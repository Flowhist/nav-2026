#!/usr/bin/env python3
import os
import signal
import shlex
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict

from state_store import StateStore


def _cmdline_references_script(args: list[str], cwd: Path, script: Path) -> bool:
    target = script.resolve()
    for arg in args:
        if not arg.endswith(".sh"):
            continue
        candidate = Path(arg)
        if not candidate.is_absolute():
            candidate = cwd / candidate
        if candidate.resolve() == target:
            return True
    return False


class RuntimeManager:
    def __init__(self, repo_dir: Path, state_store: StateStore) -> None:
        self.repo_dir = repo_dir
        self.workspace_dir = repo_dir.parent.parent
        self.tool_dir = repo_dir / "scripts" / "tool"
        self.runtime_dir = repo_dir / "server" / "runtime"
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.state_store = state_store
        self._lock = threading.Lock()
        self._procs: Dict[str, subprocess.Popen] = {}
        self._meta: Dict[str, Dict[str, object]] = {}
        self._log_cache: Dict[str, Dict[str, object]] = {}
        self._sync_status()

    def _clear_live_scene(self) -> None:
        now = time.time()
        self.state_store.update_status(
            {
                "robot": {
                    "pose_map": None,
                    "plan": {"points": 0, "length_m": 0.0, "updated_at": now},
                    "goal_pose": None,
                    "initial_pose": None,
                }
            }
        )
        self.state_store.update_scene(
            {
                "map": None,
                "scan": {
                    "frame_id": "base_link",
                    "encoding": "uint16-mm-base64",
                    "pose_map": None,
                    "angle_min": 0.0,
                    "angle_increment": 0.0,
                    "count": 0,
                    "ranges_b64": "",
                    "updated_at": now,
                },
                "plan": {"points": 0, "points_xy": [], "length_m": 0.0, "updated_at": now},
                "robot_pose_map": None,
                "goal_pose": None,
                "initial_pose": None,
            },
            map_changed=True,
            plan_changed=True,
        )

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            return self._snapshot_unlocked()

    def start(self, mode: str, launch_args: Dict[str, object] | None = None) -> Dict[str, object]:
        launch_file = {
            "mapping": "map.launch.py",
            "navigation": "nav.launch.py",
            "handle": "handle.launch.py",
        }.get(mode)
        executable = {"base": "base_control.py", "router": "base_control_router.py"}.get(mode)
        base_drive = mode == "base_drive"
        if not launch_file and not executable and not base_drive:
            raise ValueError(f"unsupported runtime mode: {mode}")

        if mode in {"mapping", "navigation"}:
            other = "navigation" if mode == "mapping" else "mapping"
            self.stop(other)
        elif mode == "base_drive":
            for control_mode in ("router", "handle", "base"):
                self.stop(control_mode)

        with self._lock:
            proc = self._procs.get(mode)
            if proc and proc.poll() is None:
                self.state_store.add_event("info", f"{mode} already running")
                return self._snapshot_unlocked()

            log_path = self.runtime_dir / f"{mode}.log"
            log_handle = log_path.open("ab")
            command = (
                self._build_launch_command(launch_file, launch_args=launch_args)
                if launch_file
                else self._build_base_drive_command(launch_args)
                if base_drive
                else self._build_run_command(executable, params=launch_args)
            )
            proc = subprocess.Popen(
                ["bash", "-lc", command],
                cwd=str(self.repo_dir),
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._procs[mode] = proc
            self._meta[mode] = {
                "stopping": False,
                "started_at": time.time(),
                "pid": proc.pid,
                "log_path": str(log_path),
                "launch_file": launch_file,
                "executable": executable,
                "script": "base_drive.sh" if base_drive else None,
                "launch_args": dict(launch_args or {}),
                "log_handle": log_handle,
            }
            self._sync_status()

        self.state_store.add_event(
            "info",
            f"{mode} start requested",
            {
                "pid": proc.pid,
                "launch": launch_file,
                "executable": executable,
                "script": "base_drive.sh" if base_drive else None,
                "launch_args": dict(launch_args or {}),
            },
        )
        watcher = threading.Thread(target=self._watch, args=(mode, proc), daemon=True)
        watcher.start()
        if mode == "mapping" and str((launch_args or {}).get("map_file", "")).strip():
            self.set_continued_mapping_localization(True, delay_s=0.5)
        return self.snapshot()

    def stop(self, mode: str) -> Dict[str, object]:
        if mode not in {"mapping", "navigation", "base", "handle", "router", "base_drive"}:
            raise ValueError(f"unsupported runtime mode: {mode}")

        if mode == "navigation":
            self._stop_relocate()

        proc = None
        with self._lock:
            proc = self._procs.get(mode)
            meta = self._meta.setdefault(mode, {})
            if meta.get("stopping"):
                return self._snapshot_unlocked()
            if proc is None or proc.poll() is not None:
                self._close_log_handle(mode)
                self._procs.pop(mode, None)
                meta["stopping"] = False
                meta["pid"] = None
                self._sync_status()
                if mode in {"base", "handle", "router", "base_drive"}:
                    self._cleanup_control_runtime(mode)
                return self._snapshot_unlocked()
            meta["stopping"] = True
            self._sync_status()

        self.state_store.add_event("info", f"{mode} cleanup started")
        script = self.tool_dir / ("clean_map.sh" if mode == "mapping" else "clean_nav.sh")
        errors = []

        if mode in {"base", "handle", "router", "base_drive"}:
            self._cleanup_control_runtime(mode)
        elif script.exists():
            try:
                subprocess.run(
                    ["bash", str(script)],
                    cwd=str(self.repo_dir),
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                    timeout=18,
                )
            except Exception as exc:
                errors.append(str(exc))
        else:
            errors.append(f"cleanup script not found: {script}")

        if proc and proc.poll() is None:
            self._terminate_process_group(proc)

        with self._lock:
            self._close_log_handle(mode)
            self._procs.pop(mode, None)
            meta = self._meta.setdefault(mode, {})
            meta["stopping"] = False
            meta["pid"] = None
            self._sync_status()

        if errors:
            self.state_store.add_event("warn", f"{mode} cleanup finished with warnings", {"detail": "; ".join(errors)})
        else:
            self.state_store.add_event("info", f"{mode} cleanup finished")
        if mode in {"mapping", "navigation"}:
            self._clear_live_scene()
        return self.snapshot()

    def restart(self, mode: str, launch_args: Dict[str, object] | None = None) -> Dict[str, object]:
        if mode == "base_drive":
            supervisor_pid = self._find_start_finav_pid()
            if supervisor_pid is not None:
                os.kill(supervisor_pid, signal.SIGUSR1)
                self.state_store.add_event(
                    "info",
                    "base drive restart requested",
                    {"supervisor_pid": supervisor_pid},
                )
                return self.snapshot()
            for control_mode in ("base_drive", "router", "handle", "base"):
                self.stop(control_mode)
            return self.start("base_drive", launch_args=launch_args)
        self.stop(mode)
        return self.start(mode, launch_args=launch_args)

    def stop_all(self) -> Dict[str, object]:
        self.stop("mapping")
        self.stop("navigation")
        for mode in ("base_drive", "router", "handle", "base"):
            self.stop(mode)
        self._stop_relocate()
        return self.snapshot()

    def finav_supervisor_status(self) -> Dict[str, object]:
        pid = self._find_start_finav_pid()
        return {
            "managed": pid is not None,
            "pid": pid,
            "script": str((self.repo_dir / "start_finav.sh").resolve()),
        }

    def request_finav_action(self, action: str, delay_s: float = 0.45) -> Dict[str, object]:
        signals = {
            "shutdown": signal.SIGTERM,
            "restart": signal.SIGUSR2,
        }
        target_signal = signals.get(action)
        if target_signal is None:
            raise ValueError(f"unsupported finav action: {action}")

        supervisor_pid = self._find_start_finav_pid()
        if supervisor_pid is None:
            raise RuntimeError("start_finav.sh supervisor is not running")

        self.state_store.add_event(
            "warn" if action == "shutdown" else "info",
            f"finav {action} requested",
            {"supervisor_pid": supervisor_pid},
        )

        def deliver_signal() -> None:
            time.sleep(max(0.1, float(delay_s)))
            try:
                os.kill(supervisor_pid, target_signal)
            except OSError as exc:
                self.state_store.add_event(
                    "error",
                    f"finav {action} signal failed",
                    {"detail": str(exc)},
                )

        threading.Thread(target=deliver_signal, daemon=True).start()
        return {
            "ok": True,
            "action": action,
            "supervisor_pid": supervisor_pid,
            "delay_s": delay_s,
        }

    def _find_start_finav_pid(self) -> int | None:
        script = (self.repo_dir / "start_finav.sh").resolve()
        proc_root = Path("/proc")
        for entry in proc_root.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                parts = (entry / "cmdline").read_bytes().split(b"\0")
                args = [part.decode("utf-8", errors="replace") for part in parts if part]
                cwd = (entry / "cwd").resolve()
            except OSError:
                continue
            if _cmdline_references_script(args, cwd, script):
                return int(entry.name)
        return None

    def run_relocate(
        self,
        params: Dict[str, object] | None = None,
        *,
        resume_mapping: bool = False,
    ) -> Dict[str, object]:
        """Run a one-shot global relocation (ros2 run finav nav_relocate.py).

        Relocate is not a long-running stack: nav_relocate.py auto-detects the
        running map, samples /scan, publishes /initialpose, then exits on its own.
        It can run while navigation is active, or while continued mapping is
        waiting in localization mode. In the latter case a successful match
        switches slam_toolbox back to mapping mode.
        """
        nav = self._status_for("navigation")
        mapping = self._status_for("mapping")
        mapping_args = mapping.get("launch_args", {})
        continued_mapping = bool(
            mapping["running"]
            and not mapping["stopping"]
            and isinstance(mapping_args, dict)
            and str(mapping_args.get("map_file", "")).strip()
        )
        if resume_mapping:
            if not continued_mapping:
                raise RuntimeError("continued mapping must be running before relocate")
            if not self.set_continued_mapping_localization(True):
                raise RuntimeError("failed to switch continued mapping to localization mode")
        elif not nav["running"] or nav["stopping"]:
            raise RuntimeError("navigation must be running before relocate")

        with self._lock:
            proc = self._procs.get("relocate")
            if proc and proc.poll() is None:
                self.state_store.add_event("info", "relocate already running")
                return self._snapshot_unlocked()

            log_path = self.runtime_dir / "relocate.log"
            log_handle = log_path.open("ab")
            command = self._build_run_command("nav_relocate.py", params=params)
            proc = subprocess.Popen(
                ["bash", "-lc", command],
                cwd=str(self.repo_dir),
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._procs["relocate"] = proc
            self._meta["relocate"] = {
                "stopping": False,
                "started_at": time.time(),
                "pid": proc.pid,
                "log_path": str(log_path),
                "params": dict(params or {}),
                "resume_mapping": bool(resume_mapping),
                "log_handle": log_handle,
            }
            self._sync_status()

        self.state_store.add_event(
            "info",
            "relocate start requested",
            {"pid": proc.pid, "params": dict(params or {})},
        )
        watcher = threading.Thread(target=self._watch, args=("relocate", proc), daemon=True)
        watcher.start()
        return self.snapshot()

    def set_continued_mapping_localization(
        self, enabled: bool, delay_s: float = 0.0
    ) -> bool:
        if delay_s > 0.0:
            timer = threading.Timer(
                delay_s,
                self.set_continued_mapping_localization,
                kwargs={"enabled": enabled},
            )
            timer.daemon = True
            timer.start()
            return True

        mapping = self._status_for("mapping")
        launch_args = mapping.get("launch_args", {})
        if (
            not mapping["running"]
            or mapping["stopping"]
            or not isinstance(launch_args, dict)
            or not str(launch_args.get("map_file", "")).strip()
        ):
            return False

        parts = self._build_ros_env_parts()
        parts.append(
            "ros2 service call /slam_toolbox/set_localization_mode "
            f"std_srvs/srv/SetBool \"{{data: {'true' if enabled else 'false'}}}\""
        )
        try:
            result = subprocess.run(
                ["bash", "-lc", " && ".join(parts)],
                cwd=str(self.repo_dir),
                capture_output=True,
                text=True,
                check=False,
                timeout=8,
            )
        except Exception as exc:
            self.state_store.add_event(
                "error", "continued mapping mode switch failed", {"detail": str(exc)}
            )
            return False

        normalized = (result.stdout + result.stderr).replace(" ", "").lower()
        success = result.returncode == 0 and (
            "success=true" in normalized or "success:true" in normalized
        )
        self.state_store.add_event(
            "info" if success else "error",
            (
                "continued mapping localization enabled"
                if enabled
                else "continued mapping activated"
            )
            if success
            else "continued mapping mode switch failed",
            {
                "localization": enabled,
                "detail": (result.stdout or result.stderr).strip()[-500:],
            },
        )
        return success

    def activate_continued_mapping(self, delay_s: float = 0.0) -> bool:
        return self.set_continued_mapping_localization(False, delay_s=delay_s)

    def _stop_relocate(self) -> None:
        proc = None
        with self._lock:
            proc = self._procs.get("relocate")
            if proc is None or proc.poll() is not None:
                self._close_log_handle("relocate")
                self._procs.pop("relocate", None)
                meta = self._meta.setdefault("relocate", {})
                meta["stopping"] = False
                meta["pid"] = None
                self._sync_status()
                return
            meta = self._meta.setdefault("relocate", {})
            meta["stopping"] = True
            self._sync_status()

        if proc and proc.poll() is None:
            self._terminate_process_group(proc)
        with self._lock:
            self._close_log_handle("relocate")
            self._procs.pop("relocate", None)
            meta = self._meta.setdefault("relocate", {})
            meta["stopping"] = False
            meta["pid"] = None
            self._sync_status()

    def read_log(self, mode: str, tail: int = 300) -> Dict[str, object]:
        if mode not in {"mapping", "navigation", "relocate", "base", "handle", "router", "base_drive"}:
            raise ValueError(f"unsupported runtime mode: {mode}")

        log_path = self.runtime_dir / f"{mode}.log"
        if not log_path.exists():
            return {"mode": mode, "lines": [], "path": str(log_path), "updated_at": None}

        stat = log_path.stat()
        line_limit = max(20, min(int(tail), 2000))
        with self._lock:
            cached = self._log_cache.get(mode)
            if cached and cached.get("mtime") == stat.st_mtime and cached.get("size") == stat.st_size and cached.get("tail") == line_limit:
                return {
                    "mode": mode,
                    "lines": list(cached.get("lines", [])),
                    "path": str(log_path),
                    "updated_at": stat.st_mtime,
                }

        try:
            lines = self._tail_log_lines(log_path, line_limit)
        except Exception:
            lines = []

        with self._lock:
            self._log_cache[mode] = {
                "mtime": stat.st_mtime,
                "size": stat.st_size,
                "tail": line_limit,
                "lines": list(lines),
            }
        return {"mode": mode, "lines": lines, "path": str(log_path), "updated_at": stat.st_mtime}

    def clear_log(self, mode: str) -> Dict[str, object]:
        if mode not in {"mapping", "navigation", "relocate", "base", "handle", "router", "base_drive"}:
            raise ValueError(f"unsupported runtime mode: {mode}")

        log_path = self.runtime_dir / f"{mode}.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text("", encoding="utf-8")
        with self._lock:
            self._log_cache.pop(mode, None)
        self.state_store.add_event("info", f"{mode} log cleared")
        return {"mode": mode, "path": str(log_path), "ok": True}

    def _build_ros_env_parts(self) -> list[str]:
        ros_setup = "/opt/ros/humble/setup.bash"
        local_setup = self.workspace_dir / "install" / "local_setup.bash"
        setup = self.workspace_dir / "install" / "setup.bash"
        fastdds = self.repo_dir / "config" / "fastdds_profiles.xml"

        parts = [f"source '{ros_setup}'"]
        if local_setup.exists():
            parts.append(f"source '{local_setup}'")
        elif setup.exists():
            parts.append(f"source '{setup}'")
        parts.append("export LANG='C.UTF-8'")
        parts.append("export LC_ALL='C.UTF-8'")
        parts.append("export PYTHONIOENCODING='utf-8'")
        parts.append("export PYTHONUTF8='1'")
        parts.append(f"export FINAV_REPO_DIR='{self.repo_dir}'")
        parts.append(f"export FINAV_MAPS_DIR='{self.repo_dir / 'maps'}'")
        if fastdds.exists():
            parts.append(f"export FASTRTPS_DEFAULT_PROFILES_FILE='{fastdds}'")
        return parts

    def _build_launch_command(self, launch_file: str, launch_args: Dict[str, object] | None = None) -> str:
        parts = self._build_ros_env_parts()
        launch_cmd = ["ros2", "launch", "finav", launch_file]
        for key, value in (launch_args or {}).items():
            if value is None:
                continue
            launch_cmd.append(f"{key}:={value}")
        parts.append("exec " + " ".join(shlex.quote(part) for part in launch_cmd))
        return " && ".join(parts)

    def _build_run_command(self, executable: str, params: Dict[str, object] | None = None) -> str:
        parts = self._build_ros_env_parts()
        run_cmd = ["ros2", "run", "finav", executable]
        ros_args: list[str] = []
        params = dict(params or {})
        params_file = {
            "base_control.py": "base_control.yaml",
            "base_control_router.py": "base_control.yaml",
            "nav_relocate.py": "nav_relocate.yaml",
        }.get(executable)
        if params_file:
            ros_args.extend(["--params-file", str(self.repo_dir / "config" / params_file)])
        for key, value in (params or {}).items():
            if value is None:
                continue
            ros_args.extend(["-p", f"{key}:={value}"])
        if ros_args:
            run_cmd.append("--ros-args")
            run_cmd.extend(ros_args)
        parts.append("exec " + " ".join(shlex.quote(part) for part in run_cmd))
        return " && ".join(parts)

    def _build_base_drive_command(self, params: Dict[str, object] | None = None) -> str:
        parts = self._build_ros_env_parts()
        run_cmd = ["bash", str(self.repo_dir / "base_drive.sh")]
        params = dict(params or {})
        handle_port = params.get("handle_port")
        if handle_port:
            run_cmd.extend(["--handle-port", str(handle_port)])
        parts.append("exec " + " ".join(shlex.quote(part) for part in run_cmd))
        return " && ".join(parts)

    def _cleanup_control_runtime(self, mode: str) -> None:
        patterns = {
            "base": ["base_control.py"],
            "handle": ["handle_control.py"],
            "router": ["base_control_router.py"],
            "base_drive": [
                "base_drive.sh",
                "base_control.py",
                "handle_control.py",
                "base_control_router.py",
            ],
        }.get(mode, [])
        for pattern in patterns:
            try:
                subprocess.run(
                    ["pkill", "-TERM", "-f", pattern],
                    cwd=str(self.repo_dir),
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                    timeout=2,
                )
            except Exception:
                pass

    def _status_for(self, mode: str) -> Dict[str, object]:
        proc = self._procs.get(mode)
        meta = self._meta.get(mode, {})
        running = proc is not None and proc.poll() is None
        return {
            "running": running,
            "stopping": bool(meta.get("stopping", False)),
            "started_at": meta.get("started_at"),
            "pid": meta.get("pid"),
            "log_path": meta.get("log_path"),
            "launch_args": dict(meta.get("launch_args", {})),
        }

    def _sync_status(self) -> None:
        mapping = self._status_for("mapping")
        navigation = self._status_for("navigation")
        relocate = self._status_for("relocate")
        base = self._status_for("base")
        handle = self._status_for("handle")
        router = self._status_for("router")
        base_drive = self._status_for("base_drive")
        self.state_store.update_status(
            {
                "runtime": {
                    "mapping": mapping,
                    "navigation": navigation,
                    "relocate": relocate,
                    "base": base,
                    "handle": handle,
                    "router": router,
                    "base_drive": base_drive,
                    "busy": bool(mapping["stopping"] or navigation["stopping"]),
                }
            }
        )

    def _snapshot_unlocked(self) -> Dict[str, object]:
        data = {
            "mapping": self._status_for("mapping"),
            "navigation": self._status_for("navigation"),
            "relocate": self._status_for("relocate"),
            "base": self._status_for("base"),
            "handle": self._status_for("handle"),
            "router": self._status_for("router"),
            "base_drive": self._status_for("base_drive"),
        }
        data["busy"] = bool(data["mapping"]["stopping"] or data["navigation"]["stopping"])
        return data

    def _watch(self, mode: str, proc: subprocess.Popen) -> None:
        code = proc.wait()
        completed_meta: Dict[str, object] = {}
        with self._lock:
            current = self._procs.get(mode)
            if current is not proc:
                self._close_log_handle(mode)
                return
            stopping = bool(self._meta.get(mode, {}).get("stopping", False))
            completed_meta = dict(self._meta.get(mode, {}))
            self._close_log_handle(mode)
            self._procs.pop(mode, None)
            meta = self._meta.setdefault(mode, {})
            meta["pid"] = None
            meta["stopping"] = False
            self._sync_status()

        level = "info" if stopping or code == 0 else "warn"
        message = f"{mode} process exited"
        self.state_store.add_event(level, message, {"code": code})
        if (
            mode == "relocate"
            and code == 0
            and not stopping
            and completed_meta.get("resume_mapping")
        ):
            self.activate_continued_mapping(delay_s=0.8)

    def _terminate_process_group(self, proc: subprocess.Popen) -> None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3)
            return
        except Exception:
            pass

        try:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=2)
        except Exception:
            pass

    def _close_log_handle(self, mode: str) -> None:
        meta = self._meta.get(mode)
        if not meta:
            return
        handle = meta.pop("log_handle", None)
        if handle:
            try:
                handle.close()
            except Exception:
                pass

    def _tail_log_lines(self, path: Path, limit: int, max_bytes: int = 256 * 1024) -> list[str]:
        if limit <= 0:
            return []

        chunk_size = 8192
        data = bytearray()
        size = path.stat().st_size
        with path.open("rb") as fh:
            pos = size
            while pos > 0 and data.count(b"\n") <= limit and len(data) < max_bytes:
                step = min(chunk_size, pos, max_bytes - len(data))
                pos -= step
                fh.seek(pos)
                data[:0] = fh.read(step)

        return data.decode("utf-8", errors="replace").splitlines()[-limit:]
