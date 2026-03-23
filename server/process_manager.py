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

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            return self._snapshot_unlocked()

    def start(self, mode: str, launch_args: Dict[str, object] | None = None) -> Dict[str, object]:
        launch_file = {"mapping": "map.launch.py", "navigation": "nav.launch.py"}.get(mode)
        if not launch_file:
            raise ValueError(f"unsupported runtime mode: {mode}")

        other = "navigation" if mode == "mapping" else "mapping"
        self.stop(other)

        with self._lock:
            proc = self._procs.get(mode)
            if proc and proc.poll() is None:
                self.state_store.add_event("info", f"{mode} already running")
                return self._snapshot_unlocked()

            log_path = self.runtime_dir / f"{mode}.log"
            log_handle = log_path.open("ab")
            command = self._build_launch_command(launch_file, launch_args=launch_args)
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
                "launch_args": dict(launch_args or {}),
                "log_handle": log_handle,
            }
            self._sync_status()

        self.state_store.add_event(
            "info",
            f"{mode} start requested",
            {"pid": proc.pid, "launch": launch_file, "launch_args": dict(launch_args or {})},
        )
        watcher = threading.Thread(target=self._watch, args=(mode, proc), daemon=True)
        watcher.start()
        return self.snapshot()

    def stop(self, mode: str) -> Dict[str, object]:
        if mode not in {"mapping", "navigation"}:
            raise ValueError(f"unsupported runtime mode: {mode}")

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
                return self._snapshot_unlocked()
            meta["stopping"] = True
            self._sync_status()

        self.state_store.add_event("info", f"{mode} cleanup started")
        script = self.tool_dir / ("clean_map.sh" if mode == "mapping" else "clean_nav.sh")
        errors = []

        if script.exists():
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
        return self.snapshot()

    def stop_all(self) -> Dict[str, object]:
        self.stop("mapping")
        self.stop("navigation")
        return self.snapshot()

    def read_log(self, mode: str, tail: int = 300) -> Dict[str, object]:
        if mode not in {"mapping", "navigation"}:
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
        if mode not in {"mapping", "navigation"}:
            raise ValueError(f"unsupported runtime mode: {mode}")

        log_path = self.runtime_dir / f"{mode}.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text("", encoding="utf-8")
        with self._lock:
            self._log_cache.pop(mode, None)
        self.state_store.add_event("info", f"{mode} log cleared")
        return {"mode": mode, "path": str(log_path), "ok": True}

    def _build_launch_command(self, launch_file: str, launch_args: Dict[str, object] | None = None) -> str:
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
        launch_cmd = ["ros2", "launch", "finav", launch_file]
        for key, value in (launch_args or {}).items():
            if value is None:
                continue
            launch_cmd.append(f"{key}:={value}")
        parts.append("exec " + " ".join(shlex.quote(part) for part in launch_cmd))
        return " && ".join(parts)

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
        self.state_store.update_status(
            {
                "runtime": {
                    "mapping": mapping,
                    "navigation": navigation,
                    "busy": bool(mapping["stopping"] or navigation["stopping"]),
                }
            }
        )

    def _snapshot_unlocked(self) -> Dict[str, object]:
        data = {
            "mapping": self._status_for("mapping"),
            "navigation": self._status_for("navigation"),
        }
        data["busy"] = bool(data["mapping"]["stopping"] or data["navigation"]["stopping"])
        return data

    def _watch(self, mode: str, proc: subprocess.Popen) -> None:
        code = proc.wait()
        with self._lock:
            current = self._procs.get(mode)
            if current is not proc:
                self._close_log_handle(mode)
                return
            stopping = bool(self._meta.get(mode, {}).get("stopping", False))
            self._close_log_handle(mode)
            self._procs.pop(mode, None)
            meta = self._meta.setdefault(mode, {})
            meta["pid"] = None
            meta["stopping"] = False
            self._sync_status()

        level = "info" if stopping or code == 0 else "warn"
        message = f"{mode} process exited"
        self.state_store.add_event(level, message, {"code": code})

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
