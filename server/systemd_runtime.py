#!/usr/bin/env python3
"""Web adapter for independently owned Finav user services."""

from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time

from process_manager import RuntimeManager


UNITS = {mode: f"finav-{mode}.service" for mode in ("base", "web", "mapping", "navigation")}
UNITS["base"] = "base_control.service"
BASE_MODES = {"base", "base_drive", "handle", "router"}
MANAGED_MODES = BASE_MODES | {"mapping", "navigation"}


def service_marker() -> Path:
    return Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "finav/services.json"


def create_runtime(repo_dir, state_store):
    marker = service_marker()
    backend = os.environ.get("FINAV_RUNTIME_BACKEND", "systemd" if marker.exists() else "process")
    if backend == "process":
        if marker.exists():
            raise RuntimeError("Services are installed; refusing a second process supervisor")
        return RuntimeManager(repo_dir, state_store)
    if backend != "systemd":
        raise ValueError(f"Unknown FINAV_RUNTIME_BACKEND: {backend}")
    state_dir = Path(os.environ.get("FINAV_STATE_DIR", Path.home() / ".local/state/finav"))
    if marker.exists():
        config = json.loads(marker.read_text(encoding="utf-8"))
        if Path(config["repo_dir"]).resolve() != repo_dir.resolve():
            raise RuntimeError("Installed Finav services belong to another checkout")
        state_dir = Path(config["state_dir"])
    return SystemdRuntimeManager(repo_dir, state_store, state_dir)


class SystemdRuntimeManager(RuntimeManager):
    def __init__(self, repo_dir, state_store, state_dir, *, monitor=True):
        self._services = {}
        self._service_error = None
        self._operation_lock = threading.RLock()
        self._operation_depth = 0
        self._closed = threading.Event()
        self._monitor_thread = None
        super().__init__(repo_dir, state_store)
        self.runtime_dir = Path(state_dir)
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self._refresh_services()
        if monitor:
            self._monitor_thread = threading.Thread(target=self._monitor, daemon=True)
            self._monitor_thread.start()

    def _systemctl(self, *arguments, timeout=35):
        result = subprocess.run(
            ["systemctl", "--user", *arguments], capture_output=True,
            text=True, timeout=timeout, check=False,
        )
        if result.returncode:
            raise RuntimeError((result.stderr or result.stdout).strip() or "systemctl failed")
        return result.stdout

    @contextmanager
    def _operation(self):
        # Serialize mode switches across HTTP threads and multiple Web processes.
        with self._operation_lock:
            if self._operation_depth:
                self._operation_depth += 1
                try:
                    yield
                finally:
                    self._operation_depth -= 1
                return
            with (self.runtime_dir / "service-operation.lock").open("a") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX)
                self._operation_depth = 1
                try:
                    yield
                finally:
                    self._operation_depth = 0

    def _read_arguments(self, mode):
        try:
            value = json.loads((self.runtime_dir / f"{mode}.json").read_text(encoding="utf-8"))
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError):
            return {}

    def _write_arguments(self, mode, values):
        values = dict(values or {})
        for key, value in values.items():
            if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key):
                raise ValueError(f"Invalid launch argument: {key}")
            if value is not None and not isinstance(value, (str, int, float, bool)):
                raise ValueError(f"Invalid launch argument value: {key}")
        path = self.runtime_dir / f"{mode}.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(values, ensure_ascii=True, allow_nan=False) + "\n", encoding="utf-8")
        temporary.replace(path)

    def _refresh_services(self):
        try:
            output = self._systemctl(
                "show", *UNITS.values(),
                "--property=Id,LoadState,ActiveState,SubState,MainPID,ActiveEnterTimestampMonotonic,Result",
                timeout=5,
            )
            services = {}
            for block in output.strip().split("\n\n"):
                props = dict(line.split("=", 1) for line in block.splitlines() if "=" in line)
                unit = props.get("Id")
                if unit not in UNITS.values():
                    continue
                mode = next(key for key, name in UNITS.items() if name == unit)
                active = props.get("ActiveState", "unknown")
                entered = int(props.get("ActiveEnterTimestampMonotonic", "0")) / 1_000_000
                services[mode] = {
                    "running": active in {"active", "activating", "reloading"},
                    "stopping": active == "deactivating",
                    "started_at": time.time() - (time.monotonic() - entered) if entered else None,
                    "pid": int(props.get("MainPID", "0")) or None,
                    "log_path": str(self.runtime_dir / f"{mode}.log"),
                    "launch_args": self._read_arguments(mode),
                    "unit": unit, "backend": "systemd", "active_state": active,
                    "load_state": props.get("LoadState"), "result": props.get("Result"),
                }
            with self._lock:
                self._services = services
                self._service_error = None
                self._sync_status()
        except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
            with self._lock:
                self._service_error = str(exc)
                self._services = {}
                self._sync_status()

    def _monitor(self):
        while not self._closed.wait(1.0):
            self._refresh_services()

    def _status_for(self, mode):
        if mode == "relocate":
            return super()._status_for(mode)
        key = "base" if mode in BASE_MODES else mode
        return dict(self._services.get(key, {
            "running": False, "stopping": False, "started_at": None,
            "pid": None, "log_path": None, "launch_args": {},
            "backend": "systemd", "active_state": "unknown", "error": self._service_error,
        }))

    def start(self, mode, launch_args=None):
        if mode not in MANAGED_MODES:
            raise ValueError(f"Unsupported runtime mode: {mode}")
        if mode in BASE_MODES and launch_args:
            raise ValueError("Configure the base service through base_control/config/base_control.yaml and base_control/config/handle.yaml")
        if mode == "navigation" and not str((launch_args or {}).get("map_file", "")).strip():
            raise ValueError("Navigation requires a selected map")
        key = "base" if mode in BASE_MODES else mode
        with self._operation():
            self._refresh_services()
            if self._service_error:
                raise RuntimeError(self._service_error)
            if self._status_for(key)["running"]:
                return self.snapshot()
            if mode in {"mapping", "navigation"}:
                self._stop_relocate()
                other = "navigation" if mode == "mapping" else "mapping"
                self._write_arguments(mode, launch_args)
                self._systemctl("stop", UNITS[other])
                self._clear_live_scene()
            self._systemctl("start", UNITS[key])
            self._refresh_services()
        self.state_store.add_event("info", f"{mode} service started")
        return self.snapshot()

    def stop(self, mode):
        if mode not in MANAGED_MODES:
            raise ValueError(f"Unsupported runtime mode: {mode}")
        key = "base" if mode in BASE_MODES else mode
        with self._operation():
            self._stop_relocate()
            self._systemctl("stop", UNITS[key])
            self._refresh_services()
            if mode in {"mapping", "navigation"} or mode in BASE_MODES:
                self._clear_live_scene()
        self.state_store.add_event("info", f"{mode} service stopped")
        return self.snapshot()

    def restart(self, mode, launch_args=None):
        if mode not in MANAGED_MODES:
            raise ValueError(f"Unsupported runtime mode: {mode}")
        with self._operation():
            values = self._read_arguments(mode) if launch_args is None else launch_args
            self.stop(mode)
            return self.start(mode, values)

    def stop_all(self):
        with self._operation():
            self._stop_relocate()
            self._systemctl("stop", UNITS["mapping"], UNITS["navigation"], UNITS["base"])
            self._refresh_services()
            self._clear_live_scene()
        return self.snapshot()

    def close(self):
        """Closing Web only stops its own one-shot relocation helper."""
        self._closed.set()
        if self._monitor_thread:
            self._monitor_thread.join(timeout=6)
        self._stop_relocate()

    def read_log(self, mode, tail=300):
        return super().read_log("base" if mode in BASE_MODES else mode, tail)

    def clear_log(self, mode):
        return super().clear_log("base" if mode in BASE_MODES else mode)

    def finav_supervisor_status(self):
        return {"managed": self._service_error is None, "pid": None,
                "script": None, "backend": "systemd", "unit": "finav.target",
                "error": self._service_error}

    def request_finav_action(self, action, delay_s=0.45):
        if action not in {"shutdown", "restart"}:
            raise ValueError(f"Unsupported finav action: {action}")
        loaded = self._systemctl("show", "finav.target", "--property=LoadState", timeout=5)
        if "LoadState=loaded" not in loaded:
            raise RuntimeError("finav.target is not installed")

        def perform():
            try:
                with self._operation():
                    self._stop_relocate()
                    self._systemctl("stop", UNITS["mapping"], UNITS["navigation"])
                    self._systemctl("--no-block", "stop" if action == "shutdown" else "restart", "finav.target")
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                self.state_store.add_event("error", f"Finav {action} failed", {"detail": str(exc)})

        timer = threading.Timer(max(0.1, delay_s), perform)
        timer.daemon = True
        timer.start()
        return {"ok": True, "action": action, "backend": "systemd"}
