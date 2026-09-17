#!/usr/bin/env python3
"""Service entry points. Never search for or kill another service's processes."""

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys


def load_arguments(state_dir: Path, mode: str) -> dict:
    path = state_dir / f"{mode}.json"
    if not path.exists():
        return {}
    values = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(values, dict):
        raise ValueError("Service arguments must be an object")
    for key, value in values.items():
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key):
            raise ValueError(f"Invalid argument name: {key}")
        if value is not None and not isinstance(value, (str, int, float, bool)):
            raise ValueError(f"Invalid argument value: {key}")
    return values


def launch_command(mode: str, values: dict) -> list[str]:
    filename = {"mapping": "map.launch.py", "navigation": "nav.launch.py"}[mode]
    if mode == "navigation" and not str(values.get("map_file", "")).strip():
        raise ValueError("Navigation requires an explicit map_file")
    command = ["ros2", "launch", "finav", filename]
    for key, value in values.items():
        if value is not None:
            rendered = str(value).lower() if isinstance(value, bool) else str(value)
            command.append(f"{key}:={rendered}")
    return command


def check_process_ownership(mode: str, proc_root: Path = Path("/proc")) -> None:
    names = {
        "base": {"base_control.py", "handle_control.py", "base_control_router.py"},
        "mapping": {"map.launch.py", "nav.launch.py"},
        "navigation": {"map.launch.py", "nav.launch.py"},
        "web": {"run_server.py"},
    }[mode]
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry / "cmdline").read_bytes().split(b"\0")
        except (OSError, PermissionError):
            continue
        if any(Path(arg.decode("utf-8", errors="replace")).name in names for arg in args if arg):
            raise RuntimeError(f"Existing {mode} process PID {entry.name}; stop the old runtime before starting services")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("web", "mapping", "navigation"))
    args = parser.parse_args()
    repo_dir = Path(os.environ["FINAV_REPO_DIR"])
    state_dir = Path(os.environ["FINAV_STATE_DIR"])
    check_process_ownership(args.mode)
    if args.mode == "web":
        os.environ["FINAV_RUNTIME_BACKEND"] = "systemd"
        values = load_arguments(state_dir, "web")
        command = [sys.executable, str(repo_dir / "server" / "run_server.py"),
                   "--host", str(values.get("host", "0.0.0.0")),
                   "--port", str(values.get("port", 8010))]
    else:
        command = launch_command(args.mode, load_arguments(state_dir, args.mode))
    os.execvp(command[0], command)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
