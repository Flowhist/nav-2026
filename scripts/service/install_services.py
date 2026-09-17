#!/usr/bin/env python3
"""Install user services without starting hardware or enabling boot startup."""

import argparse
import json
import os
from pathlib import Path
import subprocess
from base_control.service_units import render_base_unit


def unit_quote(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"').replace("%", "%%") + '"'


def render_units(repo_dir: Path, state_dir: Path) -> dict[str, str]:
    units = {}
    units["base_control.service"] = render_base_unit(repo_dir.parent.parent / "install/local_setup.bash", state_dir)
    for mode in ("web", "mapping", "navigation"):
        relationships = "PartOf=finav.target\n"
        if mode in ("mapping", "navigation"):
            other = "navigation" if mode == "mapping" else "mapping"
            relationships += f"Conflicts=finav-{other}.service\nAfter=finav-{other}.service\n"
            # Avoid ordering cycles: mapping orders after navigation for both transitions.
            if mode == "navigation":
                relationships = relationships.replace("After=finav-mapping.service\n", "")
            relationships += "BindsTo=base_control.service\nAfter=base_control.service\n"
        units[f"finav-{mode}.service"] = (
            f"[Unit]\nDescription=Finav {mode}\n{relationships}\n"
            "[Service]\nType=exec\n"
            f"Environment={unit_quote('FINAV_REPO_DIR=' + str(repo_dir))}\n"
            f"Environment={unit_quote('FINAV_STATE_DIR=' + str(state_dir))}\n"
            "Environment=FINAV_RUNTIME_BACKEND=systemd\n"
            f"ExecStart=/bin/bash {unit_quote(str(repo_dir / 'scripts/service/run_service.sh'))} {mode}\n"
            "KillMode=control-group\nKillSignal=SIGINT\nTimeoutStopSec=15\n"
            f"Restart={'on-failure' if mode == 'web' else 'no'}\nRestartSec=3\n"
            "StandardInput=null\nUMask=0077\n"
        )
    units["finav.target"] = (
        "[Unit]\nDescription=base_control and web services\n"
        "Wants=base_control.service finav-web.service\n"
        "After=base_control.service finav-web.service\n\n"
        "[Install]\nWantedBy=default.target\n"
    )
    return units


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, help="Only render units here; do not install or reload")
    args = parser.parse_args()
    if os.geteuid() == 0 and not args.output_dir:
        parser.error("Run the installer as the robot user, without sudo")
    repo_dir = Path(__file__).resolve().parents[2]
    config_home = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    state_home = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
    state_dir = state_home / "finav"
    output = args.output_dir or config_home / "systemd/user"
    output.mkdir(parents=True, exist_ok=True)
    if not args.output_dir:
        marker = config_home / "finav/services.json"
        if marker.exists():
            existing = json.loads(marker.read_text(encoding="utf-8"))
            if existing.get("repo_dir") != str(repo_dir):
                parser.error("Finav services are already installed for another checkout")
        elif any((output / name).exists() for name in render_units(repo_dir, state_dir)):
            parser.error("Existing Finav units have no ownership marker; inspect them before installing")
    for name, content in render_units(repo_dir, state_dir).items():
        (output / name).write_text(content, encoding="utf-8")
        print(output / name)
    if args.output_dir:
        return
    state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text(json.dumps({"repo_dir": str(repo_dir), "state_dir": str(state_dir)}, indent=2) + "\n", encoding="utf-8")
    subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
    print("Installed only. No service was started or enabled.")
    print("Boot startup: sudo loginctl enable-linger " + os.environ.get("USER", "<user>"))
    print("Then: systemctl --user enable finav.target")
    print("Start after checking the robot: systemctl --user start finav.target")


if __name__ == "__main__":
    main()
