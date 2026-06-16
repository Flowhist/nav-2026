#!/usr/bin/env python3
"""Terminal monitor for command routing and real wheel velocity."""

import argparse
import math
import os
import shutil
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import yaml


SPARK_BLOCKS = "▁▂▃▄▅▆▇█"


@dataclass
class CommandSnapshot:
    linear: float = 0.0
    angular: float = 0.0
    stamp: float = 0.0

    @property
    def nonzero(self) -> bool:
        return abs(self.linear) > 1e-4 or abs(self.angular) > 1e-4


def wheel_degps_to_twist(
    left_degps: float,
    right_degps: float,
    wheel_radius: float,
    wheel_separation: float,
    wheel_velocity_sign: float,
) -> tuple[float, float, float, float]:
    left_mps = wheel_velocity_sign * math.radians(float(left_degps)) * wheel_radius
    right_mps = wheel_velocity_sign * math.radians(float(right_degps)) * wheel_radius
    linear = (left_mps + right_mps) * 0.5
    angular = (right_mps - left_mps) / wheel_separation
    return linear, angular, left_mps, right_mps


def render_sparkline(
    values,
    *,
    width: int,
    min_value: float | None = None,
    max_value: float | None = None,
) -> str:
    width = max(1, int(width))
    samples = list(values)[-width:]
    if len(samples) < width:
        samples = [0.0] * (width - len(samples)) + samples

    lo = min(samples) if min_value is None else float(min_value)
    hi = max(samples) if max_value is None else float(max_value)
    span = max(abs(lo), abs(hi), 1e-6)
    lo, hi = -span, span

    zero_idx = int(round((0.0 - lo) / (hi - lo) * (len(SPARK_BLOCKS) - 1)))
    chars = []
    for value in samples:
        idx = int(round((float(value) - lo) / (hi - lo) * (len(SPARK_BLOCKS) - 1)))
        idx = max(0, min(len(SPARK_BLOCKS) - 1, idx))
        chars.append("0" if idx == zero_idx and abs(float(value)) < 1e-5 else SPARK_BLOCKS[idx])
    return "".join(chars)


def infer_command_source(
    *,
    cmd: CommandSnapshot,
    inputs: dict[str, CommandSnapshot],
    now: float,
    timeout: float,
) -> str:
    if not cmd.nonzero:
        return "stop"

    best_name = "keyboard/router"
    best_score = float("inf")
    for name, snap in inputs.items():
        if snap.stamp <= 0.0 or now - snap.stamp > timeout:
            continue
        score = abs(cmd.linear - snap.linear) + abs(cmd.angular - snap.angular)
        if score < best_score:
            best_name = name
            best_score = score

    return best_name if best_score < 1e-3 else "keyboard/router"


def _twist_to_snapshot(msg) -> CommandSnapshot:
    return CommandSnapshot(float(msg.linear.x), float(msg.angular.z), time.monotonic())


def _find_config(cli_path: str | None) -> Path:
    candidates = []
    if cli_path:
        candidates.append(Path(cli_path).expanduser())
    if os.environ.get("FINAV_REPO_DIR"):
        candidates.append(Path(os.environ["FINAV_REPO_DIR"]) / "config" / "base_control.yaml")
    cwd = Path.cwd()
    candidates.extend(
        [
            cwd / "config" / "base_control.yaml",
            cwd / "src" / "finav" / "config" / "base_control.yaml",
            Path("/home/embotic/nav_workspace/src/finav/config/base_control.yaml"),
        ]
    )

    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(Path(get_package_share_directory("finav")) / "config" / "base_control.yaml")
    except Exception:
        pass

    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError("base_control.yaml not found; pass --config PATH")


def _load_base_params(config_path: Path) -> dict:
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    return (data.get("base_control", {}) or {}).get("ros__parameters", {}) or {}


def _fmt_age(stamp: float) -> str:
    if stamp <= 0.0:
        return "stale"
    return f"{time.monotonic() - stamp:4.2f}s"


def run(args) -> None:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import Bool, Float64MultiArray, String

    params = _load_base_params(_find_config(args.config))
    wheel_radius = float(params.get("wheel_radius", 0.127))
    wheel_separation = float(params.get("wheel_separation", 0.6))
    sign = float(params.get("wheel_velocity_sign", -1.0))
    wheel_velocity_sign = 1.0 if sign >= 0.0 else -1.0

    class MonitorNode(Node):
        def __init__(self):
            super().__init__("base_monitor")
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            )
            cmd_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.cmd = CommandSnapshot()
            self.inputs = {
                "joystick": CommandSnapshot(),
                "web": CommandSnapshot(),
                "nav": CommandSnapshot(),
            }
            self.js_online = False
            self.base_fault = False
            self.router_status = ""
            self.router_status_stamp = 0.0
            self.wheel_stamp = 0.0
            self.left_degps = 0.0
            self.right_degps = 0.0
            self.left_hist = deque(maxlen=args.history)
            self.right_hist = deque(maxlen=args.history)

            self.create_subscription(Twist, "/cmd_vel", self._on_cmd, cmd_qos)
            self.create_subscription(Twist, "/js_cmd_vel", self._on_input("joystick"), cmd_qos)
            self.create_subscription(Twist, "/web_cmd_vel", self._on_input("web"), cmd_qos)
            self.create_subscription(Twist, "/nav_cmd_vel", self._on_input("nav"), cmd_qos)
            self.create_subscription(Bool, "/js_state", self._on_js_state, 10)
            self.create_subscription(Bool, "/base_fault", self._on_base_fault, 10)
            self.create_subscription(String, "/base_control_router/status", self._on_status, 10)
            self.create_subscription(Float64MultiArray, "/wheel_velocity_degps", self._on_wheel, qos)
            self.create_timer(1.0 / max(1.0, args.rate), self._render)

        def _on_cmd(self, msg):
            self.cmd = _twist_to_snapshot(msg)

        def _on_input(self, name):
            def callback(msg):
                self.inputs[name] = _twist_to_snapshot(msg)

            return callback

        def _on_js_state(self, msg):
            self.js_online = bool(msg.data)

        def _on_base_fault(self, msg):
            self.base_fault = bool(msg.data)

        def _on_status(self, msg):
            self.router_status = str(msg.data)
            self.router_status_stamp = time.monotonic()

        def _on_wheel(self, msg):
            if len(msg.data) < 2:
                return
            self.left_degps = float(msg.data[0])
            self.right_degps = float(msg.data[1])
            self.wheel_stamp = time.monotonic()
            _, _, left_mps, right_mps = wheel_degps_to_twist(
                self.left_degps,
                self.right_degps,
                wheel_radius,
                wheel_separation,
                wheel_velocity_sign,
            )
            self.left_hist.append(left_mps)
            self.right_hist.append(right_mps)

        def _render(self):
            width = shutil.get_terminal_size((100, 24)).columns
            graph_width = max(24, min(args.history, width - 24))
            now = time.monotonic()
            source = "fault" if self.base_fault else infer_command_source(
                cmd=self.cmd,
                inputs=self.inputs,
                now=now,
                timeout=args.source_timeout,
            )
            linear, angular, left_mps, right_mps = wheel_degps_to_twist(
                self.left_degps,
                self.right_degps,
                wheel_radius,
                wheel_separation,
                wheel_velocity_sign,
            )
            span = max(
                args.scale,
                max((abs(v) for v in self.left_hist), default=0.0),
                max((abs(v) for v in self.right_hist), default=0.0),
            )
            left_graph = render_sparkline(
                self.left_hist, width=graph_width, min_value=-span, max_value=span
            )
            right_graph = render_sparkline(
                self.right_hist, width=graph_width, min_value=-span, max_value=span
            )
            status_age = _fmt_age(self.router_status_stamp)
            wheel_age = _fmt_age(self.wheel_stamp)

            lines = [
                "\033[2J\033[H"
                "FINAV BASE MONITOR".ljust(width),
                "cmd_vel   "
                f"linear={self.cmd.linear:+.3f} m/s  angular={self.cmd.angular:+.3f} rad/s  "
                f"source={source:<15} age={_fmt_age(self.cmd.stamp)}",
                "router    "
                f"js={'online' if self.js_online else 'offline':<7} "
                f"fault={'YES' if self.base_fault else 'no ':<3} "
                f"status={self.router_status or '-'} ({status_age})",
                "real      "
                f"linear={linear:+.3f} m/s  angular={angular:+.3f} rad/s  "
                f"wheel_age={wheel_age}",
                "",
                f"left  {left_mps:+.3f} m/s {self.left_degps:+7.1f} deg/s |{left_graph}|",
                f"right {right_mps:+.3f} m/s {self.right_degps:+7.1f} deg/s |{right_graph}|",
                "",
                "Ctrl-C to exit. Wheel data comes from base_control.py /wheel_velocity_degps.",
            ]
            sys.stdout.write("\n".join(line[:width] for line in lines))
            sys.stdout.flush()

    rclpy.init()
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.stdout.write("\033[0m\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor finav base command and wheel velocity.")
    parser.add_argument("--config", help="Path to config/base_control.yaml")
    parser.add_argument("--rate", type=float, default=10.0, help="Terminal refresh rate in Hz")
    parser.add_argument("--history", type=int, default=80, help="Number of wheel samples to plot")
    parser.add_argument("--scale", type=float, default=0.2, help="Minimum graph scale in m/s")
    parser.add_argument("--source-timeout", type=float, default=0.45, help="Input source freshness window")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
