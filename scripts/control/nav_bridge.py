#!/usr/bin/env python3
"""Location-to-goal bridge for voice navigation.

Subscribes to voice commands (std_msgs/String) with location names,
looks up coordinates from the active map's .locations.yaml,
and publishes PoseStamped to /goal_pose.
"""

import math
import os
from pathlib import Path
from typing import Dict, Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _resolve_maps_dir(param_val: str) -> str:
    if param_val and os.path.isdir(param_val):
        return param_val
    env_maps = os.environ.get("FINAV_MAPS_DIR", "").strip()
    if env_maps and os.path.isdir(env_maps):
        return env_maps
    repo = os.environ.get("FINAV_REPO_DIR", "").strip()
    if repo:
        candidate = os.path.join(repo, "maps")
        if os.path.isdir(candidate):
            return candidate
    return ""


class NavBridge(Node):
    def __init__(self) -> None:
        super().__init__("nav_bridge")

        self.declare_parameter("voice_topic", "/nav_bridge/voice_command")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("status_topic", "/nav_bridge/status")
        self.declare_parameter("map_file", "")
        self.declare_parameter("maps_dir", "")

        self.voice_topic = str(self.get_parameter("voice_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.map_file = str(self.get_parameter("map_file").value).strip()
        self.maps_dir = _resolve_maps_dir(str(self.get_parameter("maps_dir").value).strip())

        self.locations: Dict[str, Dict[str, float]] = {}

        self.pub_goal = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(String, self.voice_topic, self._on_voice_command, 10)
        self.create_service(Trigger, "/nav_bridge/reload", self._on_reload)

        self._load_locations()
        self.get_logger().info(
            f"nav_bridge ready | map={self.map_file} | locations={len(self.locations)}"
        )

    def _locations_path(self) -> Optional[Path]:
        if not self.map_file or not self.maps_dir:
            return None
        return Path(self.maps_dir) / self.map_file / f"{self.map_file}.locations.yaml"

    def _load_locations(self) -> None:
        self.locations.clear()
        path = self._locations_path()
        if path is None:
            self.get_logger().warn("map_file or maps_dir not set, no locations loaded")
            return
        if not path.exists():
            self.get_logger().warn(f"locations file not found: {path}")
            return

        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except yaml.YAMLError as e:
            self.get_logger().error(f"failed to parse {path}: {e}")
            return

        entries = data.get("locations")
        if not isinstance(entries, dict):
            self.get_logger().warn(f"no 'locations' dict in {path}")
            return

        for name, loc in entries.items():
            if not isinstance(loc, dict) or "x" not in loc or "y" not in loc:
                self.get_logger().error(f"skipping invalid location entry: {name}")
                continue
            try:
                yaw_deg = float(loc.get("yaw_deg", 0.0))
                self.locations[str(name)] = {
                    "x": float(loc["x"]),
                    "y": float(loc["y"]),
                    "yaw_rad": math.radians(yaw_deg),
                }
            except (ValueError, TypeError) as e:
                self.get_logger().error(f"bad value in location '{name}': {e}")

        self.get_logger().info(f"loaded {len(self.locations)} location(s) from {path}")

    def _on_voice_command(self, msg: String) -> None:
        name = msg.data.strip()
        if not name:
            self.get_logger().debug("empty voice command, ignoring")
            return

        if not self.locations:
            self._publish_status("no locations loaded")
            self.get_logger().warn(f"no locations, ignoring command: '{name}'")
            return

        loc = self.locations.get(name)
        if loc is None:
            lower = name.lower()
            for k, v in self.locations.items():
                if k.lower() == lower:
                    loc = v
                    break

        if loc is None:
            self._publish_status(f"unknown location: {name}")
            self.get_logger().warn(f"unknown location: {name}")
            return

        self._publish_goal(loc["x"], loc["y"], loc["yaw_rad"])
        self._publish_status(f"navigating to {name}")
        self.get_logger().info(
            f"goal set -> {name} ({loc['x']:.2f}, {loc['y']:.2f}, "
            f"{math.degrees(loc['yaw_rad']):.1f}deg)"
        )

    def _publish_goal(self, x: float, y: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.pub_goal.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _on_reload(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._load_locations()
        response.success = True
        response.message = f"loaded {len(self.locations)} location(s)"
        return response


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = NavBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
