#!/usr/bin/env python3
"""Location-to-goal bridge for voice navigation.

Subscribes to voice commands (std_msgs/String) with location names,
looks up coordinates from the active map's .locations.yaml,
and publishes PoseStamped to /goal_pose.
"""

import math
from pathlib import Path
import sys
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

CONTROL_DIR = Path(__file__).resolve().parent
if str(CONTROL_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROL_DIR))
MAP_LOCATION_DIR = Path(__file__).resolve().parents[1] / "map_location"
if str(MAP_LOCATION_DIR) not in sys.path:
    sys.path.insert(0, str(MAP_LOCATION_DIR))

from location_utils import detect_running_map_file, load_locations, resolve_maps_dir


def set_yaw_orientation(orientation, yaw_rad: float) -> None:
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw_rad * 0.5)
    orientation.w = math.cos(yaw_rad * 0.5)


def make_map_goal(clock, x: float, y: float, yaw_rad: float, frame_id: str = "map") -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = clock.now().to_msg()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0
    set_yaw_orientation(msg.pose.orientation, yaw_rad)
    return msg


class NavBridge(Node):
    def __init__(self) -> None:
        super().__init__("nav_voice_bridge")

        self.declare_parameter("voice_topic", "/nav_voice_bridge/voice_command")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("status_topic", "/nav_voice_bridge/status")
        self.declare_parameter("map_file", "")
        self.declare_parameter("maps_dir", "")

        self.voice_topic = str(self.get_parameter("voice_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.map_file = str(self.get_parameter("map_file").value).strip()
        param_maps_dir = str(self.get_parameter("maps_dir").value).strip()
        self.maps_dir = param_maps_dir if param_maps_dir and Path(param_maps_dir).is_dir() else resolve_maps_dir()
        if not self.map_file:
            self.map_file = detect_running_map_file(self.maps_dir, require_locations=True)

        self.locations: Dict[str, Dict[str, float]] = {}

        self.pub_goal = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(String, self.voice_topic, self._on_voice_command, 10)
        self.create_service(Trigger, "/nav_voice_bridge/reload", self._on_reload)

        self._load_locations()
        self.get_logger().info(
            f"nav_voice_bridge ready | map={self.map_file} | locations={len(self.locations)}"
        )

    def _locations_path(self) -> Optional[Path]:
        if not self.map_file or not self.maps_dir:
            return None
        return Path(self.maps_dir) / self.map_file / f"{self.map_file}.locations.yaml"

    def _load_locations(self) -> None:
        self.locations.clear()
        if not self.map_file:
            self.map_file = detect_running_map_file(self.maps_dir, require_locations=True)
        path = self._locations_path()
        if path is None:
            self.get_logger().warn("map_file or maps_dir not set, no locations loaded")
            return
        if not path.exists():
            self.get_logger().warn(f"locations file not found: {path}")
            return

        for name, loc in load_locations(path).items():
            self.locations[name] = {
                "x": loc["x"],
                "y": loc["y"],
                "yaw_rad": math.radians(loc.get("yaw_deg", 0.0)),
            }

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
        self.pub_goal.publish(make_map_goal(self.get_clock(), x, y, yaw))

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
