#!/usr/bin/env python3
"""Match voice destinations to saved route endpoints or standalone locations."""

import json
import math
from pathlib import Path
import sys
from typing import Optional

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
from editor_store import EditorStore


def route_for_destination(routes, name):
    matches = [route for route in routes
               if route.get("end_name", "").strip().casefold() == name.strip().casefold()
               and route.get("end_name", "").strip()]
    if not matches:
        raise ValueError(f"unknown route destination: {name}")
    if len(matches) != 1:
        raise ValueError(f"ambiguous route destination: {name}")
    if matches[0]["closed"]:
        raise ValueError(f"closed route has no supported destination: {name}")
    return matches[0]


class NavBridge(Node):
    def __init__(self) -> None:
        super().__init__("nav_voice_bridge")

        self.declare_parameter("voice_topic", "/nav_voice_bridge/voice_command")
        self.declare_parameter("route_command_topic", "/nav_task/command")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("status_topic", "/nav_voice_bridge/status")
        self.declare_parameter("map_file", "")
        self.declare_parameter("maps_dir", "")

        self.voice_topic = str(self.get_parameter("voice_topic").value)
        self.route_command_topic = str(self.get_parameter("route_command_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.map_file = str(self.get_parameter("map_file").value).strip()
        param_maps_dir = str(self.get_parameter("maps_dir").value).strip()
        self.maps_dir = param_maps_dir if param_maps_dir and Path(param_maps_dir).is_dir() else resolve_maps_dir()
        if not self.map_file:
            self.map_file = detect_running_map_file(self.maps_dir, require_locations=False)

        self.routes = []
        self.locations = {}

        self.pub_route = self.create_publisher(String, self.route_command_topic, 10)
        self.pub_goal = self.create_publisher(PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(String, self.voice_topic, self._on_voice_command, 10)
        self.create_service(Trigger, "/nav_voice_bridge/reload", self._on_reload)

        try:
            self._load_routes()
        except (ValueError, OSError) as exc:
            self.get_logger().warn(f"destinations unavailable: {exc}")
        self.get_logger().info(
            f"nav_voice_bridge ready | map={self.map_file} | routes={len(self.routes)} | locations={len(self.locations)}"
        )

    def _load_routes(self) -> None:
        self.routes = []
        self.locations = {}
        if not self.map_file:
            self.map_file = detect_running_map_file(self.maps_dir, require_locations=False)
        if not self.map_file or not self.maps_dir:
            raise ValueError("active map is not set")
        document = EditorStore(Path(self.maps_dir)).load(self.map_file, "", .6)
        self.routes = document["routes"]
        map_dir = Path(self.maps_dir) / self.map_file
        if (map_dir / f"{self.map_file}.editor.yaml").exists():
            self.locations = {item["name"]: item for item in document["locations"]}
        else:
            self.locations = load_locations(map_dir / f"{self.map_file}.locations.yaml")

    def _on_voice_command(self, msg: String) -> None:
        name = msg.data.strip()
        if not name:
            self.get_logger().debug("empty voice command, ignoring")
            return

        try:
            # Read current saved destinations so renames and deletions take effect immediately.
            self._load_routes()
            matching_routes = [route for route in self.routes
                               if route.get("end_name", "").casefold() == name.casefold()]
            locations = [loc for key, loc in self.locations.items()
                         if key.strip().casefold() == name.casefold()]
            if len(matching_routes) + len(locations) > 1:
                raise ValueError(f"ambiguous destination: {name}")
            if matching_routes:
                route = route_for_destination(matching_routes, name)
            elif locations:
                loc = locations[0]
                x, y, yaw = float(loc["x"]), float(loc["y"]), math.radians(float(loc.get("yaw_deg", 0)))
                if not all(math.isfinite(value) for value in (x, y, yaw)):
                    raise ValueError(f"invalid location coordinates: {name}")
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x, goal.pose.position.y = x, y
                goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                self.pub_goal.publish(goal)
                self._publish_status(f"point requested: {name}")
                return
            else:
                raise ValueError(f"unknown destination: {name}")
        except (ValueError, OSError) as exc:
            self._publish_status(str(exc))
            self.get_logger().warn(f"navigation command rejected: {exc}")
            return

        command = {"action": "start", "map_name": self.map_file, "route_id": route["id"]}
        self.pub_route.publish(String(data=json.dumps(command, ensure_ascii=False)))
        self._publish_status(f"route requested: {route['name']} -> {route['end_name']}")

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _on_reload(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._load_routes()
            response.success = True
            response.message = f"loaded {len(self.routes)} route(s), {len(self.locations)} location(s)"
        except (ValueError, OSError) as exc:
            response.success = False
            response.message = str(exc)
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
