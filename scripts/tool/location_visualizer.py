#!/usr/bin/env python3
"""Publish location markers from .locations.yaml to RViz.

Usage:
  ros2 run finav location_visualizer.py --ros-args -p map_file:=simap
"""

import math
import os
from pathlib import Path
from typing import Dict, Optional

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _resolve_maps_dir() -> str:
    env_maps = os.environ.get("FINAV_MAPS_DIR", "").strip()
    if env_maps and os.path.isdir(env_maps):
        return env_maps
    repo = os.environ.get("FINAV_REPO_DIR", "").strip()
    if repo:
        candidate = os.path.join(repo, "maps")
        if os.path.isdir(candidate):
            return candidate
    exe = Path(__file__).resolve()
    for ancestor in exe.parents:
        src_maps = ancestor / "src" / "finav" / "maps"
        if src_maps.is_dir():
            return str(src_maps)
    for _ in range(6):
        candidate = exe.parent / "maps"
        if candidate.is_dir():
            return str(candidate)
        exe = exe.parent
    exe = Path(__file__).resolve()
    for ancestor in exe.parents:
        share_maps = ancestor / "share" / "finav" / "maps"
        if share_maps.is_dir():
            return str(share_maps)
    return ""


def _load_locations(path: Path) -> Dict[str, Dict[str, float]]:
    if not path.exists():
        return {}
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError:
        return {}
    entries = data.get("locations", {})
    if not isinstance(entries, dict):
        return {}
    result = {}
    for name, loc in entries.items():
        if isinstance(loc, dict) and "x" in loc and "y" in loc:
            result[str(name)] = {
                "x": float(loc["x"]),
                "y": float(loc["y"]),
                "yaw_deg": float(loc.get("yaw_deg", 0.0)),
            }
    return result


# Color palette for location markers
PALETTE = [
    (0.9, 0.2, 0.2),   # red
    (0.2, 0.6, 0.9),   # blue
    (0.2, 0.8, 0.3),   # green
    (0.9, 0.7, 0.1),   # gold
    (0.8, 0.3, 0.8),   # purple
    (0.1, 0.8, 0.8),   # cyan
    (0.9, 0.5, 0.2),   # orange
    (0.4, 0.5, 0.9),   # periwinkle
    (0.9, 0.3, 0.6),   # pink
    (0.3, 0.8, 0.6),   # teal
]


def _make_color(name: str, idx: int, alpha: float = 1.0) -> ColorRGBA:
    r, g, b = PALETTE[idx % len(PALETTE)]
    return ColorRGBA(r=r, g=g, b=b, a=alpha)


def _pose_at(x: float, y: float, yaw: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = 0.0
    p.orientation.z = math.sin(yaw * 0.5)
    p.orientation.w = math.cos(yaw * 0.5)
    return p


class LocationVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("location_visualizer")

        self.declare_parameter("map_file", "")
        self.declare_parameter("maps_dir", "")
        self.declare_parameter("marker_topic", "/locations")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("marker_height_m", 0.05)
        self.declare_parameter("text_height_m", 0.35)
        self.declare_parameter("arrow_length_m", 0.40)

        self.map_file = str(self.get_parameter("map_file").value).strip()
        self.maps_dir = _resolve_maps_dir()
        # Allow maps_dir parameter to override
        param_maps = str(self.get_parameter("maps_dir").value).strip()
        if param_maps and os.path.isdir(param_maps):
            self.maps_dir = param_maps
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.marker_h = float(self.get_parameter("marker_height_m").value)
        self.text_h = float(self.get_parameter("text_height_m").value)
        self.arrow_len = float(self.get_parameter("arrow_length_m").value)

        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        if not self.map_file:
            self.get_logger().warn("map_file not set, no locations to visualize")
            return

        self.locations = _load_locations(
            Path(self.maps_dir) / self.map_file / f"{self.map_file}.locations.yaml"
        )

        if self.locations:
            self._publish_markers()
            self.get_logger().info(
                f"published {len(self.locations)} location marker(s) to {self.marker_topic}"
            )
        else:
            self.get_logger().info("no locations to visualize")

    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        for idx, (name, loc) in enumerate(self.locations.items()):
            yaw = math.radians(loc["yaw_deg"])
            color = _make_color(name, idx)

            # Sphere at location
            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = now
            sphere.ns = "locations"
            sphere.id = idx * 3
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = _pose_at(loc["x"], loc["y"], 0.0)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2
            sphere.color = color
            sphere.lifetime.sec = 0  # persistent
            markers.markers.append(sphere)

            # Arrow showing yaw
            arrow = Marker()
            arrow.header.frame_id = self.map_frame
            arrow.header.stamp = now
            arrow.ns = "locations"
            arrow.id = idx * 3 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = _pose_at(loc["x"], loc["y"], yaw)
            arrow.scale.x = self.arrow_len
            arrow.scale.y = 0.06
            arrow.scale.z = 0.12
            arrow.color = color
            arrow.lifetime.sec = 0
            markers.markers.append(arrow)

            # Text label
            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = now
            text.ns = "locations"
            text.id = idx * 3 + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = loc["x"]
            text.pose.position.y = loc["y"]
            text.pose.position.z = self.text_h
            text.scale.z = self.text_h
            text.color = color
            text.text = name
            text.lifetime.sec = 0
            markers.markers.append(text)

        self.pub.publish(markers)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LocationVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
