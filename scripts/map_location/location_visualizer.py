#!/usr/bin/env python3
"""Publish location markers from .locations.yaml to RViz.

Usage:
  ros2 run finav location_visualizer.py                            # auto-detect
  ros2 run finav location_visualizer.py --ros-args -p map_file:=simap
"""

import math
import sys
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from location_utils import (
    discover_maps_with_locations,
    detect_running_map_file,
    load_locations,
    resolve_maps_dir,
)

# ── 颜色调色板 ──

PALETTE = [
    (0.9, 0.2, 0.2),
    (0.2, 0.6, 0.9),
    (0.2, 0.8, 0.3),
    (0.9, 0.7, 0.1),
    (0.8, 0.3, 0.8),
    (0.1, 0.8, 0.8),
    (0.9, 0.5, 0.2),
    (0.4, 0.5, 0.9),
    (0.9, 0.3, 0.6),
    (0.3, 0.8, 0.6),
]


def _make_color(_name: str, idx: int, alpha: float = 1.0) -> ColorRGBA:
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


# ── 节点 ──


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

        self.maps_dir = resolve_maps_dir()
        param_maps = str(self.get_parameter("maps_dir").value).strip()
        if param_maps and Path(param_maps).is_dir():
            self.maps_dir = param_maps

        self.map_file = str(self.get_parameter("map_file").value).strip()
        self._auto_resolve_map_file()

        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.marker_h = float(self.get_parameter("marker_height_m").value)
        self.text_h = float(self.get_parameter("text_height_m").value)
        self.arrow_len = float(self.get_parameter("arrow_length_m").value)

        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        if not self.map_file:
            self._print_available_maps_and_exit()
            return

        self.locations = load_locations(
            Path(self.maps_dir) / self.map_file / f"{self.map_file}.locations.yaml"
        )

        if self.locations:
            self._publish_markers()
            self.get_logger().info(
                f"已发布 {len(self.locations)} 个地点标记 → {self.marker_topic}"
            )
        else:
            self.get_logger().warn(f"地图 {self.map_file} 中没有已注册的地点")

    # ── 自动检测 map_file ────────────────────────────────────────

    def _auto_resolve_map_file(self) -> None:
        if self.map_file:
            return
        # 1) 尝试从运行中的 ROS 节点自动检测
        detected = detect_running_map_file(self.maps_dir)
        if detected:
            self.get_logger().info(f"自动检测到当前地图: {detected}")
            self.map_file = detected
            return
        # 2) 终端交互式选择
        available = discover_maps_with_locations(self.maps_dir)
        if not available:
            return
        if not sys.stdin.isatty():
            return  # 非终端环境不做交互
        self._interactive_select(available)

    def _print_available_maps_and_exit(self) -> None:
        self.get_logger().error("未指定 map_file 且无法自动检测")
        available = discover_maps_with_locations(self.maps_dir)
        if available:
            print("\n包含地点的可用地图:", file=sys.stderr)
            for m in available:
                print(f"  - {m}", file=sys.stderr)
            print(
                "\n用法: ros2 run finav location_visualizer.py "
                "--ros-args -p map_file:=<地图名>",
                file=sys.stderr,
            )

    def _interactive_select(self, available: list) -> None:
        print("\n=== 包含地点的可用地图 ===")
        for i, name in enumerate(available, 1):
            print(f"  {i}. {name}")
        print("=========================")
        while True:
            try:
                choice = input("请选择地图编号 (回车退出): ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return
            if not choice:
                return
            if choice.isdigit() and 1 <= int(choice) <= len(available):
                self.map_file = available[int(choice) - 1]
                self.get_logger().info(f"已选择地图: {self.map_file}")
                return
            print(f"输入无效，请选择 1-{len(available)} 或直接回车退出")

    # ── 发布 VRML 标记 ──────────────────────────────────────────

    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        for idx, (name, loc) in enumerate(self.locations.items()):
            yaw = math.radians(loc["yaw_deg"])
            color = _make_color(name, idx)

            # 球体
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
            sphere.lifetime.sec = 0
            markers.markers.append(sphere)

            # 朝向箭头
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

            # 文字标签
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
