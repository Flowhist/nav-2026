#!/usr/bin/env python3
"""Navigate to a named location from the current map's .locations.yaml.

Usage:
  ros2 run finav nav_to_location.py                                        # auto-detect
  ros2 run finav nav_to_location.py --ros-args -p map_file:=simap
  python3 scripts/control/nav_to_location.py --map-file simap
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node

CONTROL_DIR = Path(__file__).resolve().parent
if str(CONTROL_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROL_DIR))
MAP_ANNOTATE_DIR = Path(__file__).resolve().parents[1] / "map_annotate"
if str(MAP_ANNOTATE_DIR) not in sys.path:
    sys.path.insert(0, str(MAP_ANNOTATE_DIR))

from nav_goal_utils import make_map_goal
from map_utils import (
    detect_running_map_file,
    discover_maps_with_locations,
    load_locations,
    resolve_maps_dir,
)


def _select_location(locations: Dict[str, Dict[str, float]]) -> Tuple[str, Dict[str, float]]:
    names = list(locations.keys())
    if not names:
        print("没有可用地点")
        sys.exit(1)

    if len(names) == 1:
        name = names[0]
        print(f"仅有一个地点: {name}")
        return name, locations[name]

    print("\n================ 可用地点列表 ================")
    for i, name in enumerate(names, start=1):
        loc = locations[name]
        print(f"  {i}. {name}  ({loc['x']:.2f}, {loc['y']:.2f}, {loc['yaw_deg']:.1f}°)")
    print("=============================================")

    while True:
        choice = input("请输入地点编号后回车: ").strip()
        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(names):
                return names[idx - 1], locations[names[idx - 1]]
        print("输入无效")


class NavToLocation(Node):
    def __init__(self, loc_name: str, x: float, y: float, yaw_deg: float) -> None:
        super().__init__("nav_to_location")
        self.loc_name = loc_name
        self.goal_x = x
        self.goal_y = y
        self.goal_yaw = math.radians(yaw_deg)

        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        self.plan_was_nonempty = False
        self.plan_empty_count = 0
        self.cmd_vel_zero_count = 0
        self.arrived = False
        self.start_time = time.monotonic()
        self.max_wait_s = 120.0
        self.plan_timeout_s = 10.0

        self.create_timer(0.2, self._check)

    def send_goal(self) -> None:
        self.pub_goal.publish(
            make_map_goal(self.get_clock(), self.goal_x, self.goal_y, self.goal_yaw)
        )
        self.get_logger().info(
            f"目标已发送 -> {self.loc_name} "
            f"({self.goal_x:.2f}, {self.goal_y:.2f}, {math.degrees(self.goal_yaw):.1f}°)"
        )

    def _on_plan(self, msg: NavPath) -> None:
        if self.arrived:
            return
        n = len(msg.poses)
        if not self.plan_was_nonempty and n > 0:
            self.plan_was_nonempty = True
            self.get_logger().info(f"收到路径 ({n} 个点)，机器人开始移动")
        if self.plan_was_nonempty and n == 0:
            self.plan_empty_count += 1
            if self.plan_empty_count >= 3:
                self.get_logger().info(f"路径已清空 → 已到达 {self.loc_name}")
                self.arrived = True

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self.arrived:
            return
        if abs(msg.linear.x) < 0.005 and abs(msg.angular.z) < 0.005:
            self.cmd_vel_zero_count += 1
        else:
            self.cmd_vel_zero_count = 0

    def _check(self) -> None:
        if self.arrived:
            self.get_logger().info(f"OK 已到达 {self.loc_name}")
            self.destroy_node()
            rclpy.shutdown()
            return

        elapsed = time.monotonic() - self.start_time
        if elapsed > self.max_wait_s:
            self.get_logger().warn(f"超时 ({self.max_wait_s}s)，退出")
            self.destroy_node()
            rclpy.shutdown()
            return

        if not self.plan_was_nonempty and elapsed > self.plan_timeout_s:
            self.get_logger().warn("规划超时，目标可能不可达")
            self.destroy_node()
            rclpy.shutdown()
            return

        if self.plan_was_nonempty and self.cmd_vel_zero_count > 15:
            self.get_logger().info(f"机器人已停止 → 已到达 {self.loc_name}")
            self.arrived = True


def main(args: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(description="按地点名称导航")
    parser.add_argument("--map-file", type=str, default="",
                        help="地图名称 (如 simap, map4)")
    ns = parser.parse_args()

    maps_dir = resolve_maps_dir()
    if not maps_dir:
        print("未找到 maps 目录")
        sys.exit(1)

    map_file = ns.map_file.strip()
    if not map_file:
        map_file = detect_running_map_file(maps_dir)

    if not map_file:
        available = discover_maps_with_locations(maps_dir)
        if not available:
            print("未找到任何包含 .locations.yaml 的地图，请用 --map-file 指定")
            sys.exit(1)

        print("\n================ 可用地图 ================")
        for i, name in enumerate(available, start=1):
            print(f"  {i}. {name}")
        print("=========================================")
        while True:
            choice = input("请选择地图编号: ").strip()
            if choice.isdigit() and 1 <= int(choice) <= len(available):
                map_file = available[int(choice) - 1]
                break
            print("输入无效")

    locations = load_locations(Path(maps_dir) / map_file / f"{map_file}.locations.yaml")
    loc_name, loc = _select_location(locations)

    print(f"\n正在导航到: {loc_name} ({loc['x']:.2f}, {loc['y']:.2f}, {loc['yaw_deg']:.1f}°)")
    print("等待到达... (Ctrl+C 中断)\n")

    rclpy.init(args=args)
    node = NavToLocation(loc_name, loc["x"], loc["y"], loc["yaw_deg"])
    node.send_goal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n已中断")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
