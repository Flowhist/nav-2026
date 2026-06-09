#!/usr/bin/env python3
"""Navigate to a named location from the current map's .locations.yaml.

Usage:
  ros2 run finav nav_to_location.py --map-file simap
  python3 scripts/tool/nav_to_location.py --map-file simap
"""

import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node


def _resolve_maps_dir() -> str:
    env_maps = os.environ.get("FINAV_MAPS_DIR", "").strip()
    if env_maps and os.path.isdir(env_maps):
        return env_maps
    repo = os.environ.get("FINAV_REPO_DIR", "").strip()
    if repo:
        candidate = os.path.join(repo, "maps")
        if os.path.isdir(candidate):
            return candidate
    # Prefer source-tree maps/ so edits go to the right place.
    # When installed, walk up from install/ to workspace root → src/finav/maps.
    exe = Path(__file__).resolve()
    for ancestor in exe.parents:
        src_maps = ancestor / "src" / "finav" / "maps"
        if src_maps.is_dir():
            return str(src_maps)
    # Direct source-tree walk (running from source)
    for _ in range(6):
        candidate = exe.parent / "maps"
        if candidate.is_dir():
            return str(candidate)
        exe = exe.parent
    # Fallback: install share path
    exe = Path(__file__).resolve()
    for ancestor in exe.parents:
        share_maps = ancestor / "share" / "finav" / "maps"
        if share_maps.is_dir():
            return str(share_maps)
    return ""


def _parse_ros2_param_value(output: str) -> str:
    text = output.strip()
    if not text:
        return ""
    marker = "value is:"
    if marker in text:
        text = text.split(marker, 1)[1].strip()
    return text.strip().strip("'\"")


def _ros2_param_get(node_name: str, param_name: str) -> str:
    try:
        proc = subprocess.run(
            ["ros2", "param", "get", node_name, param_name],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=1.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return ""
    if proc.returncode != 0:
        return ""
    return _parse_ros2_param_value(proc.stdout)


def _map_name_from_path(value: str) -> str:
    if not value:
        return ""
    p = Path(value)
    # slam_toolbox map_file_name usually ends with maps/<map>/<map>.
    if p.name and p.parent.name == p.name:
        return p.name
    if p.suffix in (".yaml", ".posegraph"):
        return p.stem
    return p.name


def _locations_file_exists(map_file: str, maps_dir: str) -> bool:
    return bool(map_file) and (
        Path(maps_dir) / map_file / f"{map_file}.locations.yaml"
    ).exists()


def _detect_running_map_file(maps_dir: str) -> str:
    # nav.launch.py passes the selected map_file to these nodes when they are enabled.
    for node_name in ("/nav_bridge", "/location_visualizer"):
        map_file = _ros2_param_get(node_name, "map_file")
        if _locations_file_exists(map_file, maps_dir):
            print(f"检测到当前运行地图: {map_file} ({node_name})")
            return map_file

    # slam_toolbox localization exposes map_file_name as <maps_dir>/<map>/<map>.
    map_path = _ros2_param_get("/slam_toolbox", "map_file_name")
    map_file = _map_name_from_path(map_path)
    if _locations_file_exists(map_file, maps_dir):
        print(f"检测到当前运行地图: {map_file} (/slam_toolbox)")
        return map_file

    return ""


def _load_locations(map_file: str, maps_dir: str) -> Dict[str, Dict[str, float]]:
    path = Path(maps_dir) / map_file / f"{map_file}.locations.yaml"
    if not path.exists():
        print(f"错误: 地点文件不存在: {path}")
        sys.exit(1)
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    entries = data.get("locations", {})
    if not isinstance(entries, dict) or not entries:
        print(f"错误: {path} 中没有地点定义")
        sys.exit(1)
    result = {}
    for name, loc in entries.items():
        if isinstance(loc, dict) and "x" in loc and "y" in loc:
            result[str(name)] = {
                "x": float(loc["x"]),
                "y": float(loc["y"]),
                "yaw_deg": float(loc.get("yaw_deg", 0.0)),
            }
    return result


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
        print(f"  {i}. {name}  ({loc['x']:.2f}, {loc['y']:.2f}, {loc['yaw_deg']:.1f}deg)")
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
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(self.goal_yaw * 0.5)
        msg.pose.orientation.w = math.cos(self.goal_yaw * 0.5)
        self.pub_goal.publish(msg)
        self.get_logger().info(
            f"目标已发送 -> {self.loc_name} ({self.goal_x:.2f}, {self.goal_y:.2f}, {math.degrees(self.goal_yaw):.1f}deg)"
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

    maps_dir = _resolve_maps_dir()
    if not maps_dir:
        print("未找到 maps 目录")
        sys.exit(1)

    map_file = ns.map_file.strip()
    if not map_file:
        map_file = _detect_running_map_file(maps_dir)

    if not map_file:
        # Try to auto-detect from running nodes or list available maps
        available = []
        for name in sorted(os.listdir(maps_dir)):
            d = os.path.join(maps_dir, name)
            if os.path.isdir(d) and os.path.exists(os.path.join(d, f"{name}.locations.yaml")):
                available.append(name)
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

    locations = _load_locations(map_file, maps_dir)
    loc_name, loc = _select_location(locations)

    print(f"\n正在导航到: {loc_name} ({loc['x']:.2f}, {loc['y']:.2f}, {loc['yaw_deg']:.1f}deg)")
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
