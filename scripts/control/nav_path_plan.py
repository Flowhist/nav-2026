#!/usr/bin/env python3
"""nav_path_plan.py
轻量全局路径规划节点：
  - 订阅 /map, /goal_pose
  - 通过 TF 查询 map->base_link 当前位姿
  - 默认使用快速 2D A* 全局规划，基于膨胀地图保证安全边界
  - 保留离散航向 SE2 A* 作为可选精细规划模式
  - 发布 /plan (nav_msgs/Path, frame_id=map)
"""

import math
import time
from typing import Dict, List, Optional, Set, Tuple

try:
    import numpy as np
    from scipy import ndimage
except ImportError:
    np = None
    ndimage = None

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from path_planning import fast2d as fast2d_planner
from path_planning import se2 as se2_planner
from path_planning.fast2d import build_grid_corridor, state_allowed_by_corridor


GridIndex = Tuple[int, int]
WorldPoint = Tuple[float, float]
WorldPose = Tuple[float, float, float]
State3D = Tuple[int, int, int]


def build_inflated_grid(
    data,
    width: int,
    height: int,
    occupied_threshold: int,
    treat_unknown_as_occupied: bool,
    inflation_cells: int,
) -> List[int]:
    """Build a flat inflated occupancy grid; 1 means blocked, 0 means free."""
    size = int(width) * int(height)
    if size <= 0:
        return []

    if np is not None and ndimage is not None:
        values = np.asarray(data, dtype=np.int16)
        if values.size != size:
            values = values[:size]
        occupied = values.reshape((height, width)) >= int(occupied_threshold)
        if treat_unknown_as_occupied:
            occupied = np.logical_or(occupied, values.reshape((height, width)) < 0)
        if inflation_cells > 0 and occupied.any():
            free_mask = np.logical_not(occupied)
            distance_cells = ndimage.distance_transform_edt(free_mask)
            occupied = distance_cells <= float(inflation_cells)
        return occupied.astype(np.uint8, copy=False).ravel().tolist()

    binary = [0] * size
    for i in range(size):
        v = int(data[i])
        occupied = v >= occupied_threshold or (treat_unknown_as_occupied and v < 0)
        binary[i] = 1 if occupied else 0

    if inflation_cells <= 0:
        return binary

    offsets: List[Tuple[int, int]] = []
    r2 = inflation_cells * inflation_cells
    for dy in range(-inflation_cells, inflation_cells + 1):
        for dx in range(-inflation_cells, inflation_cells + 1):
            if dx * dx + dy * dy <= r2:
                offsets.append((dx, dy))

    inflated = binary[:]
    for y in range(height):
        row = y * width
        for x in range(width):
            if binary[row + x] == 0:
                continue
            for dx, dy in offsets:
                nx = x + dx
                ny = y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    inflated[ny * width + nx] = 1

    return inflated



class PathPlanner(Node):
    def __init__(self) -> None:
        """初始化参数、缓存、订阅发布与定时规划循环。"""
        super().__init__("path_plan")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("plan_rate_hz", 2.0)
        self.declare_parameter("planner_mode", "fast2d")
        self.declare_parameter("fast2d_allow_se2_fallback", False)

        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("treat_unknown_as_occupied", True)
        self.declare_parameter("inflation_radius_m", 0.08)
        self.declare_parameter("max_search_expansions", 200000)
        self.declare_parameter("max_planning_time_s", 1.5)
        self.declare_parameter("reject_occupied_goal", True)
        self.declare_parameter("goal_tolerance_m", 0.12)
        self.declare_parameter("align_final_pose_yaw_to_goal", False)
        self.declare_parameter("stop_on_new_goal_clear_path", True)
        self.declare_parameter("stop_before_replan_clear_path", True)
        self.declare_parameter("replan_on_map_update", True)
        self.declare_parameter("replan_deviation_m", 0.35)
        self.declare_parameter("smooth_max_segment_m", 1.6)
        self.declare_parameter("path_pose_spacing_m", 0.10)
        self.declare_parameter("heuristic_weight", 1.35)
        self.declare_parameter("heuristic_heading_weight", 0.10)

        self.declare_parameter("vehicle_front_m", 0.84)
        self.declare_parameter("vehicle_rear_m", 0.25)
        self.declare_parameter("vehicle_left_m", 0.45)
        self.declare_parameter("vehicle_right_m", 0.45)
        self.declare_parameter("vehicle_margin_m", 0.05)
        self.declare_parameter("footprint_sample_step_m", 0.08)

        self.declare_parameter("heading_bins", 24)
        self.declare_parameter("primitive_step_m", 0.18)
        self.declare_parameter("primitive_turn_bins", 1)
        self.declare_parameter("enable_turn_in_place", True)
        self.declare_parameter("turn_in_place_cost", 0.18)
        self.declare_parameter("turn_cost_weight", 0.20)
        self.declare_parameter("timing_log_interval_s", 1.0)
        self.declare_parameter("enable_search_corridor", True)
        self.declare_parameter("search_corridor_radius_m", 1.2)
        self.declare_parameter("search_corridor_max_expansions", 200000)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.plan_rate_hz = float(self.get_parameter("plan_rate_hz").value)
        self.planner_mode = str(self.get_parameter("planner_mode").value).strip().lower()
        self.fast2d_allow_se2_fallback = bool(self.get_parameter("fast2d_allow_se2_fallback").value)

        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.treat_unknown_as_occupied = bool(
            self.get_parameter("treat_unknown_as_occupied").value
        )
        self.inflation_radius_m = float(self.get_parameter("inflation_radius_m").value)
        self.max_search_expansions = int(
            self.get_parameter("max_search_expansions").value
        )
        self.max_planning_time_s = max(
            0.1, float(self.get_parameter("max_planning_time_s").value)
        )
        self.reject_occupied_goal = bool(
            self.get_parameter("reject_occupied_goal").value
        )
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)
        self.align_final_pose_yaw_to_goal = bool(
            self.get_parameter("align_final_pose_yaw_to_goal").value
        )
        self.stop_on_new_goal_clear_path = bool(
            self.get_parameter("stop_on_new_goal_clear_path").value
        )
        self.stop_before_replan_clear_path = bool(
            self.get_parameter("stop_before_replan_clear_path").value
        )
        self.replan_on_map_update = bool(
            self.get_parameter("replan_on_map_update").value
        )
        self.replan_deviation_m = float(self.get_parameter("replan_deviation_m").value)
        self.smooth_max_segment_m = float(
            self.get_parameter("smooth_max_segment_m").value
        )
        self.path_pose_spacing_m = float(
            self.get_parameter("path_pose_spacing_m").value
        )
        self.heuristic_weight = max(
            1.0, float(self.get_parameter("heuristic_weight").value)
        )
        self.heuristic_heading_weight = max(
            0.0, float(self.get_parameter("heuristic_heading_weight").value)
        )

        self.vehicle_front_m = float(self.get_parameter("vehicle_front_m").value)
        self.vehicle_rear_m = float(self.get_parameter("vehicle_rear_m").value)
        self.vehicle_left_m = float(self.get_parameter("vehicle_left_m").value)
        self.vehicle_right_m = float(self.get_parameter("vehicle_right_m").value)
        self.vehicle_margin_m = float(self.get_parameter("vehicle_margin_m").value)
        self.footprint_sample_step_m = float(
            self.get_parameter("footprint_sample_step_m").value
        )

        self.heading_bins = max(8, int(self.get_parameter("heading_bins").value))
        self.primitive_step_m = max(
            0.08, float(self.get_parameter("primitive_step_m").value)
        )
        self.primitive_turn_bins = max(
            0, int(self.get_parameter("primitive_turn_bins").value)
        )
        self.enable_turn_in_place = bool(
            self.get_parameter("enable_turn_in_place").value
        )
        self.turn_in_place_cost = max(
            0.01, float(self.get_parameter("turn_in_place_cost").value)
        )
        self.turn_cost_weight = max(
            0.0, float(self.get_parameter("turn_cost_weight").value)
        )
        self.timing_log_interval_s = max(
            0.0, float(self.get_parameter("timing_log_interval_s").value)
        )
        self.enable_search_corridor = bool(self.get_parameter("enable_search_corridor").value)
        self.search_corridor_radius_m = max(0.0, float(self.get_parameter("search_corridor_radius_m").value))
        self.search_corridor_max_expansions = max(1000, int(self.get_parameter("search_corridor_max_expansions").value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_w = 0
        self.map_h = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.inflated_grid: List[int] = []
        self.map_seq = -1
        self.map_dirty = False
        self._map_processed_dirty = True

        self.goal_pose_world: Optional[WorldPose] = None
        self.goal_dirty = False
        self.plan_failed_for_current_goal = False

        self.last_plan_poses: List[WorldPose] = []

        self.heading_step = 2.0 * math.pi / float(self.heading_bins)
        self.turn_options = sorted(set((-self.primitive_turn_bins, 0, self.primitive_turn_bins)))
        self.footprint_samples: List[WorldPoint] = []
        self._build_footprint_samples()
        self.pose_free_cache: Dict[State3D, bool] = {}
        self.active_search_corridor: Optional[Set[GridIndex]] = None
        self._last_timing_log_mono = 0.0
        self._last_wait_log_mono = 0.0
        self._reset_plan_stats()

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(Empty, "/nav_clear", self._on_nav_clear, 10)

        plan_period = 1.0 / max(self.plan_rate_hz, 0.5)
        self.create_timer(plan_period, self._plan_loop)

        self.get_logger().info(
            "path_plan started | map=%s goal=%s out=%s | footprint(front=%.2f rear=%.2f left=%.2f right=%.2f margin=%.2f)"
            % (
                self.map_topic,
                self.goal_topic,
                self.path_topic,
                self.vehicle_front_m,
                self.vehicle_rear_m,
                self.vehicle_left_m,
                self.vehicle_right_m,
                self.vehicle_margin_m,
            )
        )

    def _reset_plan_stats(self) -> None:
        """重置单次规划统计，用于低频耗时日志。"""
        self.plan_stats = {
            "expansions": 0,
            "neighbors": 0,
            "pose_checks": 0,
            "state_cache_hits": 0,
            "state_cache_misses": 0,
            "motion_checks": 0,
            "motion_samples": 0,
            "astar_s": 0.0,
            "shortcut_s": 0.0,
            "densify_s": 0.0,
        }

    def _maybe_log_plan_stats(self, success: bool, path_len: int) -> None:
        """按固定间隔打印规划性能统计，避免刷屏。"""
        if self.timing_log_interval_s <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_timing_log_mono < self.timing_log_interval_s:
            return
        self._last_timing_log_mono = now
        self.get_logger().info(
            "plan timing | ok=%s poses=%d expansions=%d neighbors=%d pose_checks=%d cache=%d/%d motion=%d samples=%d astar=%.3fs shortcut=%.3fs densify=%.3fs"
            % (
                str(success),
                path_len,
                int(self.plan_stats["expansions"]),
                int(self.plan_stats["neighbors"]),
                int(self.plan_stats["pose_checks"]),
                int(self.plan_stats["state_cache_hits"]),
                int(self.plan_stats["state_cache_misses"]),
                int(self.plan_stats["motion_checks"]),
                int(self.plan_stats["motion_samples"]),
                float(self.plan_stats["astar_s"]),
                float(self.plan_stats["shortcut_s"]),
                float(self.plan_stats["densify_s"]),
            )
        )

    def _build_footprint_samples(self) -> None:
        se2_planner.build_footprint_samples(self)
    @staticmethod
    def _sample_axis(v_min: float, v_max: float, step: float) -> List[float]:
        """在给定区间按步长生成包含两端点的一维采样序列。"""
        if step <= 0.0:
            return [v_min, v_max]
        values: List[float] = []
        n = max(1, int(math.ceil((v_max - v_min) / step)))
        for i in range(n + 1):
            t = i / max(1, n)
            values.append(v_min + (v_max - v_min) * t)
        return values

    def _on_map(self, msg: OccupancyGrid) -> None:
        """接收地图并重建二值/膨胀障碍栅格。"""
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"ignore map frame={msg.header.frame_id}, expected={self.map_frame}"
            )
            return

        if msg.info.width == 0 or msg.info.height == 0:
            return

        seq = int(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        if seq == self.map_seq and self.map_msg is not None:
            return

        self.map_msg = msg
        self.map_seq = seq
        self._map_processed_dirty = True
        if self.replan_on_map_update or (
            getattr(self, "plan_failed_for_current_goal", False)
            and getattr(self, "goal_pose_world", None) is not None
        ):
            self.map_dirty = True

    def _on_goal(self, msg: PoseStamped) -> None:
        """接收目标位姿并触发下一次重规划。"""
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"ignore goal frame={msg.header.frame_id}, expected={self.map_frame}"
            )
            return
        yaw = self._quat_to_yaw(msg.pose.orientation)
        self.goal_pose_world = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            yaw,
        )
        self.goal_dirty = True
        self.plan_failed_for_current_goal = False
        if self.stop_on_new_goal_clear_path and self.last_plan_poses:
            self.last_plan_poses = []
            self._publish_path([], None)
        self.get_logger().info(
            "new goal: (%.2f, %.2f, %.1fdeg)"
            % (
                self.goal_pose_world[0],
                self.goal_pose_world[1],
                math.degrees(self.goal_pose_world[2]),
            )
        )

    def _on_nav_clear(self, _msg: Empty) -> None:
        """清空当前导航目标与路径状态。"""
        self.goal_pose_world = None
        self.goal_dirty = False
        self.plan_failed_for_current_goal = False
        self.last_plan_poses = []
        self._publish_path([], None)
        self.get_logger().info("导航停止原因: 收到 /nav_clear，已清空目标和路径")

    def _handle_plan_failure(self, reason: str) -> None:
        """统一处理规划失败：清路径、复位标志并记录日志。"""
        # 规划失败时主动清空旧路径，避免控制器继续跟旧路径，
        # 也避免后续新 goal 被残留状态影响。
        self.last_plan_poses = []
        self.goal_dirty = False
        self.map_dirty = False
        self.plan_failed_for_current_goal = True
        self._publish_path([], None)
        self.get_logger().warn(f"导航停止原因: {reason}，已发布空路径")

    def _log_wait_reason(self, reason: str, interval_s: float = 2.0) -> None:
        """低频打印等待原因，避免 TF/地图缺失时刷屏。"""
        now = time.monotonic()
        if now - self._last_wait_log_mono < interval_s:
            return
        self._last_wait_log_mono = now
        self.get_logger().warn(reason)

    def _goal_point_is_acceptable(self, pose: WorldPose) -> bool:
        """快速拒绝明显落在障碍、未知或膨胀区的目标点。"""
        idx = self._world_to_grid((pose[0], pose[1]))
        if idx is None:
            return False
        return self._is_grid_free(idx)

    def _plan_fast2d(self, current_pose: WorldPose, goal_pose: WorldPose) -> List[WorldPose]:
        return fast2d_planner.plan_fast2d(self, current_pose, goal_pose)
    def _shortcut_path_2d(self, path: List[WorldPose]) -> List[WorldPose]:
        return fast2d_planner.shortcut_path_2d(self, path)
    def _grid_segment_is_free(self, a: WorldPose, b: WorldPose) -> bool:
        return fast2d_planner.grid_segment_is_free(self, a, b)
    def _plan_loop(self) -> None:
        """周期执行规划：判定是否需要重规划并发布路径。"""
        if self.map_msg is None:
            self._log_wait_reason("规划等待原因: 尚未收到有效 /map")
            return
        if self.goal_pose_world is None:
            return

        current_pose = self._get_robot_world_pose()
        if current_pose is None:
            self._log_wait_reason(
                f"规划等待原因: 无法获取 TF {self.map_frame}->{self.base_frame}"
            )
            return

        dist_to_goal = math.hypot(
            current_pose[0] - self.goal_pose_world[0],
            current_pose[1] - self.goal_pose_world[1],
        )
        if dist_to_goal <= self.goal_tolerance_m:
            if self.last_plan_poses:
                self.last_plan_poses = []
                self._publish_path([], None)
                self.get_logger().info(
                    "导航停止原因: 当前位姿已在目标容差内，dist=%.2fm <= %.2fm，已清空路径"
                    % (dist_to_goal, self.goal_tolerance_m)
                )
            return

        need_replan = False
        if self.goal_dirty or self.map_dirty:
            need_replan = True
        elif not self.last_plan_poses and not self.plan_failed_for_current_goal:
            need_replan = True
        elif (
            self._distance_to_path((current_pose[0], current_pose[1]), self.last_plan_poses)
            > self.replan_deviation_m
        ):
            need_replan = True

        if not need_replan:
            return
        if self.stop_before_replan_clear_path and self.last_plan_poses:
            self.last_plan_poses = []
            self._publish_path([], None)
            self.get_logger().info(
                "导航暂停原因: 触发重规划，先发布空路径让控制器停车，下一轮发布新路径"
            )
            return

        if not self._ensure_processed_map():
            self._log_wait_reason("规划等待原因: 地图膨胀结果为空，暂不能规划")
            return

        if self.reject_occupied_goal and not self._goal_point_is_acceptable(
            self.goal_pose_world
        ):
            self._reset_plan_stats()
            self._maybe_log_plan_stats(False, 0)
            self._handle_plan_failure(
                "规划失败: 目标点在障碍、未知区、膨胀区或地图外"
            )
            return

        self._reset_plan_stats()
        if self.planner_mode in ("fast2d", "2d", "grid"):
            astar_start = time.monotonic()
            pose_path = self._plan_fast2d(current_pose, self.goal_pose_world)
            self.plan_stats["astar_s"] = time.monotonic() - astar_start
            if pose_path:
                self.last_plan_poses = pose_path
                self.goal_dirty = False
                self.map_dirty = False
                self.plan_failed_for_current_goal = False
                self._publish_path(pose_path, self.goal_pose_world)
                self._maybe_log_plan_stats(True, len(pose_path))
                return
            if not self.fast2d_allow_se2_fallback:
                self._maybe_log_plan_stats(False, 0)
                self._handle_plan_failure("规划失败: fast2d 未找到可行路径")
                return
            self.get_logger().warn("规划提示: fast2d 失败，开始尝试 SE2 fallback")

        goal_direction = math.atan2(
            self.goal_pose_world[1] - current_pose[1],
            self.goal_pose_world[0] - current_pose[0],
        )
        fast_start_headings = self._merge_heading_candidates(
            primary_yaw=goal_direction,
            secondary_yaw=current_pose[2],
            limit=min(self.heading_bins, 4),
        )
        safe_start_headings = self._merge_heading_candidates(
            primary_yaw=current_pose[2],
            secondary_yaw=goal_direction,
            limit=min(self.heading_bins, 8),
        )
        if self.align_final_pose_yaw_to_goal:
            goal_headings = self._ordered_heading_bins(
                self.goal_pose_world[2], limit=self.heading_bins
            )
        else:
            goal_headings = self._merge_heading_candidates(
                primary_yaw=goal_direction,
                secondary_yaw=current_pose[2],
                limit=self.heading_bins,
            )

        start_pose = self._nearest_reachable_pose(
            current_pose,
            fast_start_headings,
            max_radius_cells=8,
        )
        if start_pose is None:
            start_pose = self._nearest_reachable_pose(
                current_pose,
                safe_start_headings,
            )
        goal_pose = self._nearest_free_pose(self.goal_pose_world, goal_headings)
        if start_pose is None or goal_pose is None:
            self._handle_plan_failure("规划失败: 起点或终点附近找不到无碰撞位姿")
            return

        corridor = self._build_search_corridor(start_pose, goal_pose)
        astar_start = time.monotonic()
        pose_path = self._astar(start_pose, goal_pose, corridor)
        if not pose_path and corridor is not None:
            self.get_logger().warn("规划提示: 走廊 SE2 失败，开始全图 SE2 搜索")
            pose_path = self._astar(start_pose, goal_pose, None)
        self.active_search_corridor = None
        self.plan_stats["astar_s"] = time.monotonic() - astar_start
        if not pose_path:
            self._maybe_log_plan_stats(False, 0)
            self._handle_plan_failure("规划失败: SE2 未找到可行路径")
            return

        # 先用 A* 找到一条安全路径，再做一次直线 shortcut 去掉冗余折点。
        shortcut_start = time.monotonic()
        pose_path = self._shortcut_path(pose_path)
        self.plan_stats["shortcut_s"] = time.monotonic() - shortcut_start
        transition_path = self._build_transition_path(current_pose, start_pose)
        if transition_path:
            pose_path = transition_path + pose_path[1:]
        densify_start = time.monotonic()
        pose_path = self._densify_path(pose_path)
        self.plan_stats["densify_s"] = time.monotonic() - densify_start

        self.last_plan_poses = pose_path
        self.goal_dirty = False
        self.map_dirty = False
        self.plan_failed_for_current_goal = False

        self._publish_path(pose_path, goal_pose)
        self._maybe_log_plan_stats(True, len(pose_path))
        self.get_logger().info(
            "path published: %d poses, len=%.2fm"
            % (len(pose_path), self._path_length(pose_path))
        )

    def _ensure_processed_map(self) -> bool:
        """只在真正需要规划时处理最新地图，避免 /map 高频更新造成持续负载。"""
        if self.inflated_grid and not self._map_processed_dirty:
            return True
        if self.map_msg is None:
            return False
        self._rebuild_processed_map()
        self._map_processed_dirty = False
        return bool(self.inflated_grid)

    def _rebuild_processed_map(self) -> None:
        """将 OccupancyGrid 转为可快速查询的膨胀障碍网格。"""
        assert self.map_msg is not None
        info = self.map_msg.info
        self.map_w = int(info.width)
        self.map_h = int(info.height)
        self.resolution = float(info.resolution)
        self.origin_x = float(info.origin.position.x)
        self.origin_y = float(info.origin.position.y)

        inflation_cells = max(
            0, int(math.ceil(self.inflation_radius_m / max(self.resolution, 1e-6)))
        )
        start = time.monotonic()
        self.inflated_grid = build_inflated_grid(
            self.map_msg.data,
            self.map_w,
            self.map_h,
            self.occupied_threshold,
            self.treat_unknown_as_occupied,
            inflation_cells,
        )
        elapsed = time.monotonic() - start
        backend = "scipy" if np is not None and ndimage is not None else "python"
        self.get_logger().info(
            "按需地图膨胀完成 | size=%dx%d inflation_cells=%d backend=%s time=%.3fs"
            % (self.map_w, self.map_h, inflation_cells, backend, elapsed)
        )
        self.pose_free_cache.clear()

    def _is_grid_free(self, idx: GridIndex) -> bool:
        """判断栅格索引是否在地图内且为空闲。"""
        x, y = idx
        if x < 0 or y < 0 or x >= self.map_w or y >= self.map_h:
            return False
        return self.inflated_grid[y * self.map_w + x] == 0

    def _world_to_grid(self, p: WorldPoint) -> Optional[GridIndex]:
        """世界坐标转栅格坐标，越界时返回 None。"""
        gx = int((p[0] - self.origin_x) / self.resolution)
        gy = int((p[1] - self.origin_y) / self.resolution)
        if gx < 0 or gy < 0 or gx >= self.map_w or gy >= self.map_h:
            return None
        return (gx, gy)

    def _grid_to_world(self, p: GridIndex) -> WorldPoint:
        """栅格坐标转世界坐标（栅格中心点）。"""
        x = self.origin_x + (p[0] + 0.5) * self.resolution
        y = self.origin_y + (p[1] + 0.5) * self.resolution
        return (x, y)

    def _yaw_to_bin(self, yaw: float) -> int:
        """把连续航向角量化到离散航向 bin。"""
        wrapped = self._norm_angle(yaw)
        return int(round(wrapped / self.heading_step)) % self.heading_bins

    def _bin_to_yaw(self, idx: int) -> float:
        """把离散航向 bin 反算为连续航向角。"""
        return self._norm_angle(idx * self.heading_step)

    def _pose_to_state(self, pose: WorldPose) -> Optional[State3D]:
        """把世界位姿转换为 A* 离散状态。"""
        idx = self._world_to_grid((pose[0], pose[1]))
        if idx is None:
            return None
        return (idx[0], idx[1], self._yaw_to_bin(pose[2]))

    def _state_to_pose(self, state: State3D) -> WorldPose:
        """把 A* 离散状态转换回世界位姿。"""
        x, y = self._grid_to_world((state[0], state[1]))
        return (x, y, self._bin_to_yaw(state[2]))

    def _pose_is_free(self, pose: WorldPose) -> bool:
        return se2_planner.pose_is_free(self, pose)
    def _state_is_free(self, state: State3D) -> bool:
        return se2_planner.state_is_free(self, state)
    def _ordered_heading_bins(self, center_yaw: float, limit: Optional[int] = None) -> List[int]:
        return se2_planner.ordered_heading_bins(self, center_yaw, limit)
    def _merge_heading_candidates(self, primary_yaw: float, secondary_yaw: float, limit: int) -> List[int]:
        return se2_planner.merge_heading_candidates(self, primary_yaw, secondary_yaw, limit)
    def _nearest_free_pose(self, src_pose: WorldPose, heading_candidates: List[int], max_radius_cells: int = 40) -> Optional[WorldPose]:
        return se2_planner.nearest_free_pose(self, src_pose, heading_candidates, max_radius_cells)
    def _nearest_reachable_pose(self, src_pose: WorldPose, heading_candidates: List[int], max_radius_cells: int = 40) -> Optional[WorldPose]:
        return se2_planner.nearest_reachable_pose(self, src_pose, heading_candidates, max_radius_cells)
    def _build_transition_path(self, current_pose: WorldPose, start_pose: WorldPose) -> List[WorldPose]:
        return se2_planner.build_transition_path(self, current_pose, start_pose)
    def _motion_segment_is_free(self, a: WorldPose, b: WorldPose) -> bool:
        return se2_planner.motion_segment_is_free(self, a, b)
    @staticmethod
    def _interp_angle(a: float, b: float, t: float) -> float:
        return se2_planner.interp_angle(a, b, t)
    def _heuristic(self, a: State3D, goal_idx: GridIndex, goal_pose: WorldPose) -> float:
        return se2_planner.heuristic(self, a, goal_idx, goal_pose)
    def _expand_state(self, state: State3D) -> List[Tuple[State3D, float]]:
        return se2_planner.expand_state(self, state)
    def _build_search_corridor(self, start_pose: WorldPose, goal_pose: WorldPose) -> Optional[Set[GridIndex]]:
        return se2_planner.build_search_corridor(self, start_pose, goal_pose)
    def _grid_astar(self, start: GridIndex, goal: GridIndex) -> List[GridIndex]:
        return fast2d_planner.grid_astar(self, start, goal)
    @staticmethod
    def _reconstruct_grid_path(parent: Dict[GridIndex, GridIndex], end: GridIndex) -> List[GridIndex]:
        return fast2d_planner.reconstruct_grid_path(parent, end)
    def _astar(self, start_pose: WorldPose, goal_pose: WorldPose, corridor: Optional[Set[GridIndex]] = None) -> List[WorldPose]:
        return se2_planner.astar(self, start_pose, goal_pose, corridor)
    @staticmethod
    def _reconstruct_path(parent: Dict[State3D, State3D], end: State3D) -> List[State3D]:
        return se2_planner.reconstruct_path(parent, end)
    def _straight_segment_poses(self, a: WorldPose, b: WorldPose) -> List[WorldPose]:
        """把两点间直线段按间距离散成位姿序列。"""
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return [a]
        yaw = math.atan2(dy, dx)
        step = max(self.path_pose_spacing_m, self.resolution * 0.75)
        count = max(1, int(math.ceil(dist / step)))
        poses: List[WorldPose] = []
        for i in range(count):
            t = i / count
            poses.append((a[0] + dx * t, a[1] + dy * t, yaw))
        poses.append((b[0], b[1], yaw))
        return poses

    def _shortcut_path(self, path: List[WorldPose]) -> List[WorldPose]:
        """用贪心直连进行路径 shortcut，减少冗余折点。"""
        if len(path) <= 2:
            return path

        # 贪心地把多段短边合并成更长的直线段，能减轻下游跟踪负担，
        # 同时比曲线平滑更省计算，也更容易读懂和维护。
        simplified: List[WorldPose] = [path[0]]
        i = 0
        while i < len(path) - 1:
            picked = i + 1
            for j in range(len(path) - 1, i, -1):
                if (
                    math.hypot(path[j][0] - path[i][0], path[j][1] - path[i][1])
                    <= self.smooth_max_segment_m
                ):
                    if self._motion_segment_is_free(path[i], path[j]):
                        picked = j
                        break
            simplified.append(path[picked])
            i = picked

        return self._annotate_path_yaw(simplified, self._final_goal_yaw(path[-1][2]))

    def _densify_path(self, path: List[WorldPose]) -> List[WorldPose]:
        """把稀疏路径加密到控制器更易跟踪的点间距。"""
        if len(path) <= 1:
            return path
        out: List[WorldPose] = []
        for i in range(len(path) - 1):
            segment = self._straight_segment_poses(path[i], path[i + 1])
            if out:
                out.extend(segment[1:])
            else:
                out.extend(segment)
        out = self._annotate_path_yaw(out, self._final_goal_yaw(path[-1][2]))
        return self._deduplicate_pose_path(out)

    def _final_goal_yaw(self, goal_yaw: float) -> Optional[float]:
        """根据参数决定是否强制使用 goal 朝向。"""
        return goal_yaw if self.align_final_pose_yaw_to_goal else None

    def _deduplicate_pose_path(self, path: List[WorldPose]) -> List[WorldPose]:
        """移除过密重复点，保留路径几何形状。"""
        if not path:
            return path
        min_dist = max(self.resolution * 0.15, 1e-3)
        filtered: List[WorldPose] = [path[0]]
        for pose in path[1:]:
            if (
                math.hypot(pose[0] - filtered[-1][0], pose[1] - filtered[-1][1])
                >= min_dist
            ):
                filtered.append(pose)
            else:
                filtered[-1] = pose
        return filtered

    def _annotate_path_yaw(
        self, path: List[WorldPose], goal_yaw: Optional[float] = None
    ) -> List[WorldPose]:
        """根据相邻点方向为路径补齐航向角。"""
        if not path:
            return path
        out: List[WorldPose] = []
        for i, pose in enumerate(path):
            yaw = pose[2]
            if i < len(path) - 1:
                nx, ny, _ = path[i + 1]
                yaw = math.atan2(ny - pose[1], nx - pose[0])
            elif i > 0:
                px, py, _ = path[i - 1]
                yaw = goal_yaw if goal_yaw is not None else math.atan2(
                    pose[1] - py, pose[0] - px
                )
            out.append((pose[0], pose[1], yaw))
        return out

    def _get_robot_world_pose(self) -> Optional[WorldPose]:
        """通过 TF 获取机器人在 map 坐标系下位姿。"""
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return (
                float(t.x),
                float(t.y),
                self._quat_to_yaw(q),
            )
        except TransformException:
            return None

    def _publish_path(
        self, poses: List[WorldPose], goal_pose: Optional[WorldPose]
    ) -> None:
        """发布 nav_msgs/Path；空路径时可选择发布单点 goal。"""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        if not poses:
            if goal_pose is None:
                self.path_pub.publish(msg)
                return
            poses = [goal_pose]

        for x, y, yaw in poses:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation = self._yaw_to_quaternion(yaw)
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    @staticmethod
    def _quat_to_yaw(q) -> float:
        """四元数转平面 yaw 角。"""
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """平面 yaw 角转四元数。"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    @staticmethod
    def _norm_angle(a: float) -> float:
        """把角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _path_length(path: List[WorldPose]) -> float:
        """计算路径总长度。"""
        if len(path) < 2:
            return 0.0
        s = 0.0
        for i in range(1, len(path)):
            s += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        return s

    @staticmethod
    def _distance_to_path(p: WorldPoint, path: List[WorldPose]) -> float:
        """计算点到折线路径的最短距离。"""
        if not path:
            return float("inf")
        if len(path) == 1:
            return math.hypot(p[0] - path[0][0], p[1] - path[0][1])
        best = float("inf")
        for i in range(len(path) - 1):
            ax, ay, _ = path[i]
            bx, by, _ = path[i + 1]
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy
            if l2 < 1e-12:
                d = math.hypot(p[0] - ax, p[1] - ay)
            else:
                t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2
                t = max(0.0, min(1.0, t))
                px = ax + t * dx
                py = ay + t * dy
                d = math.hypot(p[0] - px, p[1] - py)
            if d < best:
                best = d
        return best


def main(args=None) -> None:
    """节点入口函数。"""
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
