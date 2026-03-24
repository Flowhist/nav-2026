#!/usr/bin/env python3
"""path_plan.py
轻量全局路径规划节点：
  - 订阅 /map, /goal_pose
  - 通过 TF 查询 map->base_link 当前位姿
  - 基于真实矩形车体足迹做碰撞检查
  - 使用离散航向的 SE2 A* 规划，避免仅靠圆形膨胀
  - 发布 /plan (nav_msgs/Path, frame_id=map)
"""

import heapq
import math
from typing import Dict, List, Optional, Tuple

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


GridIndex = Tuple[int, int]
WorldPoint = Tuple[float, float]
WorldPose = Tuple[float, float, float]
State3D = Tuple[int, int, int]


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

        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("treat_unknown_as_occupied", True)
        self.declare_parameter("inflation_radius_m", 0.08)
        self.declare_parameter("max_search_expansions", 200000)
        self.declare_parameter("goal_tolerance_m", 0.12)
        self.declare_parameter("align_final_pose_yaw_to_goal", False)
        self.declare_parameter("stop_on_new_goal_clear_path", True)
        self.declare_parameter("replan_on_map_update", True)
        self.declare_parameter("replan_deviation_m", 0.35)
        self.declare_parameter("smooth_max_segment_m", 1.6)
        self.declare_parameter("path_pose_spacing_m", 0.10)
        self.declare_parameter("heuristic_weight", 1.35)
        self.declare_parameter("heuristic_heading_weight", 0.10)

        self.declare_parameter("vehicle_front_m", 0.70)
        self.declare_parameter("vehicle_rear_m", 0.25)
        self.declare_parameter("vehicle_left_m", 0.31)
        self.declare_parameter("vehicle_right_m", 0.31)
        self.declare_parameter("vehicle_margin_m", 0.05)
        self.declare_parameter("footprint_sample_step_m", 0.08)

        self.declare_parameter("heading_bins", 24)
        self.declare_parameter("primitive_step_m", 0.18)
        self.declare_parameter("primitive_turn_bins", 1)
        self.declare_parameter("turn_cost_weight", 0.20)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.plan_rate_hz = float(self.get_parameter("plan_rate_hz").value)

        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.treat_unknown_as_occupied = bool(
            self.get_parameter("treat_unknown_as_occupied").value
        )
        self.inflation_radius_m = float(self.get_parameter("inflation_radius_m").value)
        self.max_search_expansions = int(
            self.get_parameter("max_search_expansions").value
        )
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)
        self.align_final_pose_yaw_to_goal = bool(
            self.get_parameter("align_final_pose_yaw_to_goal").value
        )
        self.stop_on_new_goal_clear_path = bool(
            self.get_parameter("stop_on_new_goal_clear_path").value
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
        self.turn_cost_weight = max(
            0.0, float(self.get_parameter("turn_cost_weight").value)
        )

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

        self.goal_pose_world: Optional[WorldPose] = None
        self.goal_dirty = False

        self.last_plan_poses: List[WorldPose] = []

        self.heading_step = 2.0 * math.pi / float(self.heading_bins)
        self.turn_options = sorted(set((-self.primitive_turn_bins, 0, self.primitive_turn_bins)))
        self.footprint_samples: List[WorldPoint] = []
        self._build_footprint_samples()

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

    def _build_footprint_samples(self) -> None:
        """按矩形车体与边距离散采样，生成碰撞检测足迹点集。"""
        dx_step = max(0.04, self.footprint_sample_step_m)
        dy_step = max(0.04, self.footprint_sample_step_m)
        x_min = -self.vehicle_rear_m - self.vehicle_margin_m
        x_max = self.vehicle_front_m + self.vehicle_margin_m
        y_min = -self.vehicle_right_m - self.vehicle_margin_m
        y_max = self.vehicle_left_m + self.vehicle_margin_m

        xs = self._sample_axis(x_min, x_max, dx_step)
        ys = self._sample_axis(y_min, y_max, dy_step)
        self.footprint_samples = [(x, y) for x in xs for y in ys]

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
        self._rebuild_processed_map()
        if self.replan_on_map_update:
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
        self.last_plan_poses = []
        self._publish_path([], None)
        self.get_logger().info("navigation goal cleared")

    def _handle_plan_failure(self, reason: str) -> None:
        """统一处理规划失败：清路径、复位标志并记录日志。"""
        # 规划失败时主动清空旧路径，避免控制器继续跟旧路径，
        # 也避免后续新 goal 被残留状态影响。
        self.last_plan_poses = []
        self.goal_dirty = False
        self.map_dirty = False
        self._publish_path([], None)
        self.get_logger().warn(reason)

    def _plan_loop(self) -> None:
        """周期执行规划：判定是否需要重规划并发布路径。"""
        if self.map_msg is None or not self.inflated_grid:
            return
        if self.goal_pose_world is None:
            return

        current_pose = self._get_robot_world_pose()
        if current_pose is None:
            return

        dist_to_goal = math.hypot(
            current_pose[0] - self.goal_pose_world[0],
            current_pose[1] - self.goal_pose_world[1],
        )
        if dist_to_goal <= self.goal_tolerance_m:
            if self.last_plan_poses:
                self.last_plan_poses = []
                self._publish_path([], None)
            return

        need_replan = False
        if self.goal_dirty or self.map_dirty or not self.last_plan_poses:
            need_replan = True
        elif (
            self._distance_to_path((current_pose[0], current_pose[1]), self.last_plan_poses)
            > self.replan_deviation_m
        ):
            need_replan = True

        if not need_replan:
            return

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
            self._handle_plan_failure("planning failed: no collision-free start/goal pose found")
            return

        pose_path = self._astar(start_pose, goal_pose)
        if not pose_path:
            self._handle_plan_failure("planning failed: no path")
            return

        # 先用 A* 找到一条安全路径，再做一次直线 shortcut 去掉冗余折点。
        pose_path = self._shortcut_path(pose_path)
        transition_path = self._build_transition_path(current_pose, start_pose)
        if transition_path:
            pose_path = transition_path + pose_path[1:]
        pose_path = self._densify_path(pose_path)

        self.last_plan_poses = pose_path
        self.goal_dirty = False
        self.map_dirty = False

        self._publish_path(pose_path, goal_pose)
        self.get_logger().info(
            "path published: %d poses, len=%.2fm"
            % (len(pose_path), self._path_length(pose_path))
        )

    def _rebuild_processed_map(self) -> None:
        """将 OccupancyGrid 转为可快速查询的膨胀障碍网格。"""
        assert self.map_msg is not None
        info = self.map_msg.info
        self.map_w = int(info.width)
        self.map_h = int(info.height)
        self.resolution = float(info.resolution)
        self.origin_x = float(info.origin.position.x)
        self.origin_y = float(info.origin.position.y)

        data = self.map_msg.data
        size = self.map_w * self.map_h
        binary = [0] * size
        for i in range(size):
            v = int(data[i])
            occupied = v >= self.occupied_threshold or (
                self.treat_unknown_as_occupied and v < 0
            )
            binary[i] = 1 if occupied else 0

        inflation_cells = max(
            0, int(math.ceil(self.inflation_radius_m / max(self.resolution, 1e-6)))
        )
        if inflation_cells == 0:
            self.inflated_grid = binary[:]
            return

        offsets: List[Tuple[int, int]] = []
        r2 = inflation_cells * inflation_cells
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                if dx * dx + dy * dy <= r2:
                    offsets.append((dx, dy))

        inflated = binary[:]
        for y in range(self.map_h):
            row = y * self.map_w
            for x in range(self.map_w):
                if binary[row + x] == 0:
                    continue
                for dx, dy in offsets:
                    nx = x + dx
                    ny = y + dy
                    if 0 <= nx < self.map_w and 0 <= ny < self.map_h:
                        inflated[ny * self.map_w + nx] = 1

        self.inflated_grid = inflated

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
        """判断给定位姿下整车足迹是否完全无碰撞。"""
        # 用采样后的矩形足迹判断整车是否可放置，不只检查 base_link 一个点。
        px, py, yaw = pose
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        for fx, fy in self.footprint_samples:
            wx = px + fx * cy - fy * sy
            wy = py + fx * sy + fy * cy
            idx = self._world_to_grid((wx, wy))
            if idx is None or not self._is_grid_free(idx):
                return False
        return True

    def _ordered_heading_bins(
        self, center_yaw: float, limit: Optional[int] = None
    ) -> List[int]:
        """按离中心角从近到远生成航向候选 bin 列表。"""
        center = self._yaw_to_bin(center_yaw)
        ordered: List[int] = [center]
        max_count = self.heading_bins if limit is None else max(1, limit)
        for delta in range(1, self.heading_bins):
            ordered.append((center + delta) % self.heading_bins)
            if len(ordered) >= max_count:
                break
            ordered.append((center - delta) % self.heading_bins)
            if len(ordered) >= max_count:
                break
        return ordered

    def _merge_heading_candidates(
        self, primary_yaw: float, secondary_yaw: float, limit: int
    ) -> List[int]:
        """合并两组航向候选并去重，保持优先级顺序。"""
        merged: List[int] = []
        seen = set()
        for yaw in (primary_yaw, secondary_yaw):
            for heading_bin in self._ordered_heading_bins(yaw):
                if heading_bin in seen:
                    continue
                seen.add(heading_bin)
                merged.append(heading_bin)
                if len(merged) >= limit:
                    return merged
        return merged

    def _nearest_free_pose(
        self,
        src_pose: WorldPose,
        heading_candidates: List[int],
        max_radius_cells: int = 40,
    ) -> Optional[WorldPose]:
        """在邻域内搜索最近可放置的无碰撞位姿。"""
        base_idx = self._world_to_grid((src_pose[0], src_pose[1]))
        if base_idx is None:
            return None

        for radius in range(0, max_radius_cells + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if radius > 0 and max(abs(dx), abs(dy)) != radius:
                        continue
                    gx = base_idx[0] + dx
                    gy = base_idx[1] + dy
                    if not self._is_grid_free((gx, gy)):
                        continue
                    wx, wy = self._grid_to_world((gx, gy))
                    for h in heading_candidates:
                        pose = (wx, wy, self._bin_to_yaw(h))
                        if self._pose_is_free(pose):
                            return pose
        return None

    def _nearest_reachable_pose(
        self,
        src_pose: WorldPose,
        heading_candidates: List[int],
        max_radius_cells: int = 40,
    ) -> Optional[WorldPose]:
        """搜索从当前姿态可无碰撞过渡到的起点位姿。"""
        # 起点不仅要“摆得下”，还要保证从当前姿态原地转向或短距离过渡过去的
        # 整个过程都不碰撞，避免控制器起步时原地打角擦到障碍物。
        base_idx = self._world_to_grid((src_pose[0], src_pose[1]))
        if base_idx is None:
            return None

        for radius in range(0, max_radius_cells + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if radius > 0 and max(abs(dx), abs(dy)) != radius:
                        continue
                    gx = base_idx[0] + dx
                    gy = base_idx[1] + dy
                    if not self._is_grid_free((gx, gy)):
                        continue
                    wx, wy = self._grid_to_world((gx, gy))
                    for h in heading_candidates:
                        pose = (wx, wy, self._bin_to_yaw(h))
                        if not self._pose_is_free(pose):
                            continue
                        if not self._motion_segment_is_free(src_pose, pose):
                            continue
                        return pose
        return None

    def _build_transition_path(
        self, current_pose: WorldPose, start_pose: WorldPose
    ) -> List[WorldPose]:
        """构造当前位姿到规划起点的安全过渡段。"""
        dist = math.hypot(start_pose[0] - current_pose[0], start_pose[1] - current_pose[1])
        yaw_delta = abs(self._norm_angle(start_pose[2] - current_pose[2]))
        if dist < 1e-3 and yaw_delta < math.radians(2.0):
            return []
        if not self._motion_segment_is_free(current_pose, start_pose):
            return []
        return [current_pose, start_pose]

    def _motion_segment_is_free(self, a: WorldPose, b: WorldPose) -> bool:
        """对线段+航向插值过程做离散采样碰撞检查。"""
        # 对一段运动过程做离散采样，避免“端点不撞但转弯中擦碰”的情况。
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        yaw_delta = abs(self._norm_angle(b[2] - a[2]))
        yaw_arc = max(self.vehicle_front_m + self.vehicle_rear_m, 0.2) * yaw_delta
        span = max(dist, yaw_arc)
        step = max(self.resolution * 0.5, self.footprint_sample_step_m * 0.75, 0.04)
        count = max(2, int(math.ceil(span / step)))
        for i in range(count + 1):
            t = i / count
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            yaw = self._interp_angle(a[2], b[2], t)
            if not self._pose_is_free((x, y, yaw)):
                return False
        return True

    @staticmethod
    def _interp_angle(a: float, b: float, t: float) -> float:
        """按最短角距离对两个航向做插值。"""
        delta = math.atan2(math.sin(b - a), math.cos(b - a))
        return math.atan2(math.sin(a + delta * t), math.cos(a + delta * t))

    def _heuristic(self, a: State3D, goal_idx: GridIndex, goal_pose: WorldPose) -> float:
        """A* 启发式：欧氏距离 + 轻量朝向误差代价。"""
        wx, wy = self._grid_to_world((a[0], a[1]))
        gx, gy = self._grid_to_world(goal_idx)
        dist = math.hypot(wx - gx, wy - gy)
        if self.heuristic_heading_weight <= 1e-9:
            return dist

        desired_yaw = math.atan2(goal_pose[1] - wy, goal_pose[0] - wx)
        state_yaw = self._bin_to_yaw(a[2])
        yaw_err = abs(self._norm_angle(desired_yaw - state_yaw))
        return dist + self.heuristic_heading_weight * yaw_err

    def _expand_state(self, state: State3D) -> List[Tuple[State3D, float]]:
        """按运动原语展开邻居状态并计算转移代价。"""
        pose = self._state_to_pose(state)
        out: List[Tuple[State3D, float]] = []
        for turn_bins in self.turn_options:
            new_heading_bin = (state[2] + turn_bins) % self.heading_bins
            new_yaw = self._bin_to_yaw(new_heading_bin)
            avg_yaw = self._interp_angle(pose[2], new_yaw, 0.5)
            nx = pose[0] + self.primitive_step_m * math.cos(avg_yaw)
            ny = pose[1] + self.primitive_step_m * math.sin(avg_yaw)
            nb_idx = self._world_to_grid((nx, ny))
            if nb_idx is None:
                continue
            wx, wy = self._grid_to_world(nb_idx)
            nb_pose = (wx, wy, new_yaw)
            if not self._motion_segment_is_free(pose, nb_pose):
                continue
            nb_state = (nb_idx[0], nb_idx[1], new_heading_bin)
            turn_cost = self.turn_cost_weight * abs(turn_bins)
            out.append((nb_state, self.primitive_step_m + turn_cost))
        return out

    def _astar(self, start_pose: WorldPose, goal_pose: WorldPose) -> List[WorldPose]:
        """在离散 SE2 空间执行 A* 并返回位姿路径。"""
        start_state = self._pose_to_state(start_pose)
        goal_idx = self._world_to_grid((goal_pose[0], goal_pose[1]))
        if start_state is None or goal_idx is None:
            return []

        open_heap: List[Tuple[float, float, State3D]] = []
        start_h = self._heuristic(start_state, goal_idx, goal_pose)
        heapq.heappush(open_heap, (self.heuristic_weight * start_h, 0.0, start_state))

        g_cost: Dict[State3D, float] = {start_state: 0.0}
        parent: Dict[State3D, State3D] = {}
        closed = set()
        expansions = 0
        goal_radius_cells = max(
            1, int(math.ceil(self.goal_tolerance_m / max(self.resolution, 1e-6)))
        )
        reached: Optional[State3D] = None

        while open_heap:
            _, g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            expansions += 1
            if expansions > self.max_search_expansions:
                return []

            if max(abs(current[0] - goal_idx[0]), abs(current[1] - goal_idx[1])) <= goal_radius_cells:
                reached = current
                break

            for nb, step_cost in self._expand_state(current):
                if nb in closed:
                    continue
                ng = g + step_cost
                if ng < g_cost.get(nb, float("inf")):
                    g_cost[nb] = ng
                    parent[nb] = current
                    f = ng + self.heuristic_weight * self._heuristic(nb, goal_idx, goal_pose)
                    heapq.heappush(open_heap, (f, ng, nb))

        if reached is None:
            return []

        states = self._reconstruct_path(parent, reached)
        poses = [self._state_to_pose(s) for s in states]
        if poses:
            terminal = poses[-1]
            goal_heading = goal_pose[2]
            approach_yaw = (
                math.atan2(goal_pose[1] - terminal[1], goal_pose[0] - terminal[0])
                if math.hypot(goal_pose[0] - terminal[0], goal_pose[1] - terminal[1]) > 1e-3
                else goal_heading
            )
            aligned_goal = (goal_pose[0], goal_pose[1], approach_yaw)
            if self._motion_segment_is_free(terminal, aligned_goal):
                poses.append(aligned_goal)
            else:
                poses[-1] = (terminal[0], terminal[1], approach_yaw)
        return self._annotate_path_yaw(poses, self._final_goal_yaw(goal_pose[2]))

    @staticmethod
    def _reconstruct_path(parent: Dict[State3D, State3D], end: State3D) -> List[State3D]:
        """由 parent 反向回溯得到完整状态路径。"""
        path = [end]
        cur = end
        while cur in parent:
            cur = parent[cur]
            path.append(cur)
        path.reverse()
        return path

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
