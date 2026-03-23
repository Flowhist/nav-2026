#!/usr/bin/env python3
"""path_plan.py
轻量全局路径规划节点：
  - 订阅 /map, /goal_pose, /initialpose
  - 通过 TF 查询 map->base_link 当前位姿
  - 地图二值化 + 膨胀
  - A* 网格搜索 + 线段裁剪 + 曲线平滑
  - 发布 /plan (nav_msgs/Path, frame_id=map)
"""

import heapq
import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)



GridIndex = Tuple[int, int]
WorldPoint = Tuple[float, float]


class PathPlanner(Node):
    def __init__(self) -> None:
        super().__init__("path_plan")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("plan_rate_hz", 2.0)

        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("treat_unknown_as_occupied", True)
        self.declare_parameter("inflation_radius_m", 0.30)
        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("max_search_expansions", 400000)
        self.declare_parameter("goal_tolerance_m", 0.12)
        self.declare_parameter("replan_on_map_update", True)
        self.declare_parameter("replan_deviation_m", 0.35)
        self.declare_parameter("smooth_max_segment_m", 1.6)
        self.declare_parameter("curve_smooth_enable", True)
        self.declare_parameter("curve_smooth_iterations", 2)
        self.declare_parameter("curve_smooth_ratio", 0.25)
        self.declare_parameter("curve_smooth_max_points", 180)
        self.declare_parameter("start_from_initialpose_when_tf_unavailable", True)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.plan_rate_hz = float(self.get_parameter("plan_rate_hz").value)

        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.treat_unknown_as_occupied = bool(
            self.get_parameter("treat_unknown_as_occupied").value
        )
        self.inflation_radius_m = float(self.get_parameter("inflation_radius_m").value)
        self.allow_diagonal = bool(self.get_parameter("allow_diagonal").value)
        self.max_search_expansions = int(
            self.get_parameter("max_search_expansions").value
        )
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)
        self.replan_on_map_update = bool(
            self.get_parameter("replan_on_map_update").value
        )
        self.replan_deviation_m = float(self.get_parameter("replan_deviation_m").value)
        self.smooth_max_segment_m = float(
            self.get_parameter("smooth_max_segment_m").value
        )
        self.curve_smooth_enable = bool(
            self.get_parameter("curve_smooth_enable").value
        )
        self.curve_smooth_iterations = int(
            self.get_parameter("curve_smooth_iterations").value
        )
        self.curve_smooth_ratio = float(self.get_parameter("curve_smooth_ratio").value)
        self.curve_smooth_max_points = int(
            self.get_parameter("curve_smooth_max_points").value
        )
        self.start_from_initialpose_when_tf_unavailable = bool(
            self.get_parameter("start_from_initialpose_when_tf_unavailable").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_w = 0
        self.map_h = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.binary_grid: List[int] = []
        self.inflated_grid: List[int] = []
        self.map_seq = -1
        self.map_dirty = False

        self.goal_world: Optional[WorldPoint] = None
        self.goal_dirty = False
        self.initialpose_world: Optional[WorldPoint] = None

        self.last_plan_world: List[WorldPoint] = []
        self.last_goal_world: Optional[WorldPoint] = None

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)

        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.initialpose_topic, self._on_initialpose, 10
        )
        self.create_subscription(Empty, "/nav_clear", self._on_nav_clear, 10)

        plan_period = 1.0 / max(self.plan_rate_hz, 0.5)
        self.create_timer(plan_period, self._plan_loop)

        self.get_logger().info(
            "path_plan started | map=%s goal=%s init=%s out=%s"
            % (self.map_topic, self.goal_topic, self.initialpose_topic, self.path_topic)
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
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
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"ignore goal frame={msg.header.frame_id}, expected={self.map_frame}"
            )
            return
        self.goal_world = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.goal_dirty = True
        self.get_logger().info(
            "new goal: (%.2f, %.2f)" % (self.goal_world[0], self.goal_world[1])
        )

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"ignore initialpose frame={msg.header.frame_id}, expected={self.map_frame}"
            )
            return
        self.initialpose_world = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        self.goal_dirty = True

    def _on_nav_clear(self, _msg: Empty) -> None:
        self.goal_world = None
        self.goal_dirty = False
        self.last_goal_world = None
        self.last_plan_world = []
        self._publish_path([], None)
        self.get_logger().info("navigation goal cleared")

    def _plan_loop(self) -> None:
        if self.map_msg is None or not self.inflated_grid:
            return
        if self.goal_world is None:
            return

        start = self._get_robot_world_pose()
        if start is None and self.start_from_initialpose_when_tf_unavailable:
            start = self.initialpose_world
        if start is None:
            return

        dist_to_goal = math.hypot(start[0] - self.goal_world[0], start[1] - self.goal_world[1])
        if dist_to_goal <= self.goal_tolerance_m:
            if self.last_plan_world:
                self.last_plan_world = []
                self._publish_path([], self.goal_world)
            return

        need_replan = False
        if self.goal_dirty or self.map_dirty or not self.last_plan_world:
            need_replan = True
        elif self._distance_to_path(start, self.last_plan_world) > self.replan_deviation_m:
            need_replan = True

        if not need_replan:
            return

        start_idx = self._world_to_grid(start)
        goal_idx = self._world_to_grid(self.goal_world)
        if start_idx is None or goal_idx is None:
            self.get_logger().warn("start or goal outside map")
            return

        start_idx = self._nearest_free(start_idx)
        goal_idx = self._nearest_free(goal_idx)
        if start_idx is None or goal_idx is None:
            self.get_logger().warn("no free start/goal found")
            return

        grid_path = self._astar(start_idx, goal_idx)
        if not grid_path:
            self.get_logger().warn("planning failed: no path")
            return

        world_path = [self._grid_to_world(p) for p in grid_path]
        world_path = self._smooth_path(world_path)
        self.last_plan_world = world_path
        self.last_goal_world = self.goal_world
        self.goal_dirty = False
        self.map_dirty = False

        self._publish_path(world_path, self.goal_world)
        self.get_logger().info(
            "path published: %d points, len=%.2fm"
            % (len(world_path), self._path_length(world_path))
        )

    def _rebuild_processed_map(self) -> None:
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

        inflation_cells = max(0, int(math.ceil(self.inflation_radius_m / self.resolution)))
        if inflation_cells == 0:
            self.binary_grid = binary
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

        self.binary_grid = binary
        self.inflated_grid = inflated

    def _is_free(self, idx: GridIndex) -> bool:
        x, y = idx
        if x < 0 or y < 0 or x >= self.map_w or y >= self.map_h:
            return False
        return self.inflated_grid[y * self.map_w + x] == 0

    def _nearest_free(self, src: GridIndex, max_radius: int = 40) -> Optional[GridIndex]:
        if self._is_free(src):
            return src
        sx, sy = src
        for r in range(1, max_radius + 1):
            for dy in range(-r, r + 1):
                for dx in (-r, r):
                    p = (sx + dx, sy + dy)
                    if self._is_free(p):
                        return p
            for dx in range(-r + 1, r):
                for dy in (-r, r):
                    p = (sx + dx, sy + dy)
                    if self._is_free(p):
                        return p
        return None

    def _world_to_grid(self, p: WorldPoint) -> Optional[GridIndex]:
        gx = int((p[0] - self.origin_x) / self.resolution)
        gy = int((p[1] - self.origin_y) / self.resolution)
        if gx < 0 or gy < 0 or gx >= self.map_w or gy >= self.map_h:
            return None
        return (gx, gy)

    def _grid_to_world(self, p: GridIndex) -> WorldPoint:
        x = self.origin_x + (p[0] + 0.5) * self.resolution
        y = self.origin_y + (p[1] + 0.5) * self.resolution
        return (x, y)

    def _neighbors(self, node: GridIndex) -> List[Tuple[GridIndex, float]]:
        x, y = node
        out: List[Tuple[GridIndex, float]] = []
        dirs4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        for dx, dy in dirs4:
            nb = (x + dx, y + dy)
            if self._is_free(nb):
                out.append((nb, 1.0))

        if self.allow_diagonal:
            diag = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
            for dx, dy in diag:
                nb = (x + dx, y + dy)
                if not self._is_free(nb):
                    continue
                # 防止穿角
                if not self._is_free((x + dx, y)) or not self._is_free((x, y + dy)):
                    continue
                out.append((nb, math.sqrt(2.0)))
        return out

    @staticmethod
    def _heuristic(a: GridIndex, b: GridIndex) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _astar(self, start: GridIndex, goal: GridIndex) -> List[GridIndex]:
        open_heap: List[Tuple[float, float, GridIndex]] = []
        heapq.heappush(open_heap, (self._heuristic(start, goal), 0.0, start))

        g_cost = {start: 0.0}
        parent: dict[GridIndex, GridIndex] = {}
        closed = set()
        expansions = 0

        while open_heap:
            _, g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            expansions += 1
            if expansions > self.max_search_expansions:
                return []

            if current == goal:
                return self._reconstruct_path(parent, goal)

            for nb, step in self._neighbors(current):
                if nb in closed:
                    continue
                ng = g + step
                if ng < g_cost.get(nb, float("inf")):
                    g_cost[nb] = ng
                    parent[nb] = current
                    f = ng + self._heuristic(nb, goal)
                    heapq.heappush(open_heap, (f, ng, nb))

        return []

    @staticmethod
    def _reconstruct_path(
        parent: dict[GridIndex, GridIndex], end: GridIndex
    ) -> List[GridIndex]:
        path = [end]
        cur = end
        while cur in parent:
            cur = parent[cur]
            path.append(cur)
        path.reverse()
        return path

    def _line_free(self, a: WorldPoint, b: WorldPoint) -> bool:
        ax, ay = a
        bx, by = b
        d = math.hypot(bx - ax, by - ay)
        steps = max(2, int(math.ceil(d / max(self.resolution * 0.5, 1e-3))))
        for i in range(steps + 1):
            t = i / steps
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            idx = self._world_to_grid((x, y))
            if idx is None or not self._is_free(idx):
                return False
        return True

    def _smooth_path(self, path: List[WorldPoint]) -> List[WorldPoint]:
        if len(path) <= 2:
            return path

        simplified: List[WorldPoint] = [path[0]]
        i = 0
        n = len(path)
        while i < n - 1:
            j = n - 1
            picked = i + 1
            while j > i + 1:
                if (
                    math.hypot(path[j][0] - path[i][0], path[j][1] - path[i][1])
                    <= self.smooth_max_segment_m
                    and self._line_free(path[i], path[j])
                ):
                    picked = j
                    break
                j -= 1
            simplified.append(path[picked])
            i = picked

        if not self.curve_smooth_enable or len(simplified) <= 2:
            return simplified

        smoothed = simplified
        iters = max(0, self.curve_smooth_iterations)
        max_points = max(4, self.curve_smooth_max_points)
        ratio = min(0.45, max(0.05, self.curve_smooth_ratio))
        for _ in range(iters):
            if len(smoothed) >= max_points:
                break
            candidate = self._chaikin_once(smoothed, ratio)
            if len(candidate) > max_points:
                break
            if not self._path_is_collision_free(candidate):
                break
            smoothed = candidate
        return smoothed

    def _chaikin_once(self, path: List[WorldPoint], ratio: float) -> List[WorldPoint]:
        if len(path) <= 2:
            return path
        out: List[WorldPoint] = [path[0]]
        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]
            q = ((1.0 - ratio) * ax + ratio * bx, (1.0 - ratio) * ay + ratio * by)
            r = (ratio * ax + (1.0 - ratio) * bx, ratio * ay + (1.0 - ratio) * by)
            out.append(q)
            out.append(r)
        out.append(path[-1])
        return self._deduplicate_path_points(out)

    def _deduplicate_path_points(self, path: List[WorldPoint]) -> List[WorldPoint]:
        if not path:
            return path
        min_dist = max(self.resolution * 0.2, 1e-3)
        filtered: List[WorldPoint] = [path[0]]
        for p in path[1:]:
            if math.hypot(p[0] - filtered[-1][0], p[1] - filtered[-1][1]) >= min_dist:
                filtered.append(p)
        if filtered[-1] != path[-1]:
            filtered.append(path[-1])
        return filtered

    def _point_is_free_world(self, p: WorldPoint) -> bool:
        idx = self._world_to_grid(p)
        if idx is None:
            return False
        return self._is_free(idx)

    def _path_is_collision_free(self, path: List[WorldPoint]) -> bool:
        if not path:
            return False
        for p in path:
            if not self._point_is_free_world(p):
                return False
        for i in range(len(path) - 1):
            if not self._line_free(path[i], path[i + 1]):
                return False
        return True

    def _get_robot_world_pose(self) -> Optional[WorldPoint]:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            t = tf.transform.translation
            return (float(t.x), float(t.y))
        except TransformException:
            return None

    def _publish_path(self, points: List[WorldPoint], goal: WorldPoint) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        if not points:
            # 发布只含终点的空壳路径，便于下游停止并更新状态
            points = [goal]

        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            yaw = 0.0
            if i < len(points) - 1:
                nx, ny = points[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            elif i > 0:
                px, py = points[i - 1]
                yaw = math.atan2(y - py, x - px)
            pose.pose.orientation = self._yaw_to_quaternion(yaw)
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    @staticmethod
    def _path_length(path: List[WorldPoint]) -> float:
        if len(path) < 2:
            return 0.0
        s = 0.0
        for i in range(1, len(path)):
            s += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        return s

    @staticmethod
    def _distance_to_path(p: WorldPoint, path: List[WorldPoint]) -> float:
        if not path:
            return float("inf")
        if len(path) == 1:
            return math.hypot(p[0] - path[0][0], p[1] - path[0][1])
        best = float("inf")
        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]
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
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
