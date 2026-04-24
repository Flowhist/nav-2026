#!/usr/bin/env python3
"""nav_control.py  ——  导航控制器
===========================================
控制策略：Pure Pursuit 目标点 + PD 角速控制

  DRIVE   |heading_error| < rotate_threshold_deg
            linear  = v_max * cos(heading_error)
            angular = Kp*err + Kd*d_err
  ROTATE  |heading_error| >= rotate_threshold_deg
            linear  = 0
            angular = Kp*err + Kd*d_err

  迟滞：DRIVE->ROTATE 进 > rotate_threshold_deg
        ROTATE->DRIVE 出 < rotate_exit_deg

目标点（纯追踪）：
  从路径上距机器人最近的投影点出发，沿路径前行 lookahead_dist，返回插值目标点
  dist(robot, path_end) < goal_tolerance -> 停止重置
"""

import math
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class DriveMode(Enum):
    DRIVE = auto()
    ROTATE = auto()


class ChassisControlNav(Node):

    def __init__(self):
        """初始化控制参数、路径缓存与控制循环。"""
        super().__init__("nav_control")

        # 参数声明
        self.declare_parameter("output_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("accept_replanned_path", True)
        self.declare_parameter("replan_pause_s", 0.20)

        # 速度
        self.declare_parameter("linear_speed_mps", 0.20)
        self.declare_parameter("angular_speed_degps", 20.0)  # ROTATE 上限
        self.declare_parameter("drive_angular_speed_degps", 20.0)  # DRIVE 上限
        self.declare_parameter("angular_kp", 0.8)
        self.declare_parameter("angular_kd", 0.4)
        self.declare_parameter("angular_deadband_deg", 5.0)
        self.declare_parameter("angular_sign", 1.0)

        # 纯追踪
        self.declare_parameter("lookahead_dist", 0.5)  # 前向预瞄距离 m
        self.declare_parameter("goal_tolerance", 0.10)  # 到达终点阈值 m

        # 模式切换
        self.declare_parameter("rotate_threshold_deg", 40.0)
        self.declare_parameter("rotate_exit_deg", 20.0)

        # 机械 & 频率
        self.declare_parameter("wheel_separation", 0.6)
        self.declare_parameter("control_rate_hz", 10.0)

        # 读取参数
        self.output_topic = str(self.get_parameter("output_cmd_vel_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.accept_replanned_path = bool(
            self.get_parameter("accept_replanned_path").value
        )
        self.replan_pause_s = max(
            0.0, float(self.get_parameter("replan_pause_s").value)
        )

        self.v_max = float(self.get_parameter("linear_speed_mps").value)
        self.omega_max = math.radians(
            float(self.get_parameter("angular_speed_degps").value)
        )
        self.drive_omega_max = math.radians(
            float(self.get_parameter("drive_angular_speed_degps").value)
        )
        self.ang_kp = float(self.get_parameter("angular_kp").value)
        self.ang_kd = float(self.get_parameter("angular_kd").value)
        self.ang_db = math.radians(
            float(self.get_parameter("angular_deadband_deg").value)
        )
        self.ang_sign = float(self.get_parameter("angular_sign").value)

        self.lookahead_dist = float(self.get_parameter("lookahead_dist").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.goal_tol_sq = self.goal_tol * self.goal_tol

        rot_thr = float(self.get_parameter("rotate_threshold_deg").value)
        rot_exit = float(self.get_parameter("rotate_exit_deg").value)
        self.thr_rotate = math.radians(rot_thr)
        self.thr_exit = math.radians(rot_exit)

        self.wheel_sep = float(self.get_parameter("wheel_separation").value)
        self.ctrl_rate = float(self.get_parameter("control_rate_hz").value)

        # 状态参数
        self.path_points: List[Tuple[float, float]] = []
        self.path_seg_len: List[float] = []
        self.path_seg_len2: List[float] = []
        self.path_frame = self.map_frame
        self._nearest_seg_hint = 0
        self.has_started = False
        self.mode: Optional[DriveMode] = None
        self._last_log_mode: Optional[DriveMode] = None
        self._prev_err: float = 0.0
        self._prev_time: int = 0
        self._replan_pause_until_ns = 0
        self._js_active = False
        self._cmd_msg = Twist()

        # 订阅与发布
        self.pub = self.create_publisher(Twist, self.output_topic, 10)  # 发布 /cmd_vel
        self.create_subscription(Path, self.path_topic, self._on_path, 10)  # 订阅 /path
        self.create_subscription(Empty, "/nav_clear", self._on_nav_clear, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / max(self.ctrl_rate, 1.0), self._loop)

        self.get_logger().info(
            f"nav_control 启动（纯追踪）| v={self.v_max*100:.0f}cm/s | "
            f"lookahead={self.lookahead_dist:.2f}m | "
            f"omega={math.degrees(self.omega_max):.0f}deg/s "
            f"drive_omega={math.degrees(self.drive_omega_max):.0f}deg/s | "
            f"Kp={self.ang_kp} Kd={self.ang_kd} | "
            f"ROTATE: >{rot_thr:.0f}° <{rot_exit:.0f}° | sign={self.ang_sign} | "
            f"accept_replan={self.accept_replanned_path} | "
            f"replan_pause={self.replan_pause_s:.2f}s"
        )

    # === 工具 ================================================================

    @staticmethod
    def _norm(a: float) -> float:
        """把角度归一化到 [-pi, pi]。"""
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _yaw(qx, qy, qz, qw) -> float:
        """四元数转平面 yaw 角。"""
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _pub(self, lin: float, ang: float):
        """发布底盘速度指令。"""
        self._cmd_msg.linear.x = float(lin)
        self._cmd_msg.angular.z = float(ang)
        self.pub.publish(self._cmd_msg)

    def _stop(self):
        """发布零速度，立即停车。"""
        self._pub(0.0, 0.0)

    def _reset(self):
        """重置路径跟踪状态机与历史误差缓存。"""
        self.path_points = []
        self.path_seg_len = []
        self.path_seg_len2 = []
        self.path_frame = self.map_frame
        self._nearest_seg_hint = 0
        self.has_started = False
        self.mode = None
        self._last_log_mode = None
        self._prev_err = 0.0
        self._prev_time = 0
        self._replan_pause_until_ns = 0

    def _start_replan_pause(self):
        """路径切换时先停车并进入短暂停顿窗口。"""
        self._stop()
        self.mode = None
        self._last_log_mode = None
        self._prev_err = 0.0
        self._prev_time = 0
        if self.replan_pause_s > 0.0:
            now_ns = self.get_clock().now().nanoseconds
            self._replan_pause_until_ns = now_ns + int(self.replan_pause_s * 1e9)
        else:
            self._replan_pause_until_ns = 0

    def _rebuild_path_cache(self):
        """重建路径段长度缓存，加速最近点与前瞻点搜索。"""
        self.path_seg_len = []
        self.path_seg_len2 = []
        n = len(self.path_points)
        if n < 2:
            self._nearest_seg_hint = 0
            return
        for i in range(n - 1):
            ax, ay = self.path_points[i]
            bx, by = self.path_points[i + 1]
            dx = bx - ax
            dy = by - ay
            l2 = dx * dx + dy * dy
            self.path_seg_len2.append(l2)
            self.path_seg_len.append(math.sqrt(l2))
        self._nearest_seg_hint = min(self._nearest_seg_hint, n - 2)

    # === 路径回调 ============================================================

    def _on_path(self, msg: Path):
        """处理路径消息：首条路径锁定或运行中重规划切换。"""
        if not msg.poses:
            if self.has_started or self.path_points:
                self._stop()
                self._reset()
                self.get_logger().info("路径已清空，导航停止")
            return
        new_points = [
            (float(point.pose.position.x), float(point.pose.position.y))
            for point in msg.poses
        ]
        new_frame = msg.header.frame_id or self.map_frame

        if not self.has_started:
            self.path_points = new_points
            self.path_frame = new_frame
            self._rebuild_path_cache()
            self.has_started = True
            self.mode = None
            self._last_log_mode = None
            self._prev_time = 0
            end = self.path_points[-1]
            self.get_logger().info(
                f"OK 路径锁定 {len(self.path_points)} 点 | "
                f"终点=({end[0]:.2f},{end[1]:.2f}) | lookahead={self.lookahead_dist:.2f}m"
            )
            return

        if not self.accept_replanned_path:
            return

        old_end = self.path_points[-1] if self.path_points else None
        self.path_points = new_points
        self.path_frame = new_frame
        self._rebuild_path_cache()
        self._start_replan_pause()

        new_end = self.path_points[-1]
        if old_end is None:
            end_shift = 0.0
        else:
            end_shift = math.hypot(new_end[0] - old_end[0], new_end[1] - old_end[1])
        self.get_logger().info(
            f"路径更新 {len(self.path_points)} 点 | "
            f"新终点=({new_end[0]:.2f},{new_end[1]:.2f}) | 终点偏移={end_shift:.2f}m | "
            f"暂停跟踪 {self.replan_pause_s:.2f}s"
        )

    def _on_nav_clear(self, _msg: Empty):
        """处理导航清空命令并复位控制状态。"""
        self._stop()
        self._reset()
        self.get_logger().info("收到导航清空指令，已急停并重置状态")

    # === 纯追踪目标点 =========================================================

    def _lookahead_target(self, x: float, y: float) -> Tuple[float, float]:
        """计算纯追踪前瞻点（最近投影点向前 lookahead_dist）。"""
        """
        1. 在路径上找距机器人最近的投影点（最近段上的垂足）
        2. 从该投影点沿路径向前走 lookahead_dist，返回目标点
        """
        path = self.path_points
        n = len(path)
        if n == 1:
            return path[0]
        if len(self.path_seg_len) != n - 1:
            self._rebuild_path_cache()

        # 步骤1：最近投影点（优先在上次最近段附近窗口搜索）
        best_d2 = float("inf")
        best_seg = 0
        best_t = 0.0

        def search_range(i_begin: int, i_end: int):
            nonlocal best_d2, best_seg, best_t
            for i in range(i_begin, i_end + 1):
                len2 = self.path_seg_len2[i]
                if len2 < 1e-12:
                    continue
                ax, ay = path[i]
                bx, by = path[i + 1]
                dx, dy = bx - ax, by - ay
                t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / len2))
                px = ax + t * dx
                py = ay + t * dy
                ddx = px - x
                ddy = py - y
                d2 = ddx * ddx + ddy * ddy
                if d2 < best_d2:
                    best_d2 = d2
                    best_seg = i
                    best_t = t

        seg_count = n - 1
        hint = min(max(self._nearest_seg_hint, 0), seg_count - 1)
        win = 24
        i0 = max(0, hint - win)
        i1 = min(seg_count - 1, hint + win)
        search_range(i0, i1)
        if (best_seg == i0 and i0 > 0) or (best_seg == i1 and i1 < seg_count - 1):
            best_d2 = float("inf")
            best_seg = 0
            best_t = 0.0
            search_range(0, seg_count - 1)

        self._nearest_seg_hint = best_seg

        # 步骤2：从投影点沿路径前进 lookahead_dist
        remain = self.lookahead_dist
        ax, ay = path[best_seg]
        bx, by = path[best_seg + 1]
        seg_len = self.path_seg_len[best_seg]
        in_seg = seg_len * (1.0 - best_t)

        if in_seg >= remain:
            t2 = best_t + remain / max(seg_len, 1e-9)
            return (ax + t2 * (bx - ax), ay + t2 * (by - ay))

        remain -= in_seg

        for i in range(best_seg + 1, n - 1):
            seg_len = self.path_seg_len[i]
            if seg_len >= remain:
                ax, ay = path[i]
                bx, by = path[i + 1]
                t2 = remain / max(seg_len, 1e-9)
                return (ax + t2 * (bx - ax), ay + t2 * (by - ay))
            remain -= seg_len

        return path[-1]

    # === 位姿获取 =============================================================

    def _get_pose(self) -> Optional[Tuple[float, float, float]]:
        """通过 TF 获取机器人在路径坐标系下位姿。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.path_frame, self.base_frame, Time()
            )
            t, q = tf.transform.translation, tf.transform.rotation
            return (
                float(t.x),
                float(t.y),
                self._yaw(float(q.x), float(q.y), float(q.z), float(q.w)),
            )
        except TransformException:
            return None

    # === 控制计算 =============================================================

    def _compute(self, heading_error: float, now: int) -> Tuple[float, float]:
        """根据当前模式计算线速度与角速度输出（DRIVE/ROTATE）。"""
        abs_err = abs(heading_error)

        # PD 微分项（仅 ROTATE 使用）
        if self._prev_time > 0:
            dt = max((now - self._prev_time) * 1e-9, 1e-3)
            d_err = (heading_error - self._prev_err) / dt
        else:
            d_err = 0.0
        self._prev_err = heading_error
        self._prev_time = now

        # 初始化模式
        if self.mode is None:
            self.mode = (
                DriveMode.ROTATE if abs_err >= self.thr_rotate else DriveMode.DRIVE
            )
            self.get_logger().info(
                f"初始模式: {self.mode.name} | 角差={math.degrees(abs_err):.1f} deg"
            )

        # 模式切换（迟滞）
        if self.mode == DriveMode.DRIVE and abs_err >= self.thr_rotate:
            self.mode = DriveMode.ROTATE
            self._prev_time = 0
            self._prev_err = 0.0
            self.get_logger().info(
                f"DRIVE -> ROTATE | 角差={math.degrees(abs_err):.1f} deg"
            )
        elif self.mode == DriveMode.ROTATE and abs_err < self.thr_exit:
            self.mode = DriveMode.DRIVE
            self._prev_time = 0
            self._prev_err = 0.0
            self.get_logger().info(
                f"ROTATE -> DRIVE | 角差={math.degrees(abs_err):.1f} deg"
            )

        # ── ROTATE：PD 原地旋转 ──────────────────────────────────────────────
        if self.mode == DriveMode.ROTATE:
            ang_pd = self.ang_kp * heading_error + self.ang_kd * d_err
            ang_raw = ang_pd * self.ang_sign
            ang = math.copysign(min(abs(ang_raw), self.omega_max), ang_raw)
            return 0.0, ang

        # ── DRIVE：Pure Pursuit 几何公式 ω = v · 2sin(α) / L ───────────────
        # 线速：cos 缩放（角差大时自动减速）
        lin = self.v_max * max(0.0, math.cos(heading_error))

        # 纯追踪曲率：κ = 2sin(α)/L，角速 = v·κ
        ang_pp = lin * 2.0 * math.sin(heading_error) / max(self.lookahead_dist, 0.1)
        ang_raw = ang_pp * self.ang_sign

        # 死区（直线小偏差不修正，防微振）
        if abs_err < self.ang_db:
            ang_raw = 0.0

        # 硬限幅
        ang = math.copysign(min(abs(ang_raw), self.drive_omega_max), ang_raw)

        # 轮速限制
        half = 0.5 * self.wheel_sep
        v_l = lin - ang * half
        v_r = lin + ang * half
        peak = max(abs(v_l), abs(v_r))
        if peak > self.v_max + 1e-9:
            s = self.v_max / peak
            lin = lin * s
            ang = ang * s

        return lin, ang

    # === 主循环 ===============================================================

    def _loop(self):
        """控制主循环：读取位姿、判定模式并发布控制量。"""
        if self._js_active:
            return
        if not self.has_started:
            return

        now = self.get_clock().now().nanoseconds
        if now < self._replan_pause_until_ns:
            self._stop()
            return

        pose = self._get_pose()
        if pose is None:
            return
        x, y, yaw = pose

        # 终点检测
        gx, gy = self.path_points[-1]
        d2_goal = (gx - x) * (gx - x) + (gy - y) * (gy - y)
        if d2_goal <= self.goal_tol_sq:
            dist_goal = math.sqrt(d2_goal)
            self._stop()
            self.get_logger().info(f"OK 到达终点 | 距终点={dist_goal:.2f}m")
            self._reset()
            return

        # 纯追踪目标点
        tx, ty = self._lookahead_target(x, y)

        # 控制
        heading_error = self._norm(math.atan2(ty - y, tx - x) - yaw)
        lin, ang = self._compute(heading_error, now)
        self._pub(lin, ang)

        if self._last_log_mode != self.mode:
            self._last_log_mode = self.mode
            self.get_logger().info(
                f"[{self.mode.name}] "
                f"lin={lin*100:.1f}cm/s ang={math.degrees(ang):+.1f}deg/s | "
                f"err={math.degrees(heading_error):+.1f} deg "
                f"target=({tx:.2f},{ty:.2f})"
            )


def main(args=None):
    """节点入口函数。"""
    rclpy.init(args=args)
    node = ChassisControlNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
