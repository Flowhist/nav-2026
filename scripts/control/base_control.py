#!/usr/bin/env python3
"""
WHILL 底盘驱动节点（omnilibs CAN 驱动版）

能力：
  1) 读取左右轮反馈，发布 /odom_encoder 里程计
  2) 订阅 /cmd_vel，下发左右轮速度到底盘

说明：
  - 采用"单节点独占 CAN"设计，避免资源冲突。
  - TF (odom -> base_link) 由下游 EKF 节点负责发布。
  - 急停旋钮触发的是电机 STO/fault，不按 CAN 断线处理。
"""

import math
import threading
import time
from typing import Optional

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray

from base_driver_worker import DriverWorkerBusy, DriverWorkerClient, DriverWorkerError

PROJECT_ROOT = "/home/embotic/nav_workspace/src/finav"
OMNILIBS_PATH = "/home/embotic/DCCS/src/site-packages"


def select_wheel_acceleration(
    left_degps: float,
    right_degps: float,
    acceleration: int,
    acceleration_stop: int,
) -> int:
    if abs(float(left_degps)) <= 1e-6 and abs(float(right_degps)) <= 1e-6:
        return int(acceleration_stop)
    return int(acceleration)


def advance_fault_clear_count(count: int, statuses, required: int = 3):
    if any(status.get("fault", 0) != 0 for status in statuses):
        return 0, False
    count = int(count) + 1
    return count, count >= required


def should_reset_fault(statuses) -> bool:
    estop_faults = {0x5441, 0x8313}
    return any(
        status.get("fault", 0) in estop_faults
        and status.get("emergency", 0) == 0
        for status in statuses
    )


class WhillBaseDriver(Node):
    def __init__(self):
        super().__init__("base_control")

        # ==================== ROS2 参数声明 ====================
        self.declare_parameter("wheel_radius", 0.127)
        self.declare_parameter("wheel_separation", 0.6)
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("cmd_send_rate", 50.0)
        self.declare_parameter("cmd_timeout", 0.35)
        self.declare_parameter("can_channel", "PCAN_USBBUS1")
        self.declare_parameter("can_baud_rate", 500000)
        self.declare_parameter("wheel_velocity_sign", -1.0)
        self.declare_parameter("left_motor_id", 1)
        self.declare_parameter("right_motor_id", 2)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("acceleration", 100)
        self.declare_parameter("acceleration_stop", 100)
        self.declare_parameter("cmd_resend_interval", 0.5)
        self.declare_parameter("driver_startup_timeout_s", 8.0)
        self.declare_parameter("driver_request_timeout_s", 0.5)
        self.declare_parameter("driver_terminate_timeout_s", 0.5)
        self.declare_parameter("driver_restart_interval_s", 1.0)

        # ── 参数读取 ──
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.update_rate = max(1.0, float(self.get_parameter("update_rate").value))
        self.cmd_send_rate = max(1.0, float(self.get_parameter("cmd_send_rate").value))
        self.cmd_timeout = max(0.05, float(self.get_parameter("cmd_timeout").value))
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.acceleration = int(self.get_parameter("acceleration").value)
        self.acceleration_stop = int(self.get_parameter("acceleration_stop").value)
        self.cmd_resend_interval = max(
            0.1, float(self.get_parameter("cmd_resend_interval").value)
        )
        self.driver_startup_timeout_s = max(
            0.5, float(self.get_parameter("driver_startup_timeout_s").value)
        )
        self.driver_request_timeout_s = max(
            0.1, float(self.get_parameter("driver_request_timeout_s").value)
        )
        self.driver_terminate_timeout_s = max(
            0.1, float(self.get_parameter("driver_terminate_timeout_s").value)
        )
        self._driver_restart_interval_s = max(
            0.2, float(self.get_parameter("driver_restart_interval_s").value)
        )
        sign = float(self.get_parameter("wheel_velocity_sign").value)
        self.wheel_velocity_sign = 1.0 if sign >= 0.0 else -1.0
        self.left_motor_id = int(self.get_parameter("left_motor_id").value)
        self.right_motor_id = int(self.get_parameter("right_motor_id").value)

        # ── 回调组 ──
        self.cmd_sub_group = ReentrantCallbackGroup()
        self.odom_timer_group = MutuallyExclusiveCallbackGroup()
        self.cmd_timer_group = MutuallyExclusiveCallbackGroup()
        self.fault_monitor_timer_group = MutuallyExclusiveCallbackGroup()
        self._last_cmd_log_mono = 0.0
        self._last_send_state_log_mono = 0.0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom_encoder", qos)
        self.fault_pub = self.create_publisher(Bool, "/base_fault", 10)
        self.wheel_velocity_pub = self.create_publisher(
            Float64MultiArray, "/wheel_velocity_degps", qos
        )

        # ── 里程计状态 ──
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ── 驱动状态 ──
        self.whill = None
        self.running = True
        self._last_driver_restart_try_mono = 0.0

        # ── 控制状态 ──
        self.last_cmd_time = self.get_clock().now()
        self._cmd_lock = threading.Lock()
        self._latest_cmd_linear = 0.0
        self._latest_cmd_angular = 0.0
        self._latest_cmd_mono: Optional[float] = None
        self._last_sent_linear = 0.0
        self._last_sent_angular = 0.0
        self._last_sent_mono = 0.0
        self._last_zero_keepalive_mono = 0.0
        self._last_send_failed = False
        self._require_cmd_reset = True
        self._motion_fault_latched = False
        self._fault_clear_count = 0
        self._last_fault_reset_try_mono = 0.0
        self._last_fault_log_mono = 0.0

        # ── 订阅 ──
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.create_subscription(
            Twist, cmd_vel_topic, self._on_cmd_vel, cmd_qos,
            callback_group=self.cmd_sub_group,
        )

        # ── 初始化底盘驱动 ──
        self.whill = self._create_and_load_driver()
        self.last_time = self.get_clock().now()

        # ── 定时器 ──
        self.odom_timer = self.create_timer(
            1.0 / self.update_rate, self._update_odom,
            callback_group=self.odom_timer_group,
        )
        self.cmd_timer = self.create_timer(
            1.0 / self.cmd_send_rate, self._send_cmd,
            callback_group=self.cmd_timer_group,
        )
        self.fault_monitor_timer = self.create_timer(
            0.2, self._monitor_fault,
            callback_group=self.fault_monitor_timer_group,
        )

        self.get_logger().info(
            f"WHILL 底盘驱动已启动 | odom={self.update_rate:.1f}Hz "
            f"| cmd_send={self.cmd_send_rate:.1f}Hz | cmd_timeout={self.cmd_timeout:.2f}s "
            f"| accel={self.acceleration}/{self.acceleration_stop}(stop) "
            f"| motors L/R={self.left_motor_id}/{self.right_motor_id} "
            f"| wheel_sign={self.wheel_velocity_sign:.0f}"
        )

    # ── 驱动状态 ───────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self.whill is not None and self.whill.is_healthy

    @staticmethod
    def _cmd_nonzero(linear: float, angular: float) -> bool:
        return abs(linear) > 1e-6 or abs(angular) > 1e-6

    def _publish_fault(self, active: bool):
        msg = Bool()
        msg.data = bool(active)
        self.fault_pub.publish(msg)

    def _clear_cmd_cache(self, *, require_reset: bool):
        with self._cmd_lock:
            self._latest_cmd_linear = 0.0
            self._latest_cmd_angular = 0.0
            self._latest_cmd_mono = None
            self._last_sent_linear = 0.0
            self._last_sent_angular = 0.0
            self._last_sent_mono = 0.0
            self._last_zero_keepalive_mono = 0.0
            self._last_send_failed = False
            self._require_cmd_reset = bool(require_reset)

    def _latch_motion_fault(self, error):
        was_latched = self._motion_fault_latched
        self._motion_fault_latched = True
        self._fault_clear_count = 0
        if not was_latched:
            self._clear_cmd_cache(require_reset=True)
            self._publish_fault(True)

        now = time.monotonic()
        if now - self._last_fault_log_mono >= 1.0:
            text = str(error)
            if "STO" in text or "33555" in text:
                self.get_logger().error("检测到急停/STO 故障，已清空控制指令；请释放急停并先发送 0 速复位")
            else:
                self.get_logger().error(f"检测到底盘控制故障，已清空控制指令: {error}")
            self._last_fault_log_mono = now

    def _create_and_load_driver(self):
        """启动隔离驱动进程；初始化完成前会先下发零速。"""
        worker = DriverWorkerClient(
            {
                "omnilibs_path": OMNILIBS_PATH,
                "project_root": PROJECT_ROOT,
                "can_channel": self.get_parameter("can_channel").value,
                "can_baud_rate": self.get_parameter("can_baud_rate").value,
                "motor_ids": [self.left_motor_id, self.right_motor_id],
                "acceleration_stop": self.acceleration_stop,
            },
            startup_timeout_s=self.driver_startup_timeout_s,
            request_timeout_s=self.driver_request_timeout_s,
            terminate_timeout_s=self.driver_terminate_timeout_s,
        )
        try:
            worker.start()
        except DriverWorkerError as exc:
            self._motion_fault_latched = True
            self._require_cmd_reset = True
            self._publish_fault(True)
            self.get_logger().error(f"底盘驱动进程启动失败，保持故障锁定: {exc}")
        return worker

    def _try_restart_driver(self, now_mono: Optional[float] = None) -> bool:
        now = time.monotonic() if now_mono is None else float(now_mono)
        if self.connected:
            return True
        if now - self._last_driver_restart_try_mono < self._driver_restart_interval_s:
            return False
        self._last_driver_restart_try_mono = now

        try:
            self.whill.restart()
        except DriverWorkerError as exc:
            self.get_logger().error(f"底盘驱动进程重启失败，继续保持停车锁定: {exc}")
            return False

        self._clear_cmd_cache(require_reset=True)
        self._motion_fault_latched = True
        self._publish_fault(True)
        self.get_logger().info("底盘驱动进程已重启并下发零速；等待一次 0 速指令复位")
        return True

    # ── cmd_vel 接收 ───────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist):
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        now_mono = time.monotonic()

        with self._cmd_lock:
            if self._motion_fault_latched:
                self._latest_cmd_linear = 0.0
                self._latest_cmd_angular = 0.0
                self._latest_cmd_mono = None
                return
            if self._require_cmd_reset:
                if self._cmd_nonzero(linear, angular):
                    if now_mono - self._last_fault_log_mono >= 1.0:
                        self.get_logger().warn("底盘故障后仍收到非零 /cmd_vel，已忽略；请先发送 0 速复位")
                        self._last_fault_log_mono = now_mono
                    self._latest_cmd_linear = 0.0
                    self._latest_cmd_angular = 0.0
                    self._latest_cmd_mono = None
                    return

                self._require_cmd_reset = False
                self.get_logger().info("控制指令已复位，允许新的非零 /cmd_vel")

            self._latest_cmd_linear = linear
            self._latest_cmd_angular = angular
            self._latest_cmd_mono = time.monotonic()
        self.last_cmd_time = self.get_clock().now()

        if (now_mono - self._last_cmd_log_mono) >= 0.5:
            self.get_logger().info(
                f"收到 /cmd_vel | linear={self._latest_cmd_linear:.3f} m/s "
                f"| angular={self._latest_cmd_angular:.3f} rad/s"
            )
            self._last_cmd_log_mono = now_mono

    # ── 速度下发 ───────────────────────────────────────────────────

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _apply_twist(self, linear_cmd: float, angular_cmd: float) -> bool:
        linear = self._clamp(float(linear_cmd), -self.max_linear_speed, self.max_linear_speed)
        angular = self._clamp(float(angular_cmd), -self.max_angular_speed, self.max_angular_speed)

        now_mono = time.monotonic()
        unchanged = (
            abs(linear - self._last_sent_linear) < 1e-4
            and abs(angular - self._last_sent_angular) < 1e-4
        )
        if (
            unchanged
            and not self._last_send_failed
            and (now_mono - self._last_sent_mono) < self.cmd_resend_interval
        ):
            return True

        v_left = linear - angular * self.wheel_separation * 0.5
        v_right = linear + angular * self.wheel_separation * 0.5
        left_degps = self.wheel_velocity_sign * (v_left / self.wheel_radius) * (180.0 / math.pi)
        right_degps = self.wheel_velocity_sign * (v_right / self.wheel_radius) * (180.0 / math.pi)

        sent = self._send_wheel_velocity(left_degps, right_degps)
        if sent:
            self._last_sent_linear = linear
            self._last_sent_angular = angular
            self._last_sent_mono = now_mono
            self._last_send_failed = False
        else:
            self._last_sent_linear = 0.0
            self._last_sent_angular = 0.0
            self._last_send_failed = True
        return sent

    def _send_cmd(self):
        if self._motion_fault_latched:
            return
        with self._cmd_lock:
            linear = self._latest_cmd_linear
            angular = self._latest_cmd_angular
            stamp = self._latest_cmd_mono

        now_mono = time.monotonic()

        if stamp is None:
            if (now_mono - self._last_send_state_log_mono) >= 1.0:
                self.get_logger().info("未收到任何 /cmd_vel，保持 0 速保护")
                self._last_send_state_log_mono = now_mono
            self._apply_stop_if_needed(now_mono)
            return

        cmd_age = now_mono - stamp
        if cmd_age > self.cmd_timeout:
            if (now_mono - self._last_send_state_log_mono) >= 1.0:
                self.get_logger().info(
                    f"/cmd_vel 超时 {cmd_age:.3f}s > {self.cmd_timeout:.3f}s，保持 0 速保护"
                )
                self._last_send_state_log_mono = now_mono
            self._apply_stop_if_needed(now_mono)
            return

        if (now_mono - self._last_send_state_log_mono) >= 0.5:
            self.get_logger().info(
                f"下发速度 | linear={linear:.3f} m/s | angular={angular:.3f} rad/s "
                f"| cmd_age={cmd_age:.3f}s"
            )
            self._last_send_state_log_mono = now_mono

        self._apply_twist(linear, angular)

    def _apply_stop_if_needed(self, now_mono: float):
        if not self.connected or self.whill is None:
            return
        moving = abs(self._last_sent_linear) > 1e-6 or abs(self._last_sent_angular) > 1e-6
        keepalive_due = (now_mono - self._last_zero_keepalive_mono) >= 1.0
        if moving or self._last_send_failed or keepalive_due:
            if self._apply_twist(0.0, 0.0):
                self._last_zero_keepalive_mono = now_mono

    def _send_wheel_velocity(
        self,
        left_degps: float,
        right_degps: float,
    ) -> bool:
        if not self.connected:
            error = DriverWorkerError("driver worker is not connected")
            self._latch_motion_fault(error)
            return False

        try:
            l = float(left_degps)
            r = float(right_degps)
            accel = select_wheel_acceleration(
                l,
                r,
                self.acceleration,
                self.acceleration_stop,
            )
            self.whill.set_velocity(
                [self.left_motor_id, self.right_motor_id],
                l,
                r,
                accel,
            )
            return True
        except DriverWorkerBusy:
            return False
        except DriverWorkerError as exc:
            self.get_logger().error(f"下发轮速失败: {exc}")
            self._latch_motion_fault(exc)
            return False

    # ── 里程计 ─────────────────────────────────────────────────────

    def _update_odom(self):
        if not self.connected or self.whill is None or self._motion_fault_latched:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 1.0:
            self.last_time = now
            return
        self.last_time = now

        try:
            vel = self.whill.get_velocity([self.left_motor_id, self.right_motor_id])
        except DriverWorkerBusy:
            return
        except DriverWorkerError as exc:
            self.get_logger().error(f"读取轮速失败: {exc}")
            self._latch_motion_fault(exc)
            return

        wheel_msg = Float64MultiArray()
        wheel_msg.data = [float(vel[0]), float(vel[1])]
        self.wheel_velocity_pub.publish(wheel_msg)

        v_left = self.wheel_velocity_sign * math.radians(vel[0]) * self.wheel_radius
        v_right = self.wheel_velocity_sign * math.radians(vel[1]) * self.wheel_radius

        v_forward = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_separation

        theta_mid = self.theta + omega * dt * 0.5
        self.x += v_forward * math.cos(theta_mid) * dt
        self.y += v_forward * math.sin(theta_mid) * dt
        self.theta += omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self._publish_odom(now, v_forward, omega)

    def _publish_odom(self, stamp, v_forward: float, vth: float):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.01
        odom.twist.twist.linear.x = v_forward
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

    # ── 故障主动监测 ─────────────────────────────────────────────

    def _monitor_fault(self):
        """5Hz 轻量轮询：通过 TPDO 缓存检测电机故障，避免阻塞在 move_velocity。"""
        if not self.connected:
            self._try_restart_driver()
            return
        try:
            statuses = self.whill.get_fault_status(
                [self.left_motor_id, self.right_motor_id]
            )
        except DriverWorkerBusy:
            return
        except DriverWorkerError as exc:
            self.get_logger().error(f"读取底盘故障状态失败: {exc}")
            self._latch_motion_fault(exc)
            return

        for motor_id, status in zip(
            [self.left_motor_id, self.right_motor_id], statuses
        ):
            if status.get("fault", 0) != 0:
                self._latch_motion_fault(
                    RuntimeError(status.get("msg") or f"Motor{motor_id} fault")
                )
                now = time.monotonic()
                if (
                    should_reset_fault(statuses)
                    and now - self._last_fault_reset_try_mono >= 0.5
                ):
                    self._last_fault_reset_try_mono = now
                    try:
                        self.whill.reset_fault_status(
                            [self.left_motor_id, self.right_motor_id]
                        )
                    except DriverWorkerBusy:
                        pass
                    except DriverWorkerError as exc:
                        self.get_logger().warn(f"急停释放后复位故障失败: {exc}")
                return

        self._fault_clear_count, recovered = advance_fault_clear_count(
            self._fault_clear_count, statuses
        )
        if self._motion_fault_latched and recovered:
            self._motion_fault_latched = False
            self._fault_clear_count = 0
            self._publish_fault(False)
            self.get_logger().info("底盘故障状态已稳定清除，等待一次 0 速指令复位")

    # ── 清理 ───────────────────────────────────────────────────────

    def destroy_node(self):
        self.running = False
        self._stop_motion()
        if self.whill:
            self.whill.stop()
        super().destroy_node()

    def _stop_motion(self):
        if not self.connected:
            return
        try:
            self.whill.set_velocity(
                [self.left_motor_id, self.right_motor_id],
                0.0,
                0.0,
                self.acceleration_stop,
            )
        except DriverWorkerError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WhillBaseDriver()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
