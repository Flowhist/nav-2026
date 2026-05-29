#!/usr/bin/env python3
"""
WHILL 底盘驱动节点（omnilibs CAN 驱动版）

能力：
  1) 读取左右轮反馈，发布 /odom_encoder 里程计
  2) 订阅 /cmd_vel，下发左右轮速度到底盘

说明：
  - 采用"单节点独占 CAN"设计，避免资源冲突。
  - TF (odom -> base_link) 由下游 EKF 节点负责发布。
  - CAN 重连/看门狗/状态上报由 can_connection.CanConnectionManager 负责。
"""

import os
import sys
import math
import time
import threading
from typing import Optional

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# omnilibs 安装路径
sys.path.insert(0, "/home/embotic/DCCS/src/site-packages")
from omnilibs.driver.driver import Driver, ONLINE, CAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from can_connection import CanConnectionManager

PROJECT_ROOT = "/home/embotic/nav_workspace/src/finav"


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
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("acceleration", 100)

        # ── 参数读取 ──
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.update_rate = max(1.0, float(self.get_parameter("update_rate").value))
        self.cmd_send_rate = max(1.0, float(self.get_parameter("cmd_send_rate").value))
        self.cmd_timeout = max(0.05, float(self.get_parameter("cmd_timeout").value))
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.acceleration = int(self.get_parameter("acceleration").value)
        sign = float(self.get_parameter("wheel_velocity_sign").value)
        self.wheel_velocity_sign = 1.0 if sign >= 0.0 else -1.0

        # ── 回调组 ──
        self.cmd_sub_group = ReentrantCallbackGroup()
        self.odom_timer_group = MutuallyExclusiveCallbackGroup()
        self.cmd_timer_group = MutuallyExclusiveCallbackGroup()
        self._last_cmd_log_mono = 0.0
        self._last_send_state_log_mono = 0.0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_pub = self.create_publisher(Odometry, "/odom_encoder", qos)

        # ── 里程计状态 ──
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ── 驱动状态 ──
        self.whill = None
        self.running = True
        self._driver_lock = threading.Lock()

        # ── 控制状态 ──
        self.last_cmd_time = self.get_clock().now()
        self._cmd_lock = threading.Lock()
        self._latest_cmd_linear = 0.0
        self._latest_cmd_angular = 0.0
        self._latest_cmd_mono: Optional[float] = None
        self._last_sent_linear = 0.0
        self._last_sent_angular = 0.0
        self._last_zero_keepalive_mono = 0.0
        self._last_send_failed = False

        # ── CAN 连接管理（委托给 CanConnectionManager） ──
        self.can_mgr = CanConnectionManager(
            self,
            connect_fn=self._create_and_load_driver,
            on_connected=self._on_can_up,
            on_disconnected=self._on_can_down,
            connect_tag="WHILL",
        )

        # ── 订阅 ──
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.create_subscription(
            Twist, cmd_vel_topic, self._on_cmd_vel, 10,
            callback_group=self.cmd_sub_group,
        )

        # ── 定时器 ──
        self.can_mgr.start()
        self.odom_timer = self.create_timer(
            1.0 / self.update_rate, self._update_odom,
            callback_group=self.odom_timer_group,
        )
        self.cmd_timer = self.create_timer(
            1.0 / self.cmd_send_rate, self._send_cmd,
            callback_group=self.cmd_timer_group,
        )

        self.get_logger().info(
            f"WHILL 底盘驱动已启动 | odom={self.update_rate:.1f}Hz "
            f"| cmd_send={self.cmd_send_rate:.1f}Hz | cmd_timeout={self.cmd_timeout:.2f}s"
        )

    # ── CAN 连接回调（供 CanConnectionManager 使用） ─────────────────

    @property
    def connected(self) -> bool:
        return self.can_mgr.connected

    def _create_and_load_driver(self):
        """创建 Driver 并加载 CANopen 配置；失败返回 None。"""
        whill = Driver()
        prev = os.getcwd()
        try:
            os.chdir(PROJECT_ROOT)
            whill.load(
                "Whill",
                mode=ONLINE,
                parameters={
                    CAN: {
                        "channel_name": self.get_parameter("can_channel").value,
                        "interface": "pcan",
                        "baud_rate": self.get_parameter("can_baud_rate").value,
                        "canopen": 1,
                    }
                },
            )
        finally:
            os.chdir(prev)

        # 恢复后立即发一次零速，清空电机残留状态
        try:
            whill.move_velocity([1, 2], 0.0, self.acceleration)
        except Exception:
            pass
        return whill

    def _on_can_up(self, driver):
        """CAN 恢复：存储新 driver，清空残留指令，重置里程计基准。"""
        with self._cmd_lock:
            self._latest_cmd_linear = 0.0
            self._latest_cmd_angular = 0.0
            self._latest_cmd_mono = None
            self._last_sent_linear = 0.0
            self._last_sent_angular = 0.0
            self._last_zero_keepalive_mono = 0.0
            self._last_send_failed = False
        with self._driver_lock:
            old = self.whill
            self.whill = driver
            self.last_time = self.get_clock().now()
        if old is not None:
            try:
                old.finalize()
            except Exception:
                pass
        self.get_logger().warn("✓✓✓ CAN 已恢复，底盘重新上线")

    def _on_can_down(self):
        """CAN 断开：释放旧 driver，避免 fault 状态对象继续参与控制。"""
        with self._driver_lock:
            old = self.whill
            self.whill = None
        if old is not None:
            try:
                old.finalize()
            except Exception as exc:
                self.get_logger().warn(f"CAN 断开时释放旧 driver 失败: {exc}")

    # ── cmd_vel 接收 ───────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist):
        with self._cmd_lock:
            self._latest_cmd_linear = float(msg.linear.x)
            self._latest_cmd_angular = float(msg.angular.z)
            self._latest_cmd_mono = time.monotonic()
        self.last_cmd_time = self.get_clock().now()

        now_mono = time.monotonic()
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

        v_left = linear - angular * self.wheel_separation * 0.5
        v_right = linear + angular * self.wheel_separation * 0.5

        left_degps = self.wheel_velocity_sign * (v_left / self.wheel_radius) * (180.0 / math.pi)
        right_degps = self.wheel_velocity_sign * (v_right / self.wheel_radius) * (180.0 / math.pi)

        sent = self._send_wheel_velocity(left_degps, right_degps)
        if sent:
            self._last_sent_linear = linear
            self._last_sent_angular = angular
            self._last_send_failed = False
        else:
            # 失败后不要继续用同一个 fault driver 打零速；交给 CAN manager 重连恢复。
            self._last_sent_linear = 0.0
            self._last_sent_angular = 0.0
            self._last_send_failed = False
        return sent

    def _send_cmd(self):
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

    def _send_wheel_velocity(self, left_degps: float, right_degps: float) -> bool:
        error = None
        with self._driver_lock:
            if not self.connected or self.whill is None:
                return False
            try:
                l = float(left_degps)
                r = float(right_degps)
                if abs(l - r) < 1e-6:
                    self.whill.move_velocity([1, 2], l, self.acceleration)
                else:
                    self.whill.move_velocity([1], l, self.acceleration)
                    self.whill.move_velocity([2], r, self.acceleration)
            except Exception as exc:
                error = exc

        if error is not None:
            self.get_logger().error(f"下发轮速失败: {error}")
            return False

        self.can_mgr.on_success()
        return True

    # ── 里程计 ─────────────────────────────────────────────────────

    def _update_odom(self):
        if not self.connected or self.whill is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 1.0:
            self.last_time = now
            return
        self.last_time = now

        error = None
        with self._driver_lock:
            try:
                vel = self.whill.get_velocity([1, 2])
            except Exception as exc:
                error = exc

        if error is not None:
            self.get_logger().error(f"读取轮速失败: {error}")
            self.can_mgr.mark_error()
            return

        self.can_mgr.on_success()

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

    # ── 清理 ───────────────────────────────────────────────────────

    def destroy_node(self):
        self.running = False
        self.can_mgr.shutdown()
        self._stop_motion()
        with self._driver_lock:
            if self.whill:
                try:
                    self.whill.finalize()
                except Exception:
                    pass
        super().destroy_node()

    def _stop_motion(self):
        with self._driver_lock:
            if not self.connected or self.whill is None:
                return
            try:
                self.whill.move_velocity([1, 2], 0.0, self.acceleration)
            except Exception:
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
