#!/usr/bin/env python3
"""
js_kb_router.py
Keyboard/Web teleop router with joystick priority.
"""

import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class KeyboardTeleopRouter(Node):
    def __init__(self):
        super().__init__("js_kb_router")

        # ── 参数 ─────────────────────────────────────────────────────── #
        self.declare_parameter("keyboard_linear_speeds", [0.1, 0.2, 0.4, 0.6])
        self.declare_parameter("keyboard_angular_speed", 0.5)
        self.declare_parameter("router_rate", 50.0)
        self.declare_parameter("js_cmd_timeout", 0.4)
        self.declare_parameter("web_cmd_timeout", 0.4)
        self.declare_parameter("stop_timeout", 1.0)

        speeds = [float(v) for v in self.get_parameter("keyboard_linear_speeds").value]
        self._kb_speeds = [abs(v) for v in speeds if abs(float(v)) > 1e-6] or [0.1, 0.2, 0.4, 0.6]
        self._kb_rot_spd = abs(float(self.get_parameter("keyboard_angular_speed").value))
        self._router_rate = max(1.0, float(self.get_parameter("router_rate").value))
        self._js_cmd_timeout = max(0.05, float(self.get_parameter("js_cmd_timeout").value))
        self._web_cmd_timeout = max(0.05, float(self.get_parameter("web_cmd_timeout").value))
        self.stop_timeout = max(0.1, float(self.get_parameter("stop_timeout").value))

        # ── 订阅数据缓存 ──────────────────────────────────────────────── #
        self._js = False
        self._js_last_cmd = Twist()
        self._js_last_time = 0.0
        self._web_last_cmd = Twist()
        self._web_last_time = 0.0

        self.last_time = 0.0

        # ── 键盘控制状态 ──────────────────────────────────────────────── #
        self._kb_enabled = True
        self._kb_last_time = 0.0
        self._kb_linear = 0.0
        self._kb_angular = 0.0
        self._kb_spd_idx = 1
        if self._kb_spd_idx >= len(self._kb_speeds):
            self._kb_spd_idx = len(self._kb_speeds) - 1

        # ── ROS I/O ───────────────────────────────────────────────────── #
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Bool, "/js_state", self._on_js_state, 10)
        self.create_subscription(Twist, "/js_cmd_vel", self._on_js_cmd_vel, 10)
        self.create_subscription(Twist, "/web_cmd_vel", self._on_web_cmd_vel, 10)

        # ── 定时器 ────────────────────────────────────────────────────── #
        self.create_timer(1.0 / self._router_rate, self._control_tick)

        # ── 终端 cbreak 模式（非阻塞单字符读取）─────────────────────── #
        if sys.stdin.isatty():
            self._old_term = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            self._old_term = None
            self._kb_enabled = False
            self.get_logger().warn("stdin is not a tty, keyboard control disabled")

        self.get_logger().info(
            "js_kb_router started | linear_speeds=%s | angular=%.2f rad/s | router_rate=%.2f Hz | web_timeout=%.2fs | WSAD move | J up | K down | Space stop"
            % (self._kb_speeds, self._kb_rot_spd, self._router_rate, self._web_cmd_timeout)
        )

    # ── 订阅回调 ────────────────────────────────────────────────────── #
    def _on_js_state(self, m):
        self._js = m.data
        if m.data:
            self._web_last_cmd = Twist()
            self._web_last_time = time.monotonic()
            self.last_time = self._web_last_time
        if m.data and self._kb_enabled:  # 摇杆接管时自动关闭键盘
            self._kb_enabled = False
            self._kb_linear = 0.0
            self._kb_angular = 0.0

    def _on_js_cmd_vel(self, m):
        self._js_last_cmd = m
        self._js_last_time = time.monotonic()
        self.last_time = self._js_last_time

    def _on_web_cmd_vel(self, m):
        self._web_last_cmd = m
        self._web_last_time = time.monotonic()
        self.last_time = self._web_last_time

    # ── 键盘轮询 + 持续发布 ──────────────────────────────────────────── #
    def _control_tick(self):
        if self._old_term is not None:
            rlist, _, _ = select.select([sys.stdin], [], [], 0)
            if rlist:
                key = sys.stdin.read(1).lower()
                if key == "f":
                    self._kb_enabled = not self._kb_enabled
                    self._kb_linear = 0.0
                    self._kb_angular = 0.0
                elif self._kb_enabled:
                    if key == "w":
                        self._kb_linear = self._kb_speeds[self._kb_spd_idx]
                        self._kb_angular = 0.0
                    elif key == "s":
                        self._kb_linear = -self._kb_speeds[self._kb_spd_idx]
                        self._kb_angular = 0.0
                    elif key == "a":
                        self._kb_linear = 0.0
                        self._kb_angular = self._kb_rot_spd
                    elif key == "d":
                        self._kb_linear = 0.0
                        self._kb_angular = -self._kb_rot_spd
                    elif key == "j":
                        self._kb_spd_idx = min(
                            self._kb_spd_idx + 1, len(self._kb_speeds) - 1
                        )
                    elif key == "k":
                        self._kb_spd_idx = max(self._kb_spd_idx - 1, 0)
                    elif key == " ":
                        self._kb_linear = 0.0
                        self._kb_angular = 0.0
                    self._kb_last_time = time.monotonic()
                    self.last_time = self._kb_last_time

        msg = Twist()
        now = time.monotonic()
        if self._js and (now - self._js_last_time) <= self._js_cmd_timeout:
            msg = self._js_last_cmd
            self._cmd_pub.publish(msg)
        elif self._kb_enabled and self._old_term is not None:
            msg.linear.x = self._kb_linear
            msg.angular.z = self._kb_angular
            self._cmd_pub.publish(msg)
        elif (now - self._web_last_time) <= self._web_cmd_timeout:
            msg = self._web_last_cmd
            self._cmd_pub.publish(msg)
        elif (now - self.last_time) <= self.stop_timeout:
            self._cmd_pub.publish(msg)

    # ── 节点清理 ─────────────────────────────────────────────────────── #
    def destroy_node(self):
        if self._old_term is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)
            except Exception:
                pass
        try:
            self._cmd_pub.publish(Twist())  # 退出前发一次零速
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardTeleopRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
