#!/usr/bin/env python3
"""
base_control_router.py

Central command arbiter for joystick, keyboard, web teleop and navigation.
Only this node publishes /cmd_vel; base_control.py remains the hardware driver.
"""

import select
import sys
import termios
import time
import tty
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Empty, String


class Source(Enum):
    NONE = "none"
    JOYSTICK = "joystick"
    KEYBOARD = "keyboard"
    WEB = "web"
    NAV = "nav"


class BaseControlRouter(Node):
    def __init__(self):
        super().__init__("base_control_router")

        self.declare_parameter("keyboard_linear_speeds", [0.1, 0.2, 0.4, 0.6])
        self.declare_parameter("keyboard_angular_speed", 0.5)
        self.declare_parameter("router_rate", 50.0)
        self.declare_parameter("js_cmd_timeout", 0.4)
        self.declare_parameter("web_cmd_timeout", 0.4)
        self.declare_parameter("nav_cmd_timeout", 0.4)
        self.declare_parameter("stop_timeout", 1.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("js_state_topic", "/js_state")
        self.declare_parameter("js_cmd_vel_topic", "/js_cmd_vel")
        self.declare_parameter("web_cmd_vel_topic", "/web_cmd_vel")
        self.declare_parameter("nav_cmd_vel_topic", "/nav_cmd_vel")
        self.declare_parameter("nav_clear_topic", "/nav_clear")
        self.declare_parameter("base_fault_topic", "/base_fault")
        self.declare_parameter("router_status_topic", "/base_control_router/status")

        speeds = [float(v) for v in self.get_parameter("keyboard_linear_speeds").value]
        self._kb_speeds = [abs(v) for v in speeds if abs(v) > 1e-6] or [0.1, 0.2, 0.4, 0.6]
        self._kb_rot_spd = abs(float(self.get_parameter("keyboard_angular_speed").value))
        self._router_rate = max(1.0, float(self.get_parameter("router_rate").value))
        self._js_cmd_timeout = max(0.05, float(self.get_parameter("js_cmd_timeout").value))
        self._web_cmd_timeout = max(0.05, float(self.get_parameter("web_cmd_timeout").value))
        self._nav_cmd_timeout = max(0.05, float(self.get_parameter("nav_cmd_timeout").value))
        self._stop_timeout = max(0.1, float(self.get_parameter("stop_timeout").value))

        self._js_online = False
        self._js_cmd = Twist()
        self._js_time = 0.0
        self._web_cmd = Twist()
        self._web_time = 0.0
        self._nav_cmd = Twist()
        self._nav_time = 0.0

        self._source = Source.NONE
        self._joystick_stop_latched = False
        self._last_cmd_time = 0.0
        self._last_lock_msg_time = 0.0
        self._last_nav_clear_time = 0.0
        self._base_fault_active = False

        self._kb_enabled = True
        self._kb_linear = 0.0
        self._kb_angular = 0.0
        self._kb_spd_idx = min(1, len(self._kb_speeds) - 1)

        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, cmd_qos)
        self._nav_clear_pub = self.create_publisher(
            Empty, str(self.get_parameter("nav_clear_topic").value), 10
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("router_status_topic").value), 10
        )
        self.create_subscription(
            Bool, str(self.get_parameter("js_state_topic").value), self._on_js_state, 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter("js_cmd_vel_topic").value), self._on_js_cmd, cmd_qos
        )
        self.create_subscription(
            Twist, str(self.get_parameter("web_cmd_vel_topic").value), self._on_web_cmd, cmd_qos
        )
        self.create_subscription(
            Twist, str(self.get_parameter("nav_cmd_vel_topic").value), self._on_nav_cmd, cmd_qos
        )
        self.create_subscription(
            Bool, str(self.get_parameter("base_fault_topic").value), self._on_base_fault, 10
        )
        self.create_timer(1.0 / self._router_rate, self._tick)

        if sys.stdin.isatty():
            self._old_term = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            self._old_term = None
            self._kb_enabled = False
            self.get_logger().warn("stdin is not a tty, keyboard control disabled")

        self.get_logger().info(
            "base_control_router started | WSAD move | J/K speed | Space stop | "
            "joystick nonzero interrupts active keyboard/web/nav control"
        )

    @staticmethod
    def _zero() -> Twist:
        return Twist()

    @staticmethod
    def _nonzero(msg: Twist) -> bool:
        return float(msg.linear.x) != 0.0 or float(msg.angular.z) != 0.0

    def _fresh(self, stamp: float, timeout: float) -> bool:
        return stamp > 0.0 and (time.monotonic() - stamp) <= timeout

    def _publish(self, msg: Twist):
        self._cmd_pub.publish(msg)
        self._last_cmd_time = time.monotonic()

    def _publish_stop(self):
        self._publish(self._zero())

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _clear_nav(self):
        now = time.monotonic()
        if now - self._last_nav_clear_time >= 0.2:
            self._nav_clear_pub.publish(Empty())
            self._nav_cmd = Twist()
            self._nav_time = 0.0
            self._last_nav_clear_time = now

    def _warn_reset_required(self, reason: str):
        now = time.monotonic()
        if now - self._last_lock_msg_time >= 1.0:
            self.get_logger().warn(f"请先复位摇杆～ ({reason})")
            self._publish_status(f"joystick_reset_required:{reason}")
            self._last_lock_msg_time = now

    def _clear_control_state(self, *, clear_nav: bool = True):
        self._source = Source.NONE
        self._joystick_stop_latched = False
        self._kb_linear = 0.0
        self._kb_angular = 0.0
        self._web_cmd = Twist()
        self._web_time = 0.0
        self._nav_cmd = Twist()
        self._nav_time = 0.0
        self._js_cmd = Twist()
        self._js_time = 0.0
        if clear_nav:
            self._clear_nav()
        self._publish_stop()

    def _on_base_fault(self, msg: Bool):
        active = bool(msg.data)
        if active == self._base_fault_active:
            return

        self._base_fault_active = active
        if active:
            self._clear_control_state(clear_nav=True)
            self._publish_status("base_fault_active")
            self.get_logger().error("底盘急停/STO 或控制故障，已清空控制指令并保持停车")
        else:
            self._clear_control_state(clear_nav=True)
            self._publish_status("base_fault_cleared")
            self.get_logger().info("底盘故障已复位，等待新的控制指令")

    def _enter_joystick_stop_lock(self, interrupted: Source):
        self._joystick_stop_latched = True
        self._source = Source.NONE
        self._kb_linear = 0.0
        self._kb_angular = 0.0
        self._web_cmd = Twist()
        self._web_time = 0.0
        if interrupted == Source.NAV:
            self._clear_nav()
        self._publish_stop()
        self._publish_status(f"joystick_stop:{interrupted.value}")
        self.get_logger().warn(f"摇杆抢停，已中断 {interrupted.value}，请先复位摇杆～")

    def _on_js_state(self, msg: Bool):
        self._js_online = bool(msg.data)

    def _on_js_cmd(self, msg: Twist):
        self._js_cmd = msg
        self._js_time = time.monotonic()

    def _on_web_cmd(self, msg: Twist):
        self._web_cmd = msg
        self._web_time = time.monotonic()
        if self._joystick_stop_latched:
            self._warn_reset_required("web ignored")

    def _on_nav_cmd(self, msg: Twist):
        self._nav_cmd = msg
        self._nav_time = time.monotonic()
        if self._joystick_stop_latched:
            self._clear_nav()
            self._warn_reset_required("nav ignored")

    def _handle_key(self, key: str):
        if self._base_fault_active and key in ("w", "s", "a", "d", " "):
            self._publish_stop()
            self.get_logger().warn("底盘故障未复位，忽略键盘控制")
            return
        if key == "f":
            self._kb_enabled = not self._kb_enabled
            self._kb_linear = 0.0
            self._kb_angular = 0.0
            return
        if not self._kb_enabled:
            return
        if key in ("w", "s", "a", "d", " ") and self._joystick_stop_latched:
            self._warn_reset_required("keyboard ignored")
            return

        if key == "w":
            self._kb_linear = self._kb_speeds[self._kb_spd_idx]
            self._kb_angular = 0.0
            self._source = Source.KEYBOARD
        elif key == "s":
            self._kb_linear = -self._kb_speeds[self._kb_spd_idx]
            self._kb_angular = 0.0
            self._source = Source.KEYBOARD
        elif key == "a":
            self._kb_linear = 0.0
            self._kb_angular = self._kb_rot_spd
            self._source = Source.KEYBOARD
        elif key == "d":
            self._kb_linear = 0.0
            self._kb_angular = -self._kb_rot_spd
            self._source = Source.KEYBOARD
        elif key == "j":
            self._kb_spd_idx = min(self._kb_spd_idx + 1, len(self._kb_speeds) - 1)
        elif key == "k":
            self._kb_spd_idx = max(self._kb_spd_idx - 1, 0)
        elif key == " ":
            self._kb_linear = 0.0
            self._kb_angular = 0.0
            if self._source == Source.KEYBOARD:
                self._source = Source.NONE

    def _poll_keyboard(self):
        if self._old_term is None:
            return
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            self._handle_key(sys.stdin.read(1).lower())

    def _select_non_joystick_source(self) -> tuple[Source, Twist]:
        now = time.monotonic()
        if self._kb_enabled and self._old_term is not None and self._source == Source.KEYBOARD:
            msg = Twist()
            msg.linear.x = self._kb_linear
            msg.angular.z = self._kb_angular
            return Source.KEYBOARD, msg
        if self._fresh(self._web_time, self._web_cmd_timeout):
            return Source.WEB, self._web_cmd
        if self._fresh(self._nav_time, self._nav_cmd_timeout):
            return Source.NAV, self._nav_cmd
        if now - self._last_cmd_time <= self._stop_timeout:
            return Source.NONE, self._zero()
        return Source.NONE, self._zero()

    def _tick(self):
        self._poll_keyboard()

        if self._base_fault_active:
            self._publish_stop()
            return

        js_fresh = self._fresh(self._js_time, self._js_cmd_timeout)
        js_nonzero = js_fresh and self._nonzero(self._js_cmd)
        js_reset = js_fresh and not self._nonzero(self._js_cmd)

        if self._joystick_stop_latched:
            self._publish_stop()
            if js_reset:
                self._joystick_stop_latched = False
                self._source = Source.NONE
                self._publish_status("joystick_reset")
                self.get_logger().info("摇杆已复位，允许新的控制指令")
            return

        if js_nonzero:
            interrupted = self._source
            if interrupted == Source.NONE:
                pending_source, _ = self._select_non_joystick_source()
                interrupted = pending_source
            if interrupted in (Source.NONE, Source.JOYSTICK):
                self._source = Source.JOYSTICK
                self._publish(self._js_cmd)
                return
            self._enter_joystick_stop_lock(interrupted)
            return

        if self._source == Source.JOYSTICK and js_reset:
            self._source = Source.NONE
            self._publish_stop()
            return

        source, msg = self._select_non_joystick_source()
        self._source = source
        self._publish(msg)

    def destroy_node(self):
        if self._old_term is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)
            except Exception:
                pass
        try:
            self._publish_stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = BaseControlRouter()
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
