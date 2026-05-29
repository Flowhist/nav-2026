#!/usr/bin/env python3
"""
Direct Linux joystick bridge.

Reads /dev/input/js* directly and publishes the existing finav joystick
contract: /joy for diagnostics, plus /js_state and /js_cmd_vel for control.
"""

import math
import os
import struct
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

EVENT_FORMAT = "IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
DEFAULT_AXES = 8


class JoyMappingConfig:
    def __init__(
        self,
        linear_axis=0,
        linear_direction=-1.0,
        angular_axis=1,
        angular_direction=1.0,
        dead_zone=0.05,
        sat_zone=1.0,
        speed_split=0.5,
        discrete_motion_enable=True,
        linear_speed_low=0.2,
        linear_speed_high=0.4,
        angular_speed_low=math.radians(10.0),
        angular_speed_high=math.radians(25.0),
    ):
        self.linear_axis = int(linear_axis)
        self.linear_direction = float(linear_direction)
        self.angular_axis = int(angular_axis)
        self.angular_direction = float(angular_direction)
        self.dead_zone = max(0.0, float(dead_zone))
        self.sat_zone = max(self.dead_zone + 1e-6, float(sat_zone))
        self.speed_split = min(1.0, max(0.0, float(speed_split)))
        self.discrete_motion_enable = bool(discrete_motion_enable)
        self.linear_speed_low = float(linear_speed_low)
        self.linear_speed_high = float(linear_speed_high)
        self.angular_speed_low = float(angular_speed_low)
        self.angular_speed_high = float(angular_speed_high)


def _scaled_axis(value: float, direction: float, dead_zone: float, sat_zone: float) -> float:
    signed = float(value) * float(direction)
    mag = abs(signed)
    if mag <= dead_zone:
        return 0.0
    if mag >= sat_zone:
        return math.copysign(1.0, signed)
    return math.copysign((mag - dead_zone) / (sat_zone - dead_zone), signed)


def _axis_value(axes, index: int) -> float:
    if index < 0 or index >= len(axes):
        return 0.0
    return float(axes[index])


def _tiered_speed(value: float, low_speed: float, high_speed: float, split: float) -> float:
    mag = abs(value)
    if mag <= 0.0:
        return 0.0
    speed = low_speed if mag < split else high_speed
    return math.copysign(speed, value)


def joy_axes_to_twist(axes, cfg: JoyMappingConfig) -> Twist:
    msg = Twist()
    linear = _scaled_axis(
        _axis_value(axes, cfg.linear_axis),
        cfg.linear_direction,
        cfg.dead_zone,
        cfg.sat_zone,
    )
    angular = _scaled_axis(
        _axis_value(axes, cfg.angular_axis),
        cfg.angular_direction,
        cfg.dead_zone,
        cfg.sat_zone,
    )
    if cfg.discrete_motion_enable:
        if abs(linear) >= abs(angular):
            angular = 0.0
        else:
            linear = 0.0

    msg.linear.x = _tiered_speed(
        linear, cfg.linear_speed_low, cfg.linear_speed_high, cfg.speed_split
    )
    msg.angular.z = _tiered_speed(
        angular, cfg.angular_speed_low, cfg.angular_speed_high, cfg.speed_split
    )
    return msg


def _normalize_axis(raw_value: int) -> float:
    if raw_value < 0:
        return max(-1.0, float(raw_value) / 32768.0)
    return min(1.0, float(raw_value) / 32767.0)


class LinuxJoystickReader:
    def __init__(self, device_path: str, axes_count: int = DEFAULT_AXES):
        self.device_path = device_path
        self.fd = None
        self.axes = [0.0] * axes_count
        self.buttons = []

    @property
    def is_open(self) -> bool:
        return self.fd is not None

    def open(self) -> bool:
        if self.fd is not None:
            return True
        try:
            self.fd = os.open(self.device_path, os.O_RDONLY | os.O_NONBLOCK)
            return True
        except OSError:
            self.fd = None
            return False

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def read_available(self) -> bool:
        if self.fd is None:
            return False

        updated = False
        while True:
            try:
                data = os.read(self.fd, EVENT_SIZE)
            except BlockingIOError:
                break
            except OSError:
                self.close()
                break

            if len(data) != EVENT_SIZE:
                break

            _, value, event_type, number = struct.unpack(EVENT_FORMAT, data)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS:
                while number >= len(self.axes):
                    self.axes.append(0.0)
                self.axes[number] = _normalize_axis(value)
                updated = True
            elif event_type == JS_EVENT_BUTTON:
                while number >= len(self.buttons):
                    self.buttons.append(0)
                self.buttons[number] = 1 if value else 0
                updated = True

        return updated


class JoyControl(Node):
    def __init__(self):
        super().__init__("joy_control")
        self.declare_parameter("enabled", True)
        self.declare_parameter("dev", "/dev/input/js0")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("state_topic", "/js_state")
        self.declare_parameter("cmd_vel_topic", "/js_cmd_vel")
        self.declare_parameter("publish_rate", 25.0)
        self.declare_parameter("reconnect_interval", 1.0)
        self.declare_parameter("linear_axis", 0)
        self.declare_parameter("linear_direction", -1.0)
        self.declare_parameter("angular_axis", 1)
        self.declare_parameter("angular_direction", 1.0)
        self.declare_parameter("dead_zone", 0.05)
        self.declare_parameter("sat_zone", 1.0)
        self.declare_parameter("speed_split", 0.5)
        self.declare_parameter("discrete_motion_enable", True)
        self.declare_parameter("js_vel_low", 0.2)
        self.declare_parameter("js_vel_high", 0.4)
        self.declare_parameter("js_rot_low", 10.0)
        self.declare_parameter("js_rot_high", 25.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.device_path = str(self.get_parameter("dev").value)
        self.reconnect_interval = max(
            0.2, float(self.get_parameter("reconnect_interval").value)
        )
        self.cfg = JoyMappingConfig(
            linear_axis=self.get_parameter("linear_axis").value,
            linear_direction=self.get_parameter("linear_direction").value,
            angular_axis=self.get_parameter("angular_axis").value,
            angular_direction=self.get_parameter("angular_direction").value,
            dead_zone=self.get_parameter("dead_zone").value,
            sat_zone=self.get_parameter("sat_zone").value,
            speed_split=self.get_parameter("speed_split").value,
            discrete_motion_enable=self.get_parameter("discrete_motion_enable").value,
            linear_speed_low=self.get_parameter("js_vel_low").value,
            linear_speed_high=self.get_parameter("js_vel_high").value,
            angular_speed_low=math.radians(float(self.get_parameter("js_rot_low").value)),
            angular_speed_high=math.radians(float(self.get_parameter("js_rot_high").value)),
        )

        self.reader = LinuxJoystickReader(self.device_path)
        self._last_open_attempt = 0.0
        self._reported_ready = False
        self._reported_missing = False
        self._last_twist = Twist()

        self._joy_pub = self.create_publisher(
            Joy, str(self.get_parameter("joy_topic").value), 10
        )
        self._state_pub = self.create_publisher(
            Bool, str(self.get_parameter("state_topic").value), 10
        )
        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / publish_rate, self._publish)

        if self.enabled:
            self.get_logger().info(f"HID摇杆节点已启动，直接读取 {self.device_path}")
        else:
            self.get_logger().info("HID摇杆节点已禁用")

    def _ensure_open(self) -> bool:
        if not self.enabled:
            return False
        if self.reader.is_open:
            return True

        now = monotonic()
        if now - self._last_open_attempt < self.reconnect_interval:
            return False
        self._last_open_attempt = now

        if self.reader.open():
            self._reported_missing = False
            if not self._reported_ready:
                self.get_logger().info(f"joystick启动成功: {self.device_path}")
                self._reported_ready = True
            return True

        if not self._reported_missing:
            self.get_logger().warn(f"无法打开摇杆设备: {self.device_path}")
            self._reported_missing = True
        return False

    def _publish_joy(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(self.reader.axes)
        msg.buttons = list(self.reader.buttons)
        self._joy_pub.publish(msg)

    def _publish(self):
        active = self._ensure_open()
        if active:
            self.reader.read_available()
            self._last_twist = joy_axes_to_twist(self.reader.axes, self.cfg)
            self._publish_joy()
        else:
            self._last_twist = Twist()

        state = Bool()
        state.data = active
        self._state_pub.publish(state)
        if self.enabled:
            self._cmd_pub.publish(self._last_twist)

    def destroy_node(self):
        try:
            self.reader.close()
            state = Bool()
            state.data = False
            self._state_pub.publish(state)
            self._cmd_pub.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
