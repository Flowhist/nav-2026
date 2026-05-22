#!/usr/bin/env python3
"""
USB HID joystick bridge.

Subscribes to /joy from ros-humble-joy and publishes the existing finav
joystick contract: /js_state and /js_cmd_vel.
"""

import math
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


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
    msg.linear.x = _tiered_speed(
        linear, cfg.linear_speed_low, cfg.linear_speed_high, cfg.speed_split
    )
    msg.angular.z = _tiered_speed(
        angular, cfg.angular_speed_low, cfg.angular_speed_high, cfg.speed_split
    )
    return msg


class JoyControl(Node):
    def __init__(self):
        super().__init__("joy_control")
        self.declare_parameter("enabled", True)
        self.declare_parameter("input_topic", "/joy")
        self.declare_parameter("state_topic", "/js_state")
        self.declare_parameter("cmd_vel_topic", "/js_cmd_vel")
        self.declare_parameter("publish_rate", 25.0)
        self.declare_parameter("cmd_timeout", 0.35)
        self.declare_parameter("linear_axis", 0)
        self.declare_parameter("linear_direction", -1.0)
        self.declare_parameter("angular_axis", 1)
        self.declare_parameter("angular_direction", 1.0)
        self.declare_parameter("dead_zone", 0.05)
        self.declare_parameter("sat_zone", 1.0)
        self.declare_parameter("speed_split", 0.5)
        self.declare_parameter("js_vel_low", 0.2)
        self.declare_parameter("js_vel_high", 0.4)
        self.declare_parameter("js_rot_low", 10.0)
        self.declare_parameter("js_rot_high", 25.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.cmd_timeout = max(0.05, float(self.get_parameter("cmd_timeout").value))
        self.cfg = JoyMappingConfig(
            linear_axis=self.get_parameter("linear_axis").value,
            linear_direction=self.get_parameter("linear_direction").value,
            angular_axis=self.get_parameter("angular_axis").value,
            angular_direction=self.get_parameter("angular_direction").value,
            dead_zone=self.get_parameter("dead_zone").value,
            sat_zone=self.get_parameter("sat_zone").value,
            speed_split=self.get_parameter("speed_split").value,
            linear_speed_low=self.get_parameter("js_vel_low").value,
            linear_speed_high=self.get_parameter("js_vel_high").value,
            angular_speed_low=math.radians(
                float(self.get_parameter("js_rot_low").value)
            ),
            angular_speed_high=math.radians(
                float(self.get_parameter("js_rot_high").value)
            ),
        )

        self._last_joy_time = 0.0
        self._last_twist = Twist()
        self._reported_ready = False

        self._state_pub = self.create_publisher(
            Bool, str(self.get_parameter("state_topic").value), 10
        )
        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.create_subscription(
            Joy,
            str(self.get_parameter("input_topic").value),
            self._on_joy,
            10,
        )
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / publish_rate, self._publish)

        if self.enabled:
            self.get_logger().info(
                "HID摇杆桥接节点已启动，等待 /joy 数据..."
            )
        else:
            self.get_logger().info("HID摇杆桥接节点已禁用")

    def _on_joy(self, msg: Joy):
        if not self.enabled:
            return
        self._last_twist = joy_axes_to_twist(msg.axes, self.cfg)
        self._last_joy_time = monotonic()
        if not self._reported_ready:
            self.get_logger().info("joystick启动成功！")
            self._reported_ready = True

    def _publish(self):
        active = self.enabled
        if active and (monotonic() - self._last_joy_time) > self.cmd_timeout:
            self._last_twist = Twist()

        state = Bool()
        state.data = active
        self._state_pub.publish(state)
        if active:
            self._cmd_pub.publish(self._last_twist)

    def destroy_node(self):
        try:
            state = Bool()
            state.data = False
            self._state_pub.publish(state)
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
