#!/usr/bin/env python3
"""Bridge the STM32 handle registers to Finav's existing control topics."""

from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from serial import SerialException
from std_msgs.msg import Bool, Empty, UInt16

from handle_modbus import ModbusError, ModbusRtuClient
from handle_protocol import (
    DISPLAY_SPEED_REGISTER,
    HANDLE_STATE_REGISTER,
    HANDLE_STATE_REGISTER_COUNT,
    HandleRegisters,
    command_from_registers,
    navigation_button_pressed,
    speed_mps_to_register,
)


class HandleControl(Node):
    """Own the handle serial port and publish safe, scaled velocity commands."""

    def __init__(self):
        super().__init__("handle_control")
        self.declare_parameter("enabled", True)
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("slave_id", 1)
        self.declare_parameter("serial_timeout", 0.05)
        self.declare_parameter("poll_rate", 50.0)
        self.declare_parameter("speed_write_rate", 10.0)
        self.declare_parameter("reconnect_interval", 1.0)
        self.declare_parameter("axis_minimum", 0)
        self.declare_parameter("axis_center", 2048)
        self.declare_parameter("axis_maximum", 4095)
        self.declare_parameter("dead_zone", 50)
        self.declare_parameter("linear_direction", -1.0)
        self.declare_parameter("angular_direction", -1.0)
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("state_topic", "/js_state")
        self.declare_parameter("cmd_vel_topic", "/js_cmd_vel")
        self.declare_parameter("buttons_topic", "/handle/buttons")
        self.declare_parameter("gear_topic", "/handle/gear")
        self.declare_parameter(
            "navigation_button_topic", "/handle/navigation_button"
        )
        self.declare_parameter("odometry_topic", "/odom_encoder")

        self.enabled = bool(self.get_parameter("enabled").value)
        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.slave_id = int(self.get_parameter("slave_id").value)
        self.serial_timeout = max(
            0.001, float(self.get_parameter("serial_timeout").value)
        )
        self.reconnect_interval = max(
            0.2, float(self.get_parameter("reconnect_interval").value)
        )
        self.axis_minimum = int(self.get_parameter("axis_minimum").value)
        self.axis_center = int(self.get_parameter("axis_center").value)
        self.axis_maximum = int(self.get_parameter("axis_maximum").value)
        self.dead_zone = int(self.get_parameter("dead_zone").value)
        self.linear_direction = float(
            self.get_parameter("linear_direction").value
        )
        self.angular_direction = float(
            self.get_parameter("angular_direction").value
        )
        self.max_linear_speed = max(
            0.0, float(self.get_parameter("max_linear_speed").value)
        )
        self.max_angular_speed = max(
            0.0, float(self.get_parameter("max_angular_speed").value)
        )

        cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._joy_pub = self.create_publisher(
            Joy, str(self.get_parameter("joy_topic").value), 10
        )
        self._state_pub = self.create_publisher(
            Bool, str(self.get_parameter("state_topic").value), 10
        )
        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), cmd_qos
        )
        self._buttons_pub = self.create_publisher(
            UInt16, str(self.get_parameter("buttons_topic").value), 10
        )
        self._gear_pub = self.create_publisher(
            UInt16, str(self.get_parameter("gear_topic").value), 10
        )
        self._navigation_button_pub = self.create_publisher(
            Empty,
            str(self.get_parameter("navigation_button_topic").value),
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._on_odometry,
            10,
        )

        self._client = None
        self._last_connect_attempt = 0.0
        self._online = False
        self._buttons_initialized = False
        self._previous_buttons = 0
        self._actual_speed_mps = 0.0

        poll_rate = max(1.0, float(self.get_parameter("poll_rate").value))
        speed_write_rate = max(
            0.1, float(self.get_parameter("speed_write_rate").value)
        )
        self.create_timer(1.0 / poll_rate, self._poll_handle)
        self.create_timer(1.0 / speed_write_rate, self._write_display_speed)

        if self.enabled:
            self.get_logger().info(
                f"STM32手柄节点已启动，等待连接 {self.port} @ {self.baudrate}"
            )
        else:
            self.get_logger().info("STM32手柄节点已禁用")

    def _connect(self) -> bool:
        if self._client is not None:
            return True
        now = monotonic()
        if now - self._last_connect_attempt < self.reconnect_interval:
            return False
        self._last_connect_attempt = now
        try:
            self._client = ModbusRtuClient(
                port=self.port,
                baudrate=self.baudrate,
                slave_id=self.slave_id,
                timeout=self.serial_timeout,
            )
        except (OSError, SerialException, ValueError) as exc:
            self._set_online(False)
            self.get_logger().warning(f"STM32手柄连接失败: {exc}")
            return False
        self.get_logger().info(f"STM32手柄已连接: {self.port}")
        return True

    def _disconnect(self, reason: str) -> None:
        if self._client is not None:
            self._client.close()
            self._client = None
        self._buttons_initialized = False
        self._set_online(False)
        self._publish_zero()
        self.get_logger().warning(f"STM32手柄通信中断: {reason}")

    def _set_online(self, online: bool) -> None:
        online = bool(online)
        if online != self._online:
            message = "STM32手柄通信正常" if online else "STM32手柄已离线"
            self.get_logger().info(message)
        self._online = online
        state = Bool()
        state.data = online
        self._state_pub.publish(state)

    def _poll_handle(self) -> None:
        if not self.enabled:
            self._set_online(False)
            self._publish_zero()
            return
        if not self._connect():
            self._set_online(False)
            self._publish_zero()
            return

        try:
            words = self._client.read_holding_registers(
                HANDLE_STATE_REGISTER, HANDLE_STATE_REGISTER_COUNT
            )
            sample = HandleRegisters.from_words(words)
            command = command_from_registers(
                sample,
                axis_minimum=self.axis_minimum,
                axis_center=self.axis_center,
                axis_maximum=self.axis_maximum,
                dead_zone=self.dead_zone,
                linear_direction=self.linear_direction,
                angular_direction=self.angular_direction,
                max_linear_speed=self.max_linear_speed,
                max_angular_speed=self.max_angular_speed,
            )
        except (ModbusError, OSError, SerialException, ValueError) as exc:
            self._disconnect(str(exc))
            return

        self._publish_sample(sample)
        if not command.valid:
            self._set_online(False)
            self._publish_zero()
            return

        self._set_online(True)
        twist = Twist()
        twist.linear.x = command.linear
        twist.angular.z = command.angular
        self._cmd_pub.publish(twist)

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = list(command.joy_axes)
        joy.buttons = [(sample.buttons >> bit) & 1 for bit in range(7)]
        self._joy_pub.publish(joy)

    def _publish_sample(self, sample: HandleRegisters) -> None:
        buttons = UInt16()
        buttons.data = sample.buttons
        self._buttons_pub.publish(buttons)

        gear = UInt16()
        gear.data = sample.gear
        self._gear_pub.publish(gear)

        if self._buttons_initialized and navigation_button_pressed(
            self._previous_buttons, sample.buttons
        ):
            self._navigation_button_pub.publish(Empty())
        self._previous_buttons = sample.buttons
        self._buttons_initialized = True

    def _on_odometry(self, message: Odometry) -> None:
        self._actual_speed_mps = float(message.twist.twist.linear.x)

    def _write_display_speed(self) -> None:
        if not self.enabled or not self._online or self._client is None:
            return
        try:
            self._client.write_single_register(
                DISPLAY_SPEED_REGISTER,
                speed_mps_to_register(self._actual_speed_mps),
            )
        except (ModbusError, OSError, SerialException, ValueError) as exc:
            self._disconnect(str(exc))

    def _publish_zero(self) -> None:
        if self.enabled:
            self._cmd_pub.publish(Twist())

    def destroy_node(self):
        if rclpy.ok():
            self._publish_zero()
            self._set_online(False)
        if self._client is not None:
            self._client.close()
            self._client = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandleControl()
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
