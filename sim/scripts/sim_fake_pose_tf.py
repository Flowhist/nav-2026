#!/usr/bin/env python3
"""虚拟位姿 TF 发布器
发布 map->base_link，支持两种方式更新位姿：
1) 订阅 /initialpose 手动设置
2) 订阅 /cmd_vel 进行差速积分（可选）
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class SimFakePoseTF(Node):
    def __init__(self) -> None:
        super().__init__("sim_fake_pose_tf")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("use_cmd_vel", False)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw_deg", 0.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.use_cmd_vel = bool(self.get_parameter("use_cmd_vel").value)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = math.radians(float(self.get_parameter("yaw_deg").value))
        self.v = 0.0
        self.w = 0.0

        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(
            PoseWithCovarianceStamped, self.initialpose_topic, self._on_initialpose, 10
        )
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)

        self.last_t = self.get_clock().now()
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._loop)

        self.get_logger().info(
            "sim_fake_pose_tf started | pose=(%.2f, %.2f, %.1fdeg) | use_cmd_vel=%s"
            % (self.x, self.y, math.degrees(self.yaw), str(self.use_cmd_vel))
        )

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"ignore initialpose frame={msg.header.frame_id}, expected={self.map_frame}"
            )
            return
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.get_logger().info(
            "pose set by /initialpose: (%.2f, %.2f, %.1fdeg)"
            % (self.x, self.y, math.degrees(self.yaw))
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def _loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0:
            dt = 1e-3

        if self.use_cmd_vel:
            self.yaw = self._norm(self.yaw + self.w * dt)
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw * 0.5)
        t.transform.rotation.w = math.cos(self.yaw * 0.5)
        self.tf_pub.sendTransform(t)

    @staticmethod
    def _norm(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = SimFakePoseTF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
