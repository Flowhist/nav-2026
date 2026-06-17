#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import TransformBroadcaster


class GzTopicAdapter(Node):
    def __init__(self) -> None:
        super().__init__("gz_topic_adapter")

        self.declare_parameter(
            "scan_src_topic",
            "/scan_gz",
        )
        self.declare_parameter(
            "imu_src_topic",
            "/world/office_corridor_sketch/model/finav_vehicle/link/imu_link/sensor/imu_sensor/imu",
        )
        self.declare_parameter(
            "odom_src_topic",
            "/model/finav_vehicle/odometry",
        )
        self.declare_parameter(
            "joint_state_src_topic",
            "/world/office_corridor_sketch/model/finav_vehicle/joint_state",
        )
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_joint_states", True)

        self.scan_src_topic = str(self.get_parameter("scan_src_topic").value)
        self.imu_src_topic = str(self.get_parameter("imu_src_topic").value)
        self.odom_src_topic = str(self.get_parameter("odom_src_topic").value)
        self.joint_state_src_topic = str(self.get_parameter("joint_state_src_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_joint_states = bool(
            self.get_parameter("publish_joint_states").value
        )

        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.joint_state_pub = self.create_publisher(
            JointState, self.joint_states_topic, 10
        )
        self.tf_pub = TransformBroadcaster(self)

        self.create_subscription(LaserScan, self.scan_src_topic, self._on_scan, 10)
        self.create_subscription(Imu, self.imu_src_topic, self._on_imu, 10)
        self.create_subscription(Odometry, self.odom_src_topic, self._on_odom, 10)
        if self.publish_joint_states:
            self.create_subscription(
                JointState, self.joint_state_src_topic, self._on_joint_state, 10
            )

        self.get_logger().info(
            "gz_topic_adapter started | scan=%s | imu=%s | odom=%s"
            % (self.scan_src_topic, self.imu_src_topic, self.odom_src_topic)
        )

    def _on_scan(self, msg: LaserScan) -> None:
        msg.header.frame_id = "laser_frame"
        self.scan_pub.publish(msg)

    def _on_imu(self, msg: Imu) -> None:
        msg.header.frame_id = "imu_link"
        self.imu_pub.publish(msg)

    def _on_joint_state(self, msg: JointState) -> None:
        self.joint_state_pub.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        self.odom_pub.publish(msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_pub.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = GzTopicAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
