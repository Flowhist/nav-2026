#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import TransformBroadcaster


class SimGazeboDrive(Node):
    def __init__(self) -> None:
        super().__init__("sim_gazebo_drive")

        self.declare_parameter("world_name", "office_corridor_sketch")
        self.declare_parameter("model_name", "finav_vehicle")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("cmd_timeout", 0.35)
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("pose_z", 0.16)
        self.declare_parameter("x", -0.15)
        self.declare_parameter("y", -5.65)
        self.declare_parameter("yaw", 1.5707963)

        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_rate_hz = max(
            1.0, float(self.get_parameter("publish_rate_hz").value)
        )
        self.cmd_timeout = max(0.05, float(self.get_parameter("cmd_timeout").value))
        self.max_linear_speed = abs(
            float(self.get_parameter("max_linear_speed").value)
        )
        self.max_angular_speed = abs(
            float(self.get_parameter("max_angular_speed").value)
        )
        self.pose_z = float(self.get_parameter("pose_z").value)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.v = 0.0
        self.w = 0.0
        self.last_cmd_mono: Optional[float] = None
        self._pose_req_pending = False

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.initialpose_topic, self._on_initialpose, 10
        )

        self.set_pose_client = self.create_client(
            SetEntityPose, f"/world/{self.world_name}/set_pose"
        )

        self.last_t = self.get_clock().now()
        self.create_timer(1.0 / self.publish_rate_hz, self._loop)

        self.get_logger().info(
            "sim_gazebo_drive started | model=%s | service=/world/%s/set_pose | pose=(%.2f, %.2f, %.1fdeg)"
            % (
                self.model_name,
                self.world_name,
                self.x,
                self.y,
                math.degrees(self.yaw),
            )
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.v = self._clamp(float(msg.linear.x), -self.max_linear_speed, self.max_linear_speed)
        self.w = self._clamp(float(msg.angular.z), -self.max_angular_speed, self.max_angular_speed)
        self.last_cmd_mono = self._now_mono()

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        self._send_set_pose()

    def _loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0:
            dt = 1e-3

        if self.last_cmd_mono is None or (self._now_mono() - self.last_cmd_mono) > self.cmd_timeout:
            self.v = 0.0
            self.w = 0.0

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = self._norm(self.yaw + self.w * dt)

        self._publish_odom(now)
        self._send_set_pose()

    def _publish_odom(self, now) -> None:
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_pub.sendTransform(t)

    def _send_set_pose(self) -> None:
        if self._pose_req_pending or not self.set_pose_client.service_is_ready():
            return

        req = SetEntityPose.Request()
        req.entity = Entity(name=self.model_name, type=Entity.MODEL)
        req.pose.position.x = self.x
        req.pose.position.y = self.y
        req.pose.position.z = self.pose_z
        req.pose.orientation.z = math.sin(self.yaw * 0.5)
        req.pose.orientation.w = math.cos(self.yaw * 0.5)

        future = self.set_pose_client.call_async(req)
        self._pose_req_pending = True
        future.add_done_callback(self._on_set_pose_done)

    def _on_set_pose_done(self, future) -> None:
        self._pose_req_pending = False
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(f"set_pose failed: {exc}")
            return
        if not resp.success:
            self.get_logger().warn("set_pose rejected by Gazebo")

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    @staticmethod
    def _norm(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _now_mono() -> float:
        from time import monotonic

        return monotonic()


def main() -> None:
    rclpy.init()
    node = SimGazeboDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
