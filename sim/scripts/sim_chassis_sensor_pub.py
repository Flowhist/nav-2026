#!/usr/bin/env python3
"""虚拟位姿 TF 发布器
发布 parent_frame->base_link，支持三种方式更新位姿：
1) 订阅 /initialpose 手动设置
2) 订阅 /cmd_vel 进行积分
3) 订阅 /plan 自动做路径跟随演示
"""

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu, JointState
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


WorldPose = Tuple[float, float, float]


class SimFakePoseTF(Node):
    def __init__(self) -> None:
        super().__init__("sim_chassis_sensor_pub")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("odom_frame", "")
        self.declare_parameter("tf_parent_frame", "")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_odom_encoder", False)
        self.declare_parameter("odom_encoder_topic", "/odom_encoder")
        self.declare_parameter("publish_imu", False)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_joint_states", False)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("left_wheel_joint", "left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "right_wheel_joint")
        self.declare_parameter("wheel_radius", 0.127)
        self.declare_parameter("wheel_separation", 0.6)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("use_cmd_vel", False)
        self.declare_parameter("follow_plan", True)
        self.declare_parameter("replan_pause_s", 0.20)
        self.declare_parameter("demo_linear_speed_mps", 0.35)
        self.declare_parameter("demo_angular_speed_rps", 0.65)
        self.declare_parameter("demo_waypoint_tolerance_m", 0.10)
        self.declare_parameter("demo_heading_slowdown_deg", 40.0)
        self.declare_parameter("sim_startup_anchor_enable", True)
        self.declare_parameter("sim_startup_anchor_trigger_deg", 35.0)
        self.declare_parameter("sim_startup_anchor_distance_m", 0.20)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw_deg", 0.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.publish_odom = bool(self.get_parameter("publish_odom").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value).strip()
        if not self.odom_frame:
            self.odom_frame = self.map_frame
        self.tf_parent_frame = str(self.get_parameter("tf_parent_frame").value).strip()
        if not self.tf_parent_frame:
            self.tf_parent_frame = self.map_frame
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_odom_encoder = bool(
            self.get_parameter("publish_odom_encoder").value
        )
        self.odom_encoder_topic = str(self.get_parameter("odom_encoder_topic").value)
        self.publish_imu = bool(self.get_parameter("publish_imu").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.publish_joint_states = bool(
            self.get_parameter("publish_joint_states").value
        )
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.left_wheel_joint = str(self.get_parameter("left_wheel_joint").value)
        self.right_wheel_joint = str(self.get_parameter("right_wheel_joint").value)
        self.wheel_radius = max(0.01, float(self.get_parameter("wheel_radius").value))
        self.wheel_separation = max(
            0.01, float(self.get_parameter("wheel_separation").value)
        )
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.use_cmd_vel = bool(self.get_parameter("use_cmd_vel").value)
        self.follow_plan = bool(self.get_parameter("follow_plan").value)
        self.replan_pause_s = max(
            0.0, float(self.get_parameter("replan_pause_s").value)
        )
        self.demo_linear_speed_mps = float(
            self.get_parameter("demo_linear_speed_mps").value
        )
        self.demo_angular_speed_rps = float(
            self.get_parameter("demo_angular_speed_rps").value
        )
        self.demo_waypoint_tolerance_m = float(
            self.get_parameter("demo_waypoint_tolerance_m").value
        )
        self.demo_heading_slowdown = math.radians(
            float(self.get_parameter("demo_heading_slowdown_deg").value)
        )
        self.sim_startup_anchor_enable = bool(
            self.get_parameter("sim_startup_anchor_enable").value
        )
        self.sim_startup_anchor_trigger = math.radians(
            float(self.get_parameter("sim_startup_anchor_trigger_deg").value)
        )
        self.sim_startup_anchor_distance = max(
            0.0, float(self.get_parameter("sim_startup_anchor_distance_m").value)
        )

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = math.radians(float(self.get_parameter("yaw_deg").value))
        self.v = 0.0
        self.w = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        self.path_poses: List[WorldPose] = []
        self.path_active = False
        self.path_index = 0
        self.replan_pause_until_ns = 0

        self.tf_pub = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.odom_encoder_pub = self.create_publisher(Odometry, self.odom_encoder_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.joint_state_pub = self.create_publisher(JointState, self.joint_states_topic, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.initialpose_topic, self._on_initialpose, 10
        )
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(Path, self.plan_topic, self._on_plan, 10)

        self.last_t = self.get_clock().now()
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._loop)

        self.get_logger().info(
            "sim_chassis_sensor_pub started | pose=(%.2f, %.2f, %.1fdeg) | use_cmd_vel=%s | follow_plan=%s | replan_pause=%.2fs"
            % (
                self.x,
                self.y,
                math.degrees(self.yaw),
                str(self.use_cmd_vel),
                str(self.follow_plan),
                self.replan_pause_s,
            )
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
        self.yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        self.path_active = False
        self.replan_pause_until_ns = 0
        self.get_logger().info(
            "pose set by /initialpose: (%.2f, %.2f, %.1fdeg)"
            % (self.x, self.y, math.degrees(self.yaw))
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def _on_plan(self, msg: Path) -> None:
        poses: List[WorldPose] = []
        for p in msg.poses:
            q = p.pose.orientation
            poses.append(
                (
                    float(p.pose.position.x),
                    float(p.pose.position.y),
                    self._quat_to_yaw(q.x, q.y, q.z, q.w),
                )
            )
        if len(poses) < 2:
            self.path_poses = poses
            self.path_active = False
            self.replan_pause_until_ns = 0
            return

        poses = self._inject_sim_startup_anchor(poses)

        had_active_plan = self.path_active and bool(self.path_poses)
        self.path_poses = poses
        self.path_index = self._find_nearest_path_index(poses, self.x, self.y)
        self.path_active = self.follow_plan
        if had_active_plan and self.replan_pause_s > 0.0:
            self.replan_pause_until_ns = (
                self.get_clock().now().nanoseconds + int(self.replan_pause_s * 1e9)
            )
        else:
            self.replan_pause_until_ns = 0
        self.get_logger().info(
            "demo path received: %d poses, switch to new path at index %d%s"
            % (
                len(poses),
                self.path_index,
                f", pause {self.replan_pause_s:.2f}s" if had_active_plan else "",
            )
        )

    def _loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0:
            dt = 1e-3

        if self.follow_plan and self.path_active and self.path_poses:
            if now.nanoseconds < self.replan_pause_until_ns:
                return
            self._step_follow_plan(now.nanoseconds, dt)
        elif self.use_cmd_vel:
            self.yaw = self._norm(self.yaw + self.w * dt)
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt

        self._update_wheel_states(dt)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.tf_parent_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(self.yaw * 0.5)
            t.transform.rotation.w = math.cos(self.yaw * 0.5)
            self.tf_pub.sendTransform(t)

        if self.publish_odom:
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = float(self.x)
            odom.pose.pose.position.y = float(self.y)
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
            odom.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
            odom.twist.twist.linear.x = float(self.v)
            odom.twist.twist.angular.z = float(self.w)
            self.odom_pub.publish(odom)

        if self.publish_odom_encoder:
            enc = Odometry()
            enc.header.stamp = now.to_msg()
            enc.header.frame_id = self.odom_frame
            enc.child_frame_id = self.base_frame
            enc.pose.pose.position.x = float(self.x)
            enc.pose.pose.position.y = float(self.y)
            enc.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
            enc.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
            enc.twist.twist.linear.x = float(self.v)
            enc.twist.twist.angular.z = float(self.w)
            self.odom_encoder_pub.publish(enc)

        if self.publish_imu:
            imu = Imu()
            imu.header.stamp = now.to_msg()
            imu.header.frame_id = self.imu_frame
            imu.orientation.x = 0.0
            imu.orientation.y = 0.0
            imu.orientation.z = math.sin(self.yaw * 0.5)
            imu.orientation.w = math.cos(self.yaw * 0.5)
            imu.orientation_covariance[0] = 0.1
            imu.orientation_covariance[4] = 0.1
            imu.orientation_covariance[8] = 0.3
            imu.angular_velocity.z = float(self.w)
            imu.angular_velocity_covariance[0] = 0.01
            imu.angular_velocity_covariance[4] = 0.01
            imu.angular_velocity_covariance[8] = 0.01
            imu.linear_acceleration_covariance[0] = 0.01
            imu.linear_acceleration_covariance[4] = 0.01
            imu.linear_acceleration_covariance[8] = 0.01
            self.imu_pub.publish(imu)

        if self.publish_joint_states:
            js = JointState()
            js.header.stamp = now.to_msg()
            js.name = [self.left_wheel_joint, self.right_wheel_joint]
            js.position = [float(self.left_wheel_pos), float(self.right_wheel_pos)]
            vl = self.v - 0.5 * self.w * self.wheel_separation
            vr = self.v + 0.5 * self.w * self.wheel_separation
            js.velocity = [
                float(vl / self.wheel_radius),
                float(vr / self.wheel_radius),
            ]
            self.joint_state_pub.publish(js)

    def _step_follow_plan(self, _now_ns: int, dt: float) -> None:
        if self.path_index >= len(self.path_poses):
            self.v = 0.0
            self.w = 0.0
            self.path_active = False
            return

        tx, ty, tyaw = self.path_poses[self.path_index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist <= self.demo_waypoint_tolerance_m:
            if self.path_index >= len(self.path_poses) - 1:
                self.yaw = tyaw
                self.v = 0.0
                self.w = 0.0
                self.path_active = False
                return
            self.path_index += 1
            return

        target_heading = math.atan2(dy, dx)
        heading_error = self._norm(target_heading - self.yaw)

        w_cmd = max(
            -self.demo_angular_speed_rps,
            min(self.demo_angular_speed_rps, heading_error * 2.5),
        )
        slowdown = max(
            0.0,
            1.0 - min(abs(heading_error), self.demo_heading_slowdown)
            / max(self.demo_heading_slowdown, 1e-6),
        )
        v_cmd = min(self.demo_linear_speed_mps, dist * 1.5) * slowdown
        self.v = v_cmd
        self.w = w_cmd

        self.yaw = self._norm(self.yaw + w_cmd * dt)
        self.x += v_cmd * math.cos(self.yaw) * dt
        self.y += v_cmd * math.sin(self.yaw) * dt

    def _update_wheel_states(self, dt: float) -> None:
        if dt <= 0.0:
            return
        vl = self.v - 0.5 * self.w * self.wheel_separation
        vr = self.v + 0.5 * self.w * self.wheel_separation
        self.left_wheel_pos += (vl / self.wheel_radius) * dt
        self.right_wheel_pos += (vr / self.wheel_radius) * dt

    def _inject_sim_startup_anchor(self, poses: List[WorldPose]) -> List[WorldPose]:
        if not self.sim_startup_anchor_enable:
            return poses
        if len(poses) < 2 or self.sim_startup_anchor_distance <= 1e-3:
            return poses

        tx, ty, _ = poses[0]
        heading_to_first = math.atan2(ty - self.y, tx - self.x)
        heading_err = abs(self._norm(heading_to_first - self.yaw))
        if heading_err < self.sim_startup_anchor_trigger:
            return poses

        anchor = (
            self.x + self.sim_startup_anchor_distance * math.cos(self.yaw),
            self.y + self.sim_startup_anchor_distance * math.sin(self.yaw),
            self.yaw,
        )
        return [anchor] + poses

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _norm(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _find_nearest_path_index(
        poses: List[WorldPose], x: float, y: float
    ) -> int:
        if not poses:
            return 0
        best_i = 0
        best_d2 = float("inf")
        for i, pose in enumerate(poses):
            dx = pose[0] - x
            dy = pose[1] - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i


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
