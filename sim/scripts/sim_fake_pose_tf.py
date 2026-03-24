#!/usr/bin/env python3
"""虚拟位姿 TF 发布器
发布 map->base_link，支持三种方式更新位姿：
1) 订阅 /initialpose 手动设置
2) 订阅 /cmd_vel 进行积分
3) 订阅 /plan 自动做路径跟随演示
"""

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


WorldPose = Tuple[float, float, float]


class SimFakePoseTF(Node):
    def __init__(self) -> None:
        super().__init__("sim_fake_pose_tf")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("use_cmd_vel", False)
        self.declare_parameter("follow_plan", True)
        self.declare_parameter("demo_linear_speed_mps", 0.35)
        self.declare_parameter("demo_angular_speed_rps", 0.65)
        self.declare_parameter("demo_waypoint_tolerance_m", 0.10)
        self.declare_parameter("demo_goal_hold_s", 2.0)
        self.declare_parameter("demo_heading_slowdown_deg", 40.0)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw_deg", 0.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.use_cmd_vel = bool(self.get_parameter("use_cmd_vel").value)
        self.follow_plan = bool(self.get_parameter("follow_plan").value)
        self.demo_linear_speed_mps = float(
            self.get_parameter("demo_linear_speed_mps").value
        )
        self.demo_angular_speed_rps = float(
            self.get_parameter("demo_angular_speed_rps").value
        )
        self.demo_waypoint_tolerance_m = float(
            self.get_parameter("demo_waypoint_tolerance_m").value
        )
        self.demo_goal_hold_s = float(self.get_parameter("demo_goal_hold_s").value)
        self.demo_heading_slowdown = math.radians(
            float(self.get_parameter("demo_heading_slowdown_deg").value)
        )

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = math.radians(float(self.get_parameter("yaw_deg").value))
        self.v = 0.0
        self.w = 0.0

        self.path_poses: List[WorldPose] = []
        self.path_active = False
        self.path_index = 0
        self.hold_until_ns = 0
        self.start_pose_from_path: Optional[WorldPose] = None

        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(
            PoseWithCovarianceStamped, self.initialpose_topic, self._on_initialpose, 10
        )
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(Path, self.plan_topic, self._on_plan, 10)

        self.last_t = self.get_clock().now()
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self._loop)

        self.get_logger().info(
            "sim_fake_pose_tf started | pose=(%.2f, %.2f, %.1fdeg) | use_cmd_vel=%s | follow_plan=%s"
            % (
                self.x,
                self.y,
                math.degrees(self.yaw),
                str(self.use_cmd_vel),
                str(self.follow_plan),
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
        self.hold_until_ns = 0
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
            return

        self.path_poses = poses
        self.start_pose_from_path = poses[0]
        self._reset_to_start_pose()
        self.path_index = 1
        self.path_active = self.follow_plan
        self.hold_until_ns = 0
        self.get_logger().info(
            "demo path received: %d poses, restart from first pose" % len(poses)
        )

    def _reset_to_start_pose(self) -> None:
        if not self.start_pose_from_path:
            return
        self.x = self.start_pose_from_path[0]
        self.y = self.start_pose_from_path[1]
        self.yaw = self.start_pose_from_path[2]

    def _loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0:
            dt = 1e-3

        if self.follow_plan and self.path_active and self.path_poses:
            self._step_follow_plan(now.nanoseconds, dt)
        elif self.use_cmd_vel:
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

    def _step_follow_plan(self, now_ns: int, dt: float) -> None:
        if self.hold_until_ns > 0:
            if now_ns >= self.hold_until_ns:
                self._reset_to_start_pose()
                self.path_index = 1 if len(self.path_poses) > 1 else 0
                self.hold_until_ns = 0
            return

        if self.path_index >= len(self.path_poses):
            self.hold_until_ns = now_ns + int(self.demo_goal_hold_s * 1e9)
            return

        tx, ty, tyaw = self.path_poses[self.path_index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist <= self.demo_waypoint_tolerance_m:
            if self.path_index >= len(self.path_poses) - 1:
                self.yaw = tyaw
                self.hold_until_ns = now_ns + int(self.demo_goal_hold_s * 1e9)
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

        self.yaw = self._norm(self.yaw + w_cmd * dt)
        self.x += v_cmd * math.cos(self.yaw) * dt
        self.y += v_cmd * math.sin(self.yaw) * dt

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

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
