#!/usr/bin/env python3
"""仿真路径可视化
订阅 /plan，发布路径中心线和矩形车体足迹 MarkerArray。
"""

import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


WorldPose = Tuple[float, float, float]


class SimPlanVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("sim_plan_visualizer")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("footprint_topic", "/plan_footprints")
        self.declare_parameter("footprint_stride_m", 0.35)
        self.declare_parameter("footprint_max_markers", 60)
        self.declare_parameter("vehicle_front_m", 0.84)
        self.declare_parameter("vehicle_rear_m", 0.25)
        self.declare_parameter("vehicle_left_m", 0.45)
        self.declare_parameter("vehicle_right_m", 0.45)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.footprint_topic = str(self.get_parameter("footprint_topic").value)
        self.footprint_stride_m = float(
            self.get_parameter("footprint_stride_m").value
        )
        self.footprint_max_markers = int(
            self.get_parameter("footprint_max_markers").value
        )
        self.vehicle_front_m = float(self.get_parameter("vehicle_front_m").value)
        self.vehicle_rear_m = float(self.get_parameter("vehicle_rear_m").value)
        self.vehicle_left_m = float(self.get_parameter("vehicle_left_m").value)
        self.vehicle_right_m = float(self.get_parameter("vehicle_right_m").value)

        self.marker_pub = self.create_publisher(
            MarkerArray, self.footprint_topic, 10
        )
        self.create_subscription(Path, self.path_topic, self._on_path, 10)

    def _on_path(self, msg: Path) -> None:
        poses: List[WorldPose] = []
        for pose_stamped in msg.poses:
            q = pose_stamped.pose.orientation
            poses.append(
                (
                    float(pose_stamped.pose.position.x),
                    float(pose_stamped.pose.position.y),
                    self._quat_to_yaw(q.x, q.y, q.z, q.w),
                )
            )
        self._publish_markers(poses)

    def _publish_markers(self, poses: List[WorldPose]) -> None:
        msg = MarkerArray()

        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        msg.markers.append(clear)

        if not poses:
            self.marker_pub.publish(msg)
            return

        sampled = self._sample_poses(poses)
        stamp = self.get_clock().now().to_msg()

        centerline = Marker()
        centerline.header.frame_id = self.map_frame
        centerline.header.stamp = stamp
        centerline.ns = "plan_centerline"
        centerline.id = 0
        centerline.type = Marker.LINE_STRIP
        centerline.action = Marker.ADD
        centerline.scale.x = 0.04
        centerline.color.r = 0.10
        centerline.color.g = 0.85
        centerline.color.b = 0.20
        centerline.color.a = 0.95
        centerline.points = [self._world_point((p[0], p[1])) for p in poses]
        msg.markers.append(centerline)

        for i, pose in enumerate(sampled):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = "plan_footprints"
            marker.id = i + 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.018
            marker.color.r = 1.0
            marker.color.g = 0.55
            marker.color.b = 0.05
            marker.color.a = 0.75 if i < len(sampled) - 1 else 1.0
            marker.points = self._footprint_outline(pose)
            msg.markers.append(marker)

        self.marker_pub.publish(msg)

    def _sample_poses(self, poses: List[WorldPose]) -> List[WorldPose]:
        if not poses:
            return []
        max_markers = max(1, self.footprint_max_markers)
        stride = max(self.footprint_stride_m, 0.05)
        sampled = [poses[0]]
        accum = 0.0
        for i in range(1, len(poses)):
            seg = math.hypot(poses[i][0] - poses[i - 1][0], poses[i][1] - poses[i - 1][1])
            accum += seg
            if accum >= stride:
                sampled.append(poses[i])
                accum = 0.0
        if sampled[-1] != poses[-1]:
            sampled.append(poses[-1])
        if len(sampled) <= max_markers:
            return sampled
        step = max(1, int(math.ceil(len(sampled) / max_markers)))
        reduced = sampled[::step]
        if reduced[-1] != sampled[-1]:
            reduced.append(sampled[-1])
        return reduced[:max_markers]

    def _footprint_outline(self, pose: WorldPose) -> List[Point]:
        x, y, yaw = pose
        corners = [
            (self.vehicle_front_m, self.vehicle_left_m),
            (self.vehicle_front_m, -self.vehicle_right_m),
            (-self.vehicle_rear_m, -self.vehicle_right_m),
            (-self.vehicle_rear_m, self.vehicle_left_m),
            (self.vehicle_front_m, self.vehicle_left_m),
        ]
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        points: List[Point] = []
        for lx, ly in corners:
            wx = x + lx * cy - ly * sy
            wy = y + lx * sy + ly * cy
            points.append(self._world_point((wx, wy)))
        return points

    @staticmethod
    def _world_point(p: Tuple[float, float]) -> Point:
        msg = Point()
        msg.x = float(p[0])
        msg.y = float(p[1])
        msg.z = 0.03
        return msg

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimPlanVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
