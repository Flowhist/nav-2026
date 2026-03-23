#!/usr/bin/env python3
"""仿真目标点发送工具
示例：
  ros2 run finav sim_send_goal.py --x 2.0 --y -1.5 --yaw-deg 0
"""

import argparse
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GoalSender(Node):
    def __init__(self, x: float, y: float, yaw_deg: float, topic: str, frame: str) -> None:
        super().__init__("sim_send_goal")
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.x = x
        self.y = y
        self.yaw = math.radians(yaw_deg)
        self.frame = frame
        self.create_timer(0.2, self._publish_once)
        self.sent = False

    def _publish_once(self) -> None:
        if self.sent:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.orientation.w = math.cos(self.yaw * 0.5)
        self.pub.publish(msg)
        self.get_logger().info(
            f"goal sent to {self.pub.topic_name}: ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.1f}deg)"
        )
        self.sent = True
        self.create_timer(0.3, self._shutdown)

    def _shutdown(self) -> None:
        rclpy.shutdown()


def main(args: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--topic", type=str, default="/goal_pose")
    parser.add_argument("--frame", type=str, default="map")
    ns = parser.parse_args()

    rclpy.init(args=args)
    node = GoalSender(ns.x, ns.y, ns.yaw_deg, ns.topic, ns.frame)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
