#!/usr/bin/env python3
"""
激光角度过滤节点
用途：只保留指定扇区内的激光数据（默认前向180度），其余角度置为 inf。

默认：
- 输入话题: /scan
- 输出话题: /scan_filtered
- 扇区中心: 0 deg
- 扇区角宽: 180 deg
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanAngleFilter(Node):
    def __init__(self):
        super().__init__('scan_angle_filter')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('center_angle_deg', 0.0)
        self.declare_parameter('fov_deg', 180.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.center_angle = math.radians(float(self.get_parameter('center_angle_deg').value))
        self.half_fov = math.radians(float(self.get_parameter('fov_deg').value)) / 2.0

        if self.half_fov <= 0.0 or self.half_fov > math.pi:
            self.get_logger().warn('fov_deg 参数异常，已回退到 180 度')
            self.half_fov = math.pi / 2.0

        self.sub = self.create_subscription(LaserScan, input_topic, self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, output_topic, 10)

        self.get_logger().info(
            f'激光角度过滤已启动: {input_topic} -> {output_topic}, '
            f'center={math.degrees(self.center_angle):.1f}deg, '
            f'fov={math.degrees(self.half_fov * 2.0):.1f}deg'
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def in_sector(self, angle: float) -> bool:
        delta = self.normalize_angle(angle - self.center_angle)
        return abs(delta) <= self.half_fov

    def scan_callback(self, msg: LaserScan):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        ranges_out = []
        intensities_out = []

        has_intensity = len(msg.intensities) == len(msg.ranges)

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            keep = self.in_sector(angle)

            if keep:
                ranges_out.append(r)
                if has_intensity:
                    intensities_out.append(msg.intensities[i])
            else:
                ranges_out.append(float('inf'))
                if has_intensity:
                    intensities_out.append(0.0)

        filtered.ranges = ranges_out
        if has_intensity:
            filtered.intensities = intensities_out

        self.pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanAngleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
