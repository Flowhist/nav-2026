#!/usr/bin/env python3
"""
DM-IMU启动文件 - TCP远程连接模式
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("finav")
    params_file = os.path.join(pkg_share, "config", "imu.yaml")

    return LaunchDescription(
        [
            Node(
                package="finav",
                executable="dm_imu_publisher.py",
                name="dm_imu",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
