#!/usr/bin/env python3
"""
DM-IMU启动文件 - TCP远程连接模式
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _resolve_config_dir(pkg_share: str) -> str:
    repo_dir = os.environ.get("FINAV_REPO_DIR", "").strip()
    if repo_dir and os.path.isdir(repo_dir):
        return os.path.join(repo_dir, "config")
    candidate = os.path.abspath(
        os.path.join(pkg_share, "..", "..", "..", "..", "src", "finav", "config")
    )
    if os.path.isdir(candidate):
        return candidate
    return os.path.join(pkg_share, "config")


def generate_launch_description():
    pkg_share = get_package_share_directory("finav")
    config_dir = _resolve_config_dir(pkg_share)
    params_file = os.path.join(config_dir, "imu.yaml")

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
