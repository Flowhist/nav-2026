#!/usr/bin/env python3
"""
EKF融合节点启动文件
功能：融合编码器里程计(/odom_encoder) + IMU(/imu/data) -> 输出融合里程计(/odom)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


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

    # EKF节点
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(config_dir, "ekf.yaml")],
        remappings=[
            # 输入：订阅原始编码器里程计和IMU
            # 输出：融合后的里程计发布到/odom，供SLAM Toolbox使用
            ("/odometry/filtered", "/odom"),
        ],
    )

    return LaunchDescription([ekf_node])
