#!/usr/bin/env python3

"""
机器人模型发布节点
功能：发布URDF模型和静态TF变换
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("finav")
    urdf_file = os.path.join(pkg_share, "urdf", "whillcar.urdf")

    # 使用xacro处理URDF文件（处理xacro:property等宏）
    robot_description = ParameterValue(Command(["xacro ", urdf_file]), value_type=str)

    # Robot State Publisher节点（发布TF和机器人状态）
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,
            }
        ],
    )

    # Joint State Publisher节点（发布关节状态，用于轮子可视化）
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
            }
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
        ]
    )
