#!/usr/bin/env python3

"""
RViz可视化启动文件
功能：根据模式启动对应的RViz配置
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("finav")

    # 声明RViz配置参数
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="mapping",
        description="RViz配置: 'mapping'（建图模式）或 'navigation'（导航模式）",
    )

    # 动态选择RViz配置文件
    def launch_setup(context, *args, **kwargs):
        config_type = context.launch_configurations.get("rviz_config", "mapping")

        if config_type == "navigation":
            rviz_config_path = os.path.join(pkg_share, "rviz/navigation.rviz")
        else:
            rviz_config_path = os.path.join(pkg_share, "rviz/mapping.rviz")

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
        )

        return [rviz_node]

    return LaunchDescription(
        [
            rviz_config_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
