#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("finav")
    default_map_yaml = os.path.join(pkg_share, "maps", "map4", "map4.yaml")
    default_rviz = os.path.join(pkg_share, "sim", "rviz", "path_plan_sim.rviz")
    path_plan_cfg = os.path.join(pkg_share, "sim", "config", "path_plan_sim.yaml")
    fake_pose_cfg = os.path.join(pkg_share, "sim", "config", "sim_fake_pose.yaml")

    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value=default_map_yaml,
        description="地图 yaml 路径",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动 RViz",
    )
    use_cmd_vel_arg = DeclareLaunchArgument(
        "use_cmd_vel",
        default_value="false",
        description="虚拟位姿是否跟随 /cmd_vel 积分",
    )

    map_yaml = LaunchConfiguration("map_yaml")
    use_rviz = LaunchConfiguration("use_rviz")
    use_cmd_vel = LaunchConfiguration("use_cmd_vel")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="sim_map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml},
            {"use_sim_time": False},
        ],
    )

    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"autostart": True},
            {"use_sim_time": False},
            {"node_names": ["sim_map_server"]},
        ],
    )

    fake_pose = Node(
        package="finav",
        executable="sim_fake_pose_tf.py",
        name="sim_fake_pose_tf",
        output="screen",
        parameters=[
            fake_pose_cfg,
            {"use_cmd_vel": use_cmd_vel},
        ],
    )

    planner = Node(
        package="finav",
        executable="path_plan.py",
        name="path_plan",
        output="screen",
        parameters=[path_plan_cfg],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            map_yaml_arg,
            use_rviz_arg,
            use_cmd_vel_arg,
            map_server,
            map_lifecycle,
            fake_pose,
            planner,
            rviz,
        ]
    )
