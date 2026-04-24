#!/usr/bin/env python3
"""
slam_toolbox.launch.py - SLAM Toolbox启动文件（实机版本）
支持两种模式：
1. mapping - 在线建图模式
2. localization - 基于已有地图的纯定位模式
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _resolve_default_maps_dir(pkg_share: str) -> str:
    env_maps = os.environ.get("FINAV_MAPS_DIR", "").strip()
    if env_maps:
        return env_maps

    repo_dir = os.environ.get("FINAV_REPO_DIR", "").strip()
    if repo_dir:
        return os.path.join(repo_dir, "maps")

    candidate = os.path.abspath(
        os.path.join(pkg_share, "..", "..", "..", "..", "src", "finav", "maps")
    )
    if os.path.isdir(candidate):
        return candidate
    return os.path.join(pkg_share, "maps")


def generate_launch_description():
    pkg_share = get_package_share_directory("finav")

    # 地图目录（默认指向当前用户工作区）
    default_maps_dir = _resolve_default_maps_dir(pkg_share)

    # 声明launch参数：运行模式
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="mapping",
        description="运行模式: mapping (建图) 或 localization (纯定位)",
    )

    # 声明地图文件名参数（不带扩展名，用于定位模式）
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="map_test",
        description="地图文件名（不带扩展名），用于纯定位模式",
    )

    # 地图目录参数（可覆盖）
    maps_dir_arg = DeclareLaunchArgument(
        "maps_dir",
        default_value=default_maps_dir,
        description="地图根目录，默认 ~/nav_workspace/src/finav/maps",
    )

    # 声明 use_sim_time 参数
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    # 激光话题参数（实机链路直接使用 /scan）
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="输入给SLAM的激光话题",
    )

    mode = LaunchConfiguration("mode")
    map_file = LaunchConfiguration("map_file")
    maps_dir = LaunchConfiguration("maps_dir")
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")

    # SLAM Toolbox 在线建图节点
    # 使用 sync_slam_toolbox_node（同步模式）
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
        parameters=[
            os.path.join(pkg_share, "config", "slam_toolbox_map.yaml"),
            {
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("scan", scan_topic),
            ("odom", "/odom"),
            ("imu", "/imu"),
        ],
    )

    # SLAM Toolbox 纯定位节点
    # 注意:定位模式下,地图文件路径应该在slam_toolbox_nav.yaml中
    # 通过map_file_name和map_start_at_dock参数指定,节点启动时会自动加载地图
    # 地图路径格式: maps/{map_name}/{map_name}
    localization_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
        parameters=[
            os.path.join(pkg_share, "config", "slam_toolbox_nav.yaml"),
            {
                "use_sim_time": use_sim_time,
                # 地图路径格式: <maps_dir>/<map_name>/<map_name>
                "map_file_name": [
                    maps_dir,
                    "/",
                    map_file,
                    "/",
                    map_file,
                ],
                "map_start_at_dock": True,
            },
        ],
        remappings=[
            ("scan", scan_topic),
            ("odom", "/odom"),  # 实机里程计话题
            ("imu", "/imu"),  # 实机IMU话题
        ],
    )

    # 模式提示
    mapping_info = LogInfo(
        msg="启动 SLAM Toolbox 建图模式（实机）...",
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
    )

    localization_info = LogInfo(
        msg=[
            "启动 SLAM Toolbox 纯定位模式（实机），使用地图: ",
            map_file,
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
    )

    return LaunchDescription(
        [
            mode_arg,
            map_file_arg,
            maps_dir_arg,
            use_sim_time_arg,
            scan_topic_arg,
            mapping_info,
            localization_info,
            # 延迟3秒启动SLAM Toolbox，等待TF树和传感器数据稳定
            TimerAction(period=1.0, actions=[slam_toolbox_node]),
            TimerAction(period=1.0, actions=[localization_slam_toolbox_node]),
        ]
    )
