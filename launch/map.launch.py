#!/usr/bin/env python3

"""
建图模式启动文件
功能：启动雷达、激光过滤、SLAM Toolbox 建图、IMU 与 EKF
前提：底盘驱动已由项目根目录 start_finav.sh 独立启动，本文件不再启动底盘节点。
说明：建图阶段不再自动启动 RViz，地图通过 Web 实时显示。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_repo_dir(pkg_share: str) -> str:
    env_repo = os.environ.get("FINAV_REPO_DIR", "").strip()
    if env_repo and os.path.isdir(env_repo):
        return env_repo

    candidate = os.path.abspath(os.path.join(pkg_share, "..", "..", "..", "..", "src", "finav"))
    if os.path.isdir(candidate):
        return candidate
    return pkg_share


def _load_lidar_config(config_path: str):
    defaults = {
        "lidar_ip": "10.86.81.200",
        "scan_filter_center_deg": 0.0,
        "scan_filter_fov_deg": 180.0,
    }

    try:
        import yaml  # type: ignore

        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        if isinstance(data, dict):
            defaults["lidar_ip"] = str(data.get("lidar_ip", defaults["lidar_ip"]))
            defaults["scan_filter_center_deg"] = float(
                data.get("scan_filter_center_deg", defaults["scan_filter_center_deg"])
            )
            defaults["scan_filter_fov_deg"] = float(
                data.get("scan_filter_fov_deg", defaults["scan_filter_fov_deg"])
            )
    except Exception:
        pass

    return defaults


def generate_launch_description():
    pkg_share = get_package_share_directory("finav")
    repo_dir = _resolve_repo_dir(pkg_share)
    fastdds_path = os.path.join(repo_dir, "config", "fastdds_profiles.xml")
    if os.path.exists(fastdds_path):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = fastdds_path
    lidar_cfg = _load_lidar_config(os.path.join(pkg_share, "config", "lidar.yaml"))

    # 雷达IP参数（默认从 config/lidar.yaml 读取）
    lidar_ip_arg = DeclareLaunchArgument(
        "lidar_ip",
        default_value=str(lidar_cfg["lidar_ip"]),
        description="雷达IP地址（有线直连）",
    )
    lidar_ip = LaunchConfiguration("lidar_ip")

    # 激光角度过滤参数（默认从 config/lidar.yaml 读取）
    scan_filter_center_deg_arg = DeclareLaunchArgument(
        "scan_filter_center_deg",
        default_value=str(lidar_cfg["scan_filter_center_deg"]),
        description="激光保留扇区中心角(度)",
    )
    scan_filter_fov_deg_arg = DeclareLaunchArgument(
        "scan_filter_fov_deg",
        default_value=str(lidar_cfg["scan_filter_fov_deg"]),
        description="激光保留扇区角宽(度)",
    )
    scan_filter_center_deg = LaunchConfiguration("scan_filter_center_deg")
    scan_filter_fov_deg = LaunchConfiguration("scan_filter_fov_deg")

    # 向雷达发送释放命令（清理之前连接）
    release_handle = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            ["curl -s http://", lidar_ip, "/cmd/release_handle?handle=all"],
        ],
        name="release_lidar_handle",
        output="log",
    )

    # 1. 雷达驱动
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "r2.launch.py")
        ),
        launch_arguments={"scanner_ip": lidar_ip}.items(),
    )

    # 2. 激光角度过滤（/scan -> /scan_filtered）
    scan_filter_node = Node(
        package="finav",
        executable="scan_angle_filter.py",
        name="scan_angle_filter",
        output="screen",
        parameters=[
            {
                "input_topic": "/scan",
                "output_topic": "/scan_filtered",
                "center_angle_deg": scan_filter_center_deg,
                "fov_deg": scan_filter_fov_deg,
            }
        ],
    )

    # 3. SLAM Toolbox 建图模式
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "slam_toolbox.launch.py")
        ),
        launch_arguments={
            "mode": "mapping",
            "use_sim_time": "false",
            "scan_topic": "/scan_filtered",
        }.items(),
    )

    # 4. 机器人模型发布
    robot_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "robot_model.launch.py")
        )
    )

    # 5. DM-IMU
    dm_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "dm_imu.launch.py")
        )
    )

    # 6. EKF 融合
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "ekf.launch.py")
        )
    )

    return LaunchDescription(
        [
            lidar_ip_arg,
            scan_filter_center_deg_arg,
            scan_filter_fov_deg_arg,
            release_handle,
            lidar_launch,
            scan_filter_node,
            dm_imu_launch,
            ekf_launch,
            slam_toolbox_launch,
            robot_model_launch,
        ]
    )
