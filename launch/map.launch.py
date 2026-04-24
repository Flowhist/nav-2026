#!/usr/bin/env python3

"""
建图模式启动文件
功能：启动雷达、SLAM Toolbox 建图、IMU 与 EKF
前提：底盘驱动已由项目根目录 start_finav.sh 独立启动，本文件不再启动底盘节点。
说明：建图阶段不再自动启动 RViz，地图通过 Web 实时显示。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    }

    try:
        import yaml  # type: ignore

        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        if isinstance(data, dict):
            defaults["lidar_ip"] = str(data.get("lidar_ip", defaults["lidar_ip"]))
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

    # 1. 雷达驱动
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "lidar.launch.py")
        ),
        launch_arguments={"scanner_ip": lidar_ip}.items(),
    )

    # 2. SLAM Toolbox 建图模式
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "slam_toolbox.launch.py")
        ),
        launch_arguments={
            "mode": "mapping",
            "use_sim_time": "false",
            "scan_topic": "/scan",
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
            lidar_launch,
            dm_imu_launch,
            ekf_launch,
            slam_toolbox_launch,
            robot_model_launch,
        ]
    )
