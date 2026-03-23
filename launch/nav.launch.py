#!/usr/bin/env python3

"""
导航模式启动文件
功能：启动雷达、激光过滤、SLAM Toolbox 定位、EKF 融合、自定义路径规划、三状态控制器
网络拓扑：
    雷达 (有线直连) -> 上位机
    IMU  (有线直连) -> 上位机
"""

import os
import sys
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

    candidate = os.path.abspath(
        os.path.join(pkg_share, "..", "..", "..", "..", "src", "finav")
    )
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


def _discover_maps(maps_dir: str):
    if not os.path.isdir(maps_dir):
        return []

    names = []
    for name in sorted(os.listdir(maps_dir)):
        d = os.path.join(maps_dir, name)
        if not os.path.isdir(d):
            continue

        # 优先识别 <map>/<map>.yaml
        yml = os.path.join(d, f"{name}.yaml")
        if os.path.exists(yml):
            names.append(name)
            continue

        # 兜底：目录下存在任意 yaml 也视为可用地图
        has_yaml = any(fn.endswith(".yaml") for fn in os.listdir(d))
        if has_yaml:
            names.append(name)

    return names


def _select_map_from_terminal(map_names):
    # 非交互终端，默认第一个
    if not sys.stdin.isatty():
        return map_names[0]

    print()
    print("================ 可用地图列表 ================")
    for i, name in enumerate(map_names, start=1):
        print(f"  {i}. {name}")
    print("=============================================")
    print(f"默认地图: 1 ({map_names[0]})")

    while True:
        choice = input("请输入地图编号后回车（直接回车使用默认）: ").strip()
        if choice == "":
            return map_names[0]
        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(map_names):
                return map_names[idx - 1]
        print("输入无效，请输入上面列表中的数字编号。")


def generate_launch_description():
    pkg_share = get_package_share_directory("finav")
    repo_dir = _resolve_repo_dir(pkg_share)
    maps_dir = os.environ.get("FINAV_MAPS_DIR", "").strip() or os.path.join(
        repo_dir, "maps"
    )
    fastdds_path = os.path.join(repo_dir, "config", "fastdds_profiles.xml")
    if os.path.exists(fastdds_path):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = fastdds_path
    lidar_cfg = _load_lidar_config(os.path.join(pkg_share, "config", "lidar.yaml"))

    # 启动时地图选择（可被 launch 参数 map_file 覆盖）
    available_maps = _discover_maps(maps_dir)
    if available_maps:
        selected_map_default = _select_map_from_terminal(available_maps)
        print(f"[nav.launch] 已选择地图: {selected_map_default}")
    else:
        selected_map_default = "map_test"
        print(f"[nav.launch] 未发现可用地图，回退默认: {selected_map_default}")
        print(f"[nav.launch] 请检查地图目录: {maps_dir}")

    # 声明launch参数
    lidar_ip_arg = DeclareLaunchArgument(
        "lidar_ip",
        default_value=str(lidar_cfg["lidar_ip"]),
        description="雷达IP地址（有线直连）",
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=selected_map_default,
        description="地图文件名（不带扩展名）",
    )

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

    # 获取参数值
    lidar_ip = LaunchConfiguration("lidar_ip")
    map_file = LaunchConfiguration("map_file")
    scan_filter_center_deg = LaunchConfiguration("scan_filter_center_deg")
    scan_filter_fov_deg = LaunchConfiguration("scan_filter_fov_deg")

    # 向雷达发送释放命令
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

    # 3. SLAM Toolbox纯定位模式
    slam_toolbox_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "slam_toolbox.launch.py")
        ),
        launch_arguments={
            "mode": "localization",
            "map_file": map_file,
            "use_sim_time": "false",
            "scan_topic": "/scan_filtered",
        }.items(),
    )

    # 4. DM-IMU (外置IMU传感器)
    dm_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "dm_imu.launch.py")
        ),
    )

    # 5. EKF融合 (融合编码器里程计和IMU)
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "ekf.launch.py")
        ),
    )

    # 6. 三状态导航控制器：订阅 /plan 路径，按航向误差切换 STRAIGHT/ARC/ROTATE 状态并发布 /cmd_vel
    cmd_vel_relay_node = Node(
        package="finav",
        executable="nav_control.py",
        name="nav_control",
        output="screen",
        parameters=[os.path.join(pkg_share, "config", "nav.yaml")],
    )

    # 7. 轻量路径规划：/map + TF + /goal_pose -> /plan
    path_plan_node = Node(
        package="finav",
        executable="path_plan.py",
        name="path_plan",
        output="screen",
        parameters=[os.path.join(pkg_share, "config", "path_plan.yaml")],
    )

    # 8. 机器人模型发布
    robot_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "robot_model.launch.py")
        ),
    )

    return LaunchDescription(
        [
            lidar_ip_arg,
            map_file_arg,
            scan_filter_center_deg_arg,
            scan_filter_fov_deg_arg,
            release_handle,
            lidar_launch,
            scan_filter_node,
            dm_imu_launch,
            ekf_launch,
            slam_toolbox_nav_launch,
            path_plan_node,
            cmd_vel_relay_node,
            robot_model_launch,
        ]
    )
