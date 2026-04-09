#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_lidar_config(config_path: str):
    defaults = {
        "scan_filter_center_deg": 0.0,
        "scan_filter_fov_deg": 180.0,
    }
    try:
        import yaml  # type: ignore

        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if isinstance(data, dict):
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
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    default_world = os.path.join(
        pkg_share, "sim", "gazebo", "worlds", "office_corridor_sketch.world"
    )
    default_rviz = os.path.join(pkg_share, "rviz", "mapping.rviz")
    urdf_file = os.path.join(pkg_share, "urdf", "whillcar.urdf")
    lidar_cfg = _load_lidar_config(os.path.join(pkg_share, "config", "lidar.yaml"))

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz",
    )
    x_arg = DeclareLaunchArgument("x", default_value="-0.15")
    y_arg = DeclareLaunchArgument("y", default_value="-5.65")
    z_arg = DeclareLaunchArgument("z", default_value="0.16")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="1.5707963")
    scan_filter_center_deg_arg = DeclareLaunchArgument(
        "scan_filter_center_deg",
        default_value=str(lidar_cfg["scan_filter_center_deg"]),
        description="Laser kept sector center angle in degree",
    )
    scan_filter_fov_deg_arg = DeclareLaunchArgument(
        "scan_filter_fov_deg",
        default_value=str(lidar_cfg["scan_filter_fov_deg"]),
        description="Laser kept sector FOV in degree",
    )

    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    scan_filter_center_deg = LaunchConfiguration("scan_filter_center_deg")
    scan_filter_fov_deg = LaunchConfiguration("scan_filter_fov_deg")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    robot_description = ParameterValue(Command(["xacro ", urdf_file]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_finav",
        output="screen",
        arguments=[
            "-entity",
            "finav_vehicle",
            "-topic",
            "robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-Y",
            yaw,
        ],
    )

    scan_filter = Node(
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

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "slam_toolbox.launch.py")
        ),
        launch_arguments={
            "mode": "mapping",
            "use_sim_time": "true",
            "scan_topic": "/scan_filtered",
        }.items(),
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
            world_arg,
            use_rviz_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            scan_filter_center_deg_arg,
            scan_filter_fov_deg_arg,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            TimerAction(period=1.0, actions=[scan_filter]),
            TimerAction(period=2.0, actions=[slam_toolbox_launch]),
            rviz,
        ]
    )
