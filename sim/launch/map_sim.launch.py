#!/usr/bin/env python3

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
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


def _resolve_sim_backend():
    try:
        return {
            "backend": "ros_gz_sim",
            "share": get_package_share_directory("ros_gz_sim"),
        }
    except PackageNotFoundError:
        pass

    try:
        return {
            "backend": "gazebo_ros",
            "share": get_package_share_directory("gazebo_ros"),
        }
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "No simulator backend found. Install either 'ros-humble-ros-gz-sim' "
            "(Ignition/Gazebo Sim) or 'ros-humble-gazebo-ros-pkgs' (Gazebo Classic)."
        ) from exc


def generate_launch_description():
    pkg_share = get_package_share_directory("finav")
    sim_backend = _resolve_sim_backend()

    default_world = os.path.join(
        pkg_share, "sim", "gazebo", "worlds", "office_corridor_sketch.world"
    )
    default_rviz = os.path.join(pkg_share, "rviz", "mapping.rviz")
    urdf_file = os.path.join(pkg_share, "urdf", "whillcar.urdf")
    gz_model_file = os.path.join(
        pkg_share, "sim", "gazebo", "models", "whillcar", "model.sdf"
    )
    lidar_cfg = _load_lidar_config(os.path.join(pkg_share, "config", "lidar.yaml"))
    default_world_name = os.path.splitext(os.path.basename(default_world))[0]

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
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value=default_world_name,
        description="Gazebo world name used by ros_gz_bridge topic paths",
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
    world_name = LaunchConfiguration("world_name")
    use_rviz = LaunchConfiguration("use_rviz")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    scan_filter_center_deg = LaunchConfiguration("scan_filter_center_deg")
    scan_filter_fov_deg = LaunchConfiguration("scan_filter_fov_deg")
    gz_scan_topic = "/scan"
    ros_scan_bridge_topic = "/scan_gz"
    gz_clock_topic = ["/world/", world_name, "/clock"]
    gz_imu_topic = [
        "/world/",
        world_name,
        "/model/finav_vehicle/link/imu_link/sensor/imu_sensor/imu",
    ]
    gz_joint_state_topic = [
        "/world/",
        world_name,
        "/model/finav_vehicle/joint_state",
    ]
    gz_resource_path = os.path.join(pkg_share, "sim", "gazebo", "models")

    if sim_backend["backend"] == "ros_gz_sim":
        gz_resource_env = SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=[
                os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
                os.pathsep,
                gz_resource_path,
            ],
        )
        gz_sim_resource_env = SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                os.pathsep,
                gz_resource_path,
            ],
        )
        gazebo = ExecuteProcess(
            cmd=["ign", "gazebo", "-r", world],
            shell=False,
            output="screen",
        )
        spawn_entity = None
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            arguments=[
                [*gz_clock_topic, "@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                [gz_scan_topic, "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
                [*gz_imu_topic, "@sensor_msgs/msg/Imu[gz.msgs.IMU"],
                "/model/finav_vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/model/finav_vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                [*gz_joint_state_topic, "@sensor_msgs/msg/JointState[gz.msgs.Model"],
            ],
            remappings=[
                (gz_clock_topic, "/clock"),
                (gz_scan_topic, ros_scan_bridge_topic),
                ("/model/finav_vehicle/cmd_vel", "/cmd_vel"),
            ],
        )
        gz_topic_adapter = Node(
            package="finav",
            executable="gz_topic_adapter.py",
            name="gz_topic_adapter",
            output="screen",
            parameters=[
                {
                    "scan_src_topic": ros_scan_bridge_topic,
                    "imu_src_topic": gz_imu_topic,
                    "odom_src_topic": "/model/finav_vehicle/odometry",
                    "joint_state_src_topic": gz_joint_state_topic,
                    "scan_topic": "/scan",
                    "imu_topic": "/imu/data",
                    "odom_topic": "/odom",
                    "joint_states_topic": "/joint_states",
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                }
            ],
        )
        unpause_world = ExecuteProcess(
            cmd=[
                "ign",
                "service",
                "-s",
                ["/world/", world_name, "/control"],
                "--reqtype",
                "ignition.msgs.WorldControl",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "3000",
                "--req",
                "pause: false",
            ],
            shell=False,
            output="screen",
        )
        robot_description = ParameterValue(
            Command(["xacro ", urdf_file]), value_type=str
        )
    else:
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_backend["share"], "launch", "gazebo.launch.py")
            ),
            launch_arguments={"world": world}.items(),
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
        bridge = None
        gz_topic_adapter = None
        unpause_world = None
        robot_description = ParameterValue(
            Command(["xacro ", urdf_file]), value_type=str
        )

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
            world_name_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            scan_filter_center_deg_arg,
            scan_filter_fov_deg_arg,
            *([gz_resource_env, gz_sim_resource_env] if sim_backend["backend"] == "ros_gz_sim" else []),
            gazebo,
            robot_state_publisher,
            *([spawn_entity] if spawn_entity is not None else []),
            *([bridge] if bridge is not None else []),
            *([gz_topic_adapter] if gz_topic_adapter is not None else []),
            *([TimerAction(period=1.0, actions=[unpause_world])] if unpause_world is not None else []),
            TimerAction(period=1.0, actions=[scan_filter]),
            TimerAction(period=2.0, actions=[slam_toolbox_launch]),
            rviz,
        ]
    )
