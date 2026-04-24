#!/usr/bin/env python3

import os
import sys

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


def _discover_maps(maps_dir: str):
    if not os.path.isdir(maps_dir):
        return []

    names = []
    for name in sorted(os.listdir(maps_dir)):
        d = os.path.join(maps_dir, name)
        if not os.path.isdir(d):
            continue

        yml = os.path.join(d, f"{name}.yaml")
        if os.path.exists(yml):
            names.append(name)
            continue

        has_yaml = any(fn.endswith(".yaml") for fn in os.listdir(d))
        if has_yaml:
            names.append(name)

    return names


def _select_map_from_terminal(map_names):
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
    default_rviz = os.path.join(pkg_share, "rviz", "navigation.rviz")
    urdf_file = os.path.join(pkg_share, "urdf", "whillcar.urdf")
    gz_model_file = os.path.join(
        pkg_share, "sim", "gazebo", "models", "whillcar", "model.sdf"
    )
    nav_cfg = os.path.join(pkg_share, "config", "nav.yaml")
    path_plan_cfg = os.path.join(pkg_share, "config", "path_plan.yaml")
    plan_visualizer_cfg = os.path.join(
        pkg_share, "sim", "config", "sim_plan_visualizer.yaml"
    )
    default_maps_dir = _resolve_default_maps_dir(pkg_share)
    default_world_name = os.path.splitext(os.path.basename(default_world))[0]
    available_maps = _discover_maps(default_maps_dir)

    if available_maps:
        selected_map_default = _select_map_from_terminal(available_maps)
        print(f"[nav_sim.launch] 已选择地图: {selected_map_default}")
    else:
        selected_map_default = "map4"
        print(f"[nav_sim.launch] 未发现可用地图，回退默认: {selected_map_default}")
        print(f"[nav_sim.launch] 请检查地图目录: {default_maps_dir}")

    world_arg = DeclareLaunchArgument("world", default_value=default_world)
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value=default_world_name,
        description="Gazebo world name used by ros_gz_bridge topic paths",
    )
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=selected_map_default,
    )
    maps_dir_arg = DeclareLaunchArgument("maps_dir", default_value=default_maps_dir)
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    x_arg = DeclareLaunchArgument("x", default_value="-0.15")
    y_arg = DeclareLaunchArgument("y", default_value="-5.65")
    z_arg = DeclareLaunchArgument("z", default_value="0.0")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="1.5707963")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    map_file = LaunchConfiguration("map_file")
    maps_dir = LaunchConfiguration("maps_dir")
    use_rviz = LaunchConfiguration("use_rviz")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    gz_scan_topic = "/scan"
    ros_scan_bridge_topic = "/scan_gz"
    gz_clock_topic = ["/world/", world_name, "/clock"]
    gz_set_pose_service = ["/world/", world_name, "/set_pose"]
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
                [*gz_set_pose_service, "@ros_gz_interfaces/srv/SetEntityPose"],
            ],
            remappings=[
                (gz_clock_topic, "/clock"),
                (gz_scan_topic, ros_scan_bridge_topic),
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
                    "imu_src_topic": "/imu_unused",
                    "odom_src_topic": "/odom_unused",
                    "joint_state_src_topic": "/joint_states_unused",
                    "scan_topic": "/scan",
                    "imu_topic": "/imu/data",
                    "odom_topic": "/odom",
                    "joint_states_topic": "/joint_states",
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                }
            ],
        )
        sim_drive = Node(
            package="finav",
            executable="sim_gazebo_drive.py",
            name="sim_gazebo_drive",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "world_name": world_name,
                    "model_name": "finav_vehicle",
                    "cmd_vel_topic": "/cmd_vel",
                    "odom_topic": "/odom",
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "publish_rate_hz": 30.0,
                    "cmd_timeout": 0.35,
                    "pose_z": z,
                    "x": x,
                    "y": y,
                    "yaw": yaw,
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
        sim_drive = None
        unpause_world = None
        robot_description = ParameterValue(
            Command(["xacro ", urdf_file]), value_type=str
        )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    slam_toolbox_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sub", "slam_toolbox.launch.py")
        ),
        launch_arguments={
            "mode": "localization",
            "map_file": map_file,
            "maps_dir": maps_dir,
            "use_sim_time": "true",
            "scan_topic": "/scan",
        }.items(),
    )

    path_plan_node = Node(
        package="finav",
        executable="path_plan.py",
        name="path_plan",
        output="screen",
        parameters=[path_plan_cfg],
    )

    nav_control_node = Node(
        package="finav",
        executable="nav_control.py",
        name="nav_control",
        output="screen",
        parameters=[nav_cfg],
    )

    plan_visualizer = Node(
        package="finav",
        executable="sim_plan_visualizer.py",
        name="sim_plan_visualizer",
        output="screen",
        parameters=[path_plan_cfg, plan_visualizer_cfg],
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
            world_name_arg,
            map_file_arg,
            maps_dir_arg,
            use_rviz_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            *([gz_resource_env, gz_sim_resource_env] if sim_backend["backend"] == "ros_gz_sim" else []),
            gazebo,
            robot_state_publisher,
            *([spawn_entity] if spawn_entity is not None else []),
            *([bridge] if bridge is not None else []),
            *([gz_topic_adapter] if gz_topic_adapter is not None else []),
            *([sim_drive] if sim_drive is not None else []),
            *([TimerAction(period=1.0, actions=[unpause_world])] if unpause_world is not None else []),
            TimerAction(period=2.0, actions=[slam_toolbox_nav_launch]),
            TimerAction(
                period=2.5, actions=[path_plan_node, nav_control_node, plan_visualizer]
            ),
            rviz,
        ]
    )
