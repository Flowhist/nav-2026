import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _read_yaml(path):
    try:
        import yaml  # type: ignore

        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def _load_branch(data, key, defaults):
    branch = data.get(key, {})
    if not isinstance(branch, dict):
        branch = {}

    return {
        "scanner_ip": str(branch.get("scanner_ip", defaults["scanner_ip"])),
        "frame_id": str(branch.get("frame_id", defaults["frame_id"])),
        "topic_name": str(branch.get("topic_name", defaults["topic_name"])),
        "is_ethernet": bool(branch.get("is_ethernet", defaults["is_ethernet"])),
        "port_name": str(branch.get("port_name", defaults["port_name"])),
        "baud": int(branch.get("baud", defaults["baud"])),
        "scan_frequency": int(branch.get("scan_frequency", defaults["scan_frequency"])),
        "scan_resolution": int(branch.get("scan_resolution", defaults["scan_resolution"])),
        "start_angle": int(branch.get("start_angle", defaults["start_angle"])),
        "stop_angle": int(branch.get("stop_angle", defaults["stop_angle"])),
        "offset_angle": int(branch.get("offset_angle", defaults["offset_angle"])),
        "range_min": float(branch.get("range_min", defaults["range_min"])),
        "range_max": float(branch.get("range_max", defaults["range_max"])),
        "filter_switch": int(branch.get("filter_switch", defaults["filter_switch"])),
        "single_filter_enable": bool(
            branch.get("single_filter_enable", defaults["single_filter_enable"])
        ),
        "cluster_num": int(branch.get("cluster_num", defaults["cluster_num"])),
        "broad_filter_num": int(branch.get("broad_filter_num", defaults["broad_filter_num"])),
        "nor_switch": int(branch.get("nor_switch", defaults["nor_switch"])),
        "is_reverse_postion": bool(
            branch.get("is_reverse_postion", defaults["is_reverse_postion"])
        ),
        "use_recv_time_stamp": bool(
            branch.get("use_recv_time_stamp", defaults["use_recv_time_stamp"])
        ),
    }


def _load_fusion(data, defaults):
    branch = data.get("fusion", {})
    if not isinstance(branch, dict):
        branch = {}

    return {
        "output_topic": str(branch.get("output_topic", defaults["output_topic"])),
        "output_frame": str(branch.get("output_frame", defaults["output_frame"])),
        "angle_min": float(branch.get("angle_min", defaults["angle_min"])),
        "angle_max": float(branch.get("angle_max", defaults["angle_max"])),
        "angle_increment": float(
            branch.get("angle_increment", defaults["angle_increment"])
        ),
        "range_min": float(branch.get("range_min", defaults["range_min"])),
        "range_max": float(branch.get("range_max", defaults["range_max"])),
        "sync_queue_size": int(
            branch.get("sync_queue_size", defaults["sync_queue_size"])
        ),
        "input_timeout_sec": float(
            branch.get("input_timeout_sec", defaults["input_timeout_sec"])
        ),
        "tf_timeout_sec": float(branch.get("tf_timeout_sec", defaults["tf_timeout_sec"])),
    }


def _load_lidar_config():
    base = {
        "is_ethernet": True,
        "port_name": "/dev/ttyUSB0",
        "baud": 921600,
        "scan_frequency": 30,
        "scan_resolution": 1000,
        "start_angle": -45,
        "stop_angle": 225,
        "offset_angle": 0,
        "range_min": 0.05,
        "range_max": 25.0,
        "filter_switch": 0,
        "single_filter_enable": False,
        "cluster_num": 10,
        "broad_filter_num": 20,
        "nor_switch": 1,
        "is_reverse_postion": False,
        "use_recv_time_stamp": False,
    }
    defaults = {
        "left": {
            **base,
            "scanner_ip": "10.86.81.111",
            "frame_id": "laser_left_frame",
            "topic_name": "/scan_left",
        },
        "right": {
            **base,
            "scanner_ip": "10.86.81.112",
            "frame_id": "laser_right_frame",
            "topic_name": "/scan_right",
        },
        "fusion": {
            "output_topic": "/scan",
            "output_frame": "base_link",
            "angle_min": -3.141592653589793,
            "angle_max": 3.141592653589793,
            "angle_increment": 0.0017453292519943296,
            "range_min": 0.05,
            "range_max": 25.0,
            "sync_queue_size": 10,
            "input_timeout_sec": 1.0,
            "tf_timeout_sec": 0.1,
        },
    }

    config_path = os.path.join(get_package_share_directory("finav"), "config", "lidar.yaml")
    data = _read_yaml(config_path)
    return {
        "left": _load_branch(data, "left", defaults["left"]),
        "right": _load_branch(data, "right", defaults["right"]),
        "fusion": _load_fusion(data, defaults["fusion"]),
    }


def _driver_parameters(cfg, scanner_ip):
    return {
        "frame_id": str(cfg["frame_id"]),
        "is_ethernet": bool(cfg["is_ethernet"]),
        "scanner_ip": scanner_ip,
        "port_name": str(cfg["port_name"]),
        "baud": int(cfg["baud"]),
        "scan_frequency": int(cfg["scan_frequency"]),
        "scan_resolution": int(cfg["scan_resolution"]),
        "start_angle": int(cfg["start_angle"]),
        "stop_angle": int(cfg["stop_angle"]),
        "offset_angle": int(cfg["offset_angle"]),
        "range_min": float(cfg["range_min"]),
        "range_max": float(cfg["range_max"]),
        "filter_switch": int(cfg["filter_switch"]),
        "cluster_num": int(cfg["cluster_num"]),
        "broad_filter_num": int(cfg["broad_filter_num"]),
        "NOR_switch": int(cfg["nor_switch"]),
        "is_reverse_postion": bool(cfg["is_reverse_postion"]),
        "use_recv_time_stamp": bool(cfg["use_recv_time_stamp"]),
        "single_filter_enable": bool(cfg["single_filter_enable"]),
        "topic_name": str(cfg["topic_name"]),
    }


def generate_launch_description():
    cfg = _load_lidar_config()
    pkg_share = get_package_share_directory("finav")
    fastdds_path = os.path.join(pkg_share, "config", "fastdds_profiles.xml")

    left_scanner_ip_arg = DeclareLaunchArgument(
        "left_scanner_ip",
        default_value=str(cfg["left"]["scanner_ip"]),
        description="左侧雷达IP地址",
    )
    right_scanner_ip_arg = DeclareLaunchArgument(
        "right_scanner_ip",
        default_value=str(cfg["right"]["scanner_ip"]),
        description="右侧雷达IP地址",
    )

    left_scanner_ip = LaunchConfiguration("left_scanner_ip")
    right_scanner_ip = LaunchConfiguration("right_scanner_ip")

    return LaunchDescription(
        [
            left_scanner_ip_arg,
            right_scanner_ip_arg,
            SetEnvironmentVariable(
                name="FASTRTPS_DEFAULT_PROFILES_FILE",
                value=fastdds_path,
            ),
            Node(
                package="finav",
                executable="free_lidar_node",
                name="free_lidar_left_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=5.0,
                parameters=[_driver_parameters(cfg["left"], left_scanner_ip)],
            ),
            Node(
                package="finav",
                executable="free_lidar_node",
                name="free_lidar_right_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=5.0,
                parameters=[_driver_parameters(cfg["right"], right_scanner_ip)],
            ),
            Node(
                package="finav",
                executable="scan_fusion_node",
                name="scan_fusion_node",
                output="screen",
                parameters=[
                    {
                        "left_topic": str(cfg["left"]["topic_name"]),
                        "right_topic": str(cfg["right"]["topic_name"]),
                        "output_topic": str(cfg["fusion"]["output_topic"]),
                        "output_frame": str(cfg["fusion"]["output_frame"]),
                        "angle_min": float(cfg["fusion"]["angle_min"]),
                        "angle_max": float(cfg["fusion"]["angle_max"]),
                        "angle_increment": float(cfg["fusion"]["angle_increment"]),
                        "range_min": float(cfg["fusion"]["range_min"]),
                        "range_max": float(cfg["fusion"]["range_max"]),
                        "sync_queue_size": int(cfg["fusion"]["sync_queue_size"]),
                        "input_timeout_sec": float(cfg["fusion"]["input_timeout_sec"]),
                        "tf_timeout_sec": float(cfg["fusion"]["tf_timeout_sec"]),
                    }
                ],
            ),
        ]
    )
