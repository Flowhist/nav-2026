import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_lidar_config():
    defaults = {
        "lidar_ip": "10.86.81.200",
        "frame_id": "laser_frame",
        "topic_name": "/scan",
        "is_ethernet": True,
        "port_name": "/dev/ttyUSB0",
        "baud": 921600,
        "scan_frequency": 15,
        "scan_resolution": 1000,
        "start_angle": -90,
        "stop_angle": 180,
        "offset_angle": 0,
        "range_min": 0.05,
        "range_max": 25.0,
        "filter_switch": 0,
        "cluster_num": 10,
        "broad_filter_num": 20,
        "nor_switch": 1,
        "is_reverse_postion": False,
    }

    config_path = os.path.join(get_package_share_directory("finav"), "config", "lidar.yaml")
    try:
        import yaml  # type: ignore

        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        if isinstance(data, dict):
            defaults["lidar_ip"] = str(data.get("lidar_ip", defaults["lidar_ip"]))
            defaults["frame_id"] = str(data.get("frame_id", defaults["frame_id"]))
            defaults["topic_name"] = str(data.get("topic_name", defaults["topic_name"]))
            defaults["is_ethernet"] = bool(data.get("is_ethernet", defaults["is_ethernet"]))
            defaults["port_name"] = str(data.get("port_name", defaults["port_name"]))
            defaults["baud"] = int(data.get("baud", defaults["baud"]))
            defaults["scan_frequency"] = int(data.get("scan_frequency", defaults["scan_frequency"]))
            defaults["scan_resolution"] = int(data.get("scan_resolution", defaults["scan_resolution"]))
            defaults["start_angle"] = int(data.get("start_angle", defaults["start_angle"]))
            defaults["stop_angle"] = int(data.get("stop_angle", defaults["stop_angle"]))
            defaults["offset_angle"] = int(data.get("offset_angle", defaults["offset_angle"]))
            defaults["range_min"] = float(data.get("range_min", defaults["range_min"]))
            defaults["range_max"] = float(data.get("range_max", defaults["range_max"]))
            defaults["filter_switch"] = int(data.get("filter_switch", defaults["filter_switch"]))
            defaults["cluster_num"] = int(data.get("cluster_num", defaults["cluster_num"]))
            defaults["broad_filter_num"] = int(
                data.get("broad_filter_num", defaults["broad_filter_num"])
            )
            defaults["nor_switch"] = int(data.get("nor_switch", defaults["nor_switch"]))
            defaults["is_reverse_postion"] = bool(
                data.get("is_reverse_postion", defaults["is_reverse_postion"])
            )
    except Exception:
        pass

    return defaults


def generate_launch_description():
    cfg = _load_lidar_config()

    scanner_ip_arg = DeclareLaunchArgument(
        "scanner_ip", default_value=str(cfg["lidar_ip"]), description="雷达IP地址"
    )
    scanner_ip = LaunchConfiguration("scanner_ip")

    return LaunchDescription(
        [
            scanner_ip_arg,
            Node(
                package="finav",
                executable="free_lidar_node",
                name="free_lidar_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=5.0,
                parameters=[
                    {
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
                        "topic_name": str(cfg["topic_name"]),
                    }
                ],
            ),
        ]
    )
