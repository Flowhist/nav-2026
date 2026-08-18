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
        "laser_port": int(branch.get("laser_port", defaults["laser_port"])),
        "laser_type": int(branch.get("laser_type", defaults["laser_type"])),
        "use_udp": bool(branch.get("use_udp", defaults["use_udp"])),
        "synctype": bool(branch.get("synctype", defaults["synctype"])),
        "change_param": bool(branch.get("change_param", defaults["change_param"])),
        "spin_frequency_hz": int(
            branch.get("spin_frequency_hz", defaults["spin_frequency_hz"])
        ),
        "angle_increment": str(
            branch.get("angle_increment", defaults["angle_increment"])
        ),
        "noise_filter_level": int(
            branch.get("noise_filter_level", defaults["noise_filter_level"])
        ),
        "start_angle": float(branch.get("start_angle", defaults["start_angle"])),
        "end_angle": float(branch.get("end_angle", defaults["end_angle"])),
        "shadows_filter_level": int(
            branch.get("shadows_filter_level", defaults["shadows_filter_level"])
        ),
        "disturb_filter_enable": bool(
            branch.get("disturb_filter_enable", defaults["disturb_filter_enable"])
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


def _resolve_project_dir(pkg_share):
    repo_dir = os.environ.get("FINAV_REPO_DIR", "").strip()
    if repo_dir and os.path.isdir(repo_dir):
        return repo_dir
    candidate = os.path.abspath(
        os.path.join(pkg_share, "..", "..", "..", "..", "src", "finav")
    )
    if os.path.isdir(candidate):
        return candidate
    return pkg_share


def _load_lidar_config():
    base = {
        "laser_port": 8080,
        "laser_type": 1,
        "use_udp": False,
        "synctype": False,
        "change_param": False,
        "spin_frequency_hz": 30,
        "angle_increment": "0.100",
        "noise_filter_level": 1,
        "start_angle": 0.0,
        "end_angle": 0.0,
        "shadows_filter_level": 0,
        "disturb_filter_enable": False,
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

    pkg_share = get_package_share_directory("finav")
    project_dir = _resolve_project_dir(pkg_share)
    config_path = os.path.join(project_dir, "config", "lidar.yaml")
    data = _read_yaml(config_path)
    return {
        "left": _load_branch(data, "left", defaults["left"]),
        "right": _load_branch(data, "right", defaults["right"]),
        "fusion": _load_fusion(data, defaults["fusion"]),
    }


def _driver_parameters(cfg, scanner_ip):
    return {
        "frame_name": str(cfg["frame_id"]),
        "topic_name": str(cfg["topic_name"]),
        "laser_ip": scanner_ip,
        "laser_port": int(cfg["laser_port"]),
        "laser_type": int(cfg["laser_type"]),
        "use_udp": bool(cfg["use_udp"]),
        "synctype": bool(cfg["synctype"]),
        "block_enable": False,
        "change_param": bool(cfg["change_param"]),
        "spin_frequency_Hz": int(cfg["spin_frequency_hz"]),
        "angle_increment": str(cfg["angle_increment"]),
        "noise_filter_level": int(cfg["noise_filter_level"]),
        "start_angle": float(cfg["start_angle"]),
        "end_angle": float(cfg["end_angle"]),
        "shadows_filter_level": int(cfg["shadows_filter_level"]),
        "disturb_filter_enable": bool(cfg["disturb_filter_enable"]),
    }


def generate_launch_description():
    cfg = _load_lidar_config()
    pkg_share = get_package_share_directory("finav")
    project_dir = _resolve_project_dir(pkg_share)
    config_dir = os.path.join(project_dir, "config")
    fastdds_path = os.path.join(config_dir, "fastdds_profiles.xml")

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
                executable="hins_he_lidar_node",
                name="hins_he_lidar_left_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=5.0,
                parameters=[_driver_parameters(cfg["left"], left_scanner_ip)],
            ),
            Node(
                package="finav",
                executable="hins_he_lidar_node",
                name="hins_he_lidar_right_node",
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
