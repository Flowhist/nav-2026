#!/usr/bin/env python3

"""
USB HID joystick launch.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_joy_config(config_path: str):
    defaults = {
        "enabled": "true",
        "dev": "/dev/input/js0",
    }

    try:
        import yaml  # type: ignore

        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        joy_control = data.get("joy_control", {}) if isinstance(data, dict) else {}
        joy_control_params = (
            joy_control.get("ros__parameters", {})
            if isinstance(joy_control, dict)
            else {}
        )
        if isinstance(joy_control_params, dict):
            defaults["dev"] = str(joy_control_params.get("dev", defaults["dev"]))
            enabled = bool(joy_control_params.get("enabled", True))
            defaults["enabled"] = "true" if enabled else "false"
    except Exception:
        pass

    return defaults


def _resolve_config_dir(pkg_share: str) -> str:
    repo_dir = os.environ.get("FINAV_REPO_DIR", "").strip()
    if repo_dir and os.path.isdir(repo_dir):
        return os.path.join(repo_dir, "config")
    candidate = os.path.abspath(
        os.path.join(pkg_share, "..", "..", "..", "..", "src", "finav", "config")
    )
    if os.path.isdir(candidate):
        return candidate
    return os.path.join(pkg_share, "config")


def generate_launch_description():
    pkg_share = get_package_share_directory("finav")
    config_dir = _resolve_config_dir(pkg_share)
    joy_config_path = os.path.join(config_dir, "joy.yaml")
    base_control_config_path = os.path.join(config_dir, "base_control.yaml")
    joy_cfg = _load_joy_config(joy_config_path)

    use_joystick_arg = DeclareLaunchArgument(
        "use_joystick",
        default_value=joy_cfg["enabled"],
        description="是否启动 USB HID 摇杆输入",
    )
    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value=joy_cfg["dev"],
        description="Linux joystick 设备路径",
    )
    use_joystick = LaunchConfiguration("use_joystick")
    joy_dev = LaunchConfiguration("joy_dev")

    joy_control_node = Node(
        package="finav",
        executable="joy_control.py",
        name="joy_control",
        output="screen",
        condition=IfCondition(use_joystick),
        parameters=[joy_config_path, base_control_config_path, {"dev": joy_dev}],
    )

    return LaunchDescription(
        [
            use_joystick_arg,
            joy_dev_arg,
            joy_control_node,
        ]
    )
