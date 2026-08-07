#!/usr/bin/env python3
"""STM32 handle launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_handle_config(config_path: str):
    defaults = {
        "enabled": "true",
        "port": "/dev/ttyUSB0",
    }
    try:
        import yaml  # type: ignore

        with open(config_path, "r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        node_config = data.get("handle_control", {}) if isinstance(data, dict) else {}
        parameters = (
            node_config.get("ros__parameters", {})
            if isinstance(node_config, dict)
            else {}
        )
        if isinstance(parameters, dict):
            defaults["port"] = str(parameters.get("port", defaults["port"]))
            enabled = bool(parameters.get("enabled", True))
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
    config_path = os.path.join(config_dir, "handle.yaml")
    defaults = _load_handle_config(config_path)

    use_handle_arg = DeclareLaunchArgument(
        "use_handle",
        default_value=defaults["enabled"],
        description="是否启动 STM32 手柄通信",
    )
    handle_port_arg = DeclareLaunchArgument(
        "handle_port",
        default_value=defaults["port"],
        description="STM32 手柄串口路径",
    )
    handle_node = Node(
        package="finav",
        executable="handle_control.py",
        name="handle_control",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_handle")),
        parameters=[
            config_path,
            {"port": LaunchConfiguration("handle_port")},
        ],
    )

    return LaunchDescription(
        [
            use_handle_arg,
            handle_port_arg,
            handle_node,
        ]
    )
