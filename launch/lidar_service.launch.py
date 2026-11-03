"""Shared dual lidar and static robot transforms; no motion publishers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sub = Path(get_package_share_directory('finav')) / 'launch/sub'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sub / 'robot_model.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sub / 'lidar.launch.py'))),
    ])
