"""Independent observation/planning process; never starts chassis or sensors."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('finav'))
    helper = Path(get_package_prefix('finav')) / 'lib/finav/avoidance_model.py'
    spec = importlib.util.spec_from_file_location('avoidance_model', helper)
    model = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(model)
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/obstacle_avoidance.yaml')),
        Node(package='finav', executable='obstacle_avoidance_node', name='obstacle_avoidance',
             output='screen', parameters=[LaunchConfiguration('config'),
                                         model.model_parameters(share / 'urdf/whillcar.urdf')],
             additional_env={'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1'}),
    ])
