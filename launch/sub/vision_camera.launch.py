from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='finav', executable='run_vision.sh',
                                   arguments=['camera'], output='screen')])
