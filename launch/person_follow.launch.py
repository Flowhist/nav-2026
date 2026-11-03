"""Integrated person following; drive must be explicitly enabled."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera = Node(package='finav', executable='run_vision.sh', arguments=['camera'],
                  condition=IfCondition(LaunchConfiguration('start_camera')), output='screen')
    tracker = Node(package='finav', executable='run_vision.sh', arguments=['tracker'], output='screen')
    observe = Node(package='finav', executable='person_follow.py',
                   condition=UnlessCondition(LaunchConfiguration('drive')), output='screen')
    drive = Node(package='finav', executable='person_follow.py', arguments=['--drive'],
                 condition=IfCondition(LaunchConfiguration('drive')), output='screen')
    actions = [DeclareLaunchArgument('drive', default_value='false'),
               DeclareLaunchArgument('start_camera', default_value='true')]
    for node in (camera, tracker, observe, drive):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node,
                       on_exit=[EmitEvent(event=Shutdown(reason='Following component exited'))])))
    return LaunchDescription(actions+[camera, tracker, observe, drive])
