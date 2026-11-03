"""Web-controlled observation only; no chassis or motion command processes."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    repo = Path(os.environ["FINAV_REPO_DIR"])
    camera = ExecuteProcess(cmd=["/bin/bash", str(repo / "scripts/service/run_vision.sh"), "camera"],
                            name="camera", output="screen", respawn=True, respawn_delay=5.0)
    processes = [
        camera,
        ExecuteProcess(cmd=["ros2", "run", "dauxi", "dauxi", "--ros-args", "-r", "__ns:=/ultrasonic",
                            "-p", f"config_file:={repo / 'config/ultrasonic.yaml'}", "-p", "enable:=1"], name="ultrasonic", output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(repo / "launch/sub/obstacle_avoidance.launch.py"))),
    ]

    def failed(event, context):
        if event.action is camera:
            return [] if context.is_shutdown else [LogInfo(msg="Camera exited; lidar avoidance remains active")]
        if not context.is_shutdown:
            raise RuntimeError(f"Perception component exited (code {event.returncode}); see preceding log")
        return []

    return LaunchDescription([RegisterEventHandler(OnProcessExit(on_exit=failed))] + processes)
