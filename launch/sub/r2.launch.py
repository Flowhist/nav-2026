from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Connect to R2000 Lidar via Direct Ethernet Connection

    Network topology:
        Lidar (LAN IP) -> PC (LAN IP) [Direct Ethernet Connection]
    """

    # 声明launch参数
    scanner_ip_arg = DeclareLaunchArgument(
        "scanner_ip", default_value="10.86.81.200", description="雷达IP地址"
    )

    scanner_port_arg = DeclareLaunchArgument(
        "scanner_port", default_value="80", description="雷达HTTP端口"
    )

    # 获取参数
    scanner_ip = LaunchConfiguration("scanner_ip")
    scanner_port = LaunchConfiguration("scanner_port")

    return LaunchDescription(
        [
            scanner_ip_arg,
            scanner_port_arg,
            Node(
                package="finav",
                executable="r2000_node",
                name="r2000_node",
                output="log",
                emulate_tty=True,
                respawn=True,
                respawn_delay=5.0,
                arguments=["--ros-args", "--log-level", "error"],
                parameters=[
                    {
                        "frame_id": "laser_frame",
                        "scanner_ip": scanner_ip,
                        "scanner_port": scanner_port,
                        "scan_frequency": 10,
                        "samples_per_scan": 1260,
                        "scan_start_angle": -180,
                        "scan_end_angle": 180,
                    }
                ],
            ),
        ]
    )
