from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('free_lidar'),
        'rviz',
        'free_lidar.rviz')

    
    return LaunchDescription([
        # 启动free_lidar_node节点
        Node(
            package="free_lidar",
            executable="free_lidar_node",
            name="free_lidar_node",
            output="screen",
            emulate_tty=True,
            respawn=True,
            parameters=[
                {"frame_id": "scan",
                "is_ethernet": True,                 # True为网口，False为串口
                 "scanner_ip": "192.168.7.122",      # 确保这是雷达的IP地址
                 "port_name": "/dev/ttyUSB0",        # 串口端口名
                 "baud": 921600,                     # 串口波特率
                 "scan_frequency": 30,
                 "scan_resolution": 1000,
                 "start_angle": -45,
                 "stop_angle": 225,                 
                 "offset_angle": 0,
                 "filter_switch": 0,                 # 滤波开关，0为原始点云，1为开启拖尾滤波
                 "cluster_num": 10,                  # 滤波聚类点数
                 "broad_filter_num": 20,             # 展宽过滤点数
                 "NOR_switch": 1,
                 "is_reverse_postion": False,        # False为正装，True为反装
                 "topic_name": "/scan"
                }
            ]
        ),
        # 启动rviz2节点
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_dir],
            output="screen"
        )
    ])
