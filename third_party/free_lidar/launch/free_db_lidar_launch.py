from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定义第一台雷达的参数
    radar1_parameters = [
        {"frame_id": "scan",
                "is_ethernet":True,                  #True为网口，False为串口
                 "scanner_ip": "192.168.7.121",      # 确保这是雷达的IP地址
                 "port_name":"/dev/ttyUSB0",         #串口端口名
                 "baud":921600,                      #串口波特率
                 "scan_frequency": 30,
                 "scan_resolution": 1000,
                 "start_angle": -45,
                 "stop_angle": 225,                 
                 "offset_angle": 0,
                 "filter_switch": 1,    	         #滤波开关，0为原始点云，1为开启拖尾滤波
                 "cluster_num":10,       	         #滤波聚类点数
                 "broad_filter_num":20,		         #展宽过滤点数
                 "NOR_switch": 1,
                 "is_reverse_postion":False,         #False为正装，True为反装
                 "topic_name": "/scan1"
                  
         }
    ]

    # 定义第二台雷达的参数
    radar2_parameters = [
        {"frame_id": "scan",
                "is_ethernet":True,                  #True为网口，False为串口
                 "scanner_ip": "192.168.7.122",      # 确保这是雷达的IP地址
                 "port_name":"/dev/ttyUSB0",         #串口端口名
                 "baud":921600,                      #串口波特率
                 "scan_frequency": 30,
                 "scan_resolution": 1000,
                 "start_angle": -45,
                 "stop_angle": 225,                 
                 "offset_angle": 0,
                 "filter_switch": 1,    	         #滤波开关，0为原始点云，1为开启拖尾滤波
                 "cluster_num":10,       	         #滤波聚类点数
                 "broad_filter_num":20,		         #展宽过滤点数
                 "NOR_switch": 1,
                 "is_reverse_postion":False,         #False为正装，True为反装
                 "topic_name": "/scan2"
         }
    ]

    # 创建第一台雷达的节点
    radar1_node = Node(
        package="free_lidar",
        executable="free_lidar_node",
        name="free_lidar_node_1",  # 为节点命名，确保唯一
        output="screen",
        emulate_tty=True,
        respawn=True,
        parameters=radar1_parameters
    )

    # 创建第二台雷达的节点
    radar2_node = Node(
        package="free_lidar",
        executable="free_lidar_node",
        name="free_lidar_node_2",  # 为节点命名，确保唯一
        output="screen",
        emulate_tty=True,
        respawn=True,
        parameters=radar2_parameters
    )

    return LaunchDescription([
        radar1_node,
        radar2_node
    ])
