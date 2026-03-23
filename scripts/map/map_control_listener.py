#!/usr/bin/env python3
"""
建图控制监听器 - 监听 /map_control 话题并执行相应操作
功能：
1. 监听远程发来的建图控制命令
2. 调用系统命令保存地图
3. 发布建图状态变化

使用方式：
    在 map.launch.py 中自动启动
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import subprocess
import os
from datetime import datetime


class MapControlListener(Node):
    """建图控制监听器"""

    def __init__(self):
        super().__init__("map_control_listener")

        # 订阅建图控制话题
        self.map_control_sub = self.create_subscription(
            String, "/map_control", self.map_control_callback, 10
        )

        # 发布建图状态
        self.map_status_pub = self.create_publisher(Bool, "/mapping_active", 10)

        # 当前建图状态
        self.mapping_active = False

        # 地图保存路径
        self.map_save_dir = os.path.expanduser("~/workspace/real_ws/src/uninav/maps")
        os.makedirs(self.map_save_dir, exist_ok=True)

        self.get_logger().info("建图控制监听器已启动")

    def map_control_callback(self, msg):
        """处理建图控制命令"""
        command = msg.data
        self.get_logger().info(f"收到建图控制命令: {command}")

        if command == "start_mapping":
            self.start_mapping()
        elif command == "stop_mapping":
            self.stop_mapping()
        elif command.startswith("save_map:"):
            map_name = command.split(":", 1)[1] if ":" in command else "default_map"
            self.save_map(map_name)
        elif command == "pause_mapping":
            self.pause_mapping()
        elif command == "resume_mapping":
            self.resume_mapping()
        else:
            self.get_logger().warning(f"未知命令: {command}")

    def start_mapping(self):
        """开始建图"""
        self.get_logger().info("开始建图")
        self.mapping_active = True
        self.publish_status()

    def stop_mapping(self):
        """停止建图"""
        self.get_logger().info("停止建图")
        self.mapping_active = False
        self.publish_status()

    def pause_mapping(self):
        """暂停建图"""
        self.get_logger().info("暂停建图")
        # 在slam_toolbox中，可以通过服务调用暂停
        # 这里简单设置状态标志
        self.mapping_active = False
        self.publish_status()

    def resume_mapping(self):
        """恢复建图"""
        self.get_logger().info("恢复建图")
        self.mapping_active = True
        self.publish_status()

    def save_map(self, map_name):
        """保存地图"""
        self.get_logger().info(f"正在保存地图: {map_name}")

        # 确保地图名称安全
        safe_map_name = "".join(c for c in map_name if c.isalnum() or c in ("_", "-"))
        if not safe_map_name:
            safe_map_name = f'map_{datetime.now().strftime("%Y%m%d_%H%M%S")}'

        # 完整路径
        map_path = os.path.join(self.map_save_dir, safe_map_name)

        try:
            # 调用 slam_toolbox 的保存服务
            # 使用 ros2 service call 命令
            cmd = [
                "ros2",
                "service",
                "call",
                "/slam_toolbox/serialize_map",
                "slam_toolbox/srv/SerializePoseGraph",
                f'{{filename: "{map_path}"}}',
            ]

            self.get_logger().info(f'执行保存命令: {" ".join(cmd)}')

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

            if result.returncode == 0:
                self.get_logger().info(f"✓ 地图保存成功: {map_path}")

                # 同时使用 map_saver 保存 pgm/yaml 格式
                self.save_map_files(safe_map_name)
            else:
                self.get_logger().error(f"地图保存失败: {result.stderr}")

        except subprocess.TimeoutExpired:
            self.get_logger().error("保存地图超时")
        except Exception as e:
            self.get_logger().error(f"保存地图出错: {e}")

    def save_map_files(self, map_name):
        """使用 map_saver 保存 pgm/yaml 格式"""
        try:
            map_path = os.path.join(self.map_save_dir, map_name)

            cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path]

            self.get_logger().info("保存 pgm/yaml 格式地图...")

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

            if result.returncode == 0:
                self.get_logger().info(f"✓ pgm/yaml 地图保存成功")
            else:
                self.get_logger().warning(f"pgm/yaml 保存警告: {result.stderr}")

        except Exception as e:
            self.get_logger().warning(f"保存 pgm/yaml 格式时出错: {e}")

    def publish_status(self):
        """发布建图状态"""
        msg = Bool()
        msg.data = self.mapping_active
        self.map_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    listener = MapControlListener()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
