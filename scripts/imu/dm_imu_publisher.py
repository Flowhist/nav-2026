#!/usr/bin/env python3
"""
DM-IMU ROS2 发布节点（USB串口直连版）
直接通过USB串口连接IMU，无需树莓派中继
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

import sys
import os

# 添加modules目录到Python路径(适配安装后的目录结构)
imu_modules_path = os.path.join(os.path.dirname(__file__), "dm_imu_modules")
if os.path.exists(imu_modules_path):
    sys.path.insert(0, imu_modules_path)
else:
    # 开发环境路径
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "dm_imu_modules"))
from modules.dm_serial import DM_Serial


def euler_rpy_to_quat(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """ZYX intrinsic (yaw->pitch->roll); roll/pitch/yaw in radians."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class DmImuNode(Node):
    def __init__(self):
        super().__init__("dm_imu")

        # ---------- Parameters ----------
        self.declare_parameter("port", "/dev/ttyACM0")  # USB串口
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rpy_in_degree", False)
        self.declare_parameter("verbose", False)
        self.declare_parameter("qos_reliable", True)
        self.declare_parameter("yaw_offset_deg", 0.0)
        self.declare_parameter("publish_imu_data", True)
        self.declare_parameter("publish_rpy", True)

        # 零偏校准参数
        self.declare_parameter("gyro_bias_x", 0.0)
        self.declare_parameter("gyro_bias_y", 0.0)
        self.declare_parameter("gyro_bias_z", 0.0)
        self.declare_parameter("accel_bias_x", 0.0)
        self.declare_parameter("accel_bias_y", 0.0)
        self.declare_parameter("accel_bias_z", 0.0)

        def _p(name, default=None):
            try:
                v = self.get_parameter(name).value
                return default if v in (None, "") else v
            except Exception:
                return default

        self.port = _p("port", "/dev/ttyACM0")
        self.frame_id = _p("frame_id", "imu_link")
        self.publish_rpy_in_degree = bool(_p("publish_rpy_in_degree", False))
        self.verbose = bool(_p("verbose", False))
        self.yaw_offset_deg = float(_p("yaw_offset_deg", 0.0))
        qos_reliable = bool(_p("qos_reliable", True))
        self.publish_imu_data = bool(_p("publish_imu_data", True))
        self.publish_rpy = bool(_p("publish_rpy", True))

        # 读取零偏参数
        self.gyro_bias = [
            float(_p("gyro_bias_x", 0.0)),
            float(_p("gyro_bias_y", 0.0)),
            float(_p("gyro_bias_z", 0.0)),
        ]
        self.accel_bias = [
            float(_p("accel_bias_x", 0.0)),
            float(_p("accel_bias_y", 0.0)),
            float(_p("accel_bias_z", 0.0)),
        ]

        if any(abs(b) > 0.001 for b in self.gyro_bias):
            self.get_logger().info(f"陀螺仪零偏校准: {self.gyro_bias}")
        if any(abs(b) > 0.001 for b in self.accel_bias):
            self.get_logger().info(f"加速度零偏校准: {self.accel_bias}")

        baud = _p("baudrate", 921600)
        try:
            self.baudrate = int(baud)
        except (TypeError, ValueError):
            self.get_logger().warn(f'Invalid baudrate "{baud}", fallback to 921600')
            self.baudrate = 921600

        # ---------- QoS ----------
        if qos_reliable:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
                durability=DurabilityPolicy.VOLATILE,
            )
        else:
            from rclpy.qos import qos_profile_sensor_data

            qos = qos_profile_sensor_data

        # ---------- Publishers ----------
        self.pub_imu = (
            self.create_publisher(Imu, "imu/data", qos)
            if self.publish_imu_data
            else None
        )
        self.pub_rpy = (
            self.create_publisher(Vector3Stamped, "imu/rpy", qos)
            if self.publish_rpy
            else None
        )

        # ---------- Initialize USB Serial Connection ----------
        self.get_logger().info(f"连接IMU: {self.port} @ {self.baudrate}")
        self.dm = DM_Serial(port=self.port, baudrate=self.baudrate)

        if not self.dm.is_open:
            err = self.dm.last_error() or "Unknown error"
            self.get_logger().error(f"无法打开IMU串口: {err}")
            raise RuntimeError(f"Failed to open IMU serial: {err}")

        # 启动后台读取线程
        if not self.dm.start_reader(read_sleep=0.001):
            self.get_logger().error("无法启动IMU读取线程")
            raise RuntimeError("Failed to start reader thread")

        self.get_logger().info("✓ IMU连接成功，后台读取线程已启动")

        # ---------- Timer for Publishing ----------
        self.timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.last_count = 0

        self.get_logger().info('DM-IMU 节点已启动 (模式: USB)')

    def timer_callback(self):
        """定时发布IMU数据"""
        data = self.dm.get_latest()

        # 检查是否有新数据
        if data["count"] == self.last_count:
            return

        self.last_count = data["count"]
        quat_tuple = data.get("quat")
        gyro_tuple = data.get("gyro")
        accel_tuple = data.get("accel")
        euler_tuple = data.get("euler")

        # DM-IMU-L1是6轴IMU,至少需要陀螺仪或加速度计数据
        if not (gyro_tuple or accel_tuple):
            return

        now = self.get_clock().now().to_msg()

        # 发布完整IMU数据
        if self.pub_imu:
            msg = Imu()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id

            # 四元数 - 从欧拉角转换
            # 注意：6轴IMU的Yaw会漂移，但短时间内可用于辅助SLAM
            if euler_tuple and all(math.isfinite(v) for v in euler_tuple):
                try:
                    yaw_deg = euler_tuple[2] + self.yaw_offset_deg
                    yaw_rad = math.radians(yaw_deg)
                    qx, qy, qz, qw = euler_rpy_to_quat(
                        math.radians(euler_tuple[0]),  # roll
                        math.radians(euler_tuple[1]),  # pitch
                        yaw_rad,                       # yaw (含固定偏置)
                    )
                    msg.orientation.x = qx
                    msg.orientation.y = qy
                    msg.orientation.z = qz
                    msg.orientation.w = qw
                    # 设置较大的协方差，表示不完全信任（因为会漂移）
                    msg.orientation_covariance[0] = 0.1  # roll
                    msg.orientation_covariance[4] = 0.1  # pitch
                    msg.orientation_covariance[8] = 0.3  # yaw (更不确定)
                except (ValueError, OverflowError):
                    msg.orientation_covariance[0] = -1.0
            else:
                msg.orientation_covariance[0] = -1.0

            # 角速度 (IMU输出已经是弧度/秒,应用零偏校正)
            if gyro_tuple:
                msg.angular_velocity.x = gyro_tuple[0] - self.gyro_bias[0]
                msg.angular_velocity.y = gyro_tuple[1] - self.gyro_bias[1]
                msg.angular_velocity.z = gyro_tuple[2] - self.gyro_bias[2]
                msg.angular_velocity_covariance[0] = 0.01
                msg.angular_velocity_covariance[4] = 0.01
                msg.angular_velocity_covariance[8] = 0.01

            # 线性加速度 (应用零偏校正)
            if accel_tuple:
                msg.linear_acceleration.x = accel_tuple[0] - self.accel_bias[0]
                msg.linear_acceleration.y = accel_tuple[1] - self.accel_bias[1]
                msg.linear_acceleration.z = accel_tuple[2] - self.accel_bias[2]
                msg.linear_acceleration_covariance[0] = 0.01
                msg.linear_acceleration_covariance[4] = 0.01
                msg.linear_acceleration_covariance[8] = 0.01

            self.pub_imu.publish(msg)

        # 发布欧拉角
        if self.pub_rpy and euler_tuple:
            rpy_msg = Vector3Stamped()
            rpy_msg.header.stamp = now
            rpy_msg.header.frame_id = self.frame_id

            yaw_deg = euler_tuple[2] + self.yaw_offset_deg
            if self.publish_rpy_in_degree:
                rpy_msg.vector.x = euler_tuple[0]
                rpy_msg.vector.y = euler_tuple[1]
                rpy_msg.vector.z = yaw_deg
            else:
                rpy_msg.vector.x = math.radians(euler_tuple[0])
                rpy_msg.vector.y = math.radians(euler_tuple[1])
                rpy_msg.vector.z = math.radians(yaw_deg)

            self.pub_rpy.publish(rpy_msg)

        if self.verbose and data["count"] % 100 == 0:
            self.get_logger().info(f"已发布 {data['count']} 帧IMU数据")

    def destroy_node(self):
        """节点销毁时清理资源"""
        self.get_logger().info("正在关闭IMU连接...")
        if hasattr(self, "dm"):
            self.dm.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DmImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
