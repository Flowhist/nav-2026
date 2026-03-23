import math
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, PoseStamped

# 使用你的串口实现（保持路径）
from .modules.dm_serial import DM_Serial


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
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 921600)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter(
            "publish_rpy_in_degree", True
        )  # 若希望 /imu/rpy 以“度”发布，把 False 改 True
        self.declare_parameter("verbose", True)  # 终端打印
        self.declare_parameter(
            "qos_reliable", True
        )  # 发布端 QoS（默认 Reliable，RViz 直接可见）
        # 新增：三个话题的开关（默认全开）
        self.declare_parameter("publish_imu_data", False)  # /imu/data
        self.declare_parameter("publish_rpy", True)  # /imu/rpy
        self.declare_parameter("publish_pose", False)  # /imu/pose

        # 零偏补偿参数
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
        self.publish_rpy_in_degree = bool(_p("publish_rpy_in_degree", True))
        self.verbose = bool(_p("verbose", True))
        qos_reliable = bool(_p("qos_reliable", True))
        # 三个话题的总开关
        self.publish_imu_data = bool(_p("publish_imu_data", True))  # 默认开启
        self.publish_rpy = bool(_p("publish_rpy", True))
        self.publish_pose = bool(_p("publish_pose", False))

        # 零偏补偿值
        self.gyro_bias_x = float(_p("gyro_bias_x", 0.0))
        self.gyro_bias_y = float(_p("gyro_bias_y", 0.0))
        self.gyro_bias_z = float(_p("gyro_bias_z", 0.0))
        self.accel_bias_x = float(_p("accel_bias_x", 0.0))
        self.accel_bias_y = float(_p("accel_bias_y", 0.0))
        self.accel_bias_z = float(_p("accel_bias_z", 0.0))

        # 如果有零偏补偿，打印信息
        if any(
            [
                self.gyro_bias_x,
                self.gyro_bias_y,
                self.gyro_bias_z,
                self.accel_bias_x,
                self.accel_bias_y,
                self.accel_bias_z,
            ]
        ):
            self.get_logger().info("IMU零偏补偿已启用:")
            self.get_logger().info(
                f"  陀螺仪: X={self.gyro_bias_x:.6f}, Y={self.gyro_bias_y:.6f}, Z={self.gyro_bias_z:.6f} rad/s"
            )
            self.get_logger().info(
                f"  加速度计: X={self.accel_bias_x:.6f}, Y={self.accel_bias_y:.6f}, Z={self.accel_bias_z:.6f} m/s²"
            )

        baud = _p("baudrate", 921600)
        try:
            self.baudrate = int(baud)
        except (TypeError, ValueError):
            self.get_logger().warn(f'Invalid baudrate "{baud}", fallback to 921600')
            self.baudrate = 921600

        # ---------- QoS ----------
        if qos_reliable:
            from rclpy.qos import (
                QoSProfile,
                ReliabilityPolicy,
                HistoryPolicy,
                DurabilityPolicy,
            )

            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
                durability=DurabilityPolicy.VOLATILE,
            )
        else:
            from rclpy.qos import qos_profile_sensor_data  # BestEffort

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
        self.pub_pose = (
            self.create_publisher(PoseStamped, "imu/pose", 10)
            if self.publish_pose
            else None
        )

        # ---------- Serial ----------
        try:
            # 按你的要求：初始化不传 timeout
            self.ser = DM_Serial(self.port, baudrate=self.baudrate)
            self.ser.start_reader()  # 后台读线程交给你的类
            self.get_logger().info(f"Opened serial {self.port} @ {self.baudrate}")
        except Exception as e:
            self.get_logger().fatal(f"Init serial failed: {e}")
            raise

        # ---------- Timers ----------
        # 用时间戳做去重；拿不到就不去重
        self._last_stamp_ts: Optional[float] = None
        self._closing = threading.Event()
        self._logged_bad_fmt_once = False
        self._no_frame_ticks = 0
        self._pub_count = 0

        # 200 Hz 轮询
        self.timer_pub = self.create_timer(0.005, self._on_timer_publish)
        # 每 2s 打一次统计（若你的类提供 get_stats）
        self.timer_stat = self.create_timer(2.0, self._on_timer_stats)

    # ----------- Timers -----------
    def _on_timer_publish(self):
        try:
            latest = self.ser.get_latest()
        except Exception as e:
            if self.verbose:
                self.get_logger().warn(f"get_latest() exception: {e}")
            return

        if latest is None:
            self._no_frame_ticks += 1
            if self._no_frame_ticks % 200 == 0 and self.verbose:  # ≈每秒一次
                self.get_logger().warn(
                    "No frames yet from serial (≈1s). Check IMU streaming/baud/crc."
                )
            return

        # 提取传感器数据
        euler = latest.get("euler")
        if euler is None:
            self._no_frame_ticks += 1
            if self._no_frame_ticks % 200 == 0 and self.verbose:
                self.get_logger().warn(
                    "No euler angle data. Check IMU output settings."
                )
            return

        r_deg, p_deg, y_deg = euler
        accel = latest.get("accel")  # (ax, ay, az) m/s^2，可能为 None
        gyro = latest.get("gyro")  # (gx, gy, gz) rad/s，可能为 None

        # 时间戳去重
        stamp_ts = latest.get("timestamp")
        if stamp_ts == self._last_stamp_ts:
            return
        self._last_stamp_ts = stamp_ts

        # 欧拉角转四元数并归一化
        r, p, y = math.radians(r_deg), math.radians(p_deg), math.radians(y_deg)
        qx, qy, qz, qw = euler_rpy_to_quat(r, p, y)

        # 归一化四元数
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm > 1e-6 and all(math.isfinite(v) for v in (qx, qy, qz, qw)):
            qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        else:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        stamp = self.get_clock().now().to_msg()

        # /imu/data
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id
        qx, qy, qz, qw = euler_rpy_to_quat(r, p, y)

        # /imu/rpy
        if self.pub_rpy:
            rpy_msg = Vector3Stamped()
            rpy_msg.header.stamp = stamp
            rpy_msg.header.frame_id = self.frame_id
            if self.publish_rpy_in_degree:
                rpy_msg.vector.x, rpy_msg.vector.y, rpy_msg.vector.z = (
                    r_deg,
                    p_deg,
                    y_deg,
                )
            else:
                rpy_msg.vector.x, rpy_msg.vector.y, rpy_msg.vector.z = r, p, y
            self.pub_rpy.publish(rpy_msg)
            self.pub_rpy.publish(rpy_msg)

        if self.publish_imu_data == False and self.publish_pose == False:
            return

        # 有效性检查 + 归一化（防 Unvisualizable）
        def _finite(*vals):
            return all(math.isfinite(v) for v in vals)

        if not _finite(qx, qy, qz, qw):
            if self.verbose:
                self.get_logger().warn(
                    "Quaternion has NaN/Inf, publishing identity (0,0,0,1)"
                )
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
            if n < 1e-6:
                if self.verbose:
                    self.get_logger().warn(
                        "Quaternion norm ~0, publishing identity (0,0,0,1)"
                    )
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:
                qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

        # /imu/data
        if self.pub_imu is not None:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self.frame_id
            imu.orientation.x, imu.orientation.y = qx, qy
            imu.orientation.z, imu.orientation.w = qz, qw
            imu.orientation_covariance[0] = 0.02
            imu.orientation_covariance[4] = 0.02
            imu.orientation_covariance[8] = 0.02

            # 角速度（如果有数据）
            if gyro is not None:
                # 应用陀螺仪零偏补偿
                imu.angular_velocity.x = float(gyro[0]) - self.gyro_bias_x
                imu.angular_velocity.y = float(gyro[1]) - self.gyro_bias_y
                imu.angular_velocity.z = float(gyro[2]) - self.gyro_bias_z
                # 有数据时设置一个合理的协方差
                imu.angular_velocity_covariance[0] = 0.01
                imu.angular_velocity_covariance[4] = 0.01
                imu.angular_velocity_covariance[8] = 0.01
            else:
                imu.angular_velocity.x = 0.0
                imu.angular_velocity.y = 0.0
                imu.angular_velocity.z = 0.0
                for i in range(9):
                    imu.angular_velocity_covariance[i] = -1.0  # -1 表示未测量

            # 线加速度（如果有数据）
            if accel is not None:
                # 应用加速度计零偏补偿
                imu.linear_acceleration.x = float(accel[0]) - self.accel_bias_x
                imu.linear_acceleration.y = float(accel[1]) - self.accel_bias_y
                imu.linear_acceleration.z = float(accel[2]) - self.accel_bias_z
                # 有数据时设置一个合理的协方差
                imu.linear_acceleration_covariance[0] = 0.01
                imu.linear_acceleration_covariance[4] = 0.01
                imu.linear_acceleration_covariance[8] = 0.01
            else:
                imu.linear_acceleration.x = 0.0
                imu.linear_acceleration.y = 0.0
                imu.linear_acceleration.z = 0.0
                for i in range(9):
                    imu.linear_acceleration_covariance[i] = -1.0  # -1 表示未测量

            self.pub_imu.publish(imu)

        # /imu/pose
        if self.pub_pose:
            pose = PoseStamped()
            pose.header.stamp, pose.header.frame_id = stamp, self.frame_id
            pose.pose.position.x = pose.pose.position.y = pose.pose.position.z = 0.0
            pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
            pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
            self.pub_pose.publish(pose)

        # 终端打印
        self._pub_count += 1
        self._no_frame_ticks = 0
        if self.verbose:
            msg = f"#{self._pub_count} RPY(deg)=({r_deg:.2f}, {p_deg:.2f}, {y_deg:.2f})"
            if gyro:
                msg += f" Gyro(rad/s)=({gyro[0]:.3f}, {gyro[1]:.3f}, {gyro[2]:.3f})"
            if accel:
                msg += f" Accel(m/s2)=({accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f})"
            self.get_logger().info(msg)

    def _on_timer_stats(self):
        try:
            if hasattr(self.ser, "get_stats"):
                stats = self.ser.get_stats()
                msg = (
                    " ".join([f"{k}={v}" for k, v in stats.items()])
                    if isinstance(stats, dict)
                    else str(stats)
                )
                if self.verbose:
                    self.get_logger().info(f"[stats] {msg}")
        except Exception:
            pass  # 统计异常不影响主流程

    # ----------- Shutdown -----------
    def destroy_node(self):
        if getattr(self, "_closing", None) is None or self._closing.is_set():
            try:
                super().destroy_node()
            except Exception:
                pass
            return
        self._closing.set()
        try:
            if hasattr(self.ser, "stop_reader"):
                self.ser.stop_reader()
        except Exception:
            pass
        try:
            if hasattr(self.ser, "close"):
                self.ser.close()
        except Exception:
            pass
        try:
            super().destroy_node()
        except Exception:
            pass


def main():
    rclpy.init()
    node = DmImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
