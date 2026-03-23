#!/usr/bin/env python3
"""
摇杆优先级控制节点（joystick_control.py）

功能：
  1) 读取摇杆串口信号
  2) 按键单击切换摇杆激活状态，持续发布 /js_state (std_msgs/Bool)
       默认关闭(False)，单击切换为开启(True)，再次单击关闭
  3) 激活时以固定频率发布 /js_cmd_vel (geometry_msgs/Twist)：
  4) 未激活时不发布 /js_cmd_vel
"""

import math
import time
import threading
from collections import deque
from time import monotonic

import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty


# 摇杆数据新鲜度保护：超过该时间未收到有效位置帧则强制回零
_DEFAULT_JOY_STALE_TIMEOUT = 0.35  # s


# ── 串口协议工具 ──────────────────────────────────────────────────────────── #
def _checksum(payload) -> int:
    return sum(payload) & 0xFF


# 位置查询帧（固定，预计算）
# AA 55 | data_len=03 00 | id=01 | func=31 | chk=32 | 55 AA
_QUERY_PKT = bytes(
    [0xAA, 0x55, 0x03, 0x00, 0x01, 0x31, _checksum([0x01, 0x31]), 0x55, 0xAA]
)


def _find_packet(buf: bytes):
    """从字节流提取一帧完整的 AA-55 数据包。\n返回 (pkt, remaining) 或 (None, buf)。"""
    for i in range(len(buf) - 9):
        if buf[i] == 0xAA and buf[i + 1] == 0x55:
            dlen = buf[i + 2] | (buf[i + 3] << 8)
            plen = 2 + 2 + dlen + 2
            if i + plen <= len(buf):
                if buf[i + plen - 2] == 0x55 and buf[i + plen - 1] == 0xAA:
                    return buf[i : i + plen], buf[i + plen :]
    if len(buf) > 500:
        return None, buf[1:]
    return None, buf


# ── ROS 节点 ──────────────────────────────────────────────────────────────── #


class JoystickControl(Node):
    """
    发布 /js_state 和 /js_cmd_vel，供 base_control 做最高优先级仲裁。
    """

    def __init__(self):
        super().__init__("joystick_control")
        # 参数声明
        self.declare_parameter("joystick_port", "/dev/ttyUSB0")
        self.declare_parameter("joystick_baud", 115200)
        self.declare_parameter("publish_rate", 10)
        self.declare_parameter("query_interval", 0.015)
        self.declare_parameter("joy_stale_timeout", _DEFAULT_JOY_STALE_TIMEOUT)
        self.declare_parameter("dead_zone", 20)
        self.declare_parameter("sat_zone", 110)
        # 速度相关
        self.declare_parameter("js_vel_low", 0.3)  # m/s
        self.declare_parameter("js_vel_high", 0.6)
        self.declare_parameter("js_rot_low", 10)  # deg/s
        self.declare_parameter("js_rot_high", 25)
        # 参数读取
        self.js_vel_low = float(self.get_parameter("js_vel_low").value)
        self.js_vel_high = float(self.get_parameter("js_vel_high").value)
        self.js_rot_low = math.radians(
            float(self.get_parameter("js_rot_low").value)
        )  # 从deg/s转到rad/s
        self.js_rot_high = math.radians(float(self.get_parameter("js_rot_high").value))
        self.dead_zone = int(self.get_parameter("dead_zone").value)
        self.sat_zone = int(self.get_parameter("sat_zone").value)
        self.publish_rate = int(self.get_parameter("publish_rate").value)

        self.line = (self.dead_zone + self.sat_zone) // 3 * 2  # 二/三层分界线

        self._query_interval = max(
            0.005, float(self.get_parameter("query_interval").value)
        )
        self._joy_stale_timeout = max(
            0.02, float(self.get_parameter("joy_stale_timeout").value)
        )

        self._state_pub = self.create_publisher(Bool, "/js_state", self.publish_rate)
        self._cmd_vel_pub = self.create_publisher(
            Twist, "/js_cmd_vel", self.publish_rate
        )
        self._nav_clear_pub = self.create_publisher(Empty, "/nav_clear", 10)

        # 摇杆状态（线程间共享，由 _lock 保护）
        self._lock = threading.Lock()
        self._js_enabled = False  # 总开关（按键切换）
        self._joy_twist = Twist()  # 当前摇杆换算的 Twist
        self._last_joy_update = monotonic()

        # 串口缓冲区（生产者/消费者）
        self._pos_buf = (deque(maxlen=512), threading.Lock())
        self._btn_buf = (deque(maxlen=512), threading.Lock())
        self._wrt_lock = threading.Lock()

        # 状态初始化
        self._running = True
        self._ser = None
        self._port = self.get_parameter("joystick_port").value
        self._baud = self.get_parameter("joystick_baud").value

        # 定时发布
        self.create_timer(1.0 / self.publish_rate, self._publish)

        # 后台线程
        for fn in (self._read_loop, self._query_loop, self._button_loop):
            threading.Thread(target=fn, daemon=True).start()

        self.get_logger().info(
            "摇杆优先级控制节点已启动\n"
            f"  串口: {self._port } @ {self._baud}  |  发布频率: {self.publish_rate} Hz\n"
            f"  查询周期: {self._query_interval*1000:.0f} ms \n"
            f"  第二层: 前后 {self.js_vel_low*100:.0f} cm/s  左右 {math.degrees(self.js_rot_low):.0f} deg/s\n"
            f"  第三层: 前后 {self.js_vel_high*100:.0f} cm/s  左右 {math.degrees(self.js_rot_high):.0f} deg/s\n"
            "  摇杆按键单击 → 切换激活状态（当前：关闭）"
        )

    # ── 定时发布 ──────────────────────────────────────────── #

    def _publish(self):
        now = monotonic()
        with self._lock:
            enabled = self._js_enabled
            twist = self._joy_twist
            age = now - self._last_joy_update
            stale = age > self._joy_stale_timeout
            if enabled and stale:
                # 摇杆位置帧超时，立刻回零
                self._joy_twist = Twist()
                twist = self._joy_twist

        # /js_state 始终发布
        state_msg = Bool()
        state_msg.data = enabled
        self._state_pub.publish(state_msg)

        # /js_cmd_vel 在激活时发布
        if enabled:
            self._cmd_vel_pub.publish(twist)

    def _joystick_to_twist(self, x_pos: int, y_pos: int) -> Twist:
        """
        摇杆原始坐标 → geometry_msgs/Twist
        死区内返回零速 Twist，调用方仍会发布
        """
        dist = math.hypot(x_pos, y_pos)
        msg = Twist()

        if dist < self.dead_zone:
            return msg  # 零速

        lin_spd = self.js_vel_low if dist < self.line else self.js_vel_high
        ang_spd = self.js_rot_low if dist < self.line else self.js_rot_high

        if abs(y_pos) >= abs(x_pos):  # 前 / 后分区 → 旋转（前=左转，后=右转）
            msg.angular.z = ang_spd if y_pos > 0 else -ang_spd
        else:  # 左 / 右分区 → 直行（右=前进，左=后退）
            msg.linear.x = lin_spd if x_pos > 0 else -lin_spd

        return msg

    # ── 串口读取线程（生产者，将收到的帧分发到对应缓冲区）────────────── #

    def _read_loop(self):
        raw = b""
        while self._running:
            if self._ser is None or not self._ser.is_open:
                raw = b""
                try:
                    self._ser = serial.Serial(self._port, self._baud, timeout=0.005)
                    self.get_logger().info(f"✓ 摇杆串口已连接: {self._port}")
                except Exception as e:
                    self.get_logger().warn(
                        f"串口 {self._port} 连接失败: {e}，5 秒后重试...",
                        throttle_duration_sec=30.0,
                    )
                    time.sleep(5.0)
                    continue
            try:
                if self._ser.in_waiting > 0:
                    raw += self._ser.read(self._ser.in_waiting)
                    while len(raw) >= 10:
                        pkt, raw = _find_packet(raw)
                        if pkt is None:
                            break
                        func = pkt[5]
                        if func == 0x31:  # 位置响应
                            buf, lk = self._pos_buf
                            with lk:
                                buf.extend(pkt)
                        elif func == 0xE0:  # 按键事件
                            buf, lk = self._btn_buf
                            with lk:
                                buf.extend(pkt)
                else:
                    time.sleep(0.001)
            except Exception as e:
                self.get_logger().warn(f"串口读取异常: {e}，尝试重连...")
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(1.0)

    # ── 位置查询线程（发送 query + 从缓冲区解析位置 → 更新 Twist）─────── #

    def _query_loop(self):
        while self._running:
            if self._ser is None or not self._ser.is_open:
                time.sleep(0.1)
                continue
            try:
                with self._wrt_lock:
                    self._ser.write(_QUERY_PKT)
                    self._ser.flush()
            except Exception:
                time.sleep(0.05)
                continue

            time.sleep(self._query_interval)  # 等待设备响应

            buf, lk = self._pos_buf
            with lk:
                data = bytes(buf) if len(buf) >= 10 else b""
                buf.clear()

            if data:
                pkt, _ = _find_packet(data)
                if pkt and len(pkt) >= 10:
                    x_pos = 128 - int(pkt[6])
                    y_pos = 128 - int(pkt[7])
                    twist = self._joystick_to_twist(x_pos, y_pos)
                    with self._lock:
                        self._joy_twist = twist
                        self._last_joy_update = monotonic()

    # ── 按键线程（单击切换激活状态）─────────────────────────────────────── #

    def _button_loop(self):
        buf = b""
        last_evt_time = 0.0
        while self._running:
            raw, lk = self._btn_buf
            with lk:
                chunk = bytes(raw) if len(raw) > 0 else b""
                raw.clear()
            buf += chunk

            while len(buf) >= 10:
                pkt, buf = _find_packet(buf)
                if pkt is None:
                    break
                if pkt[5] == 0xE0 and len(pkt) >= 10:
                    btn = pkt[6] | (pkt[7] << 8)
                    now = time.time()
                    if now - last_evt_time > 0.3 and btn in (
                        0x0000,
                        0x0001,
                    ):  # 单击/双击，去抖 300ms
                        with self._lock:
                            was_enabled = self._js_enabled
                            self._js_enabled = not self._js_enabled
                            new_state = self._js_enabled
                        if new_state and not was_enabled:
                            self._nav_clear_pub.publish(Empty())
                            self.get_logger().info("摇杆接管，已发布 /nav_clear")
                        self.get_logger().info(
                            "摇杆 "
                            + (
                                "【激活】─ 已接管，/cmd_vel 被屏蔽"
                                if new_state
                                else "【关闭】─ 已释放，/cmd_vel 恢复"
                            )
                        )
                        last_evt_time = now
            time.sleep(0.005)

    # ── 节点清理 ─────────────────────────────────────────────────────────── #

    def destroy_node(self):
        self._running = False
        # 退出时发布 False，让 base_control 立即知道摇杆已离线
        try:
            msg = Bool()
            msg.data = False
            self._state_pub.publish(msg)
        except Exception:
            pass
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


# ── 入口 ─────────────────────────────────────────────────────────────────── #


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
