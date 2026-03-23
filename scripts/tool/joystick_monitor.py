#!/usr/bin/env python3
"""
统一监听脚本
采用专用读线程保证按键检测灵敏度
支持死区、饱和区和距离档位
"""
import serial
import time
import threading
import sys
import math
from collections import deque

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# 死区和饱和区设置
DEAD_ZONE = 20  # 死区半径（接近中心时不输出）
SATURATION_ZONE = 110  # 饱和区半径（最远距离）


def calculate_checksum(payload):
    """计算校验和"""
    return sum(payload) & 0xFF


def get_direction_and_level(x_pos, y_pos):
    """
    根据X/Y位置获取方向和距离档位
    返回 (方向字符串, 档位数字) 或 (None, None) 如果在死区内
    """
    # 计算到原点的距离
    distance = math.sqrt(x_pos**2 + y_pos**2)

    # 死区检测
    if distance < DEAD_ZONE:
        return None, None

    # 计算饱和区内的距离（DEAD_ZONE-SATURATION_ZONE映射到实际距离）
    saturated_distance = min(distance, SATURATION_ZONE)

    # 根据距离分档：近(1) 中(2) 远(3)
    if saturated_distance < DEAD_ZONE + (SATURATION_ZONE - DEAD_ZONE) / 3:
        level = 1
    elif saturated_distance < DEAD_ZONE + 2 * (SATURATION_ZONE - DEAD_ZONE) / 3:
        level = 2
    else:
        level = 3

    # 判断方向 (以Y轴为主方向，X轴为次方向)
    abs_x = abs(x_pos)
    abs_y = abs(y_pos)

    if abs_y > abs_x:
        # 上下方向为主
        direction = "上" if y_pos > 0 else "下"
    elif abs_x > abs_y:
        # 左右方向为主
        direction = "右" if x_pos > 0 else "左"
    else:
        # 对角线方向
        if y_pos > 0 and x_pos > 0:
            direction = "右上"
        elif y_pos > 0 and x_pos < 0:
            direction = "左上"
        elif y_pos < 0 and x_pos > 0:
            direction = "右下"
        else:
            direction = "左下"

    return direction, level


def find_packet(buffer):
    """
    从缓冲区中查找完整的数据包
    返回 (packet, remaining_buffer) 或 (None, updated_buffer) 如果未找到完整包
    """
    for i in range(len(buffer) - 9):
        if buffer[i] == 0xAA and buffer[i + 1] == 0x55:
            data_len = buffer[i + 2] | (buffer[i + 3] << 8)
            packet_len = 2 + 2 + data_len + 2

            if i + packet_len <= len(buffer):
                if (
                    buffer[i + packet_len - 2] == 0x55
                    and buffer[i + packet_len - 1] == 0xAA
                ):
                    packet = buffer[i : i + packet_len]
                    remaining = buffer[i + packet_len :]
                    return packet, remaining

    # 未找到完整包，清理垃圾数据
    if len(buffer) > 500:
        return None, buffer[1:]
    return None, buffer


class JoystickMonitor:
    def __init__(self):
        self.ser = None
        self.running = False
        self.buffers = {
            "button": (deque(maxlen=512), threading.Lock()),
            "position": (deque(maxlen=512), threading.Lock()),
        }
        self.write_lock = threading.Lock()

        # 按键事件映射
        self.button_events = {0x0000: "🔵 单击 - 待定", 0x0001: "🟣 双击 - 待定", 0x0002: "🔴 长按 - 开关电源"}

    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            print(f"✓ 已连接到 {SERIAL_PORT}")
            self.running = True
            return True
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False

    def read_serial_data(self):
        """专用串口读取线程 - 保证高优先级和实时性"""
        raw_buffer = b""
        try:
            while self.running:
                if self.ser.in_waiting > 0:
                    raw_buffer += self.ser.read(self.ser.in_waiting)

                    while len(raw_buffer) >= 10:
                        packet, raw_buffer = find_packet(raw_buffer)
                        if packet is None:
                            break

                        cmd = packet[5]
                        buf_type = (
                            "button"
                            if cmd == 0xE0
                            else "position" if cmd == 0x31 else None
                        )

                        if buf_type:
                            buffer, lock = self.buffers[buf_type]
                            with lock:
                                buffer.extend(packet)
                else:
                    time.sleep(0.001)
        except Exception as e:
            print(f"\n✗ 读取线程错误: {e}")

    def query_position(self):
        """查询摇杆位置线程"""
        cmd_header = [0xAA, 0x55]
        cmd_len = [0x03, 0x00]
        cmd_id = [0x01]
        cmd_func = [0x31]
        checksum = [calculate_checksum(cmd_id + cmd_func)]
        cmd_tail = [0x55, 0xAA]
        packet = bytearray(
            cmd_header + cmd_len + cmd_id + cmd_func + checksum + cmd_tail
        )

        try:
            while self.running:
                with self.write_lock:
                    self.ser.write(packet)
                    self.ser.flush()

                time.sleep(0.03)

                buffer, lock = self.buffers["position"]
                with lock:
                    response_buffer = bytes(list(buffer)) if len(buffer) >= 10 else b""
                    buffer.clear()

                if response_buffer and len(response_buffer) >= 10:
                    pkt, _ = find_packet(response_buffer)
                    if pkt and len(pkt) >= 10:
                        x_adc, y_adc = pkt[6], pkt[7]
                        x_pos, y_pos = 128 - x_adc, 128 - y_adc

                        direction, level = get_direction_and_level(x_pos, y_pos)
                        output = f"{direction}-{level}" if direction else "🟢 中心"
                        print(
                            f"\r摇杆: X={x_pos:4d} Y={y_pos:4d}  {output}    ",
                            end="",
                            flush=True,
                        )

                time.sleep(0.05)
        except Exception as e:
            print(f"\n✗ 查询线程错误: {e}")

    def listen_button(self):
        """监听按键事件线程"""
        buffer = b""
        last_event_time = 0

        try:
            while self.running:
                buf_obj, lock = self.buffers["button"]
                with lock:
                    chunk = bytes(list(buf_obj)) if len(buf_obj) > 0 else b""
                    buf_obj.clear()

                buffer += chunk

                while len(buffer) >= 10:
                    packet, buffer = find_packet(buffer)
                    if packet is None:
                        break

                    if packet[5] == 0xE0 and len(packet) >= 10:
                        button_type = packet[6] | (packet[7] << 8)
                        now = time.time()

                        if now - last_event_time > 0.1:
                            print()
                            event = self.button_events.get(
                                button_type, f"⚪ 未知事件: 0x{button_type:04X}"
                            )
                            print(event)
                            last_event_time = now
                            sys.stdout.flush()

                time.sleep(0.001)
        except Exception as e:
            print(f"\n✗ 按键监听线程错误: {e}")

    def run(self):
        """启动监听"""
        if not self.connect():
            return

        print("\n摇杆监听已启动")
        print("按下摇杆按钮查看按键事件，移动摇杆查看位置，Ctrl+C 退出\n")

        try:
            threads = [
                threading.Thread(target=self.read_serial_data, daemon=True),
                threading.Thread(target=self.listen_button, daemon=True),
                threading.Thread(target=self.query_position, daemon=True),
            ]

            for t in threads:
                t.start()
            for t in threads:
                t.join()

        except KeyboardInterrupt:
            print("\n\n程序已退出")
        finally:
            self.running = False
            if self.ser and self.ser.is_open:
                self.ser.close()


if __name__ == "__main__":
    monitor = JoystickMonitor()
    monitor.run()
