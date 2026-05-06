# -*- coding: utf-8 -*-
"""
DM_Serial: 达妙 IMU 串口读取类
支持后台线程读取、多帧类型解析（加速度、角速度、欧拉角、四元数）
"""
from __future__ import annotations

import struct
import threading
import time
from typing import Optional, Tuple, List

import serial  # pip install pyserial

from .dm_crc import dm_crc16

# 帧格式常量
HDR, TAIL = b"\x55\xaa", 0x0A
FRAME_LEN_3AXIS, FRAME_LEN_QUAT = 19, 23
RID_ACCEL, RID_GYRO, RID_EULER, RID_QUAT = 0x01, 0x02, 0x03, 0x04
VALID_RIDS = {RID_ACCEL, RID_GYRO, RID_EULER, RID_QUAT}
SKIP_HDR_IN_CRC = False


class DM_Serial:
    def __init__(self, port: str, baudrate: int):
        self.port, self.baudrate, self.timeout = port, int(baudrate), 0.0
        self.ser: Optional[serial.Serial] = None
        self._buf = bytearray()

        # 统计信息
        self.cnt_ok = self.cnt_crc = self.cnt_short = self.cnt_nohdr = 0

        # 后台线程
        self._th: Optional[threading.Thread] = None
        self._stop_evt: Optional[threading.Event] = None
        self._read_sleep = 0.001

        # 最新数据（线程安全）
        self._latest_lock = threading.Lock()
        self._latest_accel: Optional[Tuple[float, float, float]] = None
        self._latest_gyro: Optional[Tuple[float, float, float]] = None
        self._latest_euler: Optional[Tuple[float, float, float]] = None
        self._latest_quat: Optional[Tuple[float, float, float, float]] = None
        self._latest_ts, self._latest_count = 0.0, 0
        self._last_error: Optional[str] = None

        self._open()

    # ------------ 公共 API ------------
    def read(self, max_bytes: int | None = None) -> bool:
        """一次性读入串口当前可读字节，解析所有完整帧，更新内部数据缓存。返回是否有新数据。"""
        if not self.ser or not self.ser.is_open:
            return False
        self._read_into_buf(max_bytes)
        frames = self._parse_all()
        return len(frames) > 0

    def start_reader(self, read_sleep: float = 0.001) -> bool:
        """启动只负责刷新数据的后台线程；不打印。"""
        if self._th and self._th.is_alive():
            self._read_sleep = read_sleep
            return True
        if not self.is_open:
            if not self._open():
                return False
        self._stop_evt = threading.Event()
        self._read_sleep = read_sleep
        self._th = threading.Thread(target=self._reader_loop, daemon=True)
        self._th.start()
        return True

    def stop_reader(self) -> None:
        """停止后台读线程。"""
        if self._stop_evt:
            self._stop_evt.set()
        if self._th:
            self._th.join(timeout=1.0)
        self._th = None
        self._stop_evt = None

    def get_latest(self) -> dict:
        """获取最新的传感器数据（线程安全）。"""
        with self._latest_lock:
            return {
                "accel": self._latest_accel,
                "gyro": self._latest_gyro,
                "euler": self._latest_euler,
                "quat": self._latest_quat,
                "timestamp": self._latest_ts,
                "count": self._latest_count,
            }

    def last_error(self) -> Optional[str]:
        return self._last_error

    def destory(self) -> None:
        """立即关闭串口（按你的拼写保留）。"""
        self.stop_reader()
        if self.ser:
            try:
                self.ser.close()
            finally:
                self.ser = None

    # 别名
    def destroy(self) -> None:
        self.destory()

    def reopen(self) -> bool:
        """关闭并重新打开串口。"""
        self.destory()
        return self._open()

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    # ------------ 内部实现 ------------
    def _open(self) -> bool:
        try:
            self.ser = serial.Serial(
                self.port, self.baudrate, timeout=self.timeout, write_timeout=0
            )
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            return True
        except Exception as e:
            self._last_error = str(e)
            self.ser = None
            return False

    def _reader_loop(self):
        """后台线程：持续读取数据。"""
        evt = self._stop_evt
        try:
            while evt and not evt.is_set():
                has_data = self.read(None)
                if has_data:
                    with self._latest_lock:
                        self._latest_ts = time.time()
                        self._latest_count += 1
                if self._read_sleep > 0.0:
                    time.sleep(self._read_sleep)
        except Exception as e:
            self._last_error = f"reader_loop: {e!r}"

    def _read_into_buf(self, max_bytes: Optional[int]) -> int:
        """把串口里“当前可读”的字节读入缓冲；返回读取字节数。"""
        n = getattr(self.ser, "in_waiting", 0) if self.ser else 0
        if max_bytes is not None and n > max_bytes:
            n = max_bytes
        if n <= 0:
            return 0
        self._buf.extend(self.ser.read(n))
        return n

    def _parse_all(self) -> List[Tuple[int, Tuple]]:
        """解析尽可能多的完整帧，更新内部数据缓存，返回解析成功的帧列表。"""
        results: List[Tuple[int, Tuple]] = []
        buf = self._buf
        start = 0

        while True:
            j = buf.find(HDR, start)
            if j < 0:
                # 只保留最后 1 个字节，避免帧头跨包
                keep = buf[-1:] if buf else b""
                self._buf = bytearray(keep)
                if buf:
                    self.cnt_nohdr += 1
                break

            # 先读取 RID 来判断帧长度
            if len(buf) - j < 4:  # 至少需要帧头(2) + ?(1) + RID(1)
                self._buf = bytearray(buf[j:])
                self.cnt_short += 1
                break

            rid = buf[j + 3]
            if rid not in VALID_RIDS:
                start = j + 1
                continue

            # 根据 RID 确定帧长度
            if rid == RID_QUAT:
                frame_len = FRAME_LEN_QUAT
            else:
                frame_len = FRAME_LEN_3AXIS

            if len(buf) - j < frame_len:
                # 不够一帧，保留从帧头开始的尾部
                self._buf = bytearray(buf[j:])
                self.cnt_short += 1
                break

            frame = bytes(buf[j : j + frame_len])
            start = j + 1  # 快速向前移动

            # 尾字节检查
            if frame[-1] != TAIL:
                continue

            # CRC（默认含帧头，失败再试不含帧头）
            if rid == RID_QUAT:
                # 四元数帧: 4个float = 16字节数据
                data_end = 20
            else:
                # 三轴数据帧: 3个float = 12字节数据
                data_end = 16

            if SKIP_HDR_IN_CRC:
                crc_calc = dm_crc16(frame[2:data_end])
            else:
                crc_calc = dm_crc16(frame[0:data_end])

            crc_wire = frame[data_end] | (frame[data_end + 1] << 8)
            if crc_calc != crc_wire:
                alt = (
                    dm_crc16(frame[2:data_end])
                    if not SKIP_HDR_IN_CRC
                    else dm_crc16(frame[0:data_end])
                )
                if alt != crc_wire:
                    self.cnt_crc += 1
                    continue

            # 根据 RID 解析不同类型的数据
            if rid == RID_QUAT:
                # 四元数：W, X, Y, Z
                w = struct.unpack("<f", frame[4:8])[0]
                x = struct.unpack("<f", frame[8:12])[0]
                y = struct.unpack("<f", frame[12:16])[0]
                z = struct.unpack("<f", frame[16:20])[0]
                data = (w, x, y, z)
                with self._latest_lock:
                    self._latest_quat = data
            else:
                # 三轴数据：X, Y, Z
                x = struct.unpack("<f", frame[4:8])[0]
                y = struct.unpack("<f", frame[8:12])[0]
                z = struct.unpack("<f", frame[12:16])[0]
                data = (x, y, z)

                with self._latest_lock:
                    if rid == RID_ACCEL:
                        self._latest_accel = data
                    elif rid == RID_GYRO:
                        self._latest_gyro = data
                    elif rid == RID_EULER:
                        self._latest_euler = data

            results.append((rid, data))

            # 丢弃已消费的数据（到帧尾），并从头继续找
            buf = buf[j + frame_len :]
            start = 0

        if isinstance(buf, (bytes, bytearray)) and buf is not self._buf:
            self._buf = bytearray(buf)

        self.cnt_ok += len(results)
        return results
