#!/usr/bin/env python3
"""
CAN 连接生命周期管理：重连、错误计数、看门狗、状态上报。

与电机控制逻辑完全解耦，同一进程内组合使用（零 IPC 开销）。

用法：:

    from can_connection import CanConnectionManager

    class MyDriver(Node):
        def __init__(self):
            super().__init__("my_driver")
            self.can_mgr = CanConnectionManager(
                self,
                connect_fn=self._try_connect_driver,
                on_connected=self._on_can_up,
                on_disconnected=self._on_can_down,
            )
            self.can_mgr.start()

        def _try_connect_driver(self):
            whill = Driver()
            whill.load(...)
            return whill

        def _on_can_up(self, driver):
            with self._lock:
                self._driver = driver

        def _on_can_down(self):
            pass  # CAN 断开，停发指令

        def _send(self):
            if self.can_mgr.connected:
                self._driver.move_velocity(...)
            else:
                self.can_mgr.mark_error()
"""

import json
import threading
import time
from typing import Any, Callable, Optional

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String


class CanConnectionManager:
    """CAN 连接生命周期管理。

    通过组合方式嵌入底盘驱动节点，接管：
      - 首次连接 & 断线自动重连
      - 错误计数 + 阈值判定（避免偶发干扰误判）
      - 1Hz 看门狗兜底检测
      - /base_status 状态发布
    """

    def __init__(
        self,
        node: Node,
        *,
        connect_fn: Callable[[], Optional[Any]],
        on_connected: Callable[[Any], None],
        on_disconnected: Callable[[], None],
        error_threshold: int = 3,
        lost_warn_sec: float = 3.0,
        retry_delay_sec: float = 5.0,
        connect_tag: str = "CAN",
    ):
        """
        Args:
            node:                ROS 2 Node（用于 logger、timer、publisher）。
            connect_fn:          无参 → 返回 driver 对象；失败返回 None。
            on_connected:        (driver) → 存储 driver 引用。
            on_disconnected:     无参 → CAN 断开时回调（停发指令等）。
            error_threshold:     连续 I/O 失败多少次才判定断开。
            lost_warn_sec:       断开超过此秒数开始告警。
            retry_delay_sec:     重连失败后等待秒数。
            connect_tag:         日志前缀标识。
        """
        self._node = node
        self._logger = node.get_logger()
        self._connect_fn = connect_fn
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected
        self._error_threshold = max(1, int(error_threshold))
        self._lost_warn_sec = max(1.0, float(lost_warn_sec))
        self._retry_delay_sec = max(1.0, float(retry_delay_sec))
        self._tag = str(connect_tag)

        # ── 状态 ──
        self._connected = False
        self._running = True
        self._reconnecting = False
        self._reconnect_lock = threading.Lock()
        self._errors = 0
        self._lost_at: Optional[float] = None
        self._recovered_at: Optional[float] = None

        # ── 状态发布 ──
        self._status_pub = node.create_publisher(
            String, "/base_status", 10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # ── 看门狗定时器 ──
        self._watchdog_timer = node.create_timer(
            1.0, self._watchdog,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._publish_status()

    # ── 公开 API ────────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self._connected

    def start(self):
        """启动首次连接。"""
        self._start_reconnect_thread()

    def mark_error(self):
        """上报一次 CAN I/O 错误；达阈值时触发断开 + 自动重连。"""
        self._errors += 1
        if self._errors >= self._error_threshold and self._connected:
            self._logger.error(
                f"✗ {self._tag} 通信连续失败 {self._errors} 次，标记断开"
            )
            self._connected = False
            self._lost_at = time.monotonic()
            self._on_disconnected()
            self._publish_status()
            self._start_reconnect_thread()

    def on_success(self):
        """I/O 成功后调用，清零错误计数器（避免误判）。"""
        self._errors = 0

    def shutdown(self):
        """清理：停止重连、发布下线状态。"""
        self._running = False
        self._connected = False
        try:
            msg = String()
            msg.data = json.dumps({"connected": False, "shutdown": True})
            self._status_pub.publish(msg)
        except Exception:
            pass

    # ── 内部实现 ────────────────────────────────────────────────────────

    def _start_reconnect_thread(self):
        """启动重连线程（幂等：已在线则忽略）。"""
        with self._reconnect_lock:
            if self._reconnecting:
                return
            self._reconnecting = True
        t = threading.Thread(target=self._reconnect_loop, daemon=True)
        t.start()

    def _reconnect_loop(self):
        """重连循环：阻塞线程，反复尝试直到成功或 shutdown。"""
        try:
            while self._running and not self._connected:
                self._logger.info(f"正在连接 {self._tag}...")
                driver = None
                try:
                    driver = self._connect_fn()
                except Exception as e:
                    self._logger.warn(
                        f"重连失败: {e}，{self._retry_delay_sec:.0f} 秒后重试..."
                    )

                if driver is not None:
                    self._on_connected(driver)
                    self._connected = True
                    self._errors = 0
                    self._lost_at = None
                    self._recovered_at = time.monotonic()
                    self._logger.warn(f"✓✓✓ {self._tag} 已恢复，重新上线")
                    self._publish_status()
                else:
                    time.sleep(self._retry_delay_sec)
        finally:
            with self._reconnect_lock:
                self._reconnecting = False

    def _watchdog(self):
        """1 Hz 看门狗：兜底触发重连 + 长时间断开告警。"""
        if self._connected:
            return
        # 兜底：若意外未被 mark_error 触发，看门狗补一次重连
        with self._reconnect_lock:
            if not self._reconnecting:
                self._reconnecting = True
                need_reconnect = True
            else:
                need_reconnect = False
        if need_reconnect:
            t = threading.Thread(target=self._reconnect_loop, daemon=True)
            t.start()

        if self._lost_at is not None:
            elapsed = time.monotonic() - self._lost_at
            if elapsed > self._lost_warn_sec:
                self._logger.warn(
                    f"⚠ {self._tag} 已断开 {elapsed:.0f}s，请检查急停按钮或线缆",
                    throttle_duration_sec=3.0,
                )
        self._publish_status()

    def _publish_status(self):
        """发布 /base_status（JSON）。"""
        try:
            msg = String()
            msg.data = json.dumps({
                "connected": self._connected,
                "can_lost_at": self._lost_at,
                "can_recovered_at": self._recovered_at,
                "reconnecting": self._reconnecting,
                "can_errors": self._errors,
            })
            self._status_pub.publish(msg)
        except Exception:
            pass
