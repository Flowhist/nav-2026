#!/usr/bin/env python3
import base64
import math
import queue
import subprocess
import sys
import threading
import time
import zlib
from array import array
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml

from state_store import StateStore


HANDLE_MODULE_DIR = Path(__file__).resolve().parent.parent / "scripts" / "handle"
if str(HANDLE_MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(HANDLE_MODULE_DIR))

from handle_hid import HidMappingConfig, command_from_axes  # noqa: E402
from handle_protocol import gear_scale  # noqa: E402


def load_handle_mapping(config_path: Path) -> HidMappingConfig:
    """Load the same axis shaping and speed limits used by handle_control."""
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        params = raw.get("handle_control", {}).get("ros__parameters", {})
        return HidMappingConfig(
            linear_axis=max(0, int(params.get("linear_axis", 1))),
            angular_axis=max(0, int(params.get("angular_axis", 0))),
            linear_direction=float(params.get("linear_direction", 1.0)),
            angular_direction=float(params.get("angular_direction", 1.0)),
            dead_zone=float(params.get("dead_zone", 0.05)),
            saturation=float(params.get("saturation", 1.0)),
            max_linear_speed=float(params.get("max_linear_speed", 0.6)),
            max_angular_speed=float(params.get("max_angular_speed", 0.5)),
        )
    except (AttributeError, OSError, TypeError, ValueError, yaml.YAMLError):
        return HidMappingConfig()


def manual_drive_command(
    linear: float,
    angular: float,
    gear: Optional[int],
    mapping: HidMappingConfig,
):
    """Convert a normalized Web vector with the physical handle's current gear."""
    scale = gear_scale(gear) if gear is not None else None
    if scale is None:
        return None
    axis_count = max(mapping.linear_axis, mapping.angular_axis, 0) + 1
    axes = [0.0] * axis_count
    axes[mapping.linear_axis] = float(linear)
    axes[mapping.angular_axis] = float(angular)
    return command_from_axes(axes, mapping, scale)


class RosBridge:
    def __init__(self, state_store: StateStore, handle_config_path: Optional[Path] = None) -> None:
        self.state_store = state_store
        config_path = handle_config_path or (Path(__file__).resolve().parent.parent / "config" / "handle.yaml")
        self.handle_mapping = load_handle_mapping(config_path)
        self._cmd_q: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=200)
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def command(self, payload: Dict[str, Any]) -> None:
        try:
            self._cmd_q.put_nowait(payload)
        except queue.Full:
            self.state_store.add_event("warn", "command queue full, drop command", payload)

    def _run(self) -> None:
        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor

            rclpy.init(args=None)
            node = _BridgeNode(self.state_store, self._cmd_q, self)
            executor = MultiThreadedExecutor(num_threads=3)
            executor.add_node(node)

            self.state_store.update_status({"ros": {"connected": True}})
            self.state_store.add_event("info", "ROS bridge started")
            executor.spin()
        except Exception as exc:
            self.state_store.update_status({"ros": {"connected": False}})
            self.state_store.add_event("error", f"ROS bridge failed: {exc}")


class _BridgeNode:
    def __new__(cls, state_store: StateStore, cmd_q: "queue.Queue[Dict[str, Any]]", bridge: RosBridge):
        from rclpy.node import Node

        class BridgeNode(Node):
            pass

        obj = BridgeNode("web_control_server")
        cls._init_bridge(obj, state_store, cmd_q, bridge)
        return obj

    @staticmethod
    def _init_bridge(
        self: Any,
        state_store: StateStore,
        cmd_q: "queue.Queue[Dict[str, Any]]",
        bridge: RosBridge,
    ) -> None:
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from rclpy.qos import (
            QoSProfile,
            ReliabilityPolicy,
            DurabilityPolicy,
            HistoryPolicy,
        )
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool, Empty, String, UInt16
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, TransformListener

        self.state_store = state_store
        self.cmd_q = cmd_q
        self.bridge = bridge

        self.PoseStamped = PoseStamped
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.Twist = Twist
        self.Empty = Empty
        self.String = String
        self.handle_mapping = bridge.handle_mapping

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.pub_web_cmd = self.create_publisher(Twist, "/web_cmd_vel", 10)
        self.pub_initial = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.pub_nav_clear = self.create_publisher(Empty, "/nav_clear", 10)
        self.pub_voice = self.create_publisher(String, "/nav_voice_bridge/voice_command", 10)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", lambda m: _BridgeNode._on_map(self, m), map_qos)
        self.create_subscription(Odometry, "/odom", lambda m: _BridgeNode._on_odom(self, m), 20)
        self.create_subscription(Path, "/plan", lambda m: _BridgeNode._on_plan(self, m), 10)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(LaserScan, "/scan", lambda m: _BridgeNode._on_scan(self, m), scan_qos)

        self.create_subscription(Bool, "/js_state", lambda m: _BridgeNode._on_js_state(self, m), 10)
        self.create_subscription(Twist, "/js_cmd_vel", lambda m: _BridgeNode._on_js_cmd(self, m), 10)
        self.create_subscription(UInt16, "/handle/gear", lambda m: _BridgeNode._on_gear(self, m), 10)
        self.create_subscription(
            String, "/base_control_router/status", lambda m: _BridgeNode._on_router_status(self, m), 10
        )
        self.create_subscription(TFMessage, "/tf", lambda m: _BridgeNode._on_tf(self, m), 50)
        self.create_subscription(
            String, "/nav_voice_bridge/status", lambda m: _BridgeNode._on_voice_status(self, m), 10
        )

        self._counts = {
            "odom": 0,
            "plan": 0,
            "scan": 0,
            "tf_map_odom": 0,
            "tf_odom_base": 0,
        }
        self._last_tick = time.monotonic()
        self._last_seen: Dict[str, float] = {}
        self._last_map_signature: Optional[Tuple[object, ...]] = None
        self._next_web_scan_at = 0.0

        self._joystick_online = False
        self._joystick_active = False
        self._handle_gear: Optional[int] = None

        self.create_timer(0.02, lambda: _BridgeNode._process_commands(self))
        self.create_timer(0.25, lambda: _BridgeNode._refresh_pose_map(self))
        self.create_timer(1.0, lambda: _BridgeNode._publish_metrics(self))

    @staticmethod
    def _event(self: Any, level: str, message: str, extra: Optional[Dict[str, Any]] = None) -> None:
        self.state_store.add_event(level, message, extra)

    @staticmethod
    def _touch(self: Any, key: str) -> None:
        self._last_seen[key] = time.time()

    @staticmethod
    def _yaw_from_quat(q: Any) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _pose_dict(x: float, y: float, yaw_rad: float, frame_id: str = "map") -> Dict[str, Any]:
        return {
            "x": round(float(x), 4),
            "y": round(float(y), 4),
            "yaw_deg": round(math.degrees(yaw_rad), 2),
            "frame_id": frame_id,
            "updated_at": time.time(),
        }

    @staticmethod
    def _on_map(self: Any, msg: Any) -> None:
        _BridgeNode._touch(self, "map")

        try:
            raw_map = bytes(msg.data)
        except (TypeError, ValueError):
            raw_map = array("b", (int(value) for value in msg.data)).tobytes()
        origin_yaw = round(math.degrees(_BridgeNode._yaw_from_quat(msg.info.origin.orientation)), 2)
        signature = (
            int(msg.info.width),
            int(msg.info.height),
            round(float(msg.info.resolution), 9),
            round(float(msg.info.origin.position.x), 6),
            round(float(msg.info.origin.position.y), 6),
            origin_yaw,
            zlib.crc32(raw_map),
        )
        if self._last_map_signature == signature:
            return
        self._last_map_signature = signature

        self.state_store.update_scene(
            {
                "map": {
                    "source": "live",
                    "frame_id": msg.header.frame_id or "map",
                    "width": int(msg.info.width),
                    "height": int(msg.info.height),
                    "resolution": float(msg.info.resolution),
                    "origin": {
                        "x": float(msg.info.origin.position.x),
                        "y": float(msg.info.origin.position.y),
                        "yaw_deg": origin_yaw,
                    },
                    "updated_at": time.time(),
                    "encoding": "int8-base64",
                    "data_b64": base64.b64encode(raw_map).decode("ascii"),
                }
            },
            map_changed=True,
        )

    @staticmethod
    def _on_odom(self: Any, msg: Any) -> None:
        self._counts["odom"] += 1
        _BridgeNode._touch(self, "odom")
        p = msg.pose.pose.position
        yaw = _BridgeNode._yaw_from_quat(msg.pose.pose.orientation)

        self.state_store.update_status(
            {
                "robot": {
                    "pose_odom": {"x": float(p.x), "y": float(p.y), "yaw_deg": math.degrees(yaw)},
                    "velocity": {
                        "vx": float(msg.twist.twist.linear.x),
                        "wz": float(msg.twist.twist.angular.z),
                    },
                }
            }
        )

    @staticmethod
    def _on_plan(self: Any, msg: Any) -> None:
        self._counts["plan"] += 1
        _BridgeNode._touch(self, "plan")
        points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
        length = 0.0
        for i in range(1, len(points)):
            length += math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])

        plan_status = {
            "points": len(points),
            "length_m": round(length, 3),
            "updated_at": time.time(),
        }
        self.state_store.update_status({"robot": {"plan": plan_status}})
        self.state_store.update_scene(
            {"plan": {**plan_status, "points_xy": [[x, y] for x, y in points]}},
            plan_changed=True,
        )

    @staticmethod
    def _lookup_pose_in_map(self: Any, frame_id: str) -> Optional[Tuple[float, float, float]]:
        from rclpy.time import Time
        from tf2_ros import TransformException

        try:
            tf = self.tf_buffer.lookup_transform("map", frame_id, Time())
        except TransformException:
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        yaw = _BridgeNode._yaw_from_quat(r)
        return float(t.x), float(t.y), float(yaw)

    @staticmethod
    def _on_scan(self: Any, msg: Any) -> None:
        self._counts["scan"] += 1
        _BridgeNode._touch(self, "scan")

        # Web visualization is the only consumer of this transformed payload.
        # Keep topic metrics alive, but avoid TF work and allocations when no
        # mapping/navigation page has an active stream.
        stream_hz = self.state_store.live_stream_hz()
        if stream_hz <= 0:
            return
        now = time.monotonic()
        interval = 1.0 / stream_hz
        if now - self._next_web_scan_at > interval:
            self._next_web_scan_at = now
        if now < self._next_web_scan_at:
            return
        self._next_web_scan_at += interval

        pose_in_map = _BridgeNode._lookup_pose_in_map(self, msg.header.frame_id or "base_link")
        step = max(1, len(msg.ranges) // 720)
        packed_ranges = array("H")
        for i in range(0, len(msg.ranges), step):
            distance = float(msg.ranges[i])
            if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
                packed_ranges.append(0)
            else:
                packed_ranges.append(max(1, min(65535, int(round(distance * 1000.0)))))
        if sys.byteorder != "little":
            packed_ranges.byteswap()

        pose_payload = _BridgeNode._pose_dict(*pose_in_map) if pose_in_map is not None else None
        if pose_payload is not None:
            self.state_store.update_status({"robot": {"pose_map": pose_payload}})

        self.state_store.update_scene(
            {
                "scan": {
                    "frame_id": msg.header.frame_id or "base_link",
                    "updated_at": time.time(),
                    "pose_map": pose_payload,
                    "encoding": "uint16-mm-base64",
                    "angle_min": float(msg.angle_min),
                    "angle_increment": float(msg.angle_increment) * step,
                    "count": len(packed_ranges),
                    "ranges_b64": base64.b64encode(packed_ranges.tobytes()).decode("ascii"),
                },
                "robot_pose_map": pose_payload,
            }
        )

    @staticmethod
    def _on_tf(self: Any, msg: Any) -> None:
        for t in msg.transforms:
            parent = t.header.frame_id
            child = t.child_frame_id
            if parent == "map" and child == "odom":
                self._counts["tf_map_odom"] += 1
                _BridgeNode._touch(self, "tf_map_odom")
            elif parent == "odom" and child == "base_link":
                self._counts["tf_odom_base"] += 1
                _BridgeNode._touch(self, "tf_odom_base_link")

    @staticmethod
    def _on_js_state(self: Any, msg: Any) -> None:
        self._joystick_online = bool(msg.data)
        _BridgeNode._touch(self, "js_state")

    @staticmethod
    def _on_js_cmd(self: Any, msg: Any) -> None:
        self._joystick_active = abs(float(msg.linear.x)) > 1e-6 or abs(float(msg.angular.z)) > 1e-6
        _BridgeNode._touch(self, "js_cmd")

    @staticmethod
    def _on_gear(self: Any, msg: Any) -> None:
        gear = int(msg.data)
        if gear_scale(gear) is None:
            return
        self._handle_gear = gear
        _BridgeNode._touch(self, "handle_gear")

    @staticmethod
    def _on_router_status(self: Any, msg: Any) -> None:
        status = str(msg.data).strip()
        patch: Dict[str, Any] = {}
        if (
            status.startswith("joystick_stop:")
            or status.startswith("joystick_reset_required:")
            or status in {"web_reset_required", "base_fault_active"}
        ):
            patch = {"manual_locked": True, "manual_lock_reason": status}
        elif status in {"joystick_reset", "web_reset", "base_fault_cleared"}:
            patch = {"manual_locked": False, "manual_lock_reason": None}
        if patch:
            self.state_store.update_status({"control": patch})

    @staticmethod
    def _refresh_pose_map(self: Any) -> None:
        pose = _BridgeNode._lookup_pose_in_map(self, "base_link")
        if pose is None:
            return
        pose_payload = _BridgeNode._pose_dict(*pose)
        self.state_store.update_status({"robot": {"pose_map": pose_payload}})
        self.state_store.update_scene({"robot_pose_map": pose_payload})

    @staticmethod
    def _publish_web_stop(self: Any) -> None:
        msg = self.Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_web_cmd.publish(msg)

    @staticmethod
    def _publish_manual_drive(self: Any, linear: float, angular: float) -> None:
        gear = self._handle_gear
        gear_age = time.time() - self._last_seen.get("handle_gear", 0.0)
        command = manual_drive_command(
            linear,
            angular,
            gear if gear_age <= 2.0 else None,
            self.handle_mapping,
        )
        if command is None:
            _BridgeNode._publish_web_stop(self)
            return
        msg = self.Twist()
        msg.linear.x = command.linear
        msg.angular.z = command.angular
        self.pub_web_cmd.publish(msg)

    @staticmethod
    def _update_target_state(self: Any, key: str, x: float, y: float, yaw_deg: float) -> None:
        status_key = "goal_pose" if key == "goal_pose" else "initial_pose"
        target = {
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw_deg": round(yaw_deg, 2),
            "frame_id": "map",
            "updated_at": time.time(),
        }
        self.state_store.update_status({"robot": {status_key: target}})
        self.state_store.update_scene({key: target})

    @staticmethod
    def _process_commands(self: Any) -> None:
        while True:
            try:
                cmd = self.cmd_q.get_nowait()
            except queue.Empty:
                break

            _BridgeNode._handle_command(self, cmd)

    @staticmethod
    def _handle_command(self: Any, cmd: Dict[str, Any]) -> None:
        ctype = cmd.get("type", "")
        if ctype == "manual_drive":
            _BridgeNode._publish_manual_drive(
                self,
                float(cmd.get("linear", 0.0)),
                float(cmd.get("angular", 0.0)),
            )

        elif ctype == "set_goal":
            x = float(cmd.get("x", 0.0))
            y = float(cmd.get("y", 0.0))
            yaw_deg = float(cmd.get("yaw_deg", 0.0))
            yaw = math.radians(yaw_deg)

            msg = self.PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.z = math.sin(yaw * 0.5)
            msg.pose.orientation.w = math.cos(yaw * 0.5)
            self.pub_goal.publish(msg)
            _BridgeNode._update_target_state(self, "goal_pose", x, y, yaw_deg)
            _BridgeNode._event(self, "info", "goal published", {"x": x, "y": y, "yaw_deg": yaw_deg})

        elif ctype == "set_initialpose":
            x = float(cmd.get("x", 0.0))
            y = float(cmd.get("y", 0.0))
            yaw_deg = float(cmd.get("yaw_deg", 0.0))
            yaw = math.radians(yaw_deg)

            msg = self.PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
            msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.0685
            self.pub_initial.publish(msg)
            self.state_store.update_status({"robot": {"initial_pose": None}})
            self.state_store.update_scene({"initial_pose": None})
            _BridgeNode._event(self, "info", "initialpose published", {"x": x, "y": y, "yaw_deg": yaw_deg})

        elif ctype == "save_map":
            name = str(cmd.get("name", "manual_map")).strip() or "manual_map"
            _BridgeNode._save_map(self, name)

        elif ctype == "cancel_nav":
            _BridgeNode._publish_web_stop(self)
            self.pub_nav_clear.publish(self.Empty())
            self.state_store.update_status({"robot": {"goal_pose": None, "plan": {"points": 0, "length_m": 0.0, "updated_at": time.time()}}})
            self.state_store.update_scene(
                {"goal_pose": None, "plan": {"points": 0, "length_m": 0.0, "points_xy": [], "updated_at": time.time()}},
                plan_changed=True,
            )
            _BridgeNode._event(self, "info", "navigation state cleared and robot stopped")

        elif ctype == "nav_to_location":
            name = str(cmd.get("name", "")).strip()
            if not name:
                _BridgeNode._event(self, "warn", "nav_to_location ignored: empty name")
                return
            msg = self.String()
            msg.data = name
            self.pub_voice.publish(msg)
            self.state_store.update_status({"nav_voice": {"command": name, "updated_at": time.time()}})
            _BridgeNode._event(self, "info", "location command published", {"name": name})

    @staticmethod
    def _on_voice_status(self: Any, msg: Any) -> None:
        text = str(getattr(msg, "data", "")).strip()
        if not text:
            return
        self.state_store.update_status({"nav_voice": {"status": text, "updated_at": time.time()}})
        _BridgeNode._event(self, "info", "nav_voice status", {"status": text})

    @staticmethod
    def _publish_metrics(self: Any) -> None:
        now = time.monotonic()
        dt = max(0.2, now - self._last_tick)
        self._last_tick = now

        tf_map = self._counts["tf_map_odom"] / dt
        tf_ob = self._counts["tf_odom_base"] / dt
        hz_odom = self._counts["odom"] / dt
        hz_plan = self._counts["plan"] / dt
        hz_scan = self._counts["scan"] / dt

        for k in self._counts:
            self._counts[k] = 0

        now_wall = time.time()
        last_seen_age = {}
        for k, ts in self._last_seen.items():
            last_seen_age[k] = round(now_wall - ts, 3)
        js_age = last_seen_age.get("js_state")
        js_cmd_age = last_seen_age.get("js_cmd")
        gear_age = last_seen_age.get("handle_gear")
        joystick_online = self._joystick_online and js_age is not None and js_age <= 2.0
        joystick_active = self._joystick_active and js_cmd_age is not None and js_cmd_age <= 0.5
        gear_online = self._handle_gear is not None and gear_age is not None and gear_age <= 2.0

        self.state_store.update_status(
            {
                "control": {
                    "joystick_active": joystick_active,
                    "joystick_online": joystick_online,
                    "joystick_updated_at": self._last_seen.get("js_state"),
                    "gear": self._handle_gear if gear_online else None,
                    "gear_online": gear_online,
                    "gear_updated_at": self._last_seen.get("handle_gear"),
                },
                "ros": {
                    "tf_hz": {
                        "map_odom": round(tf_map, 2),
                        "odom_base_link": round(tf_ob, 2),
                    },
                    "topic_hz": {
                        "odom": round(hz_odom, 2),
                        "plan": round(hz_plan, 2),
                        "scan": round(hz_scan, 2),
                    },
                    "last_seen": last_seen_age,
                }
            }
        )

    @staticmethod
    def _save_map(self: Any, name: str) -> None:
        repo_root = Path(__file__).resolve().parent.parent
        script = repo_root / "scripts" / "tool" / "save_map.sh"
        if not script.exists():
            _BridgeNode._event(self, "error", f"map save script not found: {script}")
            return

        try:
            subprocess.Popen(
                ["bash", str(script), name],
                cwd=str(repo_root),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            _BridgeNode._event(self, "info", "map save requested", {"name": name})
        except Exception as exc:
            _BridgeNode._event(self, "error", f"map save request failed: {exc}", {"name": name})
