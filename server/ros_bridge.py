#!/usr/bin/env python3
import math
import queue
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from state_store import StateStore


class RosBridge:
    def __init__(self, state_store: StateStore) -> None:
        self.state_store = state_store
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
            node = _BridgeNode(self.state_store, self._cmd_q)
            executor = MultiThreadedExecutor(num_threads=3)
            executor.add_node(node)

            self.state_store.update_status({"ros": {"connected": True}})
            self.state_store.add_event("info", "ROS bridge started")
            executor.spin()
        except Exception as exc:
            self.state_store.update_status({"ros": {"connected": False}})
            self.state_store.add_event("error", f"ROS bridge failed: {exc}")


class _BridgeNode:
    def __new__(cls, state_store: StateStore, cmd_q: "queue.Queue[Dict[str, Any]]"):
        from rclpy.node import Node

        class BridgeNode(Node):
            pass

        obj = BridgeNode("web_control_server")
        cls._init_bridge(obj, state_store, cmd_q)
        return obj

    @staticmethod
    def _init_bridge(self: Any, state_store: StateStore, cmd_q: "queue.Queue[Dict[str, Any]]") -> None:
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool, Empty
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, TransformListener

        self.state_store = state_store
        self.cmd_q = cmd_q

        self.PoseStamped = PoseStamped
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.Twist = Twist
        self.Empty = Empty

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_initial = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.pub_nav_clear = self.create_publisher(Empty, "/nav_clear", 10)

        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", lambda m: _BridgeNode._on_map(self, m), map_qos)
        self.create_subscription(Odometry, "/odom", lambda m: _BridgeNode._on_odom(self, m), 20)
        self.create_subscription(Path, "/plan", lambda m: _BridgeNode._on_plan(self, m), 10)
        self.create_subscription(LaserScan, "/scan_filtered", lambda m: _BridgeNode._on_scan(self, m), 10)
        self.create_subscription(Bool, "/js_state", lambda m: _BridgeNode._on_js_state(self, m), 10)
        self.create_subscription(TFMessage, "/tf", lambda m: _BridgeNode._on_tf(self, m), 50)

        self._counts = {
            "odom": 0,
            "plan": 0,
            "scan": 0,
            "tf_map_odom": 0,
            "tf_odom_base": 0,
        }
        self._last_tick = time.monotonic()
        self._last_seen: Dict[str, float] = {}
        self._last_map_stamp: Optional[Tuple[int, int]] = None

        self._teleop_timeout_at: Optional[float] = None
        self._teleop_active = False
        self._joystick_active = False

        self.create_timer(0.05, lambda: _BridgeNode._process_commands(self))
        self.create_timer(0.1, lambda: _BridgeNode._teleop_guard(self))
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
        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if self._last_map_stamp == stamp:
            return
        self._last_map_stamp = stamp
        _BridgeNode._touch(self, "map")

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
                        "yaw_deg": round(math.degrees(_BridgeNode._yaw_from_quat(msg.info.origin.orientation)), 2),
                    },
                    "updated_at": time.time(),
                    "data": [int(x) for x in msg.data],
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
        self.state_store.update_scene({"plan": {**plan_status, "points_xy": [[x, y] for x, y in points]}})

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
        _BridgeNode._touch(self, "scan_filtered")

        pose_in_map = _BridgeNode._lookup_pose_in_map(self, msg.header.frame_id or "base_link")
        points: List[List[float]] = []

        if pose_in_map is not None:
            base_x, base_y, base_yaw = pose_in_map
            cos_yaw = math.cos(base_yaw)
            sin_yaw = math.sin(base_yaw)
            step = max(1, len(msg.ranges) // 720)
            for i in range(0, len(msg.ranges), step):
                r = float(msg.ranges[i])
                if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                    continue
                angle = float(msg.angle_min + i * msg.angle_increment)
                local_x = r * math.cos(angle)
                local_y = r * math.sin(angle)
                world_x = base_x + local_x * cos_yaw - local_y * sin_yaw
                world_y = base_y + local_x * sin_yaw + local_y * cos_yaw
                points.append([round(world_x, 4), round(world_y, 4)])

        self.state_store.update_scene(
            {
                "scan": {
                    "frame_id": "map",
                    "updated_at": time.time(),
                    "pose_map": _BridgeNode._pose_dict(*pose_in_map) if pose_in_map is not None else None,
                    "points": points,
                }
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
        self._joystick_active = bool(msg.data)
        _BridgeNode._touch(self, "js_state")
        self.state_store.update_status(
            {
                "teleop": {
                    "joystick_active": self._joystick_active,
                    "joystick_online": True,
                    "joystick_updated_at": time.time(),
                }
            }
        )

    @staticmethod
    def _refresh_pose_map(self: Any) -> None:
        pose = _BridgeNode._lookup_pose_in_map(self, "base_link")
        if pose is None:
            return
        pose_payload = _BridgeNode._pose_dict(*pose)
        self.state_store.update_status({"robot": {"pose_map": pose_payload}})
        self.state_store.update_scene({"robot_pose_map": pose_payload})

    @staticmethod
    def _publish_stop(self: Any) -> None:
        msg = self.Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)

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

            ctype = cmd.get("type", "")
            if ctype == "set_cmd_vel":
                lin = float(cmd.get("linear_x", 0.0))
                ang = float(cmd.get("angular_z", 0.0))
                timeout_ms = int(cmd.get("timeout_ms", 300))

                msg = self.Twist()
                msg.linear.x = lin
                msg.angular.z = ang
                self.pub_cmd.publish(msg)

                self._teleop_active = abs(lin) > 1e-6 or abs(ang) > 1e-6
                if self._teleop_active:
                    self._teleop_timeout_at = time.monotonic() + max(100, timeout_ms) / 1000.0
                else:
                    self._teleop_timeout_at = None

                self.state_store.update_status(
                    {
                        "teleop": {
                            "active": self._teleop_active,
                            "timeout_at": time.time() + max(100, timeout_ms) / 1000.0 if self._teleop_active else None,
                        }
                    }
                )

            elif ctype == "stop":
                _BridgeNode._publish_stop(self)
                self._teleop_active = False
                self._teleop_timeout_at = None
                self.state_store.update_status({"teleop": {"active": False, "timeout_at": None}})
                _BridgeNode._event(self, "info", "robot stopped")

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
                _BridgeNode._update_target_state(self, "initial_pose", x, y, yaw_deg)
                _BridgeNode._event(self, "info", "initialpose published", {"x": x, "y": y, "yaw_deg": yaw_deg})

            elif ctype == "save_map":
                name = str(cmd.get("name", "manual_map")).strip() or "manual_map"
                _BridgeNode._save_map(self, name)

            elif ctype == "cancel_nav":
                _BridgeNode._publish_stop(self)
                self.pub_nav_clear.publish(self.Empty())
                self.state_store.update_status({"robot": {"goal_pose": None, "plan": {"points": 0, "length_m": 0.0, "updated_at": time.time()}}})
                self.state_store.update_scene({"goal_pose": None, "plan": {"points": 0, "length_m": 0.0, "points_xy": [], "updated_at": time.time()}})
                _BridgeNode._event(self, "info", "navigation state cleared and robot stopped")

    @staticmethod
    def _teleop_guard(self: Any) -> None:
        if not self._teleop_active:
            return
        if self._teleop_timeout_at is None:
            return
        if time.monotonic() < self._teleop_timeout_at:
            return

        _BridgeNode._publish_stop(self)
        self._teleop_active = False
        self._teleop_timeout_at = None
        self.state_store.update_status({"teleop": {"active": False, "timeout_at": None}})
        _BridgeNode._event(self, "info", "teleop timeout stop")

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
        joystick_online = js_age is not None and js_age <= 2.0

        self.state_store.update_status(
            {
                "teleop": {
                    "joystick_active": self._joystick_active if joystick_online else False,
                    "joystick_online": joystick_online,
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
