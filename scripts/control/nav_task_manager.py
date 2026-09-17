#!/usr/bin/env python3
"""Navigation task owner and the sole external /plan publisher."""

import json
import copy
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty, Float32, String
from route_tasks import RouteTasks


TERMINAL_STAGES = {"REACHED", "FAILED", "CANCELED"}


def decode_internal_status(payload: str) -> Dict[str, str]:
    """Decode a small internal status payload without allowing malformed data through."""
    try:
        value = json.loads(payload)
    except (TypeError, ValueError):
        return {}
    if not isinstance(value, dict):
        return {}
    stage = str(value.get("stage", "")).strip().upper()
    if not stage:
        return {}
    return {
        "stage": stage,
        "reason": str(value.get("reason", "")).strip(),
        "task_token": str(value.get("task_token", "")).strip(),
    }


def stamp_token(stamp) -> str:
    return f"{int(stamp.sec)}.{int(stamp.nanosec):09d}"


def make_nav_status(
    task_id: int,
    stage: str,
    reason: str = "",
    goal: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    """Build the stable P2 navigation status schema."""
    return {
        "version": 1,
        "task_id": int(task_id),
        "task_type": "point" if task_id > 0 else "none",
        "stage": str(stage).upper(),
        "reason": str(reason),
        "goal": goal,
        "route_id": None,
        "segment_index": None,
        "progress": None,
        "updated_at": time.time(),
    }


class NavTaskManager(RouteTasks, Node):
    """Own navigation lifecycle and arbitrate the public path topic."""

    def __init__(self) -> None:
        super().__init__("nav_task_manager")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("planner_goal_topic", "/nav_task/planner_goal")
        self.declare_parameter("planner_path_topic", "/nav_task/planner_path")
        self.declare_parameter("planner_status_topic", "/nav_task/planner_status")
        self.declare_parameter("tracker_status_topic", "/nav_task/tracker_status")
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("status_topic", "/nav_status")
        self.declare_parameter("nav_clear_topic", "/nav_clear")

        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.planner_goal_topic = str(self.get_parameter("planner_goal_topic").value)
        self.planner_path_topic = str(self.get_parameter("planner_path_topic").value)
        self.planner_status_topic = str(self.get_parameter("planner_status_topic").value)
        self.tracker_status_topic = str(self.get_parameter("tracker_status_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.nav_clear_topic = str(self.get_parameter("nav_clear_topic").value)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = QoSReliabilityPolicy.RELIABLE
        status_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.goal_pub = self.create_publisher(PoseStamped, self.planner_goal_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, status_qos)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(Path, self.planner_path_topic, self._on_planner_path, 10)
        self.create_subscription(String, self.planner_status_topic, self._on_planner_status, 10)
        self.create_subscription(String, self.tracker_status_topic, self._on_tracker_status, 10)
        self.create_subscription(Empty, self.nav_clear_topic, self._on_nav_clear, 10)

        self.task_id = 0
        self.stage = "IDLE"
        self.goal: Optional[Dict[str, float]] = None
        self.task_token = ""
        self._init_routes()
        self._publish_status("IDLE", "manager_started")
        self.get_logger().info(
            f"nav_task_manager started | goal={self.goal_topic} plan={self.path_topic} "
            f"status={self.status_topic}"
        )

    def _empty_path(self, frame_id: str = "map") -> Path:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        return message

    def _publish_status(self, stage: str, reason: str = "") -> None:
        self.stage = stage.upper()
        message = String()
        message.data = json.dumps(
            {**make_nav_status(self.task_id, self.stage, reason, self.goal),
             **self._route_status_fields()},
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_pub.publish(message)
        if reason:
            self.get_logger().info(f"[nav_task_manager] task={self.task_id} stage={self.stage} reason={reason}")

    def _on_goal(self, message: PoseStamped) -> None:
        self._invalidate()
        self.route_id = ""
        self.route = None
        self.point_goal = copy.deepcopy(message)
        self.task_id += 1
        message.header.stamp = self.get_clock().now().to_msg()
        self.task_token = stamp_token(message.header.stamp)
        self.goal = {
            "x": float(message.pose.position.x),
            "y": float(message.pose.position.y),
        }
        self.path_pub.publish(self._empty_path(message.header.frame_id or "map"))
        self._publish_status("PLANNING", "new_goal")
        self.speed_limit_pub.publish(Float32(data=0.0))
        self.goal_pub.publish(message)

    def _on_planner_path(self, message: Path) -> None:
        if self.task_id <= 0 or self.stage in TERMINAL_STAGES or self.stage == "PAUSED":
            return
        if stamp_token(message.header.stamp) != self.task_token:
            return
        if self.route_id:
            if self.stage == "PLANNING" and message.poses:
                self._publish_route_plan(message)
            return
        self.path_pub.publish(message)
        if message.poses:
            self._publish_status("FOLLOWING", "path_ready")

    def _on_planner_status(self, message: String) -> None:
        status = decode_internal_status(message.data)
        if not status or self.task_id <= 0 or self.stage in TERMINAL_STAGES or self.stage == "PAUSED":
            return
        if status["task_token"] != self.task_token:
            return
        stage = status["stage"]
        reason = status["reason"]
        if self.route_id:
            if self.stage == "PLANNING":
                if stage == "FAILED":
                    self._next_entry()
                elif stage == "REACHED":
                    self._publish_route_plan()
            return
        if stage == "FAILED":
            self._invalidate()
            self._publish_status("FAILED", reason or "planning_failed")
        elif stage == "REACHED":
            self._invalidate()
            self._publish_status("REACHED", reason or "already_within_tolerance")
        elif stage == "PLANNING" and self.stage not in TERMINAL_STAGES:
            self._publish_status("PLANNING", reason or "replanning")

    def _on_tracker_status(self, message: String) -> None:
        status = decode_internal_status(message.data)
        if not status or self.task_id <= 0 or self.stage in TERMINAL_STAGES or self.stage == "PAUSED":
            return
        if status["task_token"] != self.task_token:
            return
        stage = status["stage"]
        if stage == "REACHED":
            self._invalidate()
            self._publish_status("REACHED", status["reason"] or "goal_reached")
        elif stage == "FAILED":
            self._invalidate()
            self._publish_status("FAILED", status["reason"] or "tracking_failed")

    def _on_nav_clear(self, _message: Empty) -> None:
        self._invalidate()
        if self.task_id > 0 and self.stage not in TERMINAL_STAGES:
            self._publish_status("CANCELED", "nav_clear")
        else:
            self._publish_status("IDLE", "nav_clear")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavTaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
