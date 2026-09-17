"""Adapt a saved route to the existing point planner and /plan controller."""

import copy
import json
import math
from pathlib import Path as FilePath

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

import planning_constraints  # Editor imports work in both source and install layouts.
from editor_map_io import load_map
from editor_routes import smooth_route
from editor_store import EditorStore


def entry_candidates(points, position):
    """Nearest segment projections, retaining alternatives if planning fails."""
    candidates = []
    along = 0.0
    for index, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        ratio = max(0., min(1., ((position[0] - a[0]) * dx + (position[1] - a[1]) * dy) / length**2))
        point = (a[0] + ratio * dx, a[1] + ratio * dy)
        candidates.append((math.dist(position, point), index, point, along + ratio * length))
        along += length
    selected = []
    for candidate in sorted(candidates):
        if all(abs(candidate[3] - other[3]) >= .5 for other in selected):
            selected.append(candidate)
        if len(selected) >= 12:
            break
    return [(index, point) for _, index, point, _ in selected]


class RouteTasks:
    def _init_routes(self):
        self.declare_parameter("maps_dir", "")
        self.declare_parameter("map_file", "")
        self.maps_dir = str(self.get_parameter("maps_dir").value)
        self.map_file = str(self.get_parameter("map_file").value)
        self.route = None
        self.route_id = ""
        self.route_signature = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planner_cancel_pub = self.create_publisher(String, "/nav_task/planner_cancel", 10)
        self.speed_limit_pub = self.create_publisher(Float32, "/nav_task/speed_limit", 10)
        self.create_subscription(String, "/nav_task/command", self._on_command, 10)
        self.create_timer(.5, self._check_route_changes)

    def _invalidate(self):
        if self.task_token:
            self.planner_cancel_pub.publish(String(data=self.task_token))
        self.task_token = ""
        self.path_pub.publish(self._empty_path())

    def _route_status_fields(self):
        if not self.route_id:
            return {}
        return {"task_type": "route", "route_id": self.route_id,
                "route_name": self.route["name"] if self.route else self.route_id,
                "map_name": self.map_file}

    def _on_command(self, message):
        try:
            command = json.loads(message.data)
            action = command.get("action")
            if action == "start":
                if command.get("map_name") != self.map_file or not command.get("route_id"):
                    raise ValueError("route_map_mismatch")
                self.task_id += 1
                self.route_id, self.route = str(command["route_id"]), None
                self.goal = None
                self._prepare_route()
            elif action == "pause" and self.stage in {"PLANNING", "FOLLOWING"}:
                self._invalidate()
                self._publish_status("PAUSED", "user_pause")
            elif action == "resume" and self.stage == "PAUSED":
                if self.route_id:
                    self._prepare_route()
                elif self.goal:
                    self._on_goal(copy.deepcopy(self.point_goal))
            elif action == "cancel":
                self._on_nav_clear(None)
        except (ValueError, TypeError, AttributeError) as exc:
            self.get_logger().warn(f"[nav_task_manager] rejected command: {exc}")

    def _signature(self):
        path = FilePath(self.maps_dir) / self.map_file / f"{self.map_file}.editor.yaml"
        try:
            stat = path.stat()
            return stat.st_mtime_ns, stat.st_size, stat.st_ino
        except OSError:
            return None

    def _robot_position(self):
        transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        return transform.transform.translation.x, transform.transform.translation.y

    def _prepare_route(self, remain_paused=False):
        self._invalidate()
        self.route_signature = self._signature()
        try:
            map_data = load_map(FilePath(self.maps_dir), self.map_file)
            document = EditorStore(FilePath(self.maps_dir)).load(self.map_file, map_data.source_sha256, .6)
            route = next((item for item in document["routes"] if item["id"] == self.route_id), None)
            if route is None:
                raise ValueError("route_not_found")
            if route["closed"]:
                raise ValueError("closed_route_not_supported")
            preview = smooth_route(map_data, route, document["keepouts"], document["settings"]["safety_clearance_m"])
            if preview["status"] == "invalid":
                raise ValueError("route_collision")
            self.route = route
            self.route_points = [(point["x"], point["y"]) for point in preview["points"]]
            if route["direction"] == "reverse":
                self.route_points.reverse()
            if remain_paused:
                self._publish_status("PAUSED", "route_updated_paused")
                return
            self.entries = entry_candidates(self.route_points, self._robot_position())
            self._next_entry()
        except (ValueError, OSError, TransformException) as exc:
            self._publish_status("PAUSED", str(exc))

    def _next_entry(self):
        self._invalidate()
        if not self.entries:
            self._publish_status("FAILED", "no_reachable_entry")
            return
        self.entry_index, self.entry_point = self.entries.pop(0)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        self.task_token = f"{goal.header.stamp.sec}.{goal.header.stamp.nanosec:09d}"
        goal.pose.position.x, goal.pose.position.y = self.entry_point
        a, b = self.route_points[self.entry_index:self.entry_index + 2]
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self._publish_status("PLANNING", "selecting_entry")
        self.goal_pub.publish(goal)

    def _publish_route_plan(self, connection=None):
        # Publish one path, so the existing controller never stops at the join.
        path = copy.deepcopy(connection) if connection is not None else self._empty_path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        self.planner_cancel_pub.publish(String(data=self.task_token))
        self.task_token = f"{path.header.stamp.sec}.{path.header.stamp.nanosec:09d}"
        for x, y in [self.entry_point] + self.route_points[self.entry_index + 1:]:
            if path.poses and math.hypot(path.poses[-1].pose.position.x - x, path.poses[-1].pose.position.y - y) < 1e-6:
                continue
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        self.speed_limit_pub.publish(Float32(data=float(self.route["speed_limit_mps"])))
        for pose in path.poses:
            pose.header = copy.deepcopy(path.header)
        self.path_pub.publish(path)
        self._publish_status("FOLLOWING", "route_ready")

    def _check_route_changes(self):
        if self.route_id and self.stage in {"PLANNING", "FOLLOWING", "PAUSED"} and self._signature() != self.route_signature:
            self._prepare_route(remain_paused=self.stage == "PAUSED")
