#!/usr/bin/env python3
import threading
import time
from typing import Any, Dict, List, Optional


class StateStore:
    def __init__(self, max_history: int = 2000) -> None:
        self._lock = threading.Lock()
        self._seq = 0
        self._max_history = max_history
        self._history: List[Dict[str, Any]] = []
        self._status: Dict[str, Any] = {
            "server_time": time.time(),
            "ros": {
                "connected": False,
                "node_name": "web_control_server",
                "tf_hz": {"map_odom": 0.0, "odom_base_link": 0.0},
                "topic_hz": {"odom": 0.0, "plan": 0.0, "scan": 0.0},
                "last_seen": {},
            },
            "robot": {
                "pose_map": None,
                "pose_odom": None,
                "velocity": None,
                "plan": {"points": 0, "length_m": 0.0, "updated_at": None},
                "goal_pose": None,
                "initial_pose": None,
            },
            "teleop": {
                "active": False,
                "timeout_at": None,
                "joystick_active": False,
                "joystick_online": False,
                "joystick_updated_at": None,
            },
            "runtime": {
                "mapping": {"running": False, "stopping": False, "started_at": None, "pid": None, "log_path": None},
                "navigation": {"running": False, "stopping": False, "started_at": None, "pid": None, "log_path": None},
                "busy": False,
            },
        }
        self._scene_map_version = 0
        self._scene: Dict[str, Any] = {
            "map": None,
            "scan": {"frame_id": "map", "points": [], "updated_at": None},
            "plan": {"points": 0, "points_xy": [], "length_m": 0.0, "updated_at": None},
            "robot_pose_map": None,
            "goal_pose": None,
            "initial_pose": None,
        }

    def update_status(self, patch: Dict[str, Any]) -> None:
        with self._lock:
            self._status["server_time"] = time.time()
            self._deep_update(self._status, patch)

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return self._deep_copy(self._status)

    def update_scene(self, patch: Dict[str, Any], *, map_changed: bool = False) -> None:
        with self._lock:
            # Scene payloads are polled frequently and can contain large arrays.
            # Keep previous branches immutable and replace only the updated path.
            self._scene = self._merge_dict(self._scene, patch)
            if map_changed:
                self._scene_map_version += 1

    def snapshot_scene(self, known_map_version: int = -1) -> Dict[str, Any]:
        with self._lock:
            payload = dict(self._scene)
            current_map_version = self._scene_map_version
            payload["map_version"] = current_map_version
            payload["map_changed"] = current_map_version != known_map_version
            if not payload["map_changed"]:
                payload["map"] = None
            return payload

    def add_event(self, level: str, message: str, extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self._lock:
            self._seq += 1
            evt = {
                "seq": self._seq,
                "ts": time.time(),
                "level": level,
                "message": message,
                "extra": extra or {},
            }
            self._history.append(evt)
            if len(self._history) > self._max_history:
                self._history = self._history[-self._max_history :]
            return evt

    def get_history(self, since_seq: int = 0, limit: int = 200) -> List[Dict[str, Any]]:
        with self._lock:
            data = [x for x in self._history if x["seq"] > since_seq]
            return self._deep_copy(data[-max(1, min(limit, 1000)) :])

    def _deep_update(self, base: Dict[str, Any], patch: Dict[str, Any]) -> None:
        for k, v in patch.items():
            if isinstance(v, dict) and isinstance(base.get(k), dict):
                self._deep_update(base[k], v)
            else:
                base[k] = v

    def _merge_dict(self, base: Dict[str, Any], patch: Dict[str, Any]) -> Dict[str, Any]:
        merged = dict(base)
        for k, v in patch.items():
            if isinstance(v, dict) and isinstance(base.get(k), dict):
                merged[k] = self._merge_dict(base[k], v)
            else:
                merged[k] = v
        return merged

    def _deep_copy(self, obj: Any) -> Any:
        if isinstance(obj, dict):
            return {k: self._deep_copy(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [self._deep_copy(x) for x in obj]
        return obj
