"""Small read-only perception snapshot, separate from navigation state."""
import math
import threading
import time


SENSOR_NAMES = ("前左", "前右", "左侧", "右侧", "后左", "后右")


class PerceptionView:
    def __init__(self):
        self._lock = threading.Lock()
        self._points = []
        self._points_at = None
        self._points_revision = 0
        self._values = {}
        self._status_at = None

    def update_points(self, message):
        if message.header.frame_id != "base_link":
            return
        points = [[round(p.x, 3), round(p.y, 3)] for p in message.points[:4096]
                  if math.isfinite(p.x) and math.isfinite(p.y) and abs(p.x) <= 8 and abs(p.y) <= 8]
        with self._lock:
            self._points, self._points_at = points, time.monotonic()
            self._points_revision += 1

    def update_status(self, message):
        for item in message.status:
            if item.name == "avoidance_observation":
                with self._lock:
                    self._values = {pair.key: pair.value for pair in item.values}
                    self._status_at = time.monotonic()
                break

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            values, points = self._values, self._points
            status_age = None if self._status_at is None else now - self._status_at
            points_age = None if self._points_at is None else now - self._points_at
            points_revision = self._points_revision

        def number(key):
            try:
                value = float(values[key])
                return value if math.isfinite(value) else None
            except (KeyError, ValueError, TypeError):
                return None

        online = status_age is not None and status_age < 2.5
        timeout = number("sensor_timeout") or .5
        sensors = []
        for index, name in enumerate(SENSOR_NAMES):
            address = index + 16
            x, y, z, yaw = [number(f"ultrasonic_mounts_{index * 4 + j}") for j in range(4)]
            prefix = f"us_{address}_"
            distance, age = number(prefix + "distance_m"), number(prefix + "age_s")
            valid = (all(v is not None for v in (x, y, z, yaw))
                     and online and values.get(prefix + "online") == "true"
                     and values.get(prefix + "valid") == "true"
                     and number(prefix + "matches") == 1 and number(prefix + "failures") == 0
                     and distance is not None and distance > 0 and age is not None and 0 <= age < timeout
                     and number("ultrasonic_age_s") is not None and 0 <= number("ultrasonic_age_s") < timeout)
            sensors.append(dict(address=address, name=name, x=x, y=y, z=z, yaw=yaw,
                                distance=distance if valid else None, valid=valid))
        footprint = [number(f"footprint_{i}") for i in range(5)]
        camera = [number(f"camera_mount_{i}") for i in range(4)]
        return dict(online=online, status_age=status_age, points_age=points_age, points_revision=points_revision,
                    points=points if online and points_age is not None and points_age < .8 else [],
                    depth_live=bool(online and values.get("observation_source") == "dual_lidar"
                                    and number("lidar_left_age_s") is not None
                                    and 0 <= number("lidar_left_age_s") < timeout
                                    and number("lidar_right_age_s") is not None
                                    and 0 <= number("lidar_right_age_s") < timeout
                                    and values.get("lidar_odom_matched") == "true"
                                    and number("odom_age_s") is not None and 0 <= number("odom_age_s") < timeout
                                    and points_age is not None and points_age < .8),
                    observation_source=values.get("observation_source"),
                    sensors=sensors, footprint=footprint if all(v is not None for v in footprint) else None,
                    camera=camera if all(v is not None for v in camera) else None,
                    resolution=number("resolution") or .05, observation_only=values.get("drives_chassis") == "false")
