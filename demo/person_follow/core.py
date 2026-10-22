"""Small, hardware-independent person-following policy."""
import math
from dataclasses import dataclass
import xml.etree.ElementTree as ET
import numpy as np


@dataclass(frozen=True)
class CameraMount:
    x: float
    y: float
    z: float
    yaw: float

    @classmethod
    def from_urdf(cls, path):
        # Only four literal properties are read; no xacro evaluation or ROS needed.
        props = {p.attrib['name']: p.attrib['value'] for p in
                 ET.parse(path).getroot().findall('{http://www.ros.org/wiki/xacro}property')}
        values = [float(props['front_camera_' + key]) for key in ('x', 'y', 'z', 'yaw_deg')]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Camera mount values must be finite')
        return cls(*values[:3], math.radians(values[3]))

    def target_in_base(self, depth, u, v, fx, fy, cx, cy):
        """Aligned optical Z depth -> upright camera FLU -> base_link."""
        if not all(math.isfinite(n) for n in (depth, u, v, fx, fy, cx, cy)) or min(depth, fx, fy) <= 0:
            raise ValueError('Invalid camera projection')
        forward = depth
        left = -(u-cx)*depth/fx
        up = -(v-cy)*depth/fy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + c*forward - s*left,
                self.y + s*forward + c*left, self.z + up)


def target_depth(depth, box):
    """Use the central torso, rejecting sparse or mixed foreground/background."""
    x1, y1, x2, y2 = box
    h, w = depth.shape
    left, right = max(0, int(x1 + .35 * (x2-x1))), min(w, int(x1 + .65 * (x2-x1)))
    top, bottom = max(0, int(y1 + .25 * (y2-y1))), min(h, int(y1 + .60 * (y2-y1)))
    roi = depth[top:bottom, left:right]
    values = roi[np.isfinite(roi) & (roi > .25) & (roi < 6.)]
    if values.size < max(20, roi.size * .5):
        return None
    if np.percentile(values, 80) - np.percentile(values, 20) > .5:
        return None
    return float(np.median(values))


def velocity(distance, bearing, follow_distance=2.1):
    if not all(math.isfinite(v) for v in (distance, bearing, follow_distance)):
        return 0., 0.
    if distance <= follow_distance or distance > 5.:
        return 0., 0.
    angular = max(-.3, min(.3, .8 * bearing))
    linear = min(.15, .3 * (distance-follow_distance))
    if abs(bearing) > math.radians(25):
        linear = 0.
    return linear, angular


class TargetLock:
    """Acquire one stable ID; never silently switch after losing it."""
    def __init__(self):
        self.target = None
        self.candidate = None
        self.count = 0

    def select(self, detections):
        # Each item is (track_id, xyxy). Ambiguous first acquisition waits.
        if self.target is not None:
            return next((d for d in detections if d[0] == self.target), None)
        if len(detections) != 1:
            self.candidate, self.count = None, 0
            return None
        item = detections[0]
        self.count = self.count + 1 if self.candidate == item[0] else 1
        self.candidate = item[0]
        if self.count >= 3:
            self.target = item[0]
            return item
        return None
