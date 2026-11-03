"""RGB-D geometry and robust torso depth shared by person tracking."""
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
