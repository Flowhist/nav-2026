"""Read collision bounds and ultrasonic mounting from the expanded URDF."""
import math
import xml.etree.ElementTree as ET

import numpy as np
import xacro


def transform(origin):
    result = np.eye(4)
    if origin is None:
        return result
    result[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    roll, pitch, yaw = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                      [-sp, cp*sr, cp*cr]]
    return result


def model_parameters(path):
    root = ET.fromstring(xacro.process_file(str(path)).toxml())
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    poses = {'base_link': np.eye(4)}

    def pose(name):
        if name not in poses:
            joint = joints[name]
            poses[name] = pose(joint.find('parent').get('link')) @ transform(joint.find('origin'))
        return poses[name]

    bounds = []
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            box, cylinder, sphere = geometry.find('box'), geometry.find('cylinder'), geometry.find('sphere')
            if box is not None:
                half = np.array([float(v) for v in box.get('size').split()]) / 2
            elif cylinder is not None:
                half = np.array([float(cylinder.get('radius'))]*2 + [float(cylinder.get('length'))/2])
            elif sphere is not None:
                half = np.full(3, float(sphere.get('radius')))
            else:
                raise ValueError('Unsupported collision geometry: ' + link.get('name'))
            matrix = pose(link.get('name')) @ transform(collision.find('origin'))
            extent = np.abs(matrix[:3, :3]) @ half
            bounds.append((matrix[:3, 3]-extent, matrix[:3, 3]+extent))
    if not bounds:
        raise ValueError('Robot model has no collision geometry')
    low = np.min([b[0] for b in bounds], axis=0)
    high = np.max([b[1] for b in bounds], axis=0)
    values = [*low, *high]
    if not np.all(np.isfinite(values)):
        raise ValueError('Nonfinite robot geometry')
    ultrasonic = []
    for name in ('front_left', 'front_right', 'left', 'right', 'rear_left', 'rear_right'):
        mount = pose(f'ultrasonic_{name}_link')
        ultrasonic.extend(float(v) for v in (*mount[:3, 3], math.atan2(mount[1, 0], mount[0, 0])))
    return {'footprint': [float(v) for v in (low[0], high[0], low[1], high[1], high[2])],
            'ultrasonic_mounts': ultrasonic}
