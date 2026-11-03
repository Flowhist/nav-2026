"""Latest-frame RGB-D tracking; inference never owns the camera or velocity topic."""
import json
import signal
import threading
import time
import uuid

import message_filters
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String
from finav.msg import FollowTarget

from .geometry import CameraMount
from .identity import Identity
from .settings import load_config, share_dir


class TrackingNode(Node):
    def __init__(self):
        from .inference import Tracker
        super().__init__('person_tracker')
        self.cfg = load_config()
        self.identity = Identity(self.cfg)
        self.mount = CameraMount.from_urdf(share_dir()/'urdf/whillcar.urdf')
        self.tracker = Tracker(self.cfg)
        self.pub = self.create_publisher(FollowTarget, '/follow/target', 1)
        self.status = self.create_publisher(String, '/follow/status', 1)
        self.session, self.sequence = uuid.uuid4().hex, 0
        self.lock = threading.Lock()
        self.latest = self.command = None
        self.cancelled = False
        self.last_stamp = -1.
        self.last_status = 0.
        self.subs = [message_filters.Subscriber(self, kind, topic, qos_profile=qos_profile_sensor_data)
                     for kind, topic in ((Image, '/vision/color'), (Image, '/vision/depth'),
                                         (CameraInfo, '/vision/camera_info'))]
        self.sync = message_filters.TimeSynchronizer(self.subs, 2)
        self.sync.registerCallback(self.receive)
        self.create_subscription(String, '/follow/command', self.select, 1)
        self.create_subscription(Empty, '/follow_clear', self.cancel, 10)

    def receive(self, color, depth, info):
        with self.lock:
            self.latest = (time.monotonic(), color, depth, info)

    def cancel(self, _):
        with self.lock:
            self.cancelled = True

    def select(self, msg):
        try:
            command = json.loads(msg.data)
            action = command['action']
            if action not in ('enroll', 'select', 'cancel'):
                raise ValueError('Unknown action')
            if action == 'select' and (type(command.get('track_id')) is not int or command['track_id'] <= 0):
                raise ValueError('Positive track_id required')
            with self.lock:
                self.command = command
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(str(error))

    def step(self):
        with self.lock:
            item, self.latest = self.latest, None
            command, self.command = self.command, None
            cancelled = self.cancelled
        if command:
            if command['action'] == 'cancel':
                self.cancel(None)
                cancelled = True
            elif not cancelled:
                self.identity.reset(command.get('track_id'))
        if item is None:
            if time.monotonic()-self.last_status >= 1:
                self.status.publish(String(data=json.dumps({'state': 'waiting_for_rgbd', 'valid': False,
                                    'target_id': -1, 'people': []})))
                self.last_status = time.monotonic()
            return
        received, color, depth, info = item
        stamp = color.header.stamp.sec+color.header.stamp.nanosec/1e9
        result = FollowTarget(session=self.session, sequence=self.sequence,
                              track_id=-1, state='stale')
        result.header.stamp = color.header.stamp
        result.header.frame_id = 'base_link'
        self.sequence += 1
        people = []
        age = self.get_clock().now().nanoseconds/1e9-stamp
        if cancelled:
            result.state = 'cancelled; restart required'
        elif (stamp > self.last_stamp and 0 <= age < self.cfg['frame_timeout']
              and time.monotonic()-received < self.cfg['frame_timeout']):
            self.last_stamp = stamp
            try:
                if any(self.count_publishers(topic) != 1 for topic in
                       ('/vision/color', '/vision/depth', '/vision/camera_info')):
                    raise ValueError('Camera publisher missing or conflicting')
                if (color.encoding != 'rgb8' or depth.encoding != '32FC1' or depth.is_bigendian
                        or (color.width, color.height) != (640, 480)
                        or (depth.width, depth.height) != (color.width, color.height)
                        or color.step != color.width*3 or depth.step != depth.width*4
                        or info.width != color.width or info.height != color.height
                        or not (color.header.frame_id == depth.header.frame_id == info.header.frame_id
                                == 'front_camera_upright_optical_frame')):
                    raise ValueError('Invalid RGB-D layout or frame')
                rgb = np.frombuffer(color.data, np.uint8).reshape(color.height, color.width, 3)
                meters = np.frombuffer(depth.data, '<f4').reshape(depth.height, depth.width)
                intrinsic = (info.k[0], info.k[4], info.k[2], info.k[5])
                people, state = self.tracker.process(np.ascontiguousarray(rgb[:, :, ::-1]),
                                                    meters, intrinsic, self.mount, stamp)
                person = self.identity.select(people, stamp, self.tracker.visible_count)
                result.state = self.identity.state if state == 'ok' else state
                if person is not None:
                    result.track_id = int(person.track_id)
                    result.position.x, result.position.y, result.position.z = map(float, person.position)
                    result.confidence = float(person.confidence)
                    result.similarity = float(self.identity.similarity)
                    result.valid = True
            except (ValueError, RuntimeError) as error:
                result.state = str(error)
                self.get_logger().error(str(error))
        with self.lock:
            stopped = self.cancelled or self.command is not None
        if stopped or not 0 <= self.get_clock().now().nanoseconds/1e9-stamp < self.cfg['frame_timeout']:
            result.valid = False
            result.state = 'cancelled_or_expired'
        self.pub.publish(result)
        if time.monotonic()-self.last_status >= 1:
            self.status.publish(String(data=json.dumps({'state': result.state, 'valid': result.valid,
                'target_id': result.track_id, 'people': [{'id': int(p.track_id),
                'confidence': p.confidence, 'quality': p.quality} for p in people]})))
            self.last_status = time.monotonic()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    thread = None
    executor = None
    try:
        node = TrackingNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        while rclpy.ok():
            node.step()
            time.sleep(.005)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if executor:
            executor.shutdown(timeout_sec=2)
        if thread:
            thread.join(timeout=2)
        if node:
            node.tracker.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
