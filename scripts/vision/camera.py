"""Single RGB-D acquisition shared by tracking and avoidance; no velocity output."""
import argparse
import signal
import fcntl
import os
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class Camera(Node):
    def __init__(self, upright=False):
        super().__init__('vision_camera')
        lock_path = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp')) / f'finav-camera-{os.getuid()}.lock'
        self.device_lock = open(lock_path, 'a')
        fcntl.flock(self.device_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.color_pub = self.create_publisher(Image, '/vision/color', qos_profile_sensor_data)
        self.full_depth_pub = self.create_publisher(Image, '/vision/depth', qos_profile_sensor_data)
        self.full_info_pub = self.create_publisher(CameraInfo, '/vision/camera_info', qos_profile_sensor_data)
        from pyorbbecsdk import (Pipeline, Config, Context, OBSensorType, OBFormat,
                                OBFrameAggregateOutputMode, AlignFilter, OBStreamType)
        self.depth_pub = self.create_publisher(Image, '/avoidance/depth', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/avoidance/camera_info', qos_profile_sensor_data)
        self.lock = threading.Lock()
        self.latest = None
        self.last_timestamp = -1
        self.published = 0
        self.upright = upright
        self.sdk_context = Context()
        devices = self.sdk_context.query_devices()
        cameras = [devices.get_device_by_index(i) for i in range(devices.get_count())]
        cameras = [d for d in cameras if '305' in d.get_device_info().get_name()]
        if len(cameras) != 1:
            raise RuntimeError('Expected exactly one available Gemini 305')
        self.pipeline = Pipeline(cameras[0])
        config = Config()
        profile = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR).get_video_stream_profile(
            640, 480, OBFormat.RGB, 30)
        config.enable_stream(profile)
        config.enable_stream(self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile())
        config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        self.pipeline.enable_frame_sync()
        self.align = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        self.pipeline.start(config, self.receive)
        self.timer = self.create_timer(1/15, self.publish_depth)

    def receive(self, frames):
        with self.lock:
            self.latest = (time.monotonic(), self.get_clock().now().to_msg(), frames)

    def publish_depth(self):
        with self.lock:
            item, self.latest = self.latest, None
        if item is None:
            return
        captured, stamp, frames = item
        if time.monotonic() - captured > .15:
            return
        frames = self.align.process(frames).as_frame_set()
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            return
        timestamp = depth.get_timestamp_us()
        if timestamp <= self.last_timestamp or abs(color.get_timestamp_us()-timestamp) > 50000:
            return
        self.last_timestamp = timestamp
        # Reject frames already delayed inside the SDK, not just in our latest-frame slot.
        age = time.time() - depth.get_system_timestamp_us()/1e6
        if not 0 <= age <= .15:
            return
        stamp = rclpy.time.Time(nanoseconds=self.get_clock().now().nanoseconds-int(age*1e9)).to_msg()
        width, height = depth.get_width(), depth.get_height()
        if (width, height) != (640, 480):
            raise RuntimeError('Expected aligned 640x480 depth')
        meters = np.frombuffer(depth.get_data(), np.uint16).reshape(height, width)
        intrinsic = color.get_stream_profile().as_video_stream_profile().get_intrinsic()
        cx, cy = intrinsic.cx, intrinsic.cy
        if not self.upright:
            meters = meters[::-1, ::-1]
            cx, cy = width-1-cx, height-1-cy
        full_info = CameraInfo()
        full_info.header.stamp = stamp
        full_info.header.frame_id = 'front_camera_upright_optical_frame'
        full_info.height, full_info.width = height, width
        full_info.k = [intrinsic.fx, 0., cx, 0., intrinsic.fy, cy, 0., 0., 1.]
        if time.monotonic()-captured > .15:
            return
        if self.full_info_pub.get_subscription_count():
            self.full_info_pub.publish(full_info)
        # Avoid full-frame conversion and serialization when only avoidance is active.
        if self.color_pub.get_subscription_count():
            rgb = np.frombuffer(color.get_data(), np.uint8).reshape(height, width, 3)
            if not self.upright:
                rgb = rgb[::-1, ::-1]
            self.color_pub.publish(Image(header=full_info.header, height=height, width=width,
                                         encoding='rgb8', step=width*3, data=rgb.tobytes()))
        if self.full_depth_pub.get_subscription_count():
            full_depth = meters.astype('<f4') * (depth.get_depth_scale()/1000.)
            self.full_depth_pub.publish(Image(header=full_info.header, height=height, width=width,
                                              encoding='32FC1', step=width*4, data=full_depth.tobytes()))
        # Block minimum preserves near returns; any hole keeps the block unknown.
        blocks = meters.reshape(height//4, 4, width//4, 4)
        reduced = blocks.min(axis=(1, 3)).astype(np.float32) * (depth.get_depth_scale()/1000.)
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'front_camera_upright_optical_frame'
        image.height, image.width = reduced.shape
        image.encoding = '32FC1'
        image.step = image.width*4
        image.data = reduced.astype('<f4', copy=False).tobytes()
        info = CameraInfo()
        info.header = image.header
        info.height, info.width = image.height, image.width
        info.k = [intrinsic.fx/4, 0., (cx-1.5)/4, 0., intrinsic.fy/4, (cy-1.5)/4, 0., 0., 1.]
        if time.monotonic()-captured <= .15:
            self.info_pub.publish(info)
            self.depth_pub.publish(image)
            self.published += 1

    def close(self):
        self.timer.cancel()
        self.pipeline.stop()
        self.device_lock.close()
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upright', action='store_true', help='Camera physically upright; default corrects inverted mounting')
    parser.add_argument('--seconds', type=float, default=0, help='0 runs until interrupted')
    args, ros_args = parser.parse_known_args()
    if not np.isfinite(args.seconds) or args.seconds < 0:
        parser.error('seconds must be finite and nonnegative')
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Camera(args.upright)
        began = time.monotonic()
        while rclpy.ok() and (not args.seconds or time.monotonic()-began < args.seconds):
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
    if node is not None:
        print(f'Published {node.published} reduced depth frames; no velocity output.')
        if not node.published:
            raise RuntimeError('No fresh aligned depth frames published')


if __name__ == '__main__':
    main()
