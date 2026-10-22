#!/usr/bin/env python3
"""Gemini 305 + YOLO demo. Observation only unless --drive is explicit."""
import argparse
import math
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
from core import CameraMount, TargetLock, target_depth, velocity
from ultrasonic import FOLLOW_ADDRESSES, UltrasonicGate


class MotionOutput:
    def __init__(self, addresses, topic='/ultrasonic/device/state', bus='bus_0', stop_distance=.5):
        import rclpy
        from geometry_msgs.msg import Twist
        from interfaces.msg import KS114State
        from std_msgs.msg import Empty
        self.ros, self.Twist = rclpy, Twist
        rclpy.init()
        self.node = rclpy.create_node('person_follow_demo')
        self.publisher = self.node.create_publisher(Twist, '/follow_cmd_vel', 1)
        self.lock = threading.Lock()
        self.command = (0., 0.)
        self.stamp = 0.
        self.ultrasonic_gate = UltrasonicGate(addresses, bus, stop_distance)
        self.cancelled = False
        self.output_state = ((0., 0.), 'waiting')
        self.node.create_subscription(Empty, '/follow_clear', self.cancel, 10)
        self.node.create_subscription(KS114State, topic, self.ultrasonic, 1)
        self.node.create_timer(.05, self.tick)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def cancel(self, _):
        with self.lock:
            self.cancelled = True

    def ultrasonic(self, msg):
        with self.lock:
            self.ultrasonic_gate.update(msg.sensors, time.monotonic())

    def ultrasonic_status(self):
        with self.lock:
            return self.ultrasonic_gate.summary()

    def update(self, command, stamp):
        with self.lock:
            self.command, self.stamp = command, stamp

    def tick(self):
        now = time.monotonic()
        with self.lock:
            # Conflict latches cancellation; no automatic resumption after takeover.
            if self.node.count_publishers('/follow_cmd_vel') > 1:
                self.cancelled = True
            if self.cancelled:
                reason = 'cancelled; restart required'
            elif now-self.stamp >= .35:
                reason = 'target result expired'
            else:
                reason = self.ultrasonic_gate.reason(now, time.time())
            command = self.command if reason == 'ready' else (0., 0.)
            self.output_state = (command, reason)
        msg = self.Twist()
        msg.linear.x, msg.angular.z = command
        self.publisher.publish(msg)

    def status(self):
        with self.lock:
            return self.output_state

    def close(self):
        self.cancel(None)
        self.tick()
        self.ros.shutdown()
        self.thread.join(timeout=2)
        self.node.destroy_node()


def report(frame, objects, target_id, command, reason, latency, motion):
    mode = 'DRIVE' if motion else 'OBSERVE (no ROS commands)'
    delay = '--' if latency is None else f'{latency*1000:.0f} ms'
    lines = [f'Person follow | {mode} | frame={frame} | latency={delay}',
             'Coordinates: base_link, meters; +x forward, +y left, +z up',
             'ID    CLASS          CONF      X       Y       Z   DIST(m)  BEARING(deg)']
    for obj in objects:
        ident = '--' if obj['id'] is None else str(obj['id'])
        marker = '*' if obj['id'] is not None and obj['id'] == target_id else ' '
        prefix = f"{marker}{ident:<5} {obj['label']:<14.14} {obj['confidence']:.2f}"
        pos = obj['position']
        if pos is None:
            lines.append(prefix + '   depth invalid')
        else:
            x, y, z = pos
            lines.append(prefix + f' {x:7.2f} {y:7.2f} {z:7.2f} {math.hypot(x,y):8.2f} {math.degrees(math.atan2(y,x)):13.1f}')
    if not objects:
        lines.append('(no objects detected)')
    linear, angular = command
    action = 'STOP' if linear == 0 and angular == 0 else (
        ('FORWARD' if linear > 0 else 'TURN') +
        (' + LEFT' if angular > .01 else ' + RIGHT' if angular < -.01 else ''))
    lines.extend([f'Follow person ID={target_id} | {reason}',
                  f'Proposed: {action}, v={linear:.3f} m/s, w={angular:.3f} rad/s'])
    if motion:
        sent, gate = motion.status()
        lines.append('Front/side ultrasonic: ' + (motion.ultrasonic_status() or 'waiting'))
        lines.append(f'/follow_cmd_vel: v={sent[0]:.3f}, w={sent[1]:.3f} | {gate} (not chassis feedback)')
    text = '\n'.join(lines) + '\n'
    if sys.stdout.isatty():
        text = '\033[H\033[J' + text
    print(text, end='', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=str(Path(__file__).with_name('yolov8n.pt')))
    parser.add_argument('--device', default='cpu', help='cpu or CUDA index, e.g. 0')
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--rotate180', action='store_true', help='Rotate both RGB and aligned depth for inverted mounting')
    parser.add_argument('--drive', action='store_true', help='Requires four fresh front/side ultrasonics and exclusive /follow_cmd_vel')
    parser.add_argument('--front-addresses', nargs=4, type=lambda value: int(value, 0),
                        default=FOLLOW_ADDRESSES,
                        help='Verified front/side addresses: 0x10 0x11 0x12 0x13')
    parser.add_argument('--ultrasonic-topic', default='/ultrasonic/device/state')
    parser.add_argument('--ultrasonic-bus', default='bus_0')
    parser.add_argument('--ultrasonic-stop', type=float, default=.5,
                        help='Stop clearance from each front/side transducer in meters (default 0.5)')
    parser.add_argument('--robot-model', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'urdf/whillcar.urdf')
    parser.add_argument('--distance', type=float, default=2.1, help='Planar target distance from base_link, meters')
    args = parser.parse_args()
    if args.front_addresses:
        try:
            UltrasonicGate(args.front_addresses, args.ultrasonic_bus, args.ultrasonic_stop)
        except ValueError as error:
            parser.error(str(error))
    if not math.isfinite(args.seconds) or args.seconds <= 0 or not 1.8 <= args.distance <= 4:
        parser.error('seconds must be positive; base_link distance must be 1.8..4 m')
    mount = CameraMount.from_urdf(args.robot_model)

    from ultralytics import YOLO
    import torch
    from pyorbbecsdk import (Pipeline, Config, Context, OBSensorType, OBFormat,
                            OBFrameAggregateOutputMode, AlignFilter, OBStreamType)
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    model = YOLO(args.model)
    # Initialize inference before opening the camera or publishing movement.
    model.predict(np.zeros((320, 320, 3), dtype=np.uint8), imgsz=320,
                  device=args.device, verbose=False)
    context = Context()
    devices = context.query_devices()
    cameras = [devices.get_device_by_index(i) for i in range(devices.get_count())]
    cameras = [d for d in cameras if '305' in d.get_device_info().get_name()]
    if len(cameras) != 1:
        raise RuntimeError('Expected exactly one Gemini 305')
    pipeline = Pipeline(cameras[0])
    config = Config()
    colors = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
    color_profile = colors.get_video_stream_profile(640, 480, OBFormat.RGB, 30)
    config.enable_stream(color_profile)
    depths = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    config.enable_stream(depths.get_default_video_stream_profile())
    config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
    pipeline.enable_frame_sync()
    align = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
    latest, frame_lock = [None], threading.Lock()

    def receive(frames):
        with frame_lock:
            latest[0] = (time.monotonic(), frames)

    motion = None
    started = False
    processed = 0
    lock = TargetLock()
    began = time.monotonic()
    last_timestamp = -1
    last_report = began
    try:
        pipeline.start(config, receive)
        started = True
        if args.drive:
            motion = MotionOutput(args.front_addresses, args.ultrasonic_topic,
                                  args.ultrasonic_bus, args.ultrasonic_stop)
        while time.monotonic()-began < args.seconds:
            with frame_lock:
                item, latest[0] = latest[0], None
            if item is None:
                if time.monotonic()-last_report > .5:
                    report(processed, [], lock.target, (0., 0.), 'waiting for fresh RGB-D', None, motion)
                    last_report = time.monotonic()
                time.sleep(.01)
                continue
            captured, frames = item
            if time.monotonic()-captured > .35:
                continue
            frames = align.process(frames).as_frame_set()
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if not color or not depth:
                continue
            timestamp = color.get_timestamp()
            if timestamp <= last_timestamp or abs(timestamp-depth.get_timestamp()) > 100:
                continue
            last_timestamp = timestamp
            image = np.frombuffer(color.get_data(), np.uint8).reshape(color.get_height(), color.get_width(), 3)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            meters = np.frombuffer(depth.get_data(), np.uint16).reshape(depth.get_height(), depth.get_width())
            meters = meters.astype(np.float32) * depth.get_depth_scale() / 1000.
            if meters.shape != image.shape[:2]:
                raise RuntimeError('Depth/color alignment dimensions differ')
            if args.rotate180:
                image = cv2.rotate(image, cv2.ROTATE_180)
                meters = cv2.rotate(meters, cv2.ROTATE_180)
            result = model.track(image, persist=True, tracker='bytetrack.yaml',
                                 conf=.5, imgsz=320, device=args.device, verbose=False)[0]
            boxes = result.boxes
            ids = [None]*len(boxes) if boxes.id is None else boxes.id.int().cpu().tolist()
            intrinsic = color.get_stream_profile().as_video_stream_profile().get_intrinsic()
            cx = image.shape[1]-1-intrinsic.cx if args.rotate180 else intrinsic.cx
            cy = image.shape[0]-1-intrinsic.cy if args.rotate180 else intrinsic.cy
            objects, detections = [], []
            for ident, cls, confidence, box in zip(ids, boxes.cls.int().cpu().tolist(),
                                                  boxes.conf.cpu().tolist(), boxes.xyxy.cpu().numpy()):
                depth_m = target_depth(meters, box)
                position = None
                if depth_m is not None:
                    u = (box[0]+box[2])/2
                    v = box[1] + .425*(box[3]-box[1])
                    position = mount.target_in_base(depth_m, u, v, intrinsic.fx, intrinsic.fy, cx, cy)
                objects.append(dict(id=ident, label=result.names[cls], confidence=confidence,
                                    position=position))
                if cls == 0 and ident is not None:
                    detections.append((ident, box))
            target = lock.select(detections)
            command = (0., 0.)
            reason = 'target lost' if lock.target is not None else 'waiting for one stable person'
            if target:
                position = next(obj['position'] for obj in objects if obj['id'] == target[0])
                reason = 'target depth invalid'
                if position is not None:
                    distance = math.hypot(position[0], position[1])
                    bearing = math.atan2(position[1], position[0])
                    command = velocity(distance, bearing, args.distance)
                    reason = ('within stop distance' if distance <= args.distance else
                              'target too far' if distance > 5. else 'following')
            elapsed = time.monotonic()-captured
            if elapsed > .35:
                command = (0., 0.)
                reason = 'inference result expired (>350 ms)'
            if motion:
                motion.update(command, captured)
            processed += 1
            report(processed, objects, lock.target, command, reason, elapsed, motion)
            last_report = time.monotonic()
    finally:
        if motion:
            motion.close()
        if started:
            pipeline.stop()
        print(f'Processed {processed} frames; no target-ID reset during this run.')
    if not processed:
        raise RuntimeError('No synchronized RGB-D frames processed')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
