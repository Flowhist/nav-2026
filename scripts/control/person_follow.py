#!/usr/bin/python3
"""ROS following gate. Observation mode creates no velocity publisher."""
import argparse
import json
import signal
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from interfaces.msg import KS114State
from std_msgs.msg import Empty, String
from finav.msg import FollowTarget
from ultrasonic_gate import UltrasonicGate
from vision.control import FollowControl
from vision.settings import load_config


class FollowingNode(Node):
    def __init__(self, drive=False):
        super().__init__('person_follow')
        cfg = load_config()
        self.control = FollowControl(cfg)
        self.gate = UltrasonicGate(bus=cfg['ultrasonic_bus'], stop_distance=cfg['ultrasonic_stop'])
        self.pub = self.create_publisher(Twist, '/follow_cmd_vel', 1) if drive else None
        self.status = self.create_publisher(String, '/follow/control_status', 1)
        self.last_status = 0.
        self.create_subscription(FollowTarget, '/follow/target', self.target, 1)
        self.create_subscription(Empty, '/follow_clear', self.cancel, 10)
        self.create_subscription(String, '/follow/command', self.command, 1)
        self.create_subscription(KS114State, cfg['ultrasonic_topic'],
                                 lambda msg: self.gate.update(msg.sensors, time.monotonic()), 1)
        self.create_timer(.05, self.tick)

    def cancel(self, _):
        self.control.cancelled = True

    def command(self, msg):
        try:
            if json.loads(msg.data).get('action') == 'cancel':
                self.cancel(None)
        except (ValueError, AttributeError):
            pass

    def target(self, msg):
        cfg = self.control.cfg
        valid = (msg.valid and msg.state == 'tracking' and msg.header.frame_id == 'base_link'
                 and msg.track_id > 0 and cfg['match_similarity'] <= msg.similarity <= 1.0001
                 and 0 < msg.confidence <= 1)
        self.control.receive(msg.session, msg.sequence, msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                             (msg.position.x, msg.position.y, msg.position.z), valid,
                             time.monotonic(), self.get_clock().now().nanoseconds/1e9)

    def tick(self):
        if self.count_publishers('/follow/target') > 1 or (self.pub and self.count_publishers('/follow_cmd_vel') > 1):
            self.cancel(None)
        now = time.monotonic()
        command, reason = self.control.tick(now, self.get_clock().now().nanoseconds/1e9,
                                            self.gate.reason(now, time.time()))
        if self.pub:
            msg = Twist()
            msg.linear.x, msg.angular.z = command
            self.pub.publish(msg)
        if now-self.last_status >= 1:
            self.status.publish(String(data=json.dumps({'drive': self.pub is not None,
                                 'state': reason, 'linear': command[0], 'angular': command[1]})))
            self.last_status = now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = FollowingNode(args.drive)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if rclpy.ok():
            node.cancel(None)
            node.tick()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
