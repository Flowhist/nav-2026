import unittest
import math
from pathlib import Path
import numpy as np
from core import CameraMount, TargetLock, target_depth, velocity


class FollowTests(unittest.TestCase):
    def test_turn_direction_and_stop(self):
        self.assertGreater(velocity(3., .2)[1], 0.)
        self.assertLess(velocity(3., -.2)[1], 0.)
        self.assertEqual(velocity(1., .2), (0., 0.))
        self.assertEqual(velocity(float('nan'), .2), (0., 0.))
        self.assertEqual(velocity(3., 1.)[0], 0.)

    def test_measured_mount_and_center_ray(self):
        mount = CameraMount.from_urdf(Path(__file__).resolve().parents[2] / 'urdf/whillcar.urdf')
        self.assertEqual((mount.x, mount.y, mount.z), (.575, .2775, .727))
        self.assertAlmostEqual(mount.yaw, -math.pi/6)
        x, y, z = mount.target_in_base(2., 320., 240., 500., 500., 320., 240.)
        self.assertAlmostEqual(x, .575 + 2*math.cos(math.pi/6))
        self.assertAlmostEqual(y, .2775 - 2*math.sin(math.pi/6))
        self.assertAlmostEqual(z, .727)
        self.assertLess(math.atan2(y, x), 0.)

    def test_vehicle_centerline_and_rotated_principal_point(self):
        mount = CameraMount(.575, .2775, .727, -math.pi/6)
        dx, dy = 3.-mount.x, -mount.y
        c, s = math.cos(mount.yaw), math.sin(mount.yaw)
        depth, left = c*dx+s*dy, -s*dx+c*dy
        # Principal point after rotating a 640x480 inverted image.
        cx, cy = 639.-310., 479.-235.
        u, v = cx-left*500./depth, cy-(1.-mount.z)*500./depth
        position = mount.target_in_base(depth, u, v, 500., 500., cx, cy)
        np.testing.assert_allclose(position, (3., 0., 1.), atol=1e-12)
        self.assertAlmostEqual(velocity(math.hypot(*position[:2]), math.atan2(position[1], position[0]))[1], 0.)

    def test_depth_rejects_missing_and_mixed(self):
        depth = np.full((100, 100), 2.)
        self.assertEqual(target_depth(depth, [0, 0, 100, 100]), 2.)
        depth[:, :50] = 4.
        self.assertIsNone(target_depth(depth, [0, 0, 100, 100]))
        self.assertIsNone(target_depth(np.zeros_like(depth), [0, 0, 100, 100]))

    def test_no_switch_on_loss_or_ambiguous_acquisition(self):
        lock = TargetLock()
        a, b = (1, [0, 0, 10, 10]), (2, [20, 0, 30, 10])
        self.assertIsNone(lock.select([a, b]))
        for _ in range(3):
            lock.select([a])
        self.assertEqual(lock.target, 1)
        self.assertIsNone(lock.select([b]))
        self.assertIsNone(lock.select([]))

    def test_motion_watchdog_and_takeover_latch(self):
        import threading
        import time
        from types import SimpleNamespace
        from run import MotionOutput
        sent = []
        output = MotionOutput.__new__(MotionOutput)
        output.lock = threading.Lock()
        output.Twist = lambda: SimpleNamespace(linear=SimpleNamespace(x=0.), angular=SimpleNamespace(z=0.))
        output.publisher = SimpleNamespace(publish=lambda msg: sent.append(msg.linear.x))
        output.node = SimpleNamespace(count_publishers=lambda _: 1)
        output.cancelled = False
        output.command = (.15, 0.)
        output.stamp = time.monotonic()
        output.ultrasonic_gate = SimpleNamespace(reason=lambda *_: 'ready')
        output.tick()
        self.assertEqual(sent[-1], .15)
        output.stamp -= 1
        output.tick()
        self.assertEqual(sent[-1], 0.)
        output.stamp = time.monotonic()
        output.ultrasonic_gate.reason = lambda *_: 'ultrasonic blocked'
        output.tick()
        self.assertEqual(sent[-1], 0.)
        output.ultrasonic_gate.reason = lambda *_: 'ready'
        output.node.count_publishers = lambda _: 2
        output.tick()
        output.node.count_publishers = lambda _: 1
        output.tick()
        self.assertTrue(output.cancelled)
        self.assertEqual(sent[-1], 0.)


if __name__ == '__main__':
    unittest.main()
