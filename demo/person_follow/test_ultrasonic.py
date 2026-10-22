import unittest
from types import SimpleNamespace
from ultrasonic import FOLLOW_ADDRESSES, UltrasonicGate


class UltrasonicTests(unittest.TestCase):
    def setUp(self):
        self.gate = UltrasonicGate([0x10, 0x11, 0x12, 0x13])
        self.readings = [SimpleNamespace(
            bus='bus_0', address=address, stamp=SimpleNamespace(sec=100, nanosec=0),
            online=True, valid=True, consecutive_failures=0, distance_m=1.5)
            for address in range(0x10, 0x16)]
        self.gate.update(self.readings, 10.)

    def test_front_only_and_each_front_obstacle(self):
        self.readings[4].distance_m = .1
        self.assertEqual(self.gate.reason(10.1, 100.1), 'ready')
        for reading in self.readings[:4]:
            reading.distance_m = .5
            self.assertIn('blocked', self.gate.reason(10.1, 100.1))
            reading.distance_m = .501
            self.assertEqual(self.gate.reason(10.1, 100.1), 'ready')
            reading.distance_m = 1.5

    def test_stream_timeout_and_republished_stale_sample(self):
        self.assertIn('expired', self.gate.reason(10.5, 100.1))
        self.gate.update(self.readings, 11.)
        self.assertIn('expired', self.gate.reason(11.1, 101.1))
        self.readings[0].stamp.sec = 102
        self.assertIn('expired', self.gate.reason(11.1, 101.1))

    def test_missing_duplicate_and_wrong_bus(self):
        self.gate.update(self.readings[1:], 10.)
        self.assertIn('missing', self.gate.reason(10.1, 100.1))
        self.gate.update(self.readings + [self.readings[0]], 10.)
        self.assertIn('duplicate', self.gate.reason(10.1, 100.1))
        self.gate.update(self.readings, 10.)
        self.readings[0].bus = 'other'
        self.assertIn('missing', self.gate.reason(10.1, 100.1))

    def test_invalid_readings_do_not_release_stop(self):
        for field, value in [('online', False), ('valid', False),
                             ('consecutive_failures', 1), ('distance_m', float('nan')),
                             ('distance_m', float('inf')), ('distance_m', 0.), ('distance_m', -1.)]:
            original = getattr(self.readings[0], field)
            setattr(self.readings[0], field, value)
            self.assertIn('invalid', self.gate.reason(10.1, 100.1))
            setattr(self.readings[0], field, original)

    def test_requires_four_distinct_installed_addresses(self):
        for addresses in [[], [16, 17, 18], [16, 17, 18, 18], [16, 17, 18, 22], [16, 17, 20, 21]]:
            with self.assertRaises(ValueError):
                UltrasonicGate(addresses)

    def test_measured_front_left_obstacle_and_removal(self):
        self.assertEqual(UltrasonicGate().addresses, FOLLOW_ADDRESSES)
        self.readings[0].distance_m = .065
        self.assertEqual(self.gate.reason(10.1, 100.1), 'ultrasonic front_left 0x10 blocked')
        self.readings[0].distance_m = 1.6
        self.readings[4].valid = self.readings[5].valid = False
        self.assertEqual(self.gate.reason(10.1, 100.1), 'ready')

    def test_driver_config_polls_only_follow_sensors(self):
        from pathlib import Path
        import yaml
        config = yaml.safe_load(Path(__file__).with_name('ultrasonic.yaml').read_text())
        self.assertEqual(len(config['buses']), 1)
        self.assertEqual(tuple(s['address'] for s in config['buses'][0]['sensors']), FOLLOW_ADDRESSES)


if __name__ == '__main__':
    unittest.main()
