"""KS114 stop gate and read-only terminal monitor; reuse the DCCS driver."""
import argparse
import math
import time

FOLLOW_ADDRESSES = (0x10, 0x11, 0x12, 0x13)
SENSOR_NAMES = {0x10: 'front_left', 0x11: 'front_right', 0x12: 'left', 0x13: 'right'}


class UltrasonicGate:
    def __init__(self, addresses=FOLLOW_ADDRESSES, bus='bus_0', stop_distance=.5):
        if len(addresses) != 4 or set(addresses) != set(FOLLOW_ADDRESSES):
            raise ValueError('Follow requires front_left/right and left/right: 0x10 0x11 0x12 0x13')
        if not math.isfinite(stop_distance) or stop_distance < .3:
            raise ValueError('Stop distance must be finite and >= 0.3 m')
        self.addresses = tuple(addresses)
        self.bus = bus
        self.stop_distance = stop_distance
        self.readings = []
        self.received = 0.

    def update(self, readings, now):
        self.readings = list(readings)
        self.received = now

    def reason(self, now, wall_now):
        if now - self.received >= .5:
            return 'ultrasonic missing/expired'
        for address in self.addresses:
            matches = [r for r in self.readings if r.bus == self.bus and r.address == address]
            prefix = f'ultrasonic {SENSOR_NAMES[address]} 0x{address:02x}'
            if len(matches) != 1:
                return prefix + ' missing/duplicate'
            reading = matches[0]
            age = wall_now - reading.stamp.sec - reading.stamp.nanosec / 1e9
            # Per-sensor timestamps matter: the driver republishes cached samples.
            if not 0 <= age < .5:
                return prefix + ' sample expired'
            if (not reading.online or not reading.valid or reading.consecutive_failures
                    or not math.isfinite(reading.distance_m) or reading.distance_m <= 0):
                return prefix + ' invalid/offline'
            if reading.distance_m <= self.stop_distance:
                return prefix + ' blocked'
        return 'ready'

    def summary(self):
        return ' | '.join(
            f'{SENSOR_NAMES[r.address]}(0x{r.address:02x})={r.distance_m:.3f}m'
            f'{"" if r.online and r.valid else " INVALID"}'
            for r in self.readings if r.bus == self.bus and r.address in self.addresses)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/ultrasonic/device/state')
    parser.add_argument('--seconds', type=float, default=120.)
    parser.add_argument('--bus', default='bus_0')
    parser.add_argument('--stop-distance', type=float, default=.5)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('seconds must be positive and finite')
    try:
        gate = UltrasonicGate(bus=args.bus, stop_distance=args.stop_distance)
    except ValueError as error:
        parser.error(str(error))
    import rclpy
    from interfaces.msg import KS114State
    rclpy.init()
    node = rclpy.create_node('person_follow_ultrasonic_monitor')
    latest = [None]
    def receive(msg):
        latest[0] = msg
        gate.update(msg.sensors, time.monotonic())

    node.create_subscription(KS114State, args.topic, receive, 1)
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.2)
            msg = latest[0]
            if msg is None:
                print('Waiting for ' + args.topic, flush=True)
                continue
            wall = time.time()
            reason = gate.reason(time.monotonic(), wall)
            print(f'{"READY" if reason == "ready" else "STOP"}: {reason} (read-only)', flush=True)
            print(' | '.join(
                f'{SENSOR_NAMES.get(r.address, "unused")}/{r.bus}/0x{r.address:02x}: {r.distance_m:.3f}m '
                f'{"OK" if r.online and r.valid and not r.consecutive_failures else "INVALID"} '
                f'age={wall-r.stamp.sec-r.stamp.nanosec/1e9:.2f}s'
                for r in msg.sensors), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
