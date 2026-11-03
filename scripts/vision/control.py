"""Fail-closed target freshness and bounded following commands, without ROS."""
import math


class FollowControl:
    def __init__(self, config):
        self.cfg = config
        self.session = None
        self.sequence = -1
        self.stamp = -math.inf
        self.received = -math.inf
        self.position = None
        self.cancelled = False
        self.previous = (0., 0.)
        self.last_tick = None

    def receive(self, session, sequence, stamp, position, valid, now, wall):
        self.position = None
        if self.session is not None and session != self.session:
            self.cancelled = True
        if (not session or sequence <= self.sequence or not math.isfinite(stamp)
                or stamp <= self.stamp or not 0 <= wall-stamp < self.cfg['frame_timeout']):
            return
        self.session, self.sequence, self.stamp = session, sequence, stamp
        self.received = now
        if valid and len(position) == 3 and all(math.isfinite(v) for v in position):
            self.position = position

    def tick(self, now, wall, gate='ready'):
        dt = min(.1, max(0., now-self.last_tick)) if self.last_tick is not None else .05
        self.last_tick = now
        reason = ('cancelled; restart required' if self.cancelled else
                  'target expired' if not (0 <= now-self.received < self.cfg['frame_timeout']
                      and 0 <= wall-self.stamp < self.cfg['frame_timeout']) else
                  'target invalid' if self.position is None else gate)
        desired = (0., 0.)
        if reason == 'ready':
            x, y, _ = self.position
            distance, bearing = math.hypot(x, y), math.atan2(y, x)
            if x <= 0 or distance <= self.cfg['follow_distance'] or distance > 5:
                reason = 'outside following distance'
            else:
                angular = max(-self.cfg['max_angular'], min(self.cfg['max_angular'], .8*bearing))
                linear = min(self.cfg['max_linear'], .3*(distance-self.cfg['follow_distance']))
                desired = (linear if abs(bearing) < math.radians(25) else 0., angular)
        if reason == 'ready':
            # Stops and direction reversals pass through zero immediately; only acceleration is limited.
            limited = []
            for target, previous, rate in zip(desired, self.previous,
                    (self.cfg['linear_acceleration'], self.cfg['angular_acceleration'])):
                if target*previous < 0:
                    limited.append(0.)
                elif abs(target) <= abs(previous):
                    limited.append(target)
                else:
                    limited.append(previous+max(-rate*dt, min(rate*dt, target-previous)))
            desired = tuple(limited)
        self.previous = desired
        return desired, reason
