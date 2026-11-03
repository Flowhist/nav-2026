"""Bounded identity memory, independent of ROS and inference libraries."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class Person:
    track_id: int
    position: np.ndarray
    embedding: np.ndarray
    confidence: float
    quality: bool


class Identity:
    def __init__(self, config):
        self.cfg = config
        self.reset()

    def reset(self, track_id=None):
        self.gallery = deque(maxlen=self.cfg['gallery_size'])
        self.requested = track_id
        self.track_id = None
        self.pending = None
        self.hits = 0
        self.position = None
        self.last_seen = None
        self.last_frame = None
        self.last_update = -math.inf
        self.state = 'waiting'
        self.similarity = 0.

    def select(self, people, stamp, visible_count=None):
        if not math.isfinite(stamp) or (self.last_frame is not None and stamp <= self.last_frame):
            self.state = 'stale'
            return None
        if self.last_frame is not None and stamp-self.last_frame > self.cfg['frame_timeout']:
            self.pending, self.hits = None, 0
            self.state = 'lost'
        self.last_frame = stamp
        valid = [p for p in people if p.quality and np.shape(p.position) == (3,)
                 and np.all(np.isfinite(p.position)) and np.all(np.isfinite(p.embedding))
                 and np.linalg.norm(p.embedding) > .9]
        self.similarity = 0.
        if not self.gallery:
            candidates = [p for p in valid if p.confidence >= self.cfg['enroll_confidence']
                          and (self.requested is None or p.track_id == self.requested)]
            # Automatic enrollment requires exactly one visible person, not just one good crop.
            count = len(people) if visible_count is None else visible_count
            if len(candidates) != 1 or (self.requested is None and count != 1):
                self.pending, self.hits = None, 0
                self.state = 'waiting_for_selection'
                return None
            person = candidates[0]
            self.similarity = 1.
        else:
            elapsed = stamp-self.last_seen
            if elapsed > self.cfg['recovery_seconds']:
                self.pending, self.hits = None, 0
                self.state = 'reselect_required'
                return None
            # Compare every identity before spatial gating: an ambiguous lookalike must stop us.
            ranked = sorted(((float(np.max(np.asarray(self.gallery) @ p.embedding)), p)
                             for p in valid), key=lambda v: v[0], reverse=True)
            if not ranked or ranked[0][0] < self.cfg['match_similarity']:
                self.pending, self.hits = None, 0
                self.state = 'lost'
                return None
            self.similarity, person = ranked[0]
            if len(ranked) > 1 and self.similarity-ranked[1][0] < self.cfg['match_margin']:
                self.pending, self.hits = None, 0
                self.state = 'ambiguous'
                return None
            limit = self.cfg['position_slack'] + self.cfg['max_target_speed']*elapsed
            if np.linalg.norm(person.position[:2]-self.position[:2]) > limit:
                self.pending, self.hits = None, 0
                self.state = 'position_jump'
                return None
        continuing = self.state == 'tracking' and person.track_id == self.track_id
        if not continuing:
            self.hits = self.hits+1 if self.pending == person.track_id else 1
            self.pending = person.track_id
            if self.hits < self.cfg['confirm_frames']:
                self.state = 'confirming'
                return None
        if not self.gallery or (self.similarity >= self.cfg['update_similarity']
                                and person.confidence >= self.cfg['enroll_confidence']
                                and stamp-self.last_update >= .5):
            self.gallery.append(person.embedding.copy())
            self.last_update = stamp
        # Do not smooth through identity loss; no prediction is emitted without an observation.
        self.position = (.6*person.position + .4*self.position
                         if continuing else person.position.copy())
        self.last_seen, self.track_id = stamp, person.track_id
        self.state = 'tracking'
        return Person(person.track_id, self.position.copy(), person.embedding,
                      person.confidence, person.quality)
