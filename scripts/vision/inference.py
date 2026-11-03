"""YOLO + upstream BoT-SORT with explicit OSNet embeddings."""
from types import SimpleNamespace

import cv2
import numpy as np
import torch
import yaml
from ultralytics.data.augment import LetterBox
from ultralytics.engine.results import Boxes
from ultralytics.trackers.bot_sort import BOTSORT
from ultralytics.utils import ops
from ultralytics.utils.nms import non_max_suppression

from .geometry import target_depth
from .identity import Person
from .settings import model_dir, share_dir
from .tensorrt_engine import Engine


class Tracker:
    def __init__(self, config):
        self.cfg = config
        torch.set_num_threads(2)
        cv2.setNumThreads(1)
        self.detector = Engine(model_dir()/'yolov8n.engine')
        try:
            self.reid = Engine(model_dir()/'osnet.engine')
        except Exception:
            self.detector.close()
            raise
        with open(share_dir()/'config/person_tracker.yaml') as stream:
            tracker_config = yaml.safe_load(stream)
        self.tracker = BOTSORT(SimpleNamespace(**tracker_config), frame_rate=30)
        # BYTETracker partitions supplied embeddings with their detections before init_track.
        self.tracker.encoder = lambda features, _: features
        self.letterbox = LetterBox((320, 320), auto=False, stride=32)
        self.last_stamp = None
        self.visible_count = 0
        self.mean = np.array([.485, .456, .406], np.float32)[:, None, None]
        self.std = np.array([.229, .224, .225], np.float32)[:, None, None]

    def process(self, image, depth, intrinsic, mount, stamp):
        if self.last_stamp is not None and stamp-self.last_stamp > self.cfg['frame_timeout']:
            self.tracker.reset()
        self.last_stamp = stamp
        resized = self.letterbox(image=image)
        tensor = np.ascontiguousarray(resized[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32)/255.
        output = torch.from_numpy(self.detector(tensor).copy())
        detections = non_max_suppression(output, conf_thres=.1, iou_thres=.5,
                                           classes=[0], max_det=self.cfg['max_people']+1)[0]
        self.visible_count = len(detections)
        if len(detections) > self.cfg['max_people']:
            return [], 'crowd_limit'
        ops.scale_boxes((320, 320), detections[:, :4], image.shape[:2])
        boxes = Boxes(detections.numpy(), image.shape[:2])
        embeddings = []
        for box in boxes.xyxy:
            x1, y1, x2, y2 = self.crop_box(box, image.shape)
            if x2 <= x1 or y2 <= y1:
                raise ValueError('Empty detection crop')
            crop = cv2.resize(image[y1:y2, x1:x2, ::-1], (128, 256))
            blob = (crop.transpose(2, 0, 1).astype(np.float32)/255.-self.mean)/self.std
            feature = self.reid(np.ascontiguousarray(blob[None])).reshape(-1).copy()
            norm = np.linalg.norm(feature)
            if not np.isfinite(norm) or norm < 1e-8:
                raise RuntimeError('Invalid OSNet embedding')
            embeddings.append(feature/norm)
        features = np.asarray(embeddings, dtype=np.float32).reshape(len(boxes), -1) if len(boxes) else None
        self.tracker.update(boxes, image, features)
        people = []
        for track in self.tracker.tracked_stracks:
            if not track.is_activated or track.frame_id != self.tracker.frame_id:
                continue
            # Identity validation uses this frame's feature, not the track's accumulated feature.
            box = track.xyxy
            x1, y1, x2, y2 = self.crop_box(box, image.shape)
            roi = image[y1:y2, x1:x2]
            quality = (y2-y1 >= self.cfg['min_crop_height'] and x1 > 0 and y1 > 0
                       and x2 < image.shape[1] and y2 < image.shape[0] and roi.size > 0)
            if quality:
                quality = cv2.Laplacian(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), cv2.CV_32F).var() >= self.cfg['min_sharpness']
            area = max(1., (x2-x1)*(y2-y1))
            for other in self.tracker.tracked_stracks:
                if other is track or other.frame_id != self.tracker.frame_id:
                    continue
                a, b, c, d = other.xyxy
                overlap = max(0, min(x2, c)-max(x1, a))*max(0, min(y2, d)-max(y1, b))/area
                if overlap > self.cfg['max_overlap']:
                    quality = False
            distance = target_depth(depth, box)
            position = np.full(3, np.nan)
            if distance is not None:
                fx, fy, cx, cy = intrinsic
                position = np.asarray(mount.target_in_base(distance, (x1+x2)/2,
                                      y1+.425*(y2-y1), fx, fy, cx, cy))
            people.append(Person(track.track_id, position, track.curr_feat.copy(), float(track.score), bool(quality)))
        return people, 'ok'

    @staticmethod
    def crop_box(box, shape):
        x1, y1, x2, y2 = np.rint(box).astype(int)
        return max(0, x1), max(0, y1), min(shape[1], x2), min(shape[0], y2)

    def close(self):
        self.detector.close()
        self.reid.close()
