#!/usr/bin/env python3
"""One-shot real-robot global localization from a saved 2D map and /scan.

The real robot uses scan_fusion_node, whose fused /scan is already published in
base_link. The default scan_to_base transform is therefore identity.
"""

import math
import re
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

CONTROL_DIR = Path(__file__).resolve().parent
if str(CONTROL_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROL_DIR))
MAP_ANNOTATE_DIR = Path(__file__).resolve().parents[1] / "map_annotate"
if str(MAP_ANNOTATE_DIR) not in sys.path:
    sys.path.insert(0, str(MAP_ANNOTATE_DIR))

from map_utils import detect_running_map_file, resolve_maps_dir


Pose = Tuple[float, float, float]


def set_yaw_orientation(orientation, yaw_rad: float) -> None:
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw_rad * 0.5)
    orientation.w = math.cos(yaw_rad * 0.5)


def _parse_simple_yaml(path: Path) -> Dict[str, object]:
    data: Dict[str, object] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if value.startswith("[") and value.endswith("]"):
            items: List[object] = []
            for item in value[1:-1].split(","):
                item = item.strip()
                if not item:
                    continue
                try:
                    items.append(float(item))
                except ValueError:
                    items.append(item.strip("'\""))
            data[key] = items
            continue
        try:
            data[key] = float(value) if "." in value or "e" in value.lower() else int(value)
            continue
        except ValueError:
            data[key] = value.strip("'\"")
    return data


def _read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    idx = 0
    tokens: List[bytes] = []
    while len(tokens) < 4:
        while idx < len(data) and chr(data[idx]).isspace():
            idx += 1
        if idx >= len(data):
            raise ValueError(f"invalid PGM header: {path}")
        if data[idx] == ord("#"):
            while idx < len(data) and data[idx] not in (10, 13):
                idx += 1
            continue
        start = idx
        while idx < len(data) and not chr(data[idx]).isspace():
            idx += 1
        tokens.append(data[start:idx])

    magic = tokens[0].decode("ascii")
    width = int(tokens[1])
    height = int(tokens[2])
    max_val = int(tokens[3])
    if max_val <= 0 or max_val > 255:
        raise ValueError(f"unsupported PGM max value {max_val}: {path}")

    while idx < len(data) and chr(data[idx]).isspace():
        idx += 1

    payload = data[idx:]
    if magic == "P5":
        pixels = np.frombuffer(payload[: width * height], dtype=np.uint8)
    elif magic == "P2":
        values = [int(x) for x in re.findall(rb"\d+", payload)]
        pixels = np.asarray(values[: width * height], dtype=np.uint8)
    else:
        raise ValueError(f"unsupported PGM format {magic}: {path}")

    if pixels.size != width * height:
        raise ValueError(f"PGM pixel count mismatch: {path}")
    return pixels.reshape((height, width))


def scan_xy_to_base_xy(
    laser_xy,
    scan_to_base_x_m: float,
    scan_to_base_y_m: float,
    scan_to_base_yaw_rad: float,
) -> np.ndarray:
    pts = np.asarray(laser_xy, dtype=np.float32)
    c = math.cos(scan_to_base_yaw_rad)
    s = math.sin(scan_to_base_yaw_rad)
    base_x = pts[:, 0] * c - pts[:, 1] * s + float(scan_to_base_x_m)
    base_y = pts[:, 0] * s + pts[:, 1] * c + float(scan_to_base_y_m)
    return np.column_stack((base_x, base_y)).astype(np.float32)


def build_limited_fallback_candidates(
    fast_candidates: List[Tuple[float, Pose]],
    free_xy,
    radius_m: float,
    max_candidates: int,
) -> np.ndarray:
    """Return free translations near fast coarse candidates for bounded fallback."""
    free = np.asarray(free_xy, dtype=np.float32)
    if free.size == 0 or not fast_candidates:
        return np.empty((0, 2), dtype=np.float32)
    free = free.reshape((-1, 2))
    centers = np.asarray([(pose[0], pose[1]) for _, pose in fast_candidates], dtype=np.float32)
    radius = max(0.0, float(radius_m))
    keep = np.zeros((free.shape[0],), dtype=bool)
    radius_sq = radius * radius
    for start in range(0, centers.shape[0], 256):
        chunk = centers[start : start + 256]
        delta = free[:, None, :] - chunk[None, :, :]
        keep |= np.any(np.sum(delta * delta, axis=2) <= radius_sq, axis=1)
    limited = free[keep]
    if max_candidates > 0 and limited.shape[0] > max_candidates:
        idx = np.linspace(0, limited.shape[0] - 1, max_candidates).astype(np.int32)
        limited = limited[idx]
    return limited.astype(np.float32)


class AutoLocalize(Node):
    def __init__(self) -> None:
        super().__init__("relocate")

        self.declare_parameter("maps_dir", "")
        self.declare_parameter("map_file", "")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("collect_scan_count", 5)
        self.declare_parameter("max_wait_scan_s", 12.0)
        self.declare_parameter("max_scan_points", 360)
        self.declare_parameter("min_range_m", 0.10)
        self.declare_parameter("max_range_m", 12.0)
        self.declare_parameter("scan_to_base_x_m", 0.0)
        self.declare_parameter("scan_to_base_y_m", 0.0)
        self.declare_parameter("scan_to_base_yaw_deg", 0.0)
        self.declare_parameter("coarse_xy_step_m", 0.25)
        self.declare_parameter("coarse_yaw_step_deg", 10.0)
        self.declare_parameter("enable_fast_coarse_search", True)
        self.declare_parameter("coarse_prefilter_xy_step_m", 0.50)
        self.declare_parameter("coarse_prefilter_top_k", 80)
        self.declare_parameter("coarse_refine_radius_m", 0.35)
        self.declare_parameter("fallback_full_search", True)
        self.declare_parameter("fallback_limited_search", True)
        self.declare_parameter("fallback_limited_radius_m", 1.2)
        self.declare_parameter("fallback_limited_max_candidates", 8000)
        self.declare_parameter("fine_xy_radius_m", 0.30)
        self.declare_parameter("fine_xy_step_m", 0.05)
        self.declare_parameter("fine_yaw_radius_deg", 10.0)
        self.declare_parameter("fine_yaw_step_deg", 2.0)
        self.declare_parameter("top_k", 10)
        self.declare_parameter("max_endpoint_dist_m", 0.55)
        self.declare_parameter("match_sigma_m", 0.16)
        self.declare_parameter("min_score", 0.40)
        self.declare_parameter("min_score_gap", 0.001)
        self.declare_parameter("free_candidate_stride", 1)
        self.declare_parameter("debug", False)

        self.maps_dir = str(self.get_parameter("maps_dir").value).strip() or resolve_maps_dir()
        self.map_file = str(self.get_parameter("map_file").value).strip()
        self.map_yaml = str(self.get_parameter("map_yaml").value).strip()
        if not self.map_file and not self.map_yaml:
            self.map_file = detect_running_map_file(self.maps_dir, require_locations=False)
            if self.map_file:
                self.get_logger().info(f"auto detected running map: {self.map_file}")
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.collect_scan_count = max(1, int(self.get_parameter("collect_scan_count").value))
        self.max_wait_scan_s = max(1.0, float(self.get_parameter("max_wait_scan_s").value))
        self.max_scan_points = max(20, int(self.get_parameter("max_scan_points").value))
        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.scan_to_base_x_m = float(self.get_parameter("scan_to_base_x_m").value)
        self.scan_to_base_y_m = float(self.get_parameter("scan_to_base_y_m").value)
        self.scan_to_base_yaw = math.radians(float(self.get_parameter("scan_to_base_yaw_deg").value))
        self.coarse_xy_step_m = max(0.05, float(self.get_parameter("coarse_xy_step_m").value))
        self.coarse_yaw_step = math.radians(max(1.0, float(self.get_parameter("coarse_yaw_step_deg").value)))
        self.enable_fast_coarse_search = bool(self.get_parameter("enable_fast_coarse_search").value)
        self.coarse_prefilter_xy_step_m = max(
            self.coarse_xy_step_m,
            float(self.get_parameter("coarse_prefilter_xy_step_m").value),
        )
        self.coarse_prefilter_top_k = max(1, int(self.get_parameter("coarse_prefilter_top_k").value))
        self.coarse_refine_radius_m = max(0.0, float(self.get_parameter("coarse_refine_radius_m").value))
        self.fallback_full_search = bool(self.get_parameter("fallback_full_search").value)
        self.fallback_limited_search = bool(self.get_parameter("fallback_limited_search").value)
        self.fallback_limited_radius_m = max(0.0, float(self.get_parameter("fallback_limited_radius_m").value))
        self.fallback_limited_max_candidates = max(100, int(self.get_parameter("fallback_limited_max_candidates").value))
        self.fine_xy_radius_m = max(0.0, float(self.get_parameter("fine_xy_radius_m").value))
        self.fine_xy_step_m = max(0.02, float(self.get_parameter("fine_xy_step_m").value))
        self.fine_yaw_radius = math.radians(max(0.0, float(self.get_parameter("fine_yaw_radius_deg").value)))
        self.fine_yaw_step = math.radians(max(0.5, float(self.get_parameter("fine_yaw_step_deg").value)))
        self.top_k = max(1, int(self.get_parameter("top_k").value))
        self.max_endpoint_dist_m = max(0.05, float(self.get_parameter("max_endpoint_dist_m").value))
        self.match_sigma_m = max(0.02, float(self.get_parameter("match_sigma_m").value))
        self.min_score = float(self.get_parameter("min_score").value)
        self.min_score_gap = float(self.get_parameter("min_score_gap").value)
        self.free_candidate_stride = max(1, int(self.get_parameter("free_candidate_stride").value))
        self.debug = bool(self.get_parameter("debug").value)

        self.scan_points: List[np.ndarray] = []
        self.started_mono = time.monotonic()
        self.done = False
        self.accepted = False

        self.pub_initial = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_timer(0.25, self._check_timeout)

        self._load_map()
        self.get_logger().info(
            "relocate waiting for %d scan(s) | map=%s | candidates=%d"
            % (self.collect_scan_count, self.yaml_path, len(self.free_xy))
        )

    def _load_map(self) -> None:
        if self.map_yaml:
            yaml_path = Path(self.map_yaml)
        else:
            if not self.maps_dir:
                raise RuntimeError("maps_dir/map_yaml not set and maps directory could not be resolved")
            if not self.map_file:
                raise RuntimeError("map_file could not be auto-detected; pass -p map_file:=<name>")
            yaml_path = Path(self.maps_dir) / self.map_file / f"{self.map_file}.yaml"
        if not yaml_path.exists():
            raise RuntimeError(f"map yaml not found: {yaml_path}")

        meta = _parse_simple_yaml(yaml_path)
        image_name = str(meta.get("image", yaml_path.with_suffix(".pgm").name))
        pgm_path = (yaml_path.parent / image_name).resolve()
        if not pgm_path.exists():
            raise RuntimeError(f"map image not found: {pgm_path}")

        self.yaml_path = yaml_path
        self.resolution = float(meta.get("resolution", 0.05))
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        if not isinstance(origin, list) or len(origin) < 3:
            origin = [0.0, 0.0, 0.0]
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2])
        self.negate = int(meta.get("negate", 0))
        self.occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        self.free_thresh = float(meta.get("free_thresh", 0.25))

        pixels = _read_pgm(pgm_path)
        self.height, self.width = pixels.shape
        scaled = pixels.astype(np.float32) / 255.0
        occ_prob = scaled if self.negate else 1.0 - scaled
        self.occupied = occ_prob >= self.occupied_thresh
        self.free = occ_prob <= self.free_thresh

        binary = np.where(self.occupied, 0, 255).astype(np.uint8)
        self.distance_m = cv2.distanceTransform(binary, cv2.DIST_L2, 5).astype(np.float32) * self.resolution
        self._build_free_candidates()

    def _build_free_candidates(self) -> None:
        self.free_xy = self._build_free_candidates_for_step(self.coarse_xy_step_m)
        if self.enable_fast_coarse_search and self.coarse_prefilter_xy_step_m > self.coarse_xy_step_m:
            self.prefilter_free_xy = self._build_free_candidates_for_step(self.coarse_prefilter_xy_step_m)
        else:
            self.prefilter_free_xy = self.free_xy

    def _build_free_candidates_for_step(self, step_m: float) -> np.ndarray:
        step_px = max(1, int(round(step_m / self.resolution)))
        free_rows, free_cols = np.nonzero(self.free[::step_px, ::step_px])
        rows = free_rows * step_px
        cols = free_cols * step_px
        if self.free_candidate_stride > 1:
            rows = rows[:: self.free_candidate_stride]
            cols = cols[:: self.free_candidate_stride]
        xs, ys = self._pixel_to_world(cols.astype(np.float32), rows.astype(np.float32))
        return np.column_stack((xs, ys)).astype(np.float32)

    def _on_scan(self, msg: LaserScan) -> None:
        if self.done:
            return
        pts = self._scan_to_points(msg)
        if pts.shape[0] < 20:
            self.get_logger().warn(f"scan has too few usable points: {pts.shape[0]}")
            return
        self.scan_points.append(pts)
        if len(self.scan_points) >= self.collect_scan_count:
            merged = self._merge_scans(self.scan_points)
            self._run_localization(merged)

    def _scan_to_points(self, msg: LaserScan) -> np.ndarray:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        finite = np.isfinite(ranges)
        rmin = max(float(msg.range_min), self.min_range_m)
        rmax = min(float(msg.range_max), self.max_range_m)
        valid = finite & (ranges >= rmin) & (ranges <= rmax)
        idx = np.nonzero(valid)[0]
        if idx.size > self.max_scan_points:
            keep = np.linspace(0, idx.size - 1, self.max_scan_points).astype(np.int32)
            idx = idx[keep]
        angles = float(msg.angle_min) + idx.astype(np.float32) * float(msg.angle_increment)
        rr = ranges[idx]
        laser_xy = np.column_stack((rr * np.cos(angles), rr * np.sin(angles))).astype(np.float32)
        return scan_xy_to_base_xy(
            laser_xy,
            self.scan_to_base_x_m,
            self.scan_to_base_y_m,
            self.scan_to_base_yaw,
        )

    def _merge_scans(self, scans: List[np.ndarray]) -> np.ndarray:
        pts = np.concatenate(scans, axis=0)
        if pts.shape[0] <= self.max_scan_points:
            return pts
        keep = np.linspace(0, pts.shape[0] - 1, self.max_scan_points).astype(np.int32)
        return pts[keep]

    def _run_localization(self, scan_xy: np.ndarray) -> None:
        self.done = True
        start = time.monotonic()
        coarse = self._coarse_search(scan_xy)
        fine = self._fine_search(scan_xy, coarse[: self.top_k])
        best, second_score, gap = self._rank_scores(fine)

        if getattr(self, "fallback_full_search", True) and not self._accepted_scores(best[0], gap):
            if getattr(self, "fallback_limited_search", True):
                limited_xy = build_limited_fallback_candidates(
                    coarse,
                    self.free_xy,
                    self.fallback_limited_radius_m,
                    self.fallback_limited_max_candidates,
                )
                self.get_logger().info(
                    "fast relocate confidence low; bounded fallback candidates=%d/%d"
                    % (len(limited_xy), len(self.free_xy))
                )
                coarse = self._coarse_search_over_translations(
                    scan_xy,
                    limited_xy,
                    per_yaw_top_k=self.top_k,
                    keep_count=max(self.top_k * 3, self.top_k),
                )
            else:
                self.get_logger().info(
                    "fast relocate confidence low; falling back to full coarse search"
                )
                coarse = self._full_coarse_search(scan_xy)
            fine = self._fine_search(scan_xy, coarse[: self.top_k])
            best, second_score, gap = self._rank_scores(fine)

        elapsed = time.monotonic() - start
        self.get_logger().info(
            "relocate best score=%.3f second=%.3f gap=%.3f pose=(%.3f, %.3f, %.1fdeg) time=%.2fs"
            % (best[0], second_score, gap, best[1][0], best[1][1], math.degrees(best[1][2]), elapsed)
        )
        if self.debug:
            for i, (score, pose) in enumerate(fine[: min(5, len(fine))], start=1):
                self.get_logger().info(
                    "candidate %d score=%.3f pose=(%.3f, %.3f, %.1fdeg)"
                    % (i, score, pose[0], pose[1], math.degrees(pose[2]))
                )
        if not self._accepted_scores(best[0], gap):
            self.get_logger().warn(
                "auto localization rejected: score/gap below threshold; use manual initial pose"
            )
            return

        self._publish_initialpose(best[1])
        self.accepted = True
        self.get_logger().info("auto localization accepted and /initialpose published")

    def _rank_scores(self, ranked: List[Tuple[float, Pose]]) -> Tuple[Tuple[float, Pose], float, float]:
        best = ranked[0] if ranked else (-1.0, (0.0, 0.0, 0.0))
        second_score = ranked[1][0] if len(ranked) > 1 else -1.0
        return best, second_score, best[0] - second_score

    def _accepted_scores(self, score: float, gap: float) -> bool:
        return score >= self.min_score and gap >= self.min_score_gap

    def _coarse_search(self, scan_xy: np.ndarray) -> List[Tuple[float, Pose]]:
        if (
            not self.enable_fast_coarse_search
            or self.prefilter_free_xy is self.free_xy
            or self.coarse_refine_radius_m <= 0.0
        ):
            return self._full_coarse_search(scan_xy)

        prefilter_keep = max(self.coarse_prefilter_top_k, self.top_k * 3)
        prefilter = self._coarse_search_over_translations(
            scan_xy,
            self.prefilter_free_xy,
            per_yaw_top_k=max(1, min(self.top_k, self.coarse_prefilter_top_k)),
            keep_count=prefilter_keep,
        )
        refined = self._refine_coarse_candidates(scan_xy, prefilter[:prefilter_keep])
        if self.debug:
            self.get_logger().info(
                "fast coarse candidates: prefilter=%d/%d refined=%d full_candidates=%d"
                % (len(prefilter), len(self.prefilter_free_xy), len(refined), len(self.free_xy))
            )
        return refined or prefilter[: max(self.top_k * 3, self.top_k)]

    def _full_coarse_search(self, scan_xy: np.ndarray) -> List[Tuple[float, Pose]]:
        return self._coarse_search_over_translations(
            scan_xy,
            self.free_xy,
            per_yaw_top_k=self.top_k,
            keep_count=max(self.top_k * 3, self.top_k),
        )

    def _coarse_search_over_translations(
        self,
        scan_xy: np.ndarray,
        translations: np.ndarray,
        *,
        per_yaw_top_k: int,
        keep_count: int,
    ) -> List[Tuple[float, Pose]]:
        results: List[Tuple[float, Pose]] = []
        yaws = np.arange(-math.pi, math.pi, self.coarse_yaw_step, dtype=np.float32)
        for yaw in yaws:
            local = self._rotate_points(scan_xy, float(yaw))
            scores = self._score_translations(local, translations)
            if scores.size == 0:
                continue
            count = min(max(1, per_yaw_top_k), scores.size)
            top_idx = np.argpartition(scores, -count)[-count:]
            for idx in top_idx:
                x = float(translations[idx, 0])
                y = float(translations[idx, 1])
                results.append((float(scores[idx]), (x, y, float(yaw))))
        results.sort(key=lambda item: item[0], reverse=True)
        return results[:keep_count]

    def _refine_coarse_candidates(
        self, scan_xy: np.ndarray, seeds: List[Tuple[float, Pose]]
    ) -> List[Tuple[float, Pose]]:
        if not seeds:
            return []

        results: List[Tuple[float, Pose]] = []
        steps = int(math.ceil(self.coarse_refine_radius_m / self.coarse_xy_step_m))
        offsets = (np.arange(-steps, steps + 1, dtype=np.float32) * self.coarse_xy_step_m)
        xy_offsets = np.asarray([(dx, dy) for dx in offsets for dy in offsets], dtype=np.float32)
        yaws = sorted({round(seed[1][2], 6) for seed in seeds})

        for yaw_key in yaws:
            centers = np.asarray(
                [(pose[0], pose[1]) for _, pose in seeds if round(pose[2], 6) == yaw_key],
                dtype=np.float32,
            )
            if centers.size == 0:
                continue
            translations = (centers[:, None, :] + xy_offsets[None, :, :]).reshape(-1, 2)
            cols, rows = self._world_to_pixel(translations[:, 0], translations[:, 1])
            in_bounds = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
            if not np.any(in_bounds):
                continue
            cols = cols[in_bounds]
            rows = rows[in_bounds]
            translations = translations[in_bounds]
            keep = self.free[rows, cols]
            if not np.any(keep):
                continue
            cols = cols[keep]
            rows = rows[keep]
            translations = translations[keep]
            _, unique_idx = np.unique(np.column_stack((rows, cols)), axis=0, return_index=True)
            translations = translations[np.sort(unique_idx)]

            yaw = float(yaw_key)
            local = self._rotate_points(scan_xy, yaw)
            scores = self._score_translations(local, translations)
            if scores.size == 0:
                continue
            count = min(self.top_k, scores.size)
            top_idx = np.argpartition(scores, -count)[-count:]
            for idx in top_idx:
                results.append(
                    (
                        float(scores[idx]),
                        (float(translations[idx, 0]), float(translations[idx, 1]), yaw),
                    )
                )

        results.sort(key=lambda item: item[0], reverse=True)
        return results[: max(self.top_k * 3, self.top_k)]

    def _fine_search(self, scan_xy: np.ndarray, seeds: List[Tuple[float, Pose]]) -> List[Tuple[float, Pose]]:
        results: List[Tuple[float, Pose]] = []
        if not seeds:
            return [(-1.0, (0.0, 0.0, 0.0))]
        xy_offsets = np.arange(-self.fine_xy_radius_m, self.fine_xy_radius_m + 1e-6, self.fine_xy_step_m)
        yaw_offsets = np.arange(-self.fine_yaw_radius, self.fine_yaw_radius + 1e-6, self.fine_yaw_step)
        for _, seed in seeds:
            sx, sy, syaw = seed
            translations = np.asarray(
                [(sx + dx, sy + dy) for dx in xy_offsets for dy in xy_offsets],
                dtype=np.float32,
            )
            for dyaw in yaw_offsets:
                yaw = self._norm_angle(syaw + float(dyaw))
                local = self._rotate_points(scan_xy, yaw)
                scores = self._score_translations(local, translations)
                if scores.size == 0:
                    continue
                best_idx = int(np.argmax(scores))
                results.append(
                    (float(scores[best_idx]), (float(translations[best_idx, 0]), float(translations[best_idx, 1]), yaw))
                )
        results.sort(key=lambda item: item[0], reverse=True)
        deduped: List[Tuple[float, Pose]] = []
        for score, pose in results:
            if all(math.hypot(pose[0] - p[1][0], pose[1] - p[1][1]) > 0.10 or abs(self._angle_diff(pose[2], p[1][2])) > math.radians(5.0) for p in deduped):
                deduped.append((score, pose))
            if len(deduped) >= max(2, self.top_k):
                break
        return deduped or results[: max(2, self.top_k)]

    def _score_translations(self, rotated_scan: np.ndarray, translations: np.ndarray) -> np.ndarray:
        scores = np.full((translations.shape[0],), -1.0, dtype=np.float32)
        for start in range(0, translations.shape[0], 2048):
            chunk = translations[start : start + 2048]
            center_cols, center_rows = self._world_to_pixel(chunk[:, 0], chunk[:, 1])
            center_ok = (
                (center_cols >= 0)
                & (center_cols < self.width)
                & (center_rows >= 0)
                & (center_rows < self.height)
            )
            center_rows_safe = np.clip(center_rows, 0, self.height - 1)
            center_cols_safe = np.clip(center_cols, 0, self.width - 1)
            center_ok &= self.free[center_rows_safe, center_cols_safe]
            world = rotated_scan[None, :, :] + chunk[:, None, :]
            cols, rows = self._world_to_pixel(world[:, :, 0], world[:, :, 1])
            in_bounds = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
            valid_ratio = in_bounds.mean(axis=1)
            safe_rows = np.clip(rows, 0, self.height - 1)
            safe_cols = np.clip(cols, 0, self.width - 1)
            dists = self.distance_m[safe_rows, safe_cols]
            dists = np.where(in_bounds, np.minimum(dists, self.max_endpoint_dist_m), self.max_endpoint_dist_m)
            endpoint_score = np.exp(-dists / self.match_sigma_m).mean(axis=1)
            scores[start : start + chunk.shape[0]] = endpoint_score * valid_ratio * center_ok.astype(np.float32)
        return scores

    def _publish_initialpose(self, pose: Pose) -> None:
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        set_yaw_orientation(msg.pose.pose.orientation, yaw)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        for _ in range(3):
            self.pub_initial.publish(msg)

    def _check_timeout(self) -> None:
        if self.done:
            return
        if time.monotonic() - self.started_mono > self.max_wait_scan_s:
            self.done = True
            self.get_logger().warn("auto localization timed out waiting for scans")

    def _world_to_pixel(self, x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        dx = x - self.origin_x
        dy = y - self.origin_y
        c = math.cos(-self.origin_yaw)
        s = math.sin(-self.origin_yaw)
        mx = (dx * c - dy * s) / self.resolution
        my = (dx * s + dy * c) / self.resolution
        cols = np.floor(mx).astype(np.int32)
        rows = (self.height - 1 - np.floor(my)).astype(np.int32)
        return cols, rows

    def _pixel_to_world(self, cols: np.ndarray, rows: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        mx = (cols + 0.5) * self.resolution
        my = (self.height - 1 - rows + 0.5) * self.resolution
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        x = self.origin_x + mx * c - my * s
        y = self.origin_y + mx * s + my * c
        return x, y

    @staticmethod
    def _rotate_points(points: np.ndarray, yaw: float) -> np.ndarray:
        c = math.cos(yaw)
        s = math.sin(yaw)
        x = points[:, 0] * c - points[:, 1] * s
        y = points[:, 0] * s + points[:, 1] * c
        return np.column_stack((x, y)).astype(np.float32)

    @staticmethod
    def _norm_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return AutoLocalize._norm_angle(a - b)


def main() -> None:
    rclpy.init()
    node: Optional[AutoLocalize] = None
    try:
        node = AutoLocalize()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.accepted:
            time.sleep(0.2)
        else:
            raise SystemExit(2)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if rclpy.ok():
            Node("relocate_error").get_logger().error(str(exc))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
