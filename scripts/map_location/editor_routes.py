#!/usr/bin/env python3
"""Clearance masks and deterministic obstacle-safe route smoothing."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Mapping, Sequence

import cv2
import numpy as np
import yaml

from editor_geometry import build_obstacle_mask
from editor_map_io import MapData
from editor_store import route_input_sha256

ALGORITHM = "adaptive_corner_rounding_v1"


def resolve_clearance(config_path: Path) -> tuple[float, str]:
    try:
        data = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return 0.7, "fallback"
    node = next(iter(data.values()), {})
    params = node.get("ros__parameters", {}) if isinstance(node, dict) else {}
    try:
        if "inflation_radius_m" in params:
            return round(float(params["inflation_radius_m"]), 3), "config"
        required = (
            "vehicle_front_m",
            "vehicle_rear_m",
            "vehicle_left_m",
            "vehicle_right_m",
        )
        if all(key in params for key in required):
            margin = float(params.get("vehicle_margin_m", 0.0))
            radius = max(
                math.hypot(float(params[front]), float(params[side]))
                for front in ("vehicle_front_m", "vehicle_rear_m")
                for side in ("vehicle_left_m", "vehicle_right_m")
            )
            return round(radius + margin, 3), "vehicle_footprint"
    except (TypeError, ValueError):
        return 0.7, "fallback"
    return 0.7, "fallback"


def build_clearance_mask(
    map_data: MapData,
    keepouts: Sequence[Mapping[str, object]],
    clearance_m: float,
) -> np.ndarray:
    obstacle = build_obstacle_mask(map_data, keepouts)
    free = ((1 - obstacle) * 255).astype(np.uint8)
    # The world outside the finite occupancy grid is unknown and therefore
    # non-traversable. Mark the perimeter conservatively before measuring
    # distance so a robot footprint cannot extend beyond the map.
    free[[0, -1], :] = 0
    free[:, [0, -1]] = 0
    distance_px = cv2.distanceTransform(free, cv2.DIST_L2, 5)
    threshold_px = max(
        1e-6, max(0.0, float(clearance_m)) / map_data.resolution
    )
    return distance_px >= threshold_px


def _sample_line(
    start: tuple[float, float],
    end: tuple[float, float],
    spacing: float,
) -> list[tuple[float, float]]:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(math.ceil(distance / max(spacing, 1e-6))))
    return [
        (
            start[0] + (end[0] - start[0]) * index / count,
            start[1] + (end[1] - start[1]) * index / count,
        )
        for index in range(count + 1)
    ]


def is_point_safe(
    map_data: MapData, traversable: np.ndarray, point: tuple[float, float]
) -> bool:
    px, py = map_data.map_to_pixel(*point)
    # Map coordinates address occupancy cells by their lower-left extent.
    # Horizontal indices therefore floor, while the vertically flipped image
    # row uses ceil. Small epsilons keep exact grid lines deterministic.
    ix = int(math.floor(px + 1e-9))
    iy = int(math.ceil(py - 1e-9))
    return (
        0 <= ix < map_data.width
        and 0 <= iy < map_data.height
        and bool(traversable[iy, ix])
    )


def _sample_quadratic(
    start: tuple[float, float],
    control: tuple[float, float],
    end: tuple[float, float],
    spacing: float,
) -> list[tuple[float, float]]:
    length = math.hypot(control[0] - start[0], control[1] - start[1])
    length += math.hypot(end[0] - control[0], end[1] - control[1])
    count = max(2, int(math.ceil(length / max(spacing, 1e-6))))
    points = []
    for index in range(count + 1):
        t = index / count
        one = 1.0 - t
        points.append(
            (
                one * one * start[0]
                + 2.0 * one * t * control[0]
                + t * t * end[0],
                one * one * start[1]
                + 2.0 * one * t * control[1]
                + t * t * end[1],
            )
        )
    return points


def _corner_candidate(
    previous: tuple[float, float],
    corner: tuple[float, float],
    following: tuple[float, float],
    scale: float,
    spacing: float,
) -> tuple[tuple[float, float], list[tuple[float, float]], tuple[float, float]]:
    incoming = (corner[0] - previous[0], corner[1] - previous[1])
    outgoing = (following[0] - corner[0], following[1] - corner[1])
    length_in = math.hypot(*incoming)
    length_out = math.hypot(*outgoing)
    if length_in < 1e-9 or length_out < 1e-9:
        return corner, [corner], corner
    radius = min(length_in, length_out) * 0.25 * scale
    entry = (
        corner[0] - incoming[0] / length_in * radius,
        corner[1] - incoming[1] / length_in * radius,
    )
    exit_point = (
        corner[0] + outgoing[0] / length_out * radius,
        corner[1] + outgoing[1] / length_out * radius,
    )
    return entry, _sample_quadratic(entry, corner, exit_point, spacing), exit_point


def _append_line(
    output: list[tuple[float, float]],
    target: tuple[float, float],
    spacing: float,
) -> None:
    points = _sample_line(output[-1], target, spacing)
    output.extend(points[1:])


def _round_point(point: tuple[float, float]) -> dict[str, float]:
    return {"x": round(float(point[0]), 3), "y": round(float(point[1]), 3)}


def smooth_route(
    map_data: MapData,
    route: Mapping[str, object],
    keepouts: Sequence[Mapping[str, object]],
    clearance_m: float,
) -> dict[str, object]:
    waypoints = route.get("waypoints", [])
    points = [(float(item["x"]), float(item["y"])) for item in waypoints]
    closed = bool(route.get("closed", False))
    traversable = build_clearance_mask(map_data, keepouts, clearance_m)
    validation_spacing = map_data.resolution * 0.5
    input_hash = route_input_sha256(
        map_data.source_sha256, list(keepouts), route, clearance_m
    )
    base_result = {
        "algorithm": ALGORITHM,
        "clearance_m": round(float(clearance_m), 3),
        "sample_spacing_m": round(map_data.resolution, 3),
        "input_sha256": input_hash,
        "fallback_waypoint_ids": [],
        "collisions": [],
        "points": [],
    }

    spacing = map_data.resolution
    fallback_ids = []
    corner_parts = {}
    indices = range(len(points)) if closed else range(1, len(points) - 1)
    for index in indices:
        previous = points[(index - 1) % len(points)]
        corner = points[index]
        following = points[(index + 1) % len(points)]
        chosen = None
        for scale in (1.0, 0.75, 0.5, 0.25, 0.125):
            candidate = _corner_candidate(previous, corner, following, scale, spacing)
            if all(is_point_safe(map_data, traversable, item) for item in candidate[1]):
                chosen = candidate
                break
        if chosen is None:
            chosen = (corner, [corner], corner)
            fallback_ids.append(str(waypoints[index]["id"]))
        corner_parts[index] = chosen

    if not closed:
        output = [points[0]]
        for index in range(1, len(points) - 1):
            entry, curve, _ = corner_parts[index]
            _append_line(output, entry, spacing)
            output.extend(curve[1:])
        _append_line(output, points[-1], spacing)
    else:
        _, _, first_exit = corner_parts[0]
        output = [first_exit]
        for index in range(1, len(points)):
            entry, curve, _ = corner_parts[index]
            _append_line(output, entry, spacing)
            output.extend(curve[1:])
        first_entry, first_curve, _ = corner_parts[0]
        _append_line(output, first_entry, spacing)
        output.extend(first_curve[1:])
        output[-1] = output[0]

    collisions = []
    for segment_index in range(len(output) - 1):
        start, end = output[segment_index], output[segment_index + 1]
        collision_point = next(
            (
                point
                for point in _sample_line(start, end, validation_spacing)
                if not is_point_safe(map_data, traversable, point)
            ),
            None,
        )
        if collision_point is not None:
            collisions.append(
                {
                    "segment_index": segment_index,
                    "x": round(collision_point[0], 3),
                    "y": round(collision_point[1], 3),
                }
            )

    status = (
        "invalid"
        if collisions
        else "valid_with_fallbacks" if fallback_ids else "valid"
    )
    return {
        **base_result,
        "status": status,
        "fallback_waypoint_ids": fallback_ids,
        "collisions": collisions,
        "points": [_round_point(point) for point in output],
    }
