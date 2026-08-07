#!/usr/bin/env python3
"""Geometry helpers for rotated square keepouts and occupancy masks."""

from __future__ import annotations

import math
from typing import Mapping, Sequence

import cv2
import numpy as np

from editor_map_io import MapData


def rectangle_vertices(zone: Mapping[str, object]) -> list[tuple[float, float]]:
    center = zone["center"]
    cx, cy = float(center["x"]), float(center["y"])
    half_width = float(zone["width_m"]) / 2.0
    half_height = float(zone["height_m"]) / 2.0
    yaw = math.radians(float(zone.get("yaw_deg", 0.0)))
    c, s = math.cos(yaw), math.sin(yaw)
    return [
        (cx + c * dx - s * dy, cy + s * dx + c * dy)
        for dx, dy in (
            (-half_width, -half_height),
            (half_width, -half_height),
            (half_width, half_height),
            (-half_width, half_height),
        )
    ]


def point_in_rectangle(
    point: tuple[float, float], zone: Mapping[str, object]
) -> bool:
    center = zone["center"]
    dx = float(point[0]) - float(center["x"])
    dy = float(point[1]) - float(center["y"])
    yaw = -math.radians(float(zone.get("yaw_deg", 0.0)))
    c, s = math.cos(yaw), math.sin(yaw)
    local_x = c * dx - s * dy
    local_y = s * dx + c * dy
    half_width = float(zone["width_m"]) / 2.0
    half_height = float(zone["height_m"]) / 2.0
    return abs(local_x) <= half_width and abs(local_y) <= half_height


def rasterize_keepouts(
    map_data: MapData, keepouts: Sequence[Mapping[str, object]]
) -> np.ndarray:
    mask = np.zeros((map_data.height, map_data.width), dtype=np.uint8)
    for zone in keepouts:
        pixels = np.asarray(
            [map_data.map_to_pixel(x, y) for x, y in rectangle_vertices(zone)],
            dtype=np.float64,
        )
        cv2.fillPoly(mask, [np.rint(pixels).astype(np.int32)], 1)
    return mask


def build_obstacle_mask(
    map_data: MapData, keepouts: Sequence[Mapping[str, object]]
) -> np.ndarray:
    base = (map_data.occupancy != 0).astype(np.uint8)
    return np.maximum(base, rasterize_keepouts(map_data, keepouts))
