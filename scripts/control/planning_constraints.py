#!/usr/bin/env python3
"""Load editor constraints and merge keepouts into a ROS occupancy grid."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import yaml

try:
    from editor_geometry import rectangle_vertices
    from editor_map_io import validate_map_name
    from editor_store import DocumentError, normalize_document
except ModuleNotFoundError:
    # colcon --symlink-install executes this file from scripts/control.
    editor_module_dir = Path(__file__).resolve().parents[1] / "map_location"
    if editor_module_dir.is_dir():
        sys.path.insert(0, str(editor_module_dir))
    from editor_geometry import rectangle_vertices
    from editor_map_io import validate_map_name
    from editor_store import DocumentError, normalize_document


@dataclass(frozen=True)
class PlanningConstraints:
    keepouts: tuple[dict[str, object], ...]
    clearance_m: float
    clearance_source: str
    revision: int
    editor_path: Path | None


class ConstraintError(ValueError):
    pass


def load_planning_constraints(
    maps_dir: str | Path,
    map_name: str,
    fallback_clearance_m: float,
) -> PlanningConstraints:
    """Load and normalize one map's editor document without modifying it."""
    if not str(maps_dir).strip() or not str(map_name).strip():
        return PlanningConstraints((), float(fallback_clearance_m), "config", 0, None)
    try:
        name = validate_map_name(str(map_name))
        root = Path(maps_dir).resolve()
        map_dir = (root / name).resolve()
        map_dir.relative_to(root)
        path = map_dir / f"{name}.editor.yaml"
        if not path.is_file():
            return PlanningConstraints(
                (), float(fallback_clearance_m), "config", 0, path
            )
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        document = normalize_document(raw, name)
    except (OSError, yaml.YAMLError, DocumentError, TypeError, ValueError) as exc:
        raise ConstraintError(f"无法加载规划约束: {exc}") from exc
    settings = document["settings"]
    return PlanningConstraints(
        tuple(document["keepouts"]),
        float(settings["safety_clearance_m"]),
        str(settings["safety_clearance_source"]),
        int(document["revision"]),
        path,
    )


def overlay_keepouts(
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    keepouts: Sequence[Mapping[str, object]],
    occupied_value: int = 100,
) -> tuple[list[int], int]:
    """Return a copied occupancy grid with rotated keepouts marked occupied."""
    width = int(width)
    height = int(height)
    resolution = float(resolution)
    if width <= 0 or height <= 0 or resolution <= 0:
        raise ConstraintError("OccupancyGrid 尺寸或分辨率无效")
    if len(data) != width * height:
        raise ConstraintError("OccupancyGrid data 长度与尺寸不一致")
    merged = [int(value) for value in data]
    if not keepouts:
        return merged, 0

    origin_cos = math.cos(float(origin_yaw))
    origin_sin = math.sin(float(origin_yaw))
    cell_margin = resolution * math.sqrt(2.0) * 0.5
    marked: set[int] = set()

    def world_to_grid(x: float, y: float) -> tuple[float, float]:
        dx = x - float(origin_x)
        dy = y - float(origin_y)
        local_x = origin_cos * dx + origin_sin * dy
        local_y = -origin_sin * dx + origin_cos * dy
        return local_x / resolution, local_y / resolution

    def grid_center_to_world(column: int, row: int) -> tuple[float, float]:
        local_x = (column + 0.5) * resolution
        local_y = (row + 0.5) * resolution
        return (
            float(origin_x) + origin_cos * local_x - origin_sin * local_y,
            float(origin_y) + origin_sin * local_x + origin_cos * local_y,
        )

    for zone in keepouts:
        vertices = [world_to_grid(x, y) for x, y in rectangle_vertices(zone)]
        min_column = max(0, int(math.floor(min(point[0] for point in vertices))) - 1)
        max_column = min(
            width - 1, int(math.ceil(max(point[0] for point in vertices))) + 1
        )
        min_row = max(0, int(math.floor(min(point[1] for point in vertices))) - 1)
        max_row = min(
            height - 1, int(math.ceil(max(point[1] for point in vertices))) + 1
        )
        center = zone["center"]
        zone_yaw = -math.radians(float(zone.get("yaw_deg", 0.0)))
        zone_cos = math.cos(zone_yaw)
        zone_sin = math.sin(zone_yaw)
        half_width = float(zone["width_m"]) * 0.5 + cell_margin
        half_height = float(zone["height_m"]) * 0.5 + cell_margin
        for row in range(min_row, max_row + 1):
            for column in range(min_column, max_column + 1):
                world_x, world_y = grid_center_to_world(column, row)
                dx = world_x - float(center["x"])
                dy = world_y - float(center["y"])
                local_x = zone_cos * dx - zone_sin * dy
                local_y = zone_sin * dx + zone_cos * dy
                if abs(local_x) <= half_width and abs(local_y) <= half_height:
                    index = row * width + column
                    merged[index] = int(occupied_value)
                    marked.add(index)
    return merged, len(marked)
