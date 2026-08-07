#!/usr/bin/env python3
"""ROS occupancy-map loading and coordinate transforms for the local editor."""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
import yaml

MAP_NAME_RE = re.compile(r"^[A-Za-z0-9_\-\u4e00-\u9fff]+$")


class MapFormatError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class MapData:
    name: str
    yaml_path: Path
    image_path: Path
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    negate: int
    occupied_thresh: float
    free_thresh: float
    pixels: np.ndarray
    occupancy: np.ndarray
    source_sha256: str

    def pixel_to_map(self, px: float, py: float) -> Tuple[float, float]:
        local_x = px * self.resolution
        local_y = (self.height - 1 - py) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (
            self.origin_x + c * local_x - s * local_y,
            self.origin_y + s * local_x + c * local_y,
        )

    def map_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        return (
            local_x / self.resolution,
            self.height - 1 - local_y / self.resolution,
        )

    def preview_payload(self) -> dict:
        return {
            "name": self.name,
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": {
                "x": self.origin_x,
                "y": self.origin_y,
                "yaw_deg": math.degrees(self.origin_yaw),
            },
            "source_sha256": self.source_sha256,
            "pixels": self.pixels.reshape(-1).tolist(),
            "occupancy": self.occupancy.reshape(-1).tolist(),
        }


def validate_map_name(name: str) -> str:
    value = str(name).strip()
    if not MAP_NAME_RE.fullmatch(value):
        raise MapFormatError("invalid_map_name", f"Invalid map name: {name}")
    return value


def _next_pgm_token(data: bytes, index: int) -> tuple[bytes, int]:
    while True:
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index >= len(data):
            raise MapFormatError("invalid_pgm", "Invalid PGM header")
        if data[index] != ord("#"):
            break
        while index < len(data) and data[index] not in b"\r\n":
            index += 1
    start = index
    while index < len(data) and data[index] not in b" \t\r\n":
        index += 1
    return data[start:index], index


def read_pgm(path: Path) -> tuple[int, int, int, np.ndarray]:
    data = path.read_bytes()
    index = 0
    tokens = []
    for _ in range(4):
        token, index = _next_pgm_token(data, index)
        tokens.append(token)
    try:
        magic = tokens[0].decode("ascii")
        width, height, max_value = map(int, tokens[1:])
    except (UnicodeDecodeError, ValueError) as exc:
        raise MapFormatError("invalid_pgm", f"Invalid PGM header: {path}") from exc
    if width <= 0 or height <= 0 or not 0 < max_value <= 255:
        raise MapFormatError("invalid_pgm", f"Unsupported PGM dimensions: {path}")

    if magic == "P5":
        if index >= len(data) or data[index] not in b" \t\r\n":
            raise MapFormatError("invalid_pgm", f"Missing PGM separator: {path}")
        if data[index : index + 2] == b"\r\n":
            index += 2
        else:
            index += 1
        payload = data[index : index + width * height]
        if len(payload) != width * height:
            raise MapFormatError("invalid_pgm", f"Truncated PGM payload: {path}")
        pixels = np.frombuffer(payload, dtype=np.uint8).copy()
    elif magic == "P2":
        body = data[index:].decode("ascii", errors="strict")
        body = re.sub(r"#.*?(?:\r?\n|$)", " ", body)
        try:
            values = [int(value) for value in body.split()]
        except ValueError as exc:
            raise MapFormatError("invalid_pgm", f"Invalid P2 payload: {path}") from exc
        if len(values) < width * height:
            raise MapFormatError("invalid_pgm", f"Truncated PGM payload: {path}")
        pixels = np.asarray(values[: width * height], dtype=np.uint8)
    else:
        raise MapFormatError("unsupported_pgm", f"Unsupported PGM format: {magic}")

    if max_value != 255:
        pixels = np.rint(pixels.astype(np.float64) * 255.0 / max_value).astype(
            np.uint8
        )
    return width, height, max_value, pixels.reshape((height, width))


def _safe_image_path(map_dir: Path, image_value: object) -> Path:
    candidate = (map_dir / str(image_value)).resolve()
    try:
        candidate.relative_to(map_dir.resolve())
    except ValueError as exc:
        raise MapFormatError("invalid_image_path", "Map image escapes map directory") from exc
    if not candidate.is_file():
        raise MapFormatError("missing_image", f"Map image not found: {candidate.name}")
    return candidate


def load_map(maps_dir: Path, name: str) -> MapData:
    name = validate_map_name(name)
    map_dir = (Path(maps_dir).resolve() / name).resolve()
    try:
        map_dir.relative_to(Path(maps_dir).resolve())
    except ValueError as exc:
        raise MapFormatError("invalid_map_name", f"Invalid map name: {name}") from exc
    yaml_path = map_dir / f"{name}.yaml"
    if not yaml_path.is_file():
        raise MapFormatError("missing_map", f"Map YAML not found: {name}")
    try:
        metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        raise MapFormatError("invalid_yaml", f"Invalid map YAML: {name}") from exc
    if not isinstance(metadata, dict):
        raise MapFormatError("invalid_yaml", f"Invalid map YAML: {name}")

    image_path = _safe_image_path(map_dir, metadata.get("image", f"{name}.pgm"))
    width, height, _, pixels = read_pgm(image_path)
    try:
        resolution = float(metadata["resolution"])
        origin = metadata.get("origin", [0.0, 0.0, 0.0])
        if not isinstance(origin, list) or len(origin) < 3:
            raise ValueError("origin")
        origin_x, origin_y, origin_yaw = map(float, origin[:3])
        negate = int(metadata.get("negate", 0))
        occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
        free_thresh = float(metadata.get("free_thresh", 0.25))
    except (KeyError, TypeError, ValueError) as exc:
        raise MapFormatError("invalid_yaml", f"Invalid map metadata: {name}") from exc
    if resolution <= 0 or not 0 <= free_thresh < occupied_thresh <= 1:
        raise MapFormatError("invalid_yaml", f"Invalid map thresholds: {name}")

    normalized = pixels.astype(np.float64) / 255.0
    probability = normalized if negate else 1.0 - normalized
    occupancy = np.full((height, width), -1, dtype=np.int16)
    occupancy[probability > occupied_thresh] = 100
    occupancy[probability < free_thresh] = 0
    fingerprint_meta = {
        "resolution": resolution,
        "origin": [origin_x, origin_y, origin_yaw],
        "negate": negate,
        "occupied_thresh": occupied_thresh,
        "free_thresh": free_thresh,
    }
    digest = hashlib.sha256()
    digest.update(image_path.read_bytes())
    digest.update(
        json.dumps(fingerprint_meta, sort_keys=True, separators=(",", ":")).encode()
    )
    return MapData(
        name=name,
        yaml_path=yaml_path,
        image_path=image_path,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        negate=negate,
        occupied_thresh=occupied_thresh,
        free_thresh=free_thresh,
        pixels=pixels,
        occupancy=occupancy,
        source_sha256=digest.hexdigest(),
    )


def discover_maps(maps_dir: Path) -> list[dict[str, object]]:
    root = Path(maps_dir)
    if not root.is_dir():
        return []
    results = []
    for child in sorted(root.iterdir(), key=lambda item: item.name):
        if not child.is_dir() or not MAP_NAME_RE.fullmatch(child.name):
            continue
        try:
            data = load_map(root, child.name)
        except (MapFormatError, OSError):
            continue
        results.append(
            {
                "name": data.name,
                "width": data.width,
                "height": data.height,
                "resolution": data.resolution,
                "has_editor": (child / f"{child.name}.editor.yaml").is_file(),
            }
        )
    return results
