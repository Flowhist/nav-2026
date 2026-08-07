#!/usr/bin/env python3
"""Unified editor document validation and atomic revisioned persistence."""

from __future__ import annotations

import copy
import fcntl
import hashlib
import json
import math
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator, Mapping

import yaml

from editor_map_io import validate_map_name

SCHEMA_VERSION = 1


class DocumentError(ValueError):
    def __init__(self, code: str, message: str, field: str = ""):
        super().__init__(message)
        self.code = code
        self.field = field


class RevisionConflict(DocumentError):
    def __init__(self, expected: int, actual: int):
        super().__init__(
            "revision_conflict",
            f"Document revision conflict: expected {expected}, current {actual}",
            "revision",
        )
        self.expected = expected
        self.actual = actual


def _finite(value: object, field: str, digits: int = 3) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise DocumentError("invalid_number", f"Invalid number at {field}", field) from exc
    if not math.isfinite(number):
        raise DocumentError("invalid_number", f"Invalid number at {field}", field)
    return round(number, digits)


def _identity(value: object, field: str) -> str:
    text = str(value or "").strip()
    if not text or len(text) > 128 or any(ch in text for ch in "\r\n"):
        raise DocumentError("invalid_id", f"Invalid identifier at {field}", field)
    return text


def _name(value: object, field: str) -> str:
    text = str(value or "").strip()
    if not text or len(text) > 128 or any(ch in text for ch in "\r\n"):
        raise DocumentError("invalid_name", f"Invalid name at {field}", field)
    return text


def empty_document(
    map_name: str,
    map_source_sha256: str,
    clearance: float,
    source: str = "config",
) -> dict[str, object]:
    return {
        "schema_version": SCHEMA_VERSION,
        "revision": 0,
        "map": {
            "name": validate_map_name(map_name),
            "yaml_file": f"{map_name}.yaml",
            "source_sha256": str(map_source_sha256),
        },
        "settings": {
            "safety_clearance_m": round(float(clearance), 3),
            "safety_clearance_source": str(source),
        },
        "locations": [],
        "keepouts": [],
        "routes": [],
    }


def _normalize_locations(items: object) -> list[dict[str, object]]:
    if not isinstance(items, list):
        raise DocumentError("invalid_locations", "locations must be a list", "locations")
    result = []
    ids, names = set(), set()
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise DocumentError("invalid_location", "location must be an object")
        prefix = f"locations[{index}]"
        ident = _identity(item.get("id"), f"{prefix}.id")
        name = _name(item.get("name"), f"{prefix}.name")
        if ident in ids or name in names:
            raise DocumentError("duplicate_location", f"Duplicate location: {name}", prefix)
        ids.add(ident)
        names.add(name)
        result.append(
            {
                "id": ident,
                "name": name,
                "x": _finite(item.get("x"), f"{prefix}.x"),
                "y": _finite(item.get("y"), f"{prefix}.y"),
                "yaw_deg": _finite(item.get("yaw_deg", 0.0), f"{prefix}.yaw_deg", 1),
            }
        )
    return result


def _normalize_keepouts(items: object) -> list[dict[str, object]]:
    if not isinstance(items, list):
        raise DocumentError("invalid_keepouts", "keepouts must be a list", "keepouts")
    result = []
    ids, names = set(), set()
    for index, item in enumerate(items):
        if not isinstance(item, dict) or not isinstance(item.get("center"), dict):
            raise DocumentError("invalid_keepout", "keepout must contain center")
        prefix = f"keepouts[{index}]"
        ident = _identity(item.get("id"), f"{prefix}.id")
        name = _name(item.get("name"), f"{prefix}.name")
        if ident in ids or name in names:
            raise DocumentError("duplicate_keepout", f"Duplicate keepout: {name}", prefix)
        width = _finite(item.get("width_m"), f"{prefix}.width_m")
        height = _finite(item.get("height_m"), f"{prefix}.height_m")
        if width <= 0 or height <= 0:
            raise DocumentError(
                "invalid_keepout", "Keepout width and height must be positive"
            )
        ids.add(ident)
        names.add(name)
        center = item["center"]
        result.append(
            {
                "id": ident,
                "name": name,
                "center": {
                    "x": _finite(center.get("x"), f"{prefix}.center.x"),
                    "y": _finite(center.get("y"), f"{prefix}.center.y"),
                },
                "width_m": width,
                "height_m": height,
                "yaw_deg": _finite(item.get("yaw_deg", 0), f"{prefix}.yaw_deg", 1),
            }
        )
    return result


def _normalize_routes(items: object) -> list[dict[str, object]]:
    if not isinstance(items, list):
        raise DocumentError("invalid_routes", "routes must be a list", "routes")
    result = []
    ids, names = set(), set()
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise DocumentError("invalid_route", "route must be an object")
        prefix = f"routes[{index}]"
        ident = _identity(item.get("id"), f"{prefix}.id")
        name = _name(item.get("name"), f"{prefix}.name")
        if ident in ids or name in names:
            raise DocumentError("duplicate_route", f"Duplicate route: {name}", prefix)
        ids.add(ident)
        names.add(name)
        closed = bool(item.get("closed", False))
        raw_waypoints = item.get("waypoints", [])
        minimum = 3 if closed else 2
        if not isinstance(raw_waypoints, list) or len(raw_waypoints) < minimum:
            raise DocumentError(
                "invalid_route",
                f"{'Closed' if closed else 'Open'} route requires {minimum} waypoints",
                f"{prefix}.waypoints",
            )
        waypoint_ids = set()
        waypoints = []
        for wp_index, waypoint in enumerate(raw_waypoints):
            if not isinstance(waypoint, dict):
                raise DocumentError("invalid_waypoint", "waypoint must be an object")
            wp_prefix = f"{prefix}.waypoints[{wp_index}]"
            wp_id = _identity(waypoint.get("id"), f"{wp_prefix}.id")
            if wp_id in waypoint_ids:
                raise DocumentError("duplicate_waypoint", f"Duplicate waypoint: {wp_id}")
            waypoint_ids.add(wp_id)
            waypoints.append(
                {
                    "id": wp_id,
                    "x": _finite(waypoint.get("x"), f"{wp_prefix}.x"),
                    "y": _finite(waypoint.get("y"), f"{wp_prefix}.y"),
                }
            )
        route = {
            "id": ident,
            "name": name,
            "closed": closed,
            "waypoints": waypoints,
        }
        result.append(route)
    return result


def normalize_document(data: object, map_name: str) -> dict[str, object]:
    if not isinstance(data, dict):
        raise DocumentError("invalid_document", "Editor document must be an object")
    version = int(data.get("schema_version", SCHEMA_VERSION))
    if version > SCHEMA_VERSION:
        raise DocumentError(
            "unsupported_schema",
            f"Editor schema {version} is newer than supported {SCHEMA_VERSION}",
        )
    if version != SCHEMA_VERSION:
        raise DocumentError("unsupported_schema", f"Unsupported editor schema: {version}")
    map_info = data.get("map")
    settings = data.get("settings")
    if not isinstance(map_info, dict) or not isinstance(settings, dict):
        raise DocumentError("invalid_document", "map and settings are required")
    if validate_map_name(str(map_info.get("name", ""))) != validate_map_name(map_name):
        raise DocumentError("map_mismatch", "Editor document belongs to another map")
    try:
        revision = int(data.get("revision", 0))
    except (TypeError, ValueError) as exc:
        raise DocumentError("invalid_revision", "revision must be an integer") from exc
    if revision < 0:
        raise DocumentError("invalid_revision", "revision cannot be negative")
    clearance = _finite(
        settings.get("safety_clearance_m"), "settings.safety_clearance_m"
    )
    if clearance < 0:
        raise DocumentError("invalid_clearance", "Safety clearance cannot be negative")
    return {
        "schema_version": SCHEMA_VERSION,
        "revision": revision,
        "map": {
            "name": map_name,
            "yaml_file": f"{map_name}.yaml",
            "source_sha256": str(map_info.get("source_sha256", "")),
        },
        "settings": {
            "safety_clearance_m": clearance,
            "safety_clearance_source": str(
                settings.get("safety_clearance_source", "fallback")
            ),
        },
        "locations": _normalize_locations(data.get("locations", [])),
        "keepouts": _normalize_keepouts(data.get("keepouts", [])),
        "routes": _normalize_routes(data.get("routes", [])),
    }


def route_input_sha256(
    map_source_sha256: str,
    keepouts: list[dict[str, object]],
    route: Mapping[str, object],
    clearance_m: float,
) -> str:
    zones = []
    for zone in sorted(keepouts, key=lambda value: str(value.get("id", ""))):
        zones.append(
            {
                "id": zone.get("id"),
                "center": zone.get("center"),
                "width_m": zone.get("width_m"),
                "height_m": zone.get("height_m"),
                "yaw_deg": zone.get("yaw_deg"),
            }
        )
    payload = {
        "map_source_sha256": str(map_source_sha256),
        "keepouts": zones,
        "route": {
            "id": route.get("id"),
            "closed": bool(route.get("closed", False)),
            "waypoints": route.get("waypoints", []),
        },
        "clearance_m": round(float(clearance_m), 6),
    }
    encoded = json.dumps(
        payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


class EditorStore:
    def __init__(self, maps_dir: Path):
        self.maps_dir = Path(maps_dir).resolve()

    def _map_dir(self, map_name: str) -> Path:
        name = validate_map_name(map_name)
        target = (self.maps_dir / name).resolve()
        try:
            target.relative_to(self.maps_dir)
        except ValueError as exc:
            raise DocumentError("invalid_map_name", "Invalid map path") from exc
        if not target.is_dir():
            raise DocumentError("map_not_found", f"Map not found: {name}")
        return target

    @contextmanager
    def _locked(self, map_dir: Path, map_name: str) -> Iterator[None]:
        lock_path = map_dir / f".{map_name}.editor.lock"
        with lock_path.open("a+", encoding="utf-8") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)

    @staticmethod
    def _read(path: Path, map_name: str) -> dict[str, object]:
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise DocumentError(
                "invalid_editor_yaml", f"Unable to read editor document: {path.name}"
            ) from exc
        return normalize_document(data, map_name)

    def load(
        self,
        map_name: str,
        map_source_sha256: str,
        clearance: float,
        clearance_source: str = "config",
    ) -> dict[str, object]:
        map_dir = self._map_dir(map_name)
        path = map_dir / f"{map_name}.editor.yaml"
        with self._locked(map_dir, map_name):
            if not path.exists():
                return empty_document(
                    map_name, map_source_sha256, clearance, clearance_source
                )
            return self._read(path, map_name)

    def save(
        self, map_name: str, document: object, expected_revision: int
    ) -> dict[str, object]:
        map_dir = self._map_dir(map_name)
        path = map_dir / f"{map_name}.editor.yaml"
        with self._locked(map_dir, map_name):
            current_revision = 0
            if path.exists():
                current_revision = int(self._read(path, map_name)["revision"])
            if int(expected_revision) != current_revision:
                raise RevisionConflict(int(expected_revision), current_revision)
            normalized = normalize_document(copy.deepcopy(document), map_name)
            normalized["revision"] = current_revision + 1
            fd, temp_name = tempfile.mkstemp(
                prefix=f".{map_name}.editor.", suffix=".tmp", dir=map_dir
            )
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as stream:
                    yaml.safe_dump(
                        normalized,
                        stream,
                        allow_unicode=True,
                        sort_keys=False,
                        default_flow_style=False,
                    )
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temp_name, path)
                directory_fd = os.open(map_dir, os.O_RDONLY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            except Exception:
                try:
                    os.unlink(temp_name)
                except FileNotFoundError:
                    pass
                raise
            return normalized
