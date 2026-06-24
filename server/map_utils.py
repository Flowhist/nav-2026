#!/usr/bin/env python3
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple


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
            parts = [x.strip() for x in value[1:-1].split(",") if x.strip()]
            items: List[object] = []
            for item in parts:
                try:
                    items.append(float(item))
                except ValueError:
                    items.append(item.strip("'\""))
            data[key] = items
            continue
        if value.lower() in {"true", "false"}:
            data[key] = value.lower() == "true"
            continue
        try:
            if "." in value or "e" in value.lower():
                data[key] = float(value)
            else:
                data[key] = int(value)
            continue
        except ValueError:
            data[key] = value.strip("'\"")
    return data


def _read_pgm(path: Path) -> Tuple[int, int, int, bytes]:
    data = path.read_bytes()
    idx = 0
    tokens: List[bytes] = []

    while len(tokens) < 4:
        while idx < len(data) and chr(data[idx]).isspace():
            idx += 1
        if idx >= len(data):
            raise ValueError(f"invalid pgm header: {path}")
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

    while idx < len(data) and chr(data[idx]).isspace():
        idx += 1

    payload = data[idx:]
    if magic == "P5":
        return width, height, max_val, payload

    if magic != "P2":
        raise ValueError(f"unsupported pgm format: {magic}")

    ascii_values = re.findall(rb"\d+", payload)
    return width, height, max_val, bytes(int(v) for v in ascii_values)


def _pixel_to_occupancy(pixel: int) -> int:
    if pixel <= 60:
        return 100
    if pixel >= 245:
        return 0
    return -1


def load_map_preview(map_dir: Path, name: str) -> Optional[Dict[str, object]]:
    yaml_path = map_dir / name / f"{name}.yaml"
    if not yaml_path.exists():
        return None

    meta = _parse_simple_yaml(yaml_path)
    image_name = str(meta.get("image", f"{name}.pgm"))
    pgm_path = yaml_path.parent / image_name
    if not pgm_path.exists():
        return None

    width, height, _, pixels = _read_pgm(pgm_path)
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    if not isinstance(origin, list) or len(origin) < 3:
        origin = [0.0, 0.0, 0.0]

    occupancy = [_pixel_to_occupancy(px) for px in pixels[: width * height]]
    return {
        "name": name,
        "source": "saved_map",
        "frame_id": "map",
        "width": width,
        "height": height,
        "resolution": float(meta.get("resolution", 0.05)),
        "origin": {
            "x": float(origin[0]),
            "y": float(origin[1]),
            "yaw_deg": float(origin[2]) * 57.295779513,
        },
        "occupied_thresh": float(meta.get("occupied_thresh", 0.65)),
        "free_thresh": float(meta.get("free_thresh", 0.25)),
        "data": occupancy,
    }


def list_saved_maps(map_dir: Path) -> List[Dict[str, object]]:
    if not map_dir.exists():
        return []

    items: List[Dict[str, object]] = []
    for child in sorted(map_dir.iterdir()):
        if not child.is_dir():
            continue
        name = child.name
        yaml_path = child / f"{name}.yaml"
        pgm_path = child / f"{name}.pgm"
        if not yaml_path.exists() or not pgm_path.exists():
            continue

        meta = _parse_simple_yaml(yaml_path)
        width, height, _, _ = _read_pgm(pgm_path)
        items.append(
            {
                "name": name,
                "width": width,
                "height": height,
                "resolution": float(meta.get("resolution", 0.05)),
                "continuable": (
                    (child / f"{name}.posegraph").exists()
                    and (child / f"{name}.data").exists()
                ),
            }
        )

    return items


def load_map_locations(map_dir: Path, name: str) -> List[Dict[str, object]]:
    """Read location names + coords from <map>/<map>.locations.yaml.

    Returns a list of {name, x, y, yaw_deg}. Parsing is intentionally simple
    (flat-key YAML), matching the rest of this module which avoids a yaml dep.
    """
    loc_path = map_dir / name / f"{name}.locations.yaml"
    if not loc_path.exists():
        return []

    results: List[Dict[str, object]] = []
    current: Optional[str] = None
    fields: Dict[str, float] = {}

    def _flush() -> None:
        if current is not None and "x" in fields and "y" in fields:
            results.append(
                {
                    "name": current,
                    "x": float(fields["x"]),
                    "y": float(fields["y"]),
                    "yaw_deg": float(fields.get("yaw_deg", 0.0)),
                }
            )

    in_locations = False
    for raw_line in loc_path.read_text(encoding="utf-8").splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        line = raw_line.strip()

        if indent == 0:
            in_locations = line.rstrip(":") == "locations" or line.startswith("locations:")
            continue
        if not in_locations:
            continue

        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()

        # Location header lines (e.g. "spot1:") sit one level under "locations:".
        if not value:
            _flush()
            current = key
            fields = {}
            continue

        if current is not None:
            try:
                fields[key] = float(value.strip("'\""))
            except ValueError:
                pass

    _flush()
    return results


def _normalize_location_entries(locations: object) -> List[Dict[str, object]]:
    if not isinstance(locations, list):
        raise ValueError("locations must be a list")

    normalized: List[Dict[str, object]] = []
    seen = set()
    for item in locations:
        if not isinstance(item, dict):
            raise ValueError("location entry must be an object")

        loc_name = str(item.get("name", "")).strip()
        if not loc_name:
            raise ValueError("location name is required")
        if "\n" in loc_name or ":" in loc_name:
            raise ValueError(f"invalid location name: {loc_name}")
        if loc_name in seen:
            raise ValueError(f"duplicate location name: {loc_name}")

        try:
            x = float(item["x"])
            y = float(item["y"])
            yaw_deg = float(item.get("yaw_deg", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid coordinates for location: {loc_name}") from exc

        if not all(math.isfinite(v) for v in (x, y, yaw_deg)):
            raise ValueError(f"invalid coordinates for location: {loc_name}")

        seen.add(loc_name)
        normalized.append(
            {
                "name": loc_name,
                "x": round(x, 3),
                "y": round(y, 3),
                "yaw_deg": round(yaw_deg, 1),
            }
        )
    return normalized


def save_map_locations(map_dir: Path, name: str, locations: object) -> List[Dict[str, object]]:
    normalized = _normalize_location_entries(locations)
    loc_path = map_dir / name / f"{name}.locations.yaml"
    loc_path.parent.mkdir(parents=True, exist_ok=True)

    lines = ["locations:"]
    for loc in normalized:
        lines.extend(
            [
                f"  {loc['name']}:",
                f"    x: {loc['x']:.3f}",
                f"    y: {loc['y']:.3f}",
                f"    yaw_deg: {loc['yaw_deg']:.1f}",
            ]
        )
    loc_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return normalized
