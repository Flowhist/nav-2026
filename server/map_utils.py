#!/usr/bin/env python3
from dataclasses import replace
from pathlib import Path
from typing import Dict, List

import numpy as np

from editor_map_io import discover_maps, load_map
from location_utils import load_locations


def load_map_preview(map_dir: Path, name: str) -> Dict[str, object]:
    data = load_map(map_dir, name)
    # Preserve the saved raster's gray unknown cells in the preview. Map-server
    # thresholds can classify the conventional gray value (205) as free.
    pixels = 255 - data.pixels if data.negate else data.pixels
    preview = np.full(pixels.shape, -1, dtype=np.int8)
    preview[pixels <= 60] = 100
    preview[pixels >= 245] = 0
    payload = replace(data, occupancy=preview).preview_payload(include_pixels=False)
    payload["data"] = payload.pop("occupancy")
    return {**payload, "source": "saved_map", "frame_id": "map",
            "occupied_thresh": data.occupied_thresh, "free_thresh": data.free_thresh}


def list_saved_maps(map_dir: Path) -> List[Dict[str, object]]:
    items = discover_maps(map_dir)
    for item in items:
        name = item["name"]
        item["continuable"] = all((map_dir / name / f"{name}.{suffix}").exists()
                                  for suffix in ("posegraph", "data"))
    return items


def load_map_locations(map_dir: Path, name: str) -> List[Dict[str, object]]:
    entries = load_locations(map_dir / name / f"{name}.locations.yaml")
    return [dict(name=key, **value) for key, value in entries.items()]
