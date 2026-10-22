#!/usr/bin/env python3
from dataclasses import replace
from pathlib import Path
from typing import Dict, List
import os
import tempfile
import shutil

import numpy as np

from editor_map_io import discover_maps, load_map
from location_utils import load_locations
from web_support import RequestError


def save_map_raster(map_dir: Path, name: str, body: dict) -> dict:
    data = load_map(map_dir, name)
    if body.get("source_sha256") != data.source_sha256:
        raise RequestError(409, "地图已变化，请重新加载后编辑")
    cells = body.get("cells")
    if not isinstance(cells, list) or not 1 <= len(cells) <= 100000:
        raise ValueError("每次保存需包含 1 至 100000 个栅格")
    # Choose an unknown value inside this map's actual trinary thresholds.
    unknown = round(255 * (1 - (data.free_thresh + data.occupied_thresh) / 2))
    colors = {100: 0, 0: 255, -1: unknown}
    pixels = data.pixels.copy().reshape(-1)
    for cell in cells:
        if (not isinstance(cell, list) or len(cell) != 2
                or type(cell[0]) is not int or type(cell[1]) is not int
                or not 0 <= cell[0] < pixels.size or cell[1] not in colors):
            raise ValueError("无效的栅格修改")
        color = colors[cell[1]]
        pixels[cell[0]] = 255 - color if data.negate else color
    fd, temporary = tempfile.mkstemp(prefix=".raster-", dir=data.image_path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(f"P5\n{data.width} {data.height}\n255\n".encode("ascii"))
            stream.write(pixels.tobytes())
            stream.flush()
            os.fsync(stream.fileno())
        shutil.copymode(data.image_path, temporary)
        backup = data.image_path.with_name(data.image_path.name + ".brush-backup")
        # Refuse a pre-existing link rather than following it outside the map.
        if backup.is_symlink():
            raise ValueError("地图备份路径不能为符号链接")
        shutil.copyfile(data.image_path, backup)
        os.replace(temporary, data.image_path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return {"ok": True, "name": name}


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
        paths = [map_dir / name / f"{name}.{suffix}" for suffix in ("yaml", "pgm", "posegraph", "data")]
        item["continuable"] = all(not path.is_symlink() and path.is_file() and path.stat().st_size > 0
                                  for path in paths)
    return items


def load_map_locations(map_dir: Path, name: str) -> List[Dict[str, object]]:
    entries = load_locations(map_dir / name / f"{name}.locations.yaml")
    return [dict(name=key, **value) for key, value in entries.items()]
