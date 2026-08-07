#!/usr/bin/env python3
"""Non-destructive derived-map export with keepouts baked into occupancy pixels."""

from __future__ import annotations

import copy
import os
import shutil
import tempfile
from pathlib import Path
from typing import Mapping

import yaml

from editor_geometry import rasterize_keepouts
from editor_map_io import load_map, validate_map_name
from editor_store import normalize_document


class ExportError(RuntimeError):
    pass


def _write_pgm(path: Path, width: int, height: int, pixels) -> None:
    with path.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(pixels.astype("uint8").tobytes())
        stream.flush()
        os.fsync(stream.fileno())


def export_map(
    maps_dir: Path,
    source_name: str,
    target_name: str,
    document: Mapping[str, object],
) -> dict[str, object]:
    root = Path(maps_dir).resolve()
    source_name = validate_map_name(source_name)
    target_name = validate_map_name(target_name)
    target = root / target_name
    if target.exists():
        raise ExportError(f"Target map already exists: {target_name}")

    source_map = load_map(root, source_name)
    keepouts = list(document.get("keepouts", []))
    mask = rasterize_keepouts(source_map, keepouts)
    pixels = source_map.pixels.copy()
    pixels[mask.astype(bool)] = 255 if source_map.negate else 0

    temp_parent = Path(tempfile.mkdtemp(prefix=".map-editor-export-", dir=root))
    staging = temp_parent / target_name
    staging.mkdir()
    try:
        pgm_path = staging / f"{target_name}.pgm"
        _write_pgm(pgm_path, source_map.width, source_map.height, pixels)

        metadata = yaml.safe_load(source_map.yaml_path.read_text(encoding="utf-8")) or {}
        metadata["image"] = f"{target_name}.pgm"
        yaml_path = staging / f"{target_name}.yaml"
        with yaml_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(metadata, stream, allow_unicode=True, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())

        derived_map = load_map(temp_parent, target_name)
        derived = copy.deepcopy(dict(document))
        derived["revision"] = 0
        derived["map"] = {
            "name": target_name,
            "yaml_file": f"{target_name}.yaml",
            "source_sha256": derived_map.source_sha256,
        }
        derived["keepouts"] = []
        derived = normalize_document(derived, target_name)
        editor_path = staging / f"{target_name}.editor.yaml"
        with editor_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(
                derived, stream, allow_unicode=True, sort_keys=False, default_flow_style=False
            )
            stream.flush()
            os.fsync(stream.fileno())

        os.replace(staging, target)
        directory_fd = os.open(root, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception as exc:
        shutil.rmtree(temp_parent, ignore_errors=True)
        if isinstance(exc, ExportError):
            raise
        raise ExportError(f"Unable to export map: {exc}") from exc
    shutil.rmtree(temp_parent, ignore_errors=True)
    return {"name": target_name, "path": str(target), "document": derived}
