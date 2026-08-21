#!/usr/bin/env python3
"""Map editor application service shared by the Finav web backend."""

from __future__ import annotations

import copy
from pathlib import Path

import yaml

from editor_export import export_map
from editor_map_io import load_map
from editor_routes import (
    build_clearance_mask,
    is_point_safe,
    resolve_clearance,
    smooth_route,
)
from editor_store import (
    DocumentError,
    EditorStore,
    RevisionConflict,
    empty_document,
    normalize_document,
)

class ApiError(RuntimeError):
    def __init__(self, status: int, code: str, message: str, field: str | None = None):
        super().__init__(message)
        self.status = status
        self.code = code
        self.field = field


class EditorApplication:
    def __init__(self, maps_dir: Path, planner_config: Path):
        self.maps_dir = Path(maps_dir).resolve()
        self.planner_config = Path(planner_config)
        self.store = EditorStore(self.maps_dir)

    @staticmethod
    def _refresh_map_state(document: object, map_data) -> dict[str, object]:
        refreshed = copy.deepcopy(document)
        if not isinstance(refreshed, dict):
            return refreshed
        map_info = refreshed.setdefault("map", {})
        if isinstance(map_info, dict):
            map_info["source_sha256"] = map_data.source_sha256
        return refreshed

    @staticmethod
    def _validate_locations(
        map_data,
        document: dict[str, object],
        baseline_document: dict[str, object] | None = None,
    ) -> None:
        clearance = float(document["settings"]["safety_clearance_m"])
        traversable = build_clearance_mask(
            map_data, document["keepouts"], clearance
        )
        baseline_by_id: dict[str, dict[str, object]] = {}
        baseline_invalid_ids: set[str] = set()
        if baseline_document:
            baseline_locations = baseline_document.get("locations", [])
            baseline_clearance = float(
                baseline_document["settings"]["safety_clearance_m"]
            )
            baseline_traversable = build_clearance_mask(
                map_data,
                baseline_document.get("keepouts", []),
                baseline_clearance,
            )
            for location in baseline_locations:
                ident = str(location.get("id", ""))
                baseline_by_id[ident] = location
                if not is_point_safe(
                    map_data,
                    baseline_traversable,
                    (float(location["x"]), float(location["y"])),
                ):
                    baseline_invalid_ids.add(ident)
        for index, location in enumerate(document["locations"]):
            if not is_point_safe(
                map_data,
                traversable,
                (float(location["x"]), float(location["y"])),
            ):
                ident = str(location.get("id", ""))
                previous = baseline_by_id.get(ident)
                unchanged_legacy_location = (
                    ident in baseline_invalid_ids
                    and previous is not None
                    and float(previous["x"]) == float(location["x"])
                    and float(previous["y"]) == float(location["y"])
                )
                if unchanged_legacy_location:
                    continue
                raise DocumentError(
                    "invalid_location_position",
                    f"Target location is not safely traversable: {location['name']}",
                    f"locations[{index}]",
                )

    def load_document(self, map_name: str) -> dict[str, object]:
        map_data = load_map(self.maps_dir, map_name)
        clearance, source = resolve_clearance(self.planner_config)
        try:
            document = self.store.load(
                map_name, map_data.source_sha256, clearance, source
            )
        except DocumentError as error:
            fallback = empty_document(
                map_name, map_data.source_sha256, clearance, source
            )
            editor_path = (
                self.maps_dir / map_name / f"{map_name}.editor.yaml"
            )
            try:
                raw = yaml.safe_load(editor_path.read_text(encoding="utf-8"))
                if isinstance(raw, dict):
                    compatible = copy.deepcopy(raw)
                    compatible["schema_version"] = 1
                    fallback = normalize_document(compatible, map_name)
                    fallback = self._refresh_map_state(fallback, map_data)
            except (OSError, yaml.YAMLError, DocumentError, TypeError, ValueError):
                pass
            return {
                "document": fallback,
                "map": map_data.preview_payload(),
                "read_only": True,
                "read_only_reason": str(error),
                "error": {"code": error.code, "message": str(error)},
            }
        stored_source = str(document.get("map", {}).get("source_sha256", ""))
        source_changed = bool(
            stored_source and stored_source != map_data.source_sha256
        )
        document = self._refresh_map_state(document, map_data)
        return {
            "document": document,
            "map": map_data.preview_payload(),
            "source_changed": source_changed,
        }

    def save_document(
        self,
        map_name: str,
        payload: object,
        baseline_document: dict[str, object] | None = None,
    ) -> dict[str, object]:
        if not isinstance(payload, dict):
            raise ApiError(400, "invalid_json", "Request body must be an object")
        if "expected_revision" not in payload or "document" not in payload:
            raise ApiError(
                400, "missing_field", "expected_revision and document are required"
            )
        map_data = load_map(self.maps_dir, map_name)
        document = copy.deepcopy(payload["document"])
        if not isinstance(document, dict):
            raise ApiError(400, "invalid_document", "document must be an object")
        document = self._refresh_map_state(document, map_data)
        normalized = normalize_document(document, map_name)
        self._validate_locations(map_data, normalized, baseline_document)
        saved = self.store.save(
            map_name, normalized, int(payload["expected_revision"])
        )
        return {"document": saved}

    def smooth(self, map_name: str, route_id: str, payload: object) -> dict[str, object]:
        if not isinstance(payload, dict) or "expected_revision" not in payload:
            raise ApiError(400, "missing_field", "expected_revision is required")
        map_data = load_map(self.maps_dir, map_name)
        clearance, source = resolve_clearance(self.planner_config)
        document = self.store.load(
            map_name, map_data.source_sha256, clearance, source
        )
        document = self._refresh_map_state(document, map_data)
        document = normalize_document(document, map_name)
        self._validate_locations(map_data, document)
        if int(payload["expected_revision"]) != int(document["revision"]):
            raise RevisionConflict(
                int(payload["expected_revision"]), int(document["revision"])
            )
        route = next(
            (item for item in document["routes"] if item["id"] == route_id), None
        )
        if route is None:
            raise ApiError(404, "route_not_found", f"Route not found: {route_id}")
        clearance = float(document["settings"]["safety_clearance_m"])
        preview = smooth_route(
            map_data, route, document["keepouts"], clearance
        )
        return {
            "route_id": route_id,
            "revision": document["revision"],
            "preview": preview,
        }

    def export(self, map_name: str, payload: object) -> dict[str, object]:
        if not isinstance(payload, dict) or not payload.get("target_name"):
            raise ApiError(400, "missing_field", "target_name is required", "target_name")
        map_data = load_map(self.maps_dir, map_name)
        clearance, source = resolve_clearance(self.planner_config)
        document = self.store.load(
            map_name, map_data.source_sha256, clearance, source
        )
        document = self._refresh_map_state(document, map_data)
        normalized = normalize_document(document, map_name)
        self._validate_locations(map_data, normalized)
        return {"export": export_map(
            self.maps_dir, map_name, str(payload["target_name"]), normalized
        )}
