#!/usr/bin/env python3
"""Loopback-only HTTP application for the local map editor."""

from __future__ import annotations

import copy
import json
import mimetypes
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

import yaml

from editor_export import ExportError, export_map
from editor_map_io import MapFormatError, discover_maps, load_map, validate_map_name
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

MAX_BODY_BYTES = 1024 * 1024


class ApiError(RuntimeError):
    def __init__(self, status: int, code: str, message: str, field: str | None = None):
        super().__init__(message)
        self.status = status
        self.code = code
        self.field = field


class EditorApplication:
    def __init__(self, maps_dir: Path, web_dir: Path, planner_config: Path):
        self.maps_dir = Path(maps_dir).resolve()
        self.web_dir = Path(web_dir).resolve()
        self.planner_config = Path(planner_config)
        self.store = EditorStore(self.maps_dir)

    def list_maps(self) -> dict[str, object]:
        return {"maps": discover_maps(self.maps_dir)}

    def map_payload(self, map_name: str) -> dict[str, object]:
        return {"map": load_map(self.maps_dir, map_name).preview_payload()}

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
    def _validate_locations(map_data, document: dict[str, object]) -> None:
        clearance = float(document["settings"]["safety_clearance_m"])
        traversable = build_clearance_mask(
            map_data, document["keepouts"], clearance
        )
        for index, location in enumerate(document["locations"]):
            if not is_point_safe(
                map_data,
                traversable,
                (float(location["x"]), float(location["y"])),
            ):
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

    def save_document(self, map_name: str, payload: object) -> dict[str, object]:
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
        self._validate_locations(map_data, normalized)
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

    def static_path(self, request_path: str) -> Path:
        relative = request_path.lstrip("/") or "index.html"
        candidate = (self.web_dir / relative).resolve()
        try:
            candidate.relative_to(self.web_dir)
        except ValueError as exc:
            raise ApiError(404, "not_found", "Static asset not found") from exc
        if not candidate.is_file():
            raise ApiError(404, "not_found", "Static asset not found")
        return candidate


def _segments(path: str) -> list[str]:
    raw_path = urlsplit(path).path
    parts = [unquote(part) for part in raw_path.split("/") if part]
    if any("\x00" in part for part in parts):
        raise ApiError(400, "invalid_path", "Invalid request path")
    if any(part in {".", ".."} for part in parts):
        raise ApiError(400, "invalid_path", "Invalid request path")
    return parts


def create_server(
    host: str, port: int, application: EditorApplication
) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        server_version = "FinavMapEditor/1.0"

        def log_message(self, format_string, *args):
            return

        def _json(self, status: int, payload: object) -> None:
            encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(encoded)

        def _error(self, error: Exception) -> None:
            if isinstance(error, ApiError):
                status, code, field = error.status, error.code, error.field
            elif isinstance(error, RevisionConflict):
                status, code, field = 409, "revision_conflict", None
            elif isinstance(error, DocumentError):
                status, code, field = 422, error.code, getattr(error, "field", None)
            elif isinstance(error, MapFormatError):
                status, code, field = 404 if error.code == "missing_map" else 422, error.code, None
            elif isinstance(error, ExportError):
                status, code, field = 409, "export_failed", "target_name"
            elif isinstance(error, (ValueError, TypeError, json.JSONDecodeError)):
                status, code, field = 400, "invalid_request", None
            else:
                status, code, field = 500, "internal_error", None
            detail = {"code": code, "message": str(error)}
            if field:
                detail["field"] = field
            self._json(status, {"ok": False, "error": detail})

        def _body(self) -> object:
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise ApiError(400, "invalid_content_length", "Invalid Content-Length") from exc
            if length > MAX_BODY_BYTES:
                raise ApiError(413, "request_too_large", "Request body exceeds 1 MiB")
            if length <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(length).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ApiError(400, "invalid_json", "Malformed JSON request body") from exc

        def _dispatch(self, method: str) -> None:
            parts = _segments(self.path)
            if parts == ["health"] and method == "GET":
                self._json(200, {"ok": True})
                return
            if parts == ["api", "maps"] and method == "GET":
                self._json(200, application.list_maps())
                return
            if len(parts) >= 3 and parts[:2] == ["api", "maps"]:
                map_name = validate_map_name(parts[2])
                if len(parts) == 3 and method == "GET":
                    self._json(200, application.map_payload(map_name))
                    return
                if len(parts) == 4 and parts[3] == "editor":
                    if method == "GET":
                        self._json(200, application.load_document(map_name))
                        return
                    if method == "PUT":
                        self._json(200, application.save_document(map_name, self._body()))
                        return
                if (
                    len(parts) == 6
                    and parts[3] == "routes"
                    and parts[5] == "smooth"
                    and method == "POST"
                ):
                    self._json(
                        200, application.smooth(map_name, parts[4], self._body())
                    )
                    return
                if (
                    len(parts) == 4
                    and parts[3] == "export"
                    and method == "POST"
                ):
                    self._json(200, application.export(map_name, self._body()))
                    return
                raise ApiError(405, "method_not_allowed", "Method not allowed")
            if method == "GET" and not parts[:1] == ["api"]:
                path = application.static_path(urlsplit(self.path).path)
                content = path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header(
                    "Content-Type",
                    mimetypes.guess_type(path.name)[0] or "application/octet-stream",
                )
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return
            raise ApiError(404, "not_found", "Endpoint not found")

        def do_GET(self):
            try:
                self._dispatch("GET")
            except Exception as error:
                self._error(error)

        def do_PUT(self):
            try:
                self._dispatch("PUT")
            except Exception as error:
                self._error(error)

        def do_POST(self):
            try:
                self._dispatch("POST")
            except Exception as error:
                self._error(error)

    return ThreadingHTTPServer((host, int(port)), Handler)
