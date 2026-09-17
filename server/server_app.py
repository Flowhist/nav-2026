#!/usr/bin/env python3
import hashlib
import json
import math
import mimetypes
import socket
import subprocess
import shutil
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, unquote, urlparse

MAP_EDITOR_DIR = Path(__file__).resolve().parent.parent / "scripts" / "map_location"
if str(MAP_EDITOR_DIR) not in sys.path:
    sys.path.insert(0, str(MAP_EDITOR_DIR))

from editor_export import ExportError
from editor_map_io import MapFormatError, validate_map_name
from editor_server import ApiError, EditorApplication
from editor_store import DocumentError, RevisionConflict
from base_control.paths import config_dir as base_config_dir
from map_utils import list_saved_maps, load_map_preview, load_map_locations
from systemd_runtime import create_runtime
from ros_bridge import RosBridge
from state_store import StateStore
from web_support import (ConfigStore, LimitedHTTPServer, RequestError,
                         access_password, authorized, finite_fields, query_integer, request_errors)


RESTART_TARGETS = {
    "base_drive": "底盘控制",
}

CONFIG_IMPACTS: Dict[str, Dict[str, object]] = {
    "base_control.yaml": {
        "message": "保存后需要重启底盘驱动，运行中的节点不会自动读取新速度参数。",
        "restart_targets": ["base_drive"],
    },
    "handle.yaml": {
        "message": "保存后需要重启底盘控制。",
        "restart_targets": ["base_drive"],
    },
    "slam_toolbox_map.yaml": {
        "message": "保存后重启建图链路生效；Web 启动时会读取 src/finav/config，不需要 colcon build。",
        "restart_targets": [],
    },
    "slam_toolbox_nav.yaml": {
        "message": "保存后重启导航链路生效；Web 启动时会读取 src/finav/config，不需要 colcon build。",
        "restart_targets": [],
    },
    "nav.yaml": {
        "message": "保存后重启导航链路生效。",
        "restart_targets": [],
    },
    "path_plan.yaml": {
        "message": "保存后重启导航链路生效。",
        "restart_targets": [],
    },
    "lidar.yaml": {
        "message": "保存后重启建图或导航链路中的雷达节点生效。",
        "restart_targets": [],
    },
    "imu.yaml": {
        "message": "保存后重启建图或导航链路中的 IMU 节点生效。",
        "restart_targets": [],
    },
    "ekf.yaml": {
        "message": "保存后重启建图或导航链路中的 EKF 节点生效。",
        "restart_targets": [],
    },
}


def describe_config_impact(name: str) -> Dict[str, object]:
    impact = CONFIG_IMPACTS.get(name, {})
    target_modes = list(impact.get("restart_targets", []))
    return {
        "requires_restart": True,
        "message": str(
            impact.get(
                "message",
                "保存后通常需要重启相关节点或 launch 才会生效；运行中的节点不会自动读取 YAML。",
            )
        ),
        "restart_targets": [
            {"mode": mode, "label": RESTART_TARGETS.get(mode, mode)}
            for mode in target_modes
        ],
    }


class ServerApp:
    def __init__(self, host: str = "0.0.0.0", port: int = 8010) -> None:
        self.host = host
        self.port = port
        self.base_dir = Path(__file__).resolve().parent
        self.web_dir = self.base_dir / "web"
        self.repo_dir = self.base_dir.parent
        self.maps_dir = self.repo_dir / "maps"
        self.config_dir = self.repo_dir / "config"
        self.base_config_dir = base_config_dir()
        self.config_store = ConfigStore()
        self.access_password = access_password()
        self._map_lock = threading.RLock()
        self.editor = EditorApplication(
            self.maps_dir,
            self.config_dir / "path_plan.yaml",
        )

        self.state = StateStore()
        self.bridge = RosBridge(
            self.state,
            self.base_config_dir / "handle.yaml",
            self.base_config_dir / "base_control.yaml",
        )
        self.runtime = create_runtime(self.repo_dir, self.state)

        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self) -> None:
        handler_cls = self._build_handler()
        self._httpd = LimitedHTTPServer((self.host, self.port), handler_cls)
        self.bridge.start()
        self._running = True
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        self.state.add_event("info", f"web server listening on http://{self.host}:{self.port}")

    def stop(self) -> None:
        self._running = False
        self.bridge.stop()
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.close_clients()
            self._httpd.server_close()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self.runtime.close()

    def _build_handler(self):
        app = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "FinavWeb/0.1"
            protocol_version = "HTTP/1.1"

            def _authorize(self):
                if authorized(self.headers.get("Authorization"), app.access_password):
                    return True
                self.close_connection = True
                self.send_response(HTTPStatus.UNAUTHORIZED)
                self.send_header("WWW-Authenticate", 'Basic realm="Finav", charset="UTF-8"')
                self.send_header("Content-Length", "0")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                return False

            @request_errors
            def do_GET(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path

                if path == "/api/health":
                    self._json(HTTPStatus.OK, {"ok": True, "ts": time.time()})
                    return
                if path == "/api/status":
                    self._json(HTTPStatus.OK, app.state.snapshot())
                    return
                if path == "/api/system":
                    self._json(HTTPStatus.OK, app.runtime.finav_supervisor_status())
                    return
                if path == "/api/stream":
                    self._stream_scene(parsed)
                    return
                if path == "/api/scene":
                    query = parse_qs(parsed.query)
                    map_version = query_integer(query, "map_version", -1, -1, 2**63 - 1)
                    planning_map_version = query_integer(query, "planning_map_version", -1, -1, 2**63 - 1)
                    self._json(
                        HTTPStatus.OK,
                        app.state.snapshot_scene(
                            known_map_version=map_version,
                            known_planning_map_version=planning_map_version,
                        ),
                    )
                    return
                if path == "/api/history":
                    query = parse_qs(parsed.query)
                    since = query_integer(query, "since", 0, 0, 2**63 - 1)
                    limit = query_integer(query, "limit", 200, 1, 1000)
                    self._json(HTTPStatus.OK, app.state.history_page(
                        since, limit, query.get("session_id", [None])[0]))
                    return
                if path.startswith("/api/runtime/logs/"):
                    mode = unquote(path.removeprefix("/api/runtime/logs/")).strip("/")
                    query = parse_qs(parsed.query)
                    tail = query_integer(query, "tail", 300, 1, 2000)
                    try:
                        payload = app.runtime.read_log(mode, tail=tail)
                    except ValueError as exc:
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                        return
                    version = payload["version"]
                    if query.get("version", [None])[0] == version:
                        payload = {"unchanged": True}
                    self._json(HTTPStatus.OK, {**payload, "version": version})
                    return
                if path == "/api/maps":
                    self._json(HTTPStatus.OK, {"maps": list_saved_maps(app.maps_dir)})
                    return
                parts = self._path_segments(path)
                if (
                    len(parts) == 4
                    and parts[:2] == ["api", "maps"]
                    and parts[3] == "editor"
                ):
                    self._editor_json(lambda: app._load_editor_document(parts[2]))
                    return
                if path.startswith("/api/maps/") and path.endswith("/locations"):
                    name = unquote(path.removeprefix("/api/maps/").removesuffix("/locations"))
                    if app._resolve_map_dir(name) is None:
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid map name"})
                        return
                    self._editor_json(lambda: app._locations_snapshot(name))
                    return
                if path.startswith("/api/maps/"):
                    name = unquote(path.removeprefix("/api/maps/"))
                    if app._resolve_map_dir(name) is None:
                        raise RequestError(400, "invalid map name")
                    self._editor_json(lambda: load_map_preview(app.maps_dir, name))
                    return
                if path == "/api/configs":
                    self._json(HTTPStatus.OK, {"files": app._list_config_files()})
                    return
                if path.startswith("/api/configs/"):
                    rel_name = unquote(path.removeprefix("/api/configs/")).strip("/")
                    target = app._resolve_config_path(rel_name)
                    if target is None or not target.exists():
                        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"config not found: {rel_name}"})
                        return
                    self._json(
                        HTTPStatus.OK,
                        {
                            "name": target.name,
                            "path": f"config/{target.name}",
                            **app.config_store.read(target),
                            "impact": describe_config_impact(target.name),
                        },
                    )
                    return

                self._serve_static(path)

            @request_errors
            def do_POST(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path
                body = self._read_json()

                parts = self._path_segments(path)
                if (
                    len(parts) == 6
                    and parts[:2] == ["api", "maps"]
                    and parts[3] == "routes"
                    and parts[5] == "smooth"
                ):
                    self._editor_json(
                        lambda: app.editor.smooth(parts[2], parts[4], body)
                    )
                    return
                if (
                    len(parts) == 4
                    and parts[:2] == ["api", "maps"]
                    and parts[3] == "export"
                ):
                    self._editor_json(lambda: app.editor.export(parts[2], body))
                    return

                if path == "/api/nav/route":
                    self._editor_json(lambda: app._route_command(body))
                    return

                if path == "/api/nav/goal":
                    finite_fields(body, "x", "y", "yaw_deg")
                    app.bridge.command(
                        {
                            "type": "set_goal",
                            "x": body.get("x", 0.0),
                            "y": body.get("y", 0.0),
                            "yaw_deg": body.get("yaw_deg", 0.0),
                        }
                    )
                    self._json(HTTPStatus.OK, {"ok": True})
                    return

                if path == "/api/control/manual":
                    finite_fields(body, "linear", "angular")
                    try:
                        linear = float(body.get("linear", 0.0))
                        angular = float(body.get("angular", 0.0))
                    except (TypeError, ValueError):
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "控制量必须是数字"})
                        return
                    if not math.isfinite(linear) or not math.isfinite(angular):
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "控制量必须是有限数字"})
                        return

                    magnitude = math.hypot(linear, angular)
                    if magnitude > 1.0:
                        linear /= magnitude
                        angular /= magnitude
                    is_stop = abs(linear) <= 1e-6 and abs(angular) <= 1e-6
                    control = app.state.snapshot().get("control", {})
                    if not is_stop and control.get("manual_locked"):
                        self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "手柄抢停后，请先将网页控制回中"})
                        return
                    if not is_stop and not control.get("gear_online"):
                        self._json(HTTPStatus.CONFLICT, {"ok": False, "error": "未读取到手柄档位，已禁止网页移动"})
                        return

                    app.bridge.command({"type": "manual_drive", "linear": linear, "angular": angular})
                    self._json(
                        HTTPStatus.OK,
                        {"ok": True, "linear": linear, "angular": angular, "gear": control.get("gear")},
                    )
                    return

                if path == "/api/nav/initialpose":
                    finite_fields(body, "x", "y", "yaw_deg")
                    app.bridge.command(
                        {
                            "type": "set_initialpose",
                            "x": body.get("x", 0.0),
                            "y": body.get("y", 0.0),
                            "yaw_deg": body.get("yaw_deg", 0.0),
                        }
                    )
                    self._json(HTTPStatus.OK, {"ok": True})
                    return

                if path == "/api/nav/cancel":
                    app.bridge.command({"type": "cancel_nav"})
                    self._json(HTTPStatus.OK, {"ok": True})
                    return

                if path == "/api/nav/relocate":
                    map_file = body.get("map_file", "")
                    if map_file and (not isinstance(map_file, str) or app._resolve_map_dir(map_file) is None):
                        raise RequestError(400, "invalid map name")
                    params = {"map_file": map_file} if isinstance(map_file, str) and map_file.strip() else {}
                    try:
                        with app._map_lock:
                            runtime = app.runtime.run_relocate(params=params)
                    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                        self._json(HTTPStatus.CONFLICT, {"ok": False, "error": str(exc)})
                        return
                    self._json(HTTPStatus.OK, {"ok": True, "runtime": runtime})
                    return

                if path == "/api/nav/location":
                    name = body.get("name", "")
                    if not isinstance(name, str) or not name.strip():
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "name is required"})
                        return
                    app.bridge.command({"type": "nav_to_location", "name": name.strip()})
                    self._json(HTTPStatus.OK, {"ok": True, "name": name.strip()})
                    return

                if path == "/api/map/save":
                    name = body.get("name")
                    if not isinstance(name, str) or app._resolve_map_dir(name) is None or name.startswith("-"):
                        raise RequestError(400, "invalid map name")
                    with app._map_lock:
                        mapping = app.runtime.snapshot().get("mapping", {})
                        if not mapping.get("running") or mapping.get("stopping"):
                            raise RequestError(409, "Mapping must be running before saving")
                        job = app.bridge.save_map(name)
                    self._json(HTTPStatus.ACCEPTED, {"ok": True, "job": job})
                    return
                if path in {"/api/system/shutdown", "/api/system/restart"}:
                    action = path.rsplit("/", 1)[-1]
                    try:
                        payload = app.runtime.request_finav_action(action)
                    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                        self._json(HTTPStatus.CONFLICT, {"ok": False, "error": str(exc)})
                        return
                    self._json(HTTPStatus.ACCEPTED, payload)
                    return
                if path.startswith("/api/maps/") and path.endswith("/locations"):
                    name = unquote(path.removeprefix("/api/maps/").removesuffix("/locations"))
                    target = app._resolve_map_dir(name)
                    if target is None:
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid map name"})
                        return
                    if not target.exists() or not target.is_dir():
                        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"map not found: {name}"})
                        return
                    self._editor_json(lambda: app._save_locations(name, body))
                    return
                if path.startswith("/api/maps/") and path.endswith("/delete"):
                    name = unquote(path.removeprefix("/api/maps/").removesuffix("/delete"))
                    target = app._resolve_map_dir(name)
                    if target is None:
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid map name"})
                        return
                    if not target.exists() or not target.is_dir():
                        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"map not found: {name}"})
                        return
                    with app._map_lock:
                        runtime = app.runtime.snapshot()
                        if runtime.get("busy") or any(
                            item.get("running") or item.get("stopping")
                            for key, item in runtime.items()
                            if key in {"mapping", "navigation", "relocate"} and isinstance(item, dict)
                        ) or app.state.snapshot().get("map_save", {}).get("state") == "running":
                            raise RequestError(409, "Stop mapping/navigation/relocation and finish map saving before deletion")
                        shutil.rmtree(target)
                    app.state.add_event("info", "map deleted", {"name": name})
                    self._json(HTTPStatus.OK, {"ok": True, "name": name})
                    return
                if path == "/api/runtime/mapping/start":
                    self._runtime_json(lambda: app.runtime.start("mapping", launch_args={}))
                    return
                if path == "/api/runtime/mapping/stop":
                    self._runtime_json(lambda: app.runtime.stop("mapping"))
                    return
                if path == "/api/runtime/navigation/start":
                    map_file = body.get("map_file", "")
                    target = app._resolve_map_dir(map_file) if isinstance(map_file, str) else None
                    if target is None or not (target / f"{map_file}.yaml").is_file():
                        raise RequestError(400, "A saved map is required")
                    launch_args = {"map_file": map_file} if isinstance(map_file, str) and map_file.strip() else {}
                    self._runtime_json(lambda: app.runtime.start("navigation", launch_args=launch_args))
                    return
                if path == "/api/runtime/navigation/stop":
                    self._runtime_json(lambda: app.runtime.stop("navigation"))
                    return
                if path.startswith("/api/runtime/logs/") and path.endswith("/clear"):
                    mode = unquote(path.removeprefix("/api/runtime/logs/").removesuffix("/clear")).strip("/")
                    try:
                        payload = app.runtime.clear_log(mode)
                    except ValueError as exc:
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                        return
                    self._json(HTTPStatus.OK, payload)
                    return
                if path.startswith("/api/runtime/") and path.endswith("/restart"):
                    mode = unquote(path.removeprefix("/api/runtime/").removesuffix("/restart")).strip("/")
                    self._runtime_json(lambda: app.runtime.restart(mode))
                    return
                if path.startswith("/api/configs/"):
                    rel_name = unquote(path.removeprefix("/api/configs/")).strip("/")
                    target = app._resolve_config_path(rel_name)
                    if target is None:
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid config path"})
                        return
                    if not target.exists():
                        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"config not found: {rel_name}"})
                        return
                    revision = app.config_store.save(target, body)
                    app.state.add_event("info", "config saved", {"file": target.name})
                    self._json(HTTPStatus.OK, {"ok": True, "name": target.name, "revision": revision,
                                               "impact": describe_config_impact(target.name)})
                    return

                self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"unknown endpoint: {path}"})

            @request_errors
            def do_PUT(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                parts = self._path_segments(parsed.path)
                if (
                    len(parts) == 4
                    and parts[:2] == ["api", "maps"]
                    and parts[3] == "editor"
                ):
                    body = self._read_json()
                    self._editor_json(
                        lambda: app._save_editor_document(parts[2], body)
                    )
                    return
                self._json(
                    HTTPStatus.NOT_FOUND,
                    {"ok": False, "error": f"unknown endpoint: {parsed.path}"},
                )

            @staticmethod
            def _path_segments(path: str) -> list[str]:
                return [unquote(part) for part in path.split("/") if part]

            def _runtime_json(self, operation) -> None:
                try:
                    with app._map_lock:
                        payload = operation()
                except ValueError as exc:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                    return
                except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                    self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "error": str(exc)})
                    return
                self._json(HTTPStatus.OK, {"ok": True, "runtime": payload})

            def _editor_json(self, operation) -> None:
                try:
                    with app._map_lock:
                        payload = operation()
                except RequestError:
                    raise
                except ApiError as error:
                    status, code, field = error.status, error.code, error.field
                    message = str(error)
                except RevisionConflict as error:
                    status, code, field = 409, "revision_conflict", error.field
                    message = str(error)
                except DocumentError as error:
                    status, code, field = 422, error.code, error.field
                    message = str(error)
                except MapFormatError as error:
                    status = 404 if error.code == "missing_map" else 422
                    code, field = error.code, None
                    message = str(error)
                except ExportError as error:
                    status, code, field = 409, "export_failed", "target_name"
                    message = str(error)
                except (TypeError, ValueError) as error:
                    status, code, field = 400, "invalid_request", None
                    message = str(error)
                else:
                    self._json(HTTPStatus.OK, payload)
                    return
                detail = {"code": code, "message": message}
                if field:
                    detail["field"] = field
                self._json(status, {"ok": False, "error": detail})

            def _stream_scene(self, parsed) -> None:
                query = parse_qs(parsed.query)
                mode = str(query.get("mode", ["mapping"])[0]).strip().lower()
                if mode not in {"mapping", "navigation"}:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid stream mode"})
                    return
                map_version = query_integer(query, "map_version", -1, -1, 2**63 - 1)
                planning_map_version = query_integer(query, "planning_map_version", -1, -1, 2**63 - 1)
                plan_version = query_integer(query, "plan_version", -1, -1, 2**63 - 1)
                if not self.server.stream_slots.acquire(blocking=False):
                    raise RequestError(503, "Too many live streams")
                try:
                    self._send_scene_stream(mode, map_version, planning_map_version, plan_version)
                finally:
                    self.server.stream_slots.release()

            def _send_scene_stream(self, mode, map_version, planning_map_version, plan_version):
                target_hz = 10.0 if mode == "mapping" else 15.0
                interval = 1.0 / target_hz
                frames = 0
                bytes_sent = 0
                stats_started = time.monotonic()

                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Cache-Control", "no-cache, no-transform")
                self.send_header("Connection", "keep-alive")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                try:
                    self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass

                app.state.add_live_viewer(mode)
                try:
                    self.wfile.write(b"retry: 1000\n\n")
                    self.wfile.flush()
                    while app._running:
                        started = time.monotonic()
                        payload = app.state.snapshot_scene(
                            known_map_version=map_version,
                            known_planning_map_version=planning_map_version,
                            known_plan_version=plan_version,
                        )
                        map_version = int(payload.get("map_version", map_version))
                        planning_map_version = int(
                            payload.get("planning_map_version", planning_map_version)
                        )
                        plan_version = int(payload.get("plan_version", plan_version))
                        payload["stream"] = {
                            "mode": mode,
                            "target_hz": target_hz,
                            "server_sent_at": time.time(),
                        }
                        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                        packet = b"data: " + encoded + b"\n\n"
                        self.wfile.write(packet)
                        self.wfile.flush()
                        frames += 1
                        bytes_sent += len(packet)

                        stats_elapsed = time.monotonic() - stats_started
                        if stats_elapsed >= 1.0:
                            app.state.update_status(
                                {
                                    "transport": {
                                        "stream_hz": round(frames / stats_elapsed, 2),
                                        "bytes_per_sec": round(bytes_sent / stats_elapsed),
                                        "last_frame_at": time.time(),
                                    }
                                }
                            )
                            frames = 0
                            bytes_sent = 0
                            stats_started = time.monotonic()

                        remaining = interval - (time.monotonic() - started)
                        if remaining > 0:
                            time.sleep(remaining)
                except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                    pass
                finally:
                    app.state.remove_live_viewer(mode)

            def _read_json(self) -> Dict[str, Any]:
                if self.headers.get("Transfer-Encoding"):
                    raise RequestError(400, "Transfer-Encoding is not supported")
                if self.headers.get_content_type() != "application/json":
                    raise RequestError(415, "Content-Type must be application/json")
                origin = self.headers.get("Origin")
                parsed_origin = urlparse(origin or "")
                if (self.headers.get("Sec-Fetch-Site") == "cross-site"
                        or (origin and (parsed_origin.scheme not in {"http", "https"}
                                        or parsed_origin.netloc != self.headers.get("Host", "")))):
                    raise RequestError(403, "Cross-origin writes are not allowed")
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                except ValueError as exc:
                    raise RequestError(400, "Invalid Content-Length") from exc
                if size < 0:
                    raise RequestError(400, "Invalid Content-Length")
                if size > 4 * 1024 * 1024:
                    raise RequestError(413, "Request body is too large")
                raw = self.rfile.read(size) if size else b"{}"
                if size and len(raw) != size:
                    raise RequestError(400, "Incomplete request body")
                try:
                    data = json.loads(raw.decode("utf-8"), parse_constant=self._invalid_constant)
                except (ValueError, UnicodeError) as exc:
                    raise RequestError(400, "Invalid JSON") from exc
                if not isinstance(data, dict):
                    raise RequestError(400, "JSON body must be an object")
                return data

            @staticmethod
            def _invalid_constant(value):
                raise ValueError(f"Invalid JSON constant: {value}")

            def _serve_static(self, path: str) -> None:
                if path == "/":
                    path = "/index.html"

                target = (app.web_dir / unquote(path).lstrip("/")).resolve()
                if not target.is_relative_to(app.web_dir.resolve()) or not target.is_file():
                    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
                    return

                ctype, _ = mimetypes.guess_type(str(target))
                ctype = ctype or "application/octet-stream"
                content = target.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(content)

            def _json(self, code: HTTPStatus, payload: Dict[str, Any]) -> None:
                data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                try:
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass

            def log_message(self, fmt: str, *args: Any) -> None:
                return

        return Handler

    def _resolve_config_path(self, rel_name: str) -> Optional[Path]:
        if not rel_name:
            return None
        if "/" in rel_name or "\\" in rel_name:
            return None
        if not rel_name.endswith((".yaml", ".yml")):
            return None
        root = self.base_config_dir if rel_name in {"base_control.yaml", "handle.yaml"} else self.config_dir
        target = (root / rel_name).resolve()
        if target.parent != root.resolve():
            return None
        return target

    def _route_command(self, body: Dict[str, object]) -> Dict[str, object]:
        action = body.get("action")
        if action not in {"start", "pause", "resume", "cancel"}:
            raise ApiError(400, "invalid_action", "Invalid route action")
        runtime = self.runtime.snapshot().get("navigation", {})
        if not runtime.get("running") or runtime.get("stopping"):
            raise ApiError(409, "navigation_not_running", "Navigation service is not running")
        if action == "start":
            map_name = str(body.get("map_name", ""))
            active_map = str(runtime.get("launch_args", {}).get("map_file", ""))
            if map_name != active_map:
                raise ApiError(409, "route_map_mismatch", "Route map differs from active navigation map")
            data = self._load_editor_document(map_name)
            if data.get("read_only"):
                raise ApiError(409, "invalid_editor_document", "Route document cannot be executed")
            route = next((item for item in data["document"]["routes"] if item["id"] == body.get("route_id")), None)
            if route is None:
                raise ApiError(404, "route_not_found", "Route not found")
            if route["closed"]:
                raise ApiError(400, "closed_route_not_supported", "Only open routes can be executed")
        self.bridge.command({"type": "route_command", "action": action,
                             "map_name": body.get("map_name", ""), "route_id": body.get("route_id", "")})
        return {"ok": True, "accepted": True}

    def _load_editor_document(self, map_name: str) -> Dict[str, object]:
        payload = self.editor.load_document(map_name, include_map=False)
        document = payload.get("document", {})
        editor_path = self.maps_dir / map_name / f"{map_name}.editor.yaml"
        if (
            isinstance(document, dict)
            and not editor_path.exists()
            and not document.get("locations")
        ):
            locations = []
            for item in load_map_locations(self.maps_dir, map_name):
                digest = hashlib.sha1(
                    f"{map_name}\0{item['name']}".encode("utf-8")
                ).hexdigest()[:16]
                locations.append({"id": f"location-{digest}", **item})
            document["locations"] = locations
        return payload

    def _save_editor_document(
        self, map_name: str, body: object
    ) -> Dict[str, object]:
        with self._map_lock:
            baseline = self._load_editor_document(map_name).get("document", {})
            return self.editor.save_document(map_name, body, baseline)

    def _save_locations(self, map_name, body):
        with self._map_lock:
            document = self._load_editor_document(map_name)["document"]
            if "locations" not in body or "expected_revision" not in body:
                raise RequestError(400, "locations and expected_revision are required")
            locations = body["locations"]
            if not isinstance(locations, list):
                raise RequestError(400, "locations must be a list")
            document["locations"] = [dict(item, id=item.get("id") or
                "location-" + hashlib.sha256(str(item.get("name", "")).encode()).hexdigest()[:16])
                for item in locations if isinstance(item, dict)]
            if len(document["locations"]) != len(locations):
                raise RequestError(400, "Invalid location entry")
            result = self._save_editor_document(map_name, {
                "document": document, "expected_revision": body["expected_revision"]})
            return {"ok": True, "map_file": map_name, "locations": result["document"]["locations"],
                    "revision": result["document"]["revision"]}

    def _locations_snapshot(self, map_name):
        document = self._load_editor_document(map_name)["document"]
        return {"map_file": map_name, "locations": document["locations"], "revision": document["revision"]}

    def _resolve_map_dir(self, name: str) -> Optional[Path]:
        if not isinstance(name, str) or not name or name != name.strip():
            return None
        try:
            validate_map_name(name)
        except MapFormatError:
            return None
        target = (self.maps_dir / name).resolve()
        if target.parent != self.maps_dir.resolve() or (self.maps_dir / name).is_symlink():
            return None
        return target

    def _list_config_files(self) -> list[dict[str, object]]:
        if not self.config_dir.exists():
            return []

        files = []
        paths = list(self.config_dir.iterdir()) + [self.base_config_dir / name for name in ("base_control.yaml", "handle.yaml")]
        for path in sorted(paths, key=lambda item: item.name):
            if not path.is_file() or path.suffix not in {".yaml", ".yml"}:
                continue
            stat = path.stat()
            files.append(
                {
                    "name": path.name,
                    "path": f"{'base_control' if path.parent == self.base_config_dir else 'finav'}/config/{path.name}",
                    "size": stat.st_size,
                    "modified_at": stat.st_mtime,
                    "impact": describe_config_impact(path.name),
                }
            )
        return files
