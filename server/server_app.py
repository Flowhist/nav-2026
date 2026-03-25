#!/usr/bin/env python3
import json
import mimetypes
import shutil
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, unquote, urlparse

from map_utils import list_saved_maps, load_map_preview
from process_manager import RuntimeManager
from ros_bridge import RosBridge
from state_store import StateStore


class ServerApp:
    def __init__(self, host: str = "0.0.0.0", port: int = 8010) -> None:
        self.host = host
        self.port = port
        self.base_dir = Path(__file__).resolve().parent
        self.web_dir = self.base_dir / "web"
        self.repo_dir = self.base_dir.parent
        self.maps_dir = self.repo_dir / "maps"
        self.config_dir = self.repo_dir / "config"

        self.state = StateStore()
        self.bridge = RosBridge(self.state)
        self.runtime = RuntimeManager(self.repo_dir, self.state)

        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.bridge.start()
        handler_cls = self._build_handler()
        self._httpd = ThreadingHTTPServer((self.host, self.port), handler_cls)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        self.state.add_event("info", f"web server listening on http://{self.host}:{self.port}")

    def stop(self) -> None:
        self.runtime.stop_all()
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)

    def _build_handler(self):
        app = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "FinavWeb/0.1"

            def do_GET(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path

                if path == "/api/health":
                    self._json(HTTPStatus.OK, {"ok": True, "ts": time.time()})
                    return
                if path == "/api/status":
                    self._json(HTTPStatus.OK, app.state.snapshot())
                    return
                if path == "/api/scene":
                    query = parse_qs(parsed.query)
                    map_version = int(query.get("map_version", [-1])[0])
                    self._json(HTTPStatus.OK, app.state.snapshot_scene(known_map_version=map_version))
                    return
                if path == "/api/history":
                    query = parse_qs(parsed.query)
                    since = int(query.get("since", [0])[0])
                    limit = int(query.get("limit", [200])[0])
                    data = app.state.get_history(since_seq=since, limit=limit)
                    self._json(HTTPStatus.OK, {"events": data, "latest_seq": data[-1]["seq"] if data else since})
                    return
                if path.startswith("/api/runtime/logs/"):
                    mode = unquote(path.removeprefix("/api/runtime/logs/")).strip("/")
                    query = parse_qs(parsed.query)
                    tail = int(query.get("tail", [300])[0])
                    try:
                        payload = app.runtime.read_log(mode, tail=tail)
                    except ValueError as exc:
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                        return
                    self._json(HTTPStatus.OK, payload)
                    return
                if path == "/api/maps":
                    self._json(HTTPStatus.OK, {"maps": list_saved_maps(app.maps_dir)})
                    return
                if path.startswith("/api/maps/"):
                    name = unquote(path.removeprefix("/api/maps/")).strip("/")
                    payload = load_map_preview(app.maps_dir, name)
                    if payload is None:
                        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"map not found: {name}"})
                        return
                    self._json(HTTPStatus.OK, payload)
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
                            "content": target.read_text(encoding="utf-8"),
                        },
                    )
                    return

                self._serve_static(path)

            def do_POST(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path
                body = self._read_json()

                if path == "/api/nav/goal":
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

                if path == "/api/nav/initialpose":
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

                if path == "/api/teleop/cmd_vel":
                    app.bridge.set_teleop_state(
                        float(body.get("linear_x", 0.0)),
                        float(body.get("angular_z", 0.0)),
                    )
                    self._json(HTTPStatus.OK, {"ok": True})
                    return

                if path == "/api/teleop/state":
                    app.bridge.set_teleop_state(
                        float(body.get("linear_x", 0.0)),
                        float(body.get("angular_z", 0.0)),
                    )
                    self._json(HTTPStatus.OK, {"ok": True})
                    return

                if path == "/api/teleop/stop":
                    app.bridge.stop_teleop()
                    self._json(HTTPStatus.OK, {"ok": True})
                    return

                if path == "/api/map/save":
                    app.bridge.command({"type": "save_map", "name": body.get("name", "manual_map")})
                    self._json(HTTPStatus.OK, {"ok": True})
                    return
                if path.startswith("/api/maps/") and path.endswith("/delete"):
                    name = unquote(path.removeprefix("/api/maps/").removesuffix("/delete")).strip("/")
                    target = app._resolve_map_dir(name)
                    if target is None:
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid map name"})
                        return
                    if not target.exists() or not target.is_dir():
                        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"map not found: {name}"})
                        return
                    shutil.rmtree(target)
                    app.state.add_event("info", "map deleted", {"name": name})
                    self._json(HTTPStatus.OK, {"ok": True, "name": name})
                    return
                if path == "/api/runtime/mapping/start":
                    self._json(HTTPStatus.OK, {"ok": True, "runtime": app.runtime.start("mapping")})
                    return
                if path == "/api/runtime/mapping/stop":
                    self._json(HTTPStatus.OK, {"ok": True, "runtime": app.runtime.stop("mapping")})
                    return
                if path == "/api/runtime/navigation/start":
                    map_file = body.get("map_file", "")
                    launch_args = {"map_file": map_file} if isinstance(map_file, str) and map_file.strip() else {}
                    self._json(
                        HTTPStatus.OK,
                        {"ok": True, "runtime": app.runtime.start("navigation", launch_args=launch_args)},
                    )
                    return
                if path == "/api/runtime/navigation/stop":
                    self._json(HTTPStatus.OK, {"ok": True, "runtime": app.runtime.stop("navigation")})
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
                if path.startswith("/api/configs/"):
                    rel_name = unquote(path.removeprefix("/api/configs/")).strip("/")
                    target = app._resolve_config_path(rel_name)
                    if target is None:
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid config path"})
                        return
                    if not target.exists():
                        self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"config not found: {rel_name}"})
                        return
                    content = body.get("content", "")
                    if not isinstance(content, str):
                        self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "content must be string"})
                        return
                    target.write_text(content, encoding="utf-8")
                    app.state.add_event("info", "config saved", {"file": target.name})
                    self._json(HTTPStatus.OK, {"ok": True, "name": target.name})
                    return

                self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": f"unknown endpoint: {path}"})

            def _read_json(self) -> Dict[str, Any]:
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                except ValueError:
                    size = 0
                raw = self.rfile.read(size) if size > 0 else b"{}"
                if not raw:
                    return {}
                try:
                    data = json.loads(raw.decode("utf-8"))
                    return data if isinstance(data, dict) else {}
                except Exception:
                    return {}

            def _serve_static(self, path: str) -> None:
                if path == "/":
                    path = "/index.html"

                target = (app.web_dir / path.lstrip("/")).resolve()
                if not str(target).startswith(str(app.web_dir.resolve())) or not target.exists() or not target.is_file():
                    self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
                    return

                ctype, _ = mimetypes.guess_type(str(target))
                ctype = ctype or "application/octet-stream"
                content = target.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def _json(self, code: HTTPStatus, payload: Dict[str, Any]) -> None:
                data = json.dumps(payload).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

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
        target = (self.config_dir / rel_name).resolve()
        if not str(target).startswith(str(self.config_dir.resolve())):
            return None
        return target

    def _resolve_map_dir(self, name: str) -> Optional[Path]:
        if not name or "/" in name or "\\" in name:
            return None
        target = (self.maps_dir / name).resolve()
        if not str(target).startswith(str(self.maps_dir.resolve())):
            return None
        return target

    def _list_config_files(self) -> list[dict[str, object]]:
        if not self.config_dir.exists():
            return []

        files = []
        for path in sorted(self.config_dir.iterdir()):
            if not path.is_file() or path.suffix not in {".yaml", ".yml"}:
                continue
            stat = path.stat()
            files.append(
                {
                    "name": path.name,
                    "path": f"config/{path.name}",
                    "size": stat.st_size,
                    "modified_at": stat.st_mtime,
                }
            )
        return files
