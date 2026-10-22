"""Offline regressions: temporary maps/configs, no ROS node or systemd calls."""

import base64
import http.client
import json
import socket
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import zlib
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from server_app import ServerApp
from editor_server import EditorApplication
from location_utils import load_locations
from map_utils import load_map_preview
from process_manager import RuntimeManager
from ros_bridge import RosBridge, _BridgeNode
from state_store import StateStore
from web_support import ConfigStore, LimitedHTTPServer, RequestError, access_password, authorized


def isolated_app(root):
    app = ServerApp.__new__(ServerApp)
    app.repo_dir = root
    app.web_dir = Path(__file__).resolve().parents[1] / "web"
    app.maps_dir = root / "maps"
    app.config_dir = root / "config"
    app.base_config_dir = root / "base"
    for directory in (app.maps_dir, app.config_dir, app.base_config_dir):
        directory.mkdir()
    app.access_password = "offline-test-password"
    app.config_store = ConfigStore()
    app._map_lock = threading.RLock()
    app.state = StateStore()
    app.bridge = RosBridge(app.state, root / "missing-handle.yaml", root / "missing-router.yaml")
    app.editor = EditorApplication(app.maps_dir, app.config_dir / "path_plan.yaml")
    logs = RuntimeManager.__new__(RuntimeManager)
    logs.runtime_dir = root
    logs._lock = threading.RLock()
    logs._log_cache = {}
    logs.state_store = app.state
    app.runtime = SimpleNamespace(snapshot=lambda: app.state.snapshot()["runtime"],
        read_log=logs.read_log, clear_log=logs.clear_log, close=lambda: None,
        finav_supervisor_status=lambda: {"managed": False})
    app._running = True
    return app


def make_map(app, name="test"):
    directory = app.maps_dir / name
    directory.mkdir()
    (directory / f"{name}.pgm").write_bytes(b"P5\n20 20\n255\n" + b"\xff" * 400)
    (directory / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nresolution: 1.0\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return directory


class WebTests(unittest.TestCase):
    def test_route_end_name_survives_http_save_and_fresh_reader(self):
        from editor_store import EditorStore
        directory = make_map(self.app)
        status, loaded = self.request("GET", "/api/maps/test/editor")
        self.assertEqual(status, 200)
        document = loaded["document"]
        document["routes"] = [{"id": "route", "name": "Corridor", "end_name": "Lobby",
            "closed": False, "direction": "forward", "repeat_count": 1,
            "speed_limit_mps": .25, "waypoints": [
                {"id": "a", "x": 5, "y": 5}, {"id": "b", "x": 12, "y": 5}]}]
        status, saved = self.request("PUT", "/api/maps/test/editor", {
            "expected_revision": document["revision"], "document": document})
        self.assertEqual(status, 200)
        self.assertEqual(saved["document"]["routes"][0]["end_name"], "Lobby")
        status, reloaded = self.request("GET", "/api/maps/test/editor")
        self.assertEqual(status, 200)
        self.assertEqual(reloaded["document"]["routes"][0]["end_name"], "Lobby")
        fresh = EditorStore(directory.parent).load("test", "hash", .6)
        self.assertEqual(fresh["routes"][0]["end_name"], "Lobby")

    def test_navigation_speed_accepts_raised_limits(self):
        import math
        path = self.app.config_dir / "nav.yaml"
        path.write_text("nav_control:\n  ros__parameters:\n    linear_speed_mps: 0.6\n    angular_speed_degps: 28.0\n    drive_angular_speed_degps: 25.0\n")
        _, loaded = self.request("GET", "/api/nav/speed")
        self.assertEqual(loaded["limits"]["linear_speed_mps"][1], 2.0)
        self.assertAlmostEqual(math.radians(loaded["limits"]["angular_speed_degps"][1]), 0.8)
        body = {"linear_speed_mps": 2.0, "angular_speed_degps": math.degrees(0.8),
                "drive_angular_speed_degps": math.degrees(0.8), "expected_revision": loaded["revision"]}
        status, saved = self.request("POST", "/api/nav/speed", body)
        self.assertEqual(status, 200)
        body["expected_revision"] = saved["revision"]
        for key in ("linear_speed_mps", "angular_speed_degps", "drive_angular_speed_degps"):
            self.assertEqual(self.request("POST", "/api/nav/speed", {**body, key: body[key] + 0.01})[0], 400)

    def test_navigation_speed_preserves_yaml_and_checks_revision(self):
        path = self.app.config_dir / "nav.yaml"
        original = "# navigation\nnav_control:\n  ros__parameters:\n    linear_speed_mps: 0.5 # limit\n    angular_speed_degps: 20.0\n    drive_angular_speed_degps: 25.0\n"
        path.write_text(original)
        status, data = self.request("GET", "/api/nav/speed")
        self.assertEqual(status, 200)
        body = {"linear_speed_mps": 0.3, "angular_speed_degps": 15.0, "drive_angular_speed_degps": 18.0, "expected_revision": data["revision"]}
        status, result = self.request("POST", "/api/nav/speed", body)
        self.assertEqual(status, 200)
        self.assertTrue(result["saved"])
        self.assertFalse(result["applied"])
        self.assertEqual(path.read_text(), original.replace("0.5", "0.3").replace("20.0", "15.0").replace("25.0", "18.0"))
        self.assertEqual(self.request("POST", "/api/nav/speed", body)[0], 409)

    def test_navigation_speed_rejects_invalid_and_reports_apply_failure(self):
        path = self.app.config_dir / "nav.yaml"
        path.write_text("nav_control:\n  ros__parameters:\n    linear_speed_mps: 0.5\n    angular_speed_degps: 20.0\n    drive_angular_speed_degps: 25.0\n")
        _, data = self.request("GET", "/api/nav/speed")
        for value in (True, "0.3", -1, 0, 2.01, float("nan")):
            self.assertEqual(self.request("POST", "/api/nav/speed", {
                "linear_speed_mps": value, "expected_revision": data["revision"]})[0], 400)
        self.app.runtime.snapshot = lambda: {"navigation": {"running": True}}
        with patch.object(self.app.bridge, "request_setting", side_effect=RequestError(503, "offline")):
            status, result = self.request("POST", "/api/nav/speed", {
                "linear_speed_mps": 0.25, "angular_speed_degps": 20.0, "drive_angular_speed_degps": 25.0, "expected_revision": data["revision"]})
        self.assertEqual(status, 200)
        self.assertTrue(result["saved"])
        self.assertFalse(result["applied"])
        self.assertEqual(result["apply_error"], "offline")
        with patch.object(self.app.bridge, "request_setting") as apply:
            status, result = self.request("POST", "/api/nav/speed", {
                "linear_speed_mps": 0.35, "angular_speed_degps": 20.0, "drive_angular_speed_degps": 25.0, "expected_revision": result["revision"]})
        self.assertEqual(status, 200)
        self.assertTrue(result["applied"])
        apply.assert_called_once_with("navigation_speed", {"linear_speed_mps": 0.35, "angular_speed_degps": 20.0, "drive_angular_speed_degps": 25.0})
        previous = path.read_text()
        self.assertEqual(self.request("POST", "/api/nav/speed", {
            "linear_speed_mps": 0.2, "angular_speed_degps": 90, "drive_angular_speed_degps": 20.0, "expected_revision": result["revision"]})[0], 400)
        self.assertEqual(path.read_text(), previous)

    def test_remote_gear_requires_online_and_acknowledgment(self):
        self.assertEqual(self.request("POST", "/api/control/gear", {"gear": 2})[0], 409)
        self.app.state.update_status({"control": {"gear_online": True}})
        for gear in (0, 6, True, 2.0, "2"):
            self.assertEqual(self.request("POST", "/api/control/gear", {"gear": gear})[0], 400)
        with patch.object(self.app.bridge, "request_setting") as apply:
            self.assertEqual(self.request("POST", "/api/control/gear", {"gear": 3})[0], 200)
            apply.assert_called_once_with("handle_gear", 3)
        with patch.object(self.app.bridge, "request_setting", side_effect=RequestError(504, "timeout")):
            self.assertEqual(self.request("POST", "/api/control/gear", {"gear": 4})[0], 504)

    def test_identical_maps_repopulate_scene_after_runtime_clear(self):
        state = self.app.state
        node = SimpleNamespace(state_store=state, _last_map_signature=None,
                               _last_planning_map_signature=None)
        msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"), data=[-1, 0, 100],
            info=SimpleNamespace(width=3, height=1, resolution=0.05,
                origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0),
                    orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
        with patch.object(_BridgeNode, "_touch"):
            for key, callback in (("map", _BridgeNode._on_map),
                                  ("planning_map", _BridgeNode._on_planning_map)):
                with self.subTest(map_type=key):
                    callback(node, msg)
                    first = state.snapshot_scene()
                    callback(node, msg)
                    self.assertEqual(state.snapshot_scene()[key + "_version"],
                                     first[key + "_version"])
                    RuntimeManager._clear_live_scene(SimpleNamespace(state_store=state))
                    cleared = state.snapshot_scene()
                    self.assertIsNone(cleared[key])
                    callback(node, msg)
                    restored = state.snapshot_scene()
                    self.assertEqual(restored[key]["width"], 3)
                    self.assertEqual(restored[key]["encoding"], first[key]["encoding"])
                    self.assertGreater(restored[key + "_version"], cleared[key + "_version"])

    def test_raster_save_preserves_other_pixels_and_checks_version(self):
        directory = make_map(self.app)
        original = (directory / "test.pgm").read_bytes()
        preview = load_map_preview(self.app.maps_dir, "test")
        body = {"source_sha256": preview["source_sha256"], "cells": [[0, 100], [21, -1], [399, 0]]}
        self.assertEqual(self.request("PUT", "/api/maps/test/raster", body)[0], 200)
        from editor_map_io import load_map
        data = load_map(self.app.maps_dir, "test")
        self.assertEqual(data.occupancy[0, 0], 100)
        self.assertEqual(data.occupancy[1, 1], -1)
        self.assertEqual(data.pixels[0, 1], 255)
        self.assertEqual((directory / "test.pgm.brush-backup").read_bytes(), original)
        self.assertEqual(self.request("PUT", "/api/maps/test/raster", body)[0], 409)

    def test_raster_rejects_invalid_cells_and_active_navigation(self):
        directory = make_map(self.app)
        original = (directory / "test.pgm").read_bytes()
        body = {"source_sha256": load_map_preview(self.app.maps_dir, "test")["source_sha256"], "cells": [[400, 0]]}
        self.assertEqual(self.request("PUT", "/api/maps/test/raster", body)[0], 400)
        self.assertEqual((directory / "test.pgm").read_bytes(), original)
        body["cells"] = [[0, 100]]
        self.app.runtime.snapshot = lambda: {"navigation": {"running": True}}
        self.assertEqual(self.request("PUT", "/api/maps/test/raster", body)[0], 409)

    def test_raster_negated_map_unknown(self):
        directory = make_map(self.app)
        metadata = directory / "test.yaml"
        metadata.write_text(metadata.read_text().replace("negate: 0", "negate: 1"))
        body = {"source_sha256": load_map_preview(self.app.maps_dir, "test")["source_sha256"], "cells": [[0, 100], [1, 0], [2, -1]]}
        self.assertEqual(self.request("PUT", "/api/maps/test/raster", body)[0], 200)
        from editor_map_io import load_map
        self.assertEqual(load_map(self.app.maps_dir, "test").occupancy[0, :3].tolist(), [100, 0, -1])

    def test_raster_failed_replace_keeps_original(self):
        directory = make_map(self.app)
        original = (directory / "test.pgm").read_bytes()
        body = {"source_sha256": load_map_preview(self.app.maps_dir, "test")["source_sha256"], "cells": [[0, 100]]}
        with patch("map_utils.os.replace", side_effect=OSError("disk error")):
            with self.assertRaises(OSError):
                self.app._save_map_raster("test", body)
        self.assertEqual((directory / "test.pgm").read_bytes(), original)
        self.assertEqual(list(directory.glob(".raster-*")), [])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.app = isolated_app(self.root)
        self.server = LimitedHTTPServer(("127.0.0.1", 0), self.app._build_handler())
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.01})
        self.thread.start()

    def tearDown(self):
        self.app._running = False
        self.server.shutdown()
        self.server.close_clients()
        self.server.server_close()
        self.thread.join()
        self.app.bridge.stop()
        self.tmp.cleanup()

    def request(self, method, path, body=None, headers=None, raw=None, authenticate=True):
        client = http.client.HTTPConnection(*self.server.server_address, timeout=3)
        options = {"Content-Type": "application/json"}
        if authenticate:
            options["Authorization"] = "Basic " + base64.b64encode(b"finav:offline-test-password").decode()
        options.update(headers or {})
        client.request(method, path, body=raw if raw is not None else json.dumps(body) if body is not None else None,
                       headers=options)
        response = client.getresponse()
        data = response.read()
        status = response.status
        client.close()
        return status, json.loads(data) if data else None

    def test_authentication_required_for_reads_and_writes(self):
        for method, path in (("GET", "/api/status"), ("POST", "/api/nav/cancel"), ("PUT", "/api/maps/test/editor")):
            self.assertEqual(self.request(method, path, {}, authenticate=False)[0], 401)
        self.assertFalse(authorized("Basic ###", "anything"))
        self.assertEqual(self.request("GET", "/api/health")[0], 200)

    def test_authentication_disabled_by_default(self):
        with patch.dict("os.environ", {"FINAV_WEB_PASSWORD": "configured-but-disabled"}, clear=True), \
             patch("web_support.Path.read_text") as read, patch("web_support.os.open") as create:
            self.app.access_password = access_password()
            self.assertIsNone(self.app.access_password)
            read.assert_not_called()
            create.assert_not_called()
        self.assertEqual(self.request("GET", "/api/status", authenticate=False)[0], 200)
        # Validation still runs after the disabled authentication gate.
        self.assertEqual(self.request("POST", "/api/nav/goal", {}, authenticate=False)[0], 400)
        make_map(self.app)
        self.assertEqual(self.request("PUT", "/api/maps/test/editor", {}, authenticate=False)[0], 400)

    def test_authentication_requires_explicit_opt_in(self):
        with patch.dict("os.environ", {"FINAV_WEB_AUTH": "1", "FINAV_WEB_PASSWORD": "test-opt-in"}, clear=True):
            self.assertEqual(access_password(), "test-opt-in")
        self.assertFalse(authorized(None, "test-opt-in"))

    def test_malformed_requests_never_queue_commands(self):
        for raw in ("{", "[]", '{"x":NaN}', '{"x":1e999}', "null"):
            self.assertEqual(self.request("POST", "/api/nav/goal", raw=raw)[0], 400)
        for body in ({}, {"x": 1, "y": 2}, {"x": True, "y": 2, "yaw_deg": 0}):
            self.assertEqual(self.request("POST", "/api/nav/goal", body)[0], 400)
        self.assertTrue(self.app.bridge._cmd_q.empty())

    def test_bad_query_parameters(self):
        for path in ("/api/history?since=no", "/api/history?limit=-1", "/api/scene?map_version=x",
                     "/api/runtime/logs/mapping?tail=999999", "/api/stream?plan_version=x"):
            self.assertEqual(self.request("GET", path)[0], 400)

    def test_body_limits_and_origin(self):
        self.assertEqual(self.request("POST", "/api/nav/cancel", {}, headers={"Origin": "http://other.example"})[0], 403)
        self.assertEqual(self.request("POST", "/api/nav/cancel", {}, headers={"Content-Type": "text/plain"})[0], 415)
        self.assertEqual(self.request("POST", "/api/nav/cancel", raw="", headers={"Content-Length": str(5 * 1024 * 1024)})[0], 413)

    def test_map_root_and_symlink_cannot_be_deleted(self):
        make_map(self.app)
        outside = self.root / "maps-backup"
        outside.mkdir()
        (self.app.maps_dir / "escape").symlink_to(outside, target_is_directory=True)
        for name in (".", "%2E", "..", "escape", "%2Ftmp"):
            self.assertEqual(self.request("POST", f"/api/maps/{name}/delete", {})[0], 400)
        self.assertTrue(self.app.maps_dir.exists())
        self.assertTrue(outside.exists())

    def test_active_map_not_deleted(self):
        target = make_map(self.app)
        self.app.state.update_status({"runtime": {"navigation": {"running": True}}})
        self.assertEqual(self.request("POST", "/api/maps/test/delete", {})[0], 409)
        self.assertTrue(target.exists())
        self.app.state.update_status({"runtime": {"navigation": {"running": False}}})
        self.assertEqual(self.request("POST", "/api/maps/test/delete", {})[0], 200)
        self.assertFalse(target.exists())

    def test_static_sibling_prefix_is_rejected(self):
        self.app.web_dir = self.root / "web"
        self.app.web_dir.mkdir()
        sibling = self.root / "web-backup"
        sibling.mkdir()
        (sibling / "data.txt").write_text("private")
        self.assertEqual(self.request("GET", "/../web-backup/data.txt")[0], 404)

    def test_config_validation_and_revision(self):
        path = self.app.config_dir / "nav.yaml"
        original = "node:\n  ros__parameters:\n    speed: 1\n"
        path.write_text(original)
        _, loaded = self.request("GET", "/api/configs/nav.yaml")
        for content in ("", "[]", "node: [", "node:\n  ros__parameters: 7\n"):
            self.assertEqual(self.request("POST", "/api/configs/nav.yaml",
                {"content": content, "expected_revision": loaded["revision"]})[0], 422)
            self.assertEqual(path.read_text(), original)
        self.assertEqual(self.request("POST", "/api/configs/nav.yaml", {})[0], 400)
        new = "node:\n  ros__parameters:\n    speed: 2\n"
        self.assertEqual(self.request("POST", "/api/configs/nav.yaml",
            {"content": new, "expected_revision": loaded["revision"]})[0], 200)
        self.assertEqual(self.request("POST", "/api/configs/nav.yaml",
            {"content": original, "expected_revision": loaded["revision"]})[0], 409)
        self.assertEqual(path.read_text(), new)

    def test_config_atomic_failure_preserves_original(self):
        path = self.app.config_dir / "nav.yaml"
        path.write_text("node: 1\n")
        revision = self.app.config_store.read(path)["revision"]
        with patch("web_support.os.replace", side_effect=OSError("disk failure")):
            self.assertEqual(self.request("POST", "/api/configs/nav.yaml",
                {"content": "node: 2\n", "expected_revision": revision})[0], 503)
        self.assertEqual(path.read_text(), "node: 1\n")
        self.assertEqual(len(list(path.parent.iterdir())), 1)

    def test_offline_and_full_queue_report_failure(self):
        goal = {"x": 1, "y": 2, "yaw_deg": 0}
        self.assertEqual(self.request("POST", "/api/nav/goal", goal)[0], 503)
        self.app.state.update_status({"ros": {"connected": True}})
        for _ in range(200):
            self.app.bridge.command({"type": "set_goal", **goal})
        self.assertEqual(self.request("POST", "/api/nav/goal", goal)[0], 503)
        self.assertEqual(self.request("POST", "/api/nav/cancel", {})[0], 200)
        self.assertEqual(self.app.bridge.pending_commands()[0]["type"], "cancel_nav")
        self.assertTrue(self.app.bridge._cmd_q.empty())

    def test_manual_latest_value_and_stop_priority(self):
        bridge = self.app.bridge
        self.app.state.update_status({"ros": {"connected": True}})
        for value in (0.1, 0.2, 0.3):
            bridge.command({"type": "manual_drive", "linear": value, "angular": 0})
        self.assertEqual([item["linear"] for item in bridge.pending_commands()], [0.3])
        bridge.command({"type": "cancel_nav"})
        bridge.command({"type": "manual_drive", "linear": 0, "angular": 0})
        self.assertEqual(bridge.pending_commands()[0]["type"], "cancel_nav")

    def test_expired_manual_command_not_published(self):
        self.app.state.update_status({"ros": {"connected": True}})
        bridge = self.app.bridge
        bridge.command({"type": "manual_drive", "linear": 1, "angular": 0})
        node = SimpleNamespace(bridge=bridge, state_store=self.app.state)
        with patch("ros_bridge.time.monotonic", return_value=time.monotonic() + 1), patch.object(_BridgeNode, "_handle_command") as handler:
            _BridgeNode._process_commands(node)
            handler.assert_not_called()

    def test_history_restarts_and_pages_without_skipping(self):
        state = self.app.state
        for index in range(5):
            state.add_event("info", str(index))
        status, first = self.request("GET", "/api/history?since=999&limit=2&session_id=old")
        self.assertEqual(status, 200)
        self.assertTrue(first["reset"])
        self.assertEqual([item["seq"] for item in first["events"]], [1, 2])
        _, second = self.request("GET", f"/api/history?since=2&limit=2&session_id={state.session_id}")
        self.assertEqual([item["seq"] for item in second["events"]], [3, 4])
        self.assertFalse(second["reset"])

    def test_readiness_does_not_require_plan(self):
        self.app.state.update_status({"ros": {"connected": True, "last_seen": {
            "scan": 0, "odom": 0, "tf_odom_base_link": 0, "tf_map_odom": 0, "map": 50}},
            "runtime": {"navigation": {"running": True, "started_at": time.time() - 60}}})
        self.assertEqual(self.app.state.snapshot()["runtime"]["navigation"]["readiness"]["state"], "ready")
        self.app.state.update_status({"ros": {"last_seen": {"scan": 9}}})
        self.assertEqual(self.app.state.snapshot()["runtime"]["navigation"]["readiness"]["state"], "not_ready")

    def test_log_unchanged_response(self):
        (self.root / "mapping.log").write_text("one\n")
        _, first = self.request("GET", "/api/runtime/logs/mapping")
        _, second = self.request("GET", "/api/runtime/logs/mapping?version=" + first["version"])
        self.assertTrue(second["unchanged"])
        self.assertNotIn("lines", second)
        (self.root / "mapping.log").write_text("two\n")
        _, third = self.request("GET", "/api/runtime/logs/mapping?version=" + first["version"])
        self.assertEqual(third["lines"], ["two"])

    def test_locations_have_one_atomic_source(self):
        directory = make_map(self.app)
        legacy = directory / "test.locations.yaml"
        legacy.write_text("locations:\n  old:\n    x: 5\n    y: 5\n")
        _, loaded = self.request("GET", "/api/maps/test/locations")
        self.assertEqual(loaded["locations"][0]["name"], "old")
        body = {"expected_revision": loaded["revision"], "locations": [{"name": "new", "x": 6, "y": 6, "yaw_deg": 0}]}
        status, result = self.request("POST", "/api/maps/test/locations", body)
        self.assertEqual(status, 200, result)
        self.assertEqual(list(load_locations(legacy)), ["new"])
        _, current = self.request("GET", "/api/maps/test/locations")
        self.assertEqual(current["locations"][0]["name"], "new")
        self.assertEqual(self.request("POST", "/api/maps/test/locations", body)[0], 409)
        with patch("editor_store.os.replace", side_effect=OSError("disk failure")):
            body["expected_revision"] = current["revision"]
            body["locations"] = []
            self.assertEqual(self.request("POST", "/api/maps/test/locations", body)[0], 503)
        self.assertEqual(list(load_locations(legacy)), ["new"])

    def test_binary_map_pixel_whitespace_is_preserved(self):
        directory = make_map(self.app)
        (directory / "test.pgm").write_bytes(b"P5\n20 20\n255\n" + b"\x0a" + b"\xff" * 399)
        data = load_map_preview(self.app.maps_dir, "test")
        self.assertEqual(len(data["data"]), 400)
        self.assertEqual(data["data"][0], 100)

    def test_map_preview_preserves_unknown_gray(self):
        directory = make_map(self.app)
        image = directory / "test.pgm"
        image.write_bytes(b"P5\n20 20\n255\n" + bytes([0, 205, 254]) + b"\xcd" * 397)
        data = load_map_preview(self.app.maps_dir, "test")
        self.assertEqual(data["data"][:3], [100, -1, 0])
        self.assertEqual(data["data"].count(-1), 398)
        metadata = directory / "test.yaml"
        metadata.write_text(metadata.read_text().replace("negate: 0", "negate: 1"))
        image.write_bytes(b"P5\n20 20\n255\n" + bytes([255, 50, 1]) + b"\x32" * 397)
        self.assertEqual(load_map_preview(self.app.maps_dir, "test")["data"], data["data"])

    def test_stream_limit(self):
        for _ in range(8):
            self.assertTrue(self.server.stream_slots.acquire(blocking=False))
        try:
            self.assertEqual(self.request("GET", "/api/stream")[0], 503)
        finally:
            for _ in range(8):
                self.server.stream_slots.release()

    def test_stream_flushes_events_and_preserves_map_versions(self):
        self.app.state.update_scene({"map": {"width": 400, "data": [0] * 160000}}, map_changed=True)
        for encoding in ("gzip, deflate", "identity", "gzip;q=0"):
            with self.subTest(encoding=encoding):
                client = http.client.HTTPConnection(*self.server.server_address, timeout=3)
                client.request("GET", "/api/stream?mode=mapping", headers={
                    "Accept-Encoding": encoding,
                    "Authorization": "Basic " + base64.b64encode(b"finav:offline-test-password").decode(),
                })
                response = client.getresponse()
                compressed = encoding == "gzip, deflate"
                self.assertEqual(response.getheader("Content-Encoding"), "gzip" if compressed else None)
                decoder = zlib.decompressobj(31) if compressed else None
                received = b""
                wire_bytes = 0
                try:
                    while received.count(b"data: ") < 2 or not received.endswith(b"\n\n"):
                        chunk = response.read1(65536)
                        self.assertTrue(chunk)
                        wire_bytes += len(chunk)
                        received += decoder.decompress(chunk) if decoder else chunk
                    events = [json.loads(part[6:]) for part in received.split(b"\n\n") if part.startswith(b"data: ")]
                    self.assertEqual(events[0]["map"]["width"], 400)
                    self.assertIsNone(events[1]["map"])
                    self.assertFalse(events[1]["map_changed"])
                    if compressed:
                        self.assertLess(wire_bytes, len(received) // 10)
                finally:
                    response.close()
                    client.close()

    def test_config_concurrent_writers(self):
        path = self.app.config_dir / "nav.yaml"
        path.write_text("node: 1\n")
        revision = self.app.config_store.read(path)["revision"]
        results = []
        barrier = threading.Barrier(2)
        def save(number):
            barrier.wait()
            results.append(self.request("POST", "/api/configs/nav.yaml",
                {"content": f"node: {number}\n", "expected_revision": revision})[0])
        threads = [threading.Thread(target=save, args=(number,)) for number in (2, 3)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(sorted(results), [200, 409])

    def test_save_map_job_success_and_failure(self):
        bridge = self.app.bridge
        self.app.state.update_status({"ros": {"connected": True}})
        for code, expected in ((0, "succeeded"), (1, "failed")):
            process = MagicMock(returncode=code)
            process.poll.return_value = code
            with patch("ros_bridge.subprocess.Popen", return_value=process):
                job = bridge.save_map("test")
                bridge._save_thread.join(timeout=2)
            result = self.app.state.snapshot()["map_save"]
            self.assertEqual(result["id"], job["id"])
            self.assertEqual(result["state"], expected)
            self.assertIsNotNone(result["finished_at"])

    def test_save_map_waits_for_annotations_and_reports_failure(self):
        bridge = self.app.bridge
        self.app.state.update_status({"ros": {"connected": True}})
        def inherit():
            self.assertEqual(self.app.state.snapshot()["map_save"]["state"], "running")
            raise RuntimeError("annotation write failed")
        for code in (0, 1):
            process = MagicMock(returncode=code)
            process.poll.return_value = code
            callback = MagicMock(side_effect=inherit)
            with patch("ros_bridge.subprocess.Popen", return_value=process):
                bridge.save_map("test", on_saved=callback)
                bridge._save_thread.join(timeout=2)
            result = self.app.state.snapshot()["map_save"]
            self.assertEqual(result["state"], "failed")
            if code == 0:
                callback.assert_called_once_with()
                self.assertIn("annotation write failed", result["error"])
            else:
                callback.assert_not_called()

    def test_save_map_shutdown_kills_only_its_process_group(self):
        bridge = self.app.bridge
        process = MagicMock(pid=123456, returncode=None)
        process.poll.return_value = None
        bridge._closed.set()
        bridge._save_lock.acquire()
        with patch("ros_bridge.subprocess.Popen", return_value=process), patch("ros_bridge.os.killpg") as kill:
            bridge._save_map_job({"name": "test", "id": "job"})
        self.assertEqual(self.app.state.snapshot()["map_save"]["state"], "failed")
        kill.assert_called_once()
        process.wait.assert_called_once()
        self.assertTrue(bridge._save_lock.acquire(blocking=False))
        bridge._save_lock.release()

    def test_ros_cleanup_without_starting_ros(self):
        bridge = self.app.bridge
        bridge._closed.set()
        context = MagicMock()
        executor = MagicMock()
        node = MagicMock()
        with patch("rclpy.init"), patch("rclpy.context.Context", return_value=context), \
             patch("rclpy.executors.MultiThreadedExecutor", return_value=executor), \
             patch("ros_bridge._BridgeNode", return_value=node) as factory:
            bridge._run()
        executor.shutdown.assert_called_once()
        node.destroy_node.assert_called_once()
        context.shutdown.assert_called_once()
        factory._publish_web_stop.assert_called_once_with(node)
        self.assertFalse(self.app.state.snapshot()["ros"]["connected"])

    def test_history_retention_reports_gap(self):
        state = StateStore(max_history=2)
        for _ in range(4):
            state.add_event("info", "event")
        result = state.history_page(1, 100, state.session_id)
        self.assertTrue(result["truncated"])
        self.assertEqual([item["seq"] for item in result["events"]], [3, 4])

    def test_connection_limit_timeout_and_cleanup(self):
        self.server._slots = threading.BoundedSemaphore(1)
        first = socket.create_connection(self.server.server_address, timeout=2)
        try:
            deadline = time.monotonic() + 1
            while not self.server._clients and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(len(self.server._clients), 1)
            self.assertEqual(next(iter(self.server._clients)).gettimeout(), 10.0)
            second = socket.create_connection(self.server.server_address, timeout=2)
            try:
                self.assertEqual(second.recv(1), b"")
            finally:
                second.close()
            self.server.close_clients()
            self.assertEqual(first.recv(1), b"")
        finally:
            first.close()

    def test_scene_reset_clears_planning_overlay(self):
        self.app.state.update_scene({"map": {"width": 1}, "planning_map": {"width": 1}},
                                    map_changed=True, planning_map_changed=True)
        manager = RuntimeManager.__new__(RuntimeManager)
        manager.state_store = self.app.state
        manager._clear_live_scene()
        scene = self.app.state.snapshot_scene(1, 1)
        self.assertTrue(scene["planning_map_changed"])
        self.assertIsNone(scene["planning_map"])

    def test_map_save_timeout(self):
        bridge = self.app.bridge
        process = MagicMock(pid=123456, returncode=None)
        process.poll.return_value = None
        bridge._save_lock.acquire()
        with patch("ros_bridge.subprocess.Popen", return_value=process), \
             patch("ros_bridge.time.monotonic", side_effect=[0.0, 121.0]), \
             patch("ros_bridge.os.killpg") as kill:
            bridge._save_map_job({"name": "test", "id": "timeout"})
        self.assertIn("timed out", self.app.state.snapshot()["map_save"]["error"])
        kill.assert_called_once()


def preview(port):
    """Serve fixture data only; never construct ROS nodes or run services."""
    with tempfile.TemporaryDirectory(prefix="finav-web-preview-") as directory:
        app = isolated_app(Path(directory))
        make_map(app)
        (app.config_dir / "nav.yaml").write_text("nav_control:\n  ros__parameters:\n    linear_speed_mps: 0.3\n    angular_speed_degps: 20.0\n    drive_angular_speed_degps: 25.0\n")
        def unavailable(*args, **kwargs):
            raise RequestError(503, "Offline preview: hardware operations are disabled")
        app.runtime.start = app.runtime.stop = app.runtime.restart = unavailable
        app.runtime.run_relocate = app.runtime.request_finav_action = unavailable
        app.state.add_event("info", "Offline preview")
        server = LimitedHTTPServer(("127.0.0.1", port), app._build_handler())
        try:
            server.serve_forever()
        finally:
            app._running = False
            server.close_clients()
            server.server_close()


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--preview":
        preview(int(sys.argv[2]))
    else:
        unittest.main()
