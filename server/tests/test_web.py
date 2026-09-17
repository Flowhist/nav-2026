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
        (app.config_dir / "nav.yaml").write_text("nav_control:\n  ros__parameters:\n    max_speed: 0.3\n")
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
