import copy
import importlib.util
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "server"))

from map_utils import load_map_locations, save_map_locations  # noqa: E402
from server_app import ServerApp  # noqa: E402
from editor_server import EditorApplication  # noqa: E402
from editor_store import DocumentError  # noqa: E402


def _load_runtime_location_utils():
    module_path = ROOT / "scripts" / "map_location" / "location_utils.py"
    spec = importlib.util.spec_from_file_location("runtime_location_utils", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_web_location_registration_round_trips_without_desktop_tool(tmp_path):
    maps_dir = tmp_path / "maps"

    save_map_locations(
        maps_dir,
        "ward",
        [
            {
                "name": "bed",
                "x": 1.2345,
                "y": -2.3456,
                "yaw_deg": 89.96,
            }
        ],
    )

    expected_web_locations = [
        {
            "name": "bed",
            "x": 1.234,
            "y": -2.346,
            "yaw_deg": 90.0,
        }
    ]
    assert load_map_locations(maps_dir, "ward") == expected_web_locations

    runtime_location_utils = _load_runtime_location_utils()
    assert runtime_location_utils.load_locations(
        maps_dir / "ward" / "ward.locations.yaml"
    ) == {
        "bed": {
            "x": 1.234,
            "y": -2.346,
            "yaw_deg": 90.0,
        }
    }


def test_runtime_location_modules_use_current_names():
    assert not (ROOT / "scripts" / "map_annotate").exists()
    assert (
        ROOT / "scripts" / "map_location" / "location_visualizer.py"
    ).is_file()

    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "scripts/map_location/location_utils.py" in cmake
    assert "scripts/map_location/location_visualizer.py" in cmake
    assert "scripts/map_annotate" not in cmake


def _write_editor_map(root: Path, name: str = "ward") -> None:
    map_dir = root / name
    map_dir.mkdir(parents=True)
    (map_dir / f"{name}.pgm").write_bytes(b"P5\n20 20\n255\n" + bytes([255]) * 400)
    (map_dir / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    (map_dir / f"{name}.locations.yaml").write_text(
        "locations:\n  entrance:\n    x: 0.500\n    y: 0.500\n    yaw_deg: 0.0\n",
        encoding="utf-8",
    )


def test_integrated_editor_bootstraps_and_syncs_navigation_locations(tmp_path):
    maps_dir = tmp_path / "maps"
    _write_editor_map(maps_dir)
    planner_config = tmp_path / "path_plan.yaml"
    planner_config.write_text(
        "path_plan:\n  ros__parameters:\n    inflation_radius_m: 0.1\n",
        encoding="utf-8",
    )
    app = ServerApp(host="127.0.0.1", port=0)
    app.maps_dir = maps_dir
    app.editor = EditorApplication(maps_dir, planner_config)

    payload = app._load_editor_document("ward")
    document = payload["document"]
    assert "map" not in payload
    assert [item["name"] for item in document["locations"]] == ["entrance"]

    document["locations"][0]["name"] = "lobby"
    document["routes"] = [
        {
            "id": "route-1",
            "name": "daily",
            "closed": False,
            "waypoints": [
                {"id": "point-1", "x": 0.5, "y": 0.5},
                {"id": "point-2", "x": 1.4, "y": 0.5},
            ],
        }
    ]
    saved = app._save_editor_document(
        "ward",
        {"expected_revision": 0, "document": document},
    )["document"]

    assert saved["revision"] == 1
    assert load_map_locations(maps_dir, "ward") == [
        {"name": "lobby", "x": 0.5, "y": 0.5, "yaw_deg": 0.0}
    ]
    preview = app.editor.smooth("ward", "route-1", {"expected_revision": 1})
    assert preview["preview"]["status"] == "valid"


def test_invalid_smooth_preview_keeps_points_and_marks_conflict_segments(tmp_path):
    maps_dir = tmp_path / "maps"
    _write_editor_map(maps_dir)
    pixels = bytearray([255] * 400)
    for row in range(20):
        pixels[row * 20 + 10] = 0
    (maps_dir / "ward" / "ward.pgm").write_bytes(
        b"P5\n20 20\n255\n" + bytes(pixels)
    )
    planner_config = tmp_path / "path_plan.yaml"
    planner_config.write_text(
        "path_plan:\n  ros__parameters:\n    inflation_radius_m: 0.1\n",
        encoding="utf-8",
    )
    app = ServerApp(host="127.0.0.1", port=0)
    app.maps_dir = maps_dir
    app.editor = EditorApplication(maps_dir, planner_config)

    document = app._load_editor_document("ward")["document"]
    document["routes"] = [{
        "id": "route-conflict",
        "name": "conflict",
        "closed": False,
        "waypoints": [
            {"id": "point-a", "x": 0.5, "y": 0.5},
            {"id": "point-b", "x": 1.5, "y": 0.5},
        ],
    }]
    saved = app._save_editor_document(
        "ward", {"expected_revision": 0, "document": document}
    )["document"]
    preview = app.editor.smooth(
        "ward", "route-conflict", {"expected_revision": saved["revision"]}
    )["preview"]

    assert preview["status"] == "invalid"
    assert len(preview["points"]) > 2
    assert preview["collisions"]
    assert all(
        0 <= item["segment_index"] < len(preview["points"]) - 1
        for item in preview["collisions"]
    )


def test_legacy_invalid_location_does_not_block_unrelated_editor_saves(tmp_path):
    maps_dir = tmp_path / "maps"
    _write_editor_map(maps_dir)
    (maps_dir / "ward" / "ward.locations.yaml").write_text(
        "locations:\n  legacy:\n    x: 0.0\n    y: 0.0\n    yaw_deg: 0.0\n",
        encoding="utf-8",
    )
    planner_config = tmp_path / "path_plan.yaml"
    planner_config.write_text(
        "path_plan:\n  ros__parameters:\n    inflation_radius_m: 0.1\n",
        encoding="utf-8",
    )
    app = ServerApp(host="127.0.0.1", port=0)
    app.maps_dir = maps_dir
    app.editor = EditorApplication(maps_dir, planner_config)

    document = app._load_editor_document("ward")["document"]
    document["keepouts"].append({
        "id": "zone-1",
        "name": "zone",
        "center": {"x": 1.5, "y": 1.5},
        "width_m": 0.1,
        "height_m": 0.1,
        "yaw_deg": 0.0,
    })
    saved = app._save_editor_document(
        "ward", {"expected_revision": 0, "document": document}
    )["document"]
    assert saved["revision"] == 1
    assert [item["name"] for item in saved["keepouts"]] == ["zone"]

    moved = copy.deepcopy(saved)
    moved["locations"][0]["x"] = 0.01
    with pytest.raises(DocumentError, match="not safely traversable"):
        app._save_editor_document(
            "ward", {"expected_revision": 1, "document": moved}
        )


def test_map_page_uses_one_workspace_for_view_and_edit_modes():
    html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "server" / "web" / "styles.css").read_text(encoding="utf-8")
    editor_js = (ROOT / "server" / "web" / "map-editor.js").read_text(encoding="utf-8")
    pages_js = (ROOT / "server" / "web" / "app-pages.js").read_text(encoding="utf-8")
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert 'id="previewMapPanel"' in html
    assert 'id="previewMapToggle"' in html
    assert 'id="previewMapOptions"' in html
    assert 'id="btnMapViewMode"' in html
    assert 'id="btnMapEditMode"' in html
    assert 'id="previewObjectSidebar"' in html
    assert 'id="editorPropertyInspector"' in html
    assert "grid-template-columns: 250px minmax(0, 1fr) 290px" in css
    assert 'id="btnEditPreviewMap"' not in html
    assert 'id="btnFinishMapEdit"' not in html
    assert 'id="previewBrowseSidebar"' not in html
    assert 'id="previewEditorSidebar"' not in html
    assert 'prefix: "preview"' in editor_js
    assert editor_js.count("drawArrow(") >= 2
    assert "function createEditorId()" in editor_js
    assert "crypto.randomUUID()" not in editor_js
    assert editor_js.count("createEditorId()") >= 6
    assert "drawRoutePreview(ctx, preview, selected)" in editor_js
    assert 'ctx.strokeStyle = "#ff1010"' in editor_js
    assert "preview.collisions || []" in editor_js
    assert 'ctx.fillStyle = "#ff1010"' in editor_js
    assert "corridorWidth" in editor_js
    assert "if (rect.width < 1 || rect.height < 1) return;" in editor_js
    assert 'if (appState.page !== "preview") return;' in pages_js
    assert 'appState.page === "preview"' in pages_js
    assert "请选择一个地图对象以编辑属性" in editor_js
    assert html.index('id="btnEditorMore"') < html.index('class="map-workspace-actions"')
    assert html.index('id="previewEditActions"') < html.index('id="editorSaveState"') < html.index('id="btnMapViewMode"')
    assert "iframe" not in html
    assert 'self.send_header("Cache-Control", "no-store")' in (
        ROOT / "server" / "server_app.py"
    ).read_text(encoding="utf-8")
    assert not (ROOT / "scripts" / "map_location" / "map_editor.py").exists()
    assert "scripts/map_location/map_editor.py" not in cmake
    assert "scripts/map_location/editor_web" not in cmake
