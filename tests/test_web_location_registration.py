import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "server"))

from map_utils import load_map_locations, save_map_locations  # noqa: E402
from server_app import ServerApp  # noqa: E402
from editor_server import EditorApplication  # noqa: E402


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


def test_map_page_owns_editor_ui_and_legacy_entry_is_removed():
    html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert 'id="btnEditPreviewMap"' in html
    assert 'id="previewEditorSidebar"' in html
    assert 'id="editorPropertyInspector"' in html
    assert "iframe" not in html
    assert not (ROOT / "scripts" / "map_location" / "map_editor.py").exists()
    assert "scripts/map_location/map_editor.py" not in cmake
    assert "scripts/map_location/editor_web" not in cmake
