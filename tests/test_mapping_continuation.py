import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "server"))

from map_utils import list_saved_maps  # noqa: E402


def _write_map(root: Path, name: str, *, posegraph: bool):
    target = root / name
    target.mkdir()
    (target / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    (target / f"{name}.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    if posegraph:
        (target / f"{name}.posegraph").write_bytes(b"posegraph")
        (target / f"{name}.data").write_bytes(b"data")


def test_saved_map_list_marks_posegraph_maps_as_continuable(tmp_path):
    _write_map(tmp_path, "complete", posegraph=True)
    _write_map(tmp_path, "image_only", posegraph=False)

    maps = {item["name"]: item for item in list_saved_maps(tmp_path)}

    assert maps["complete"]["continuable"] is True
    assert maps["image_only"]["continuable"] is False


def test_mapping_page_contains_continue_flow_and_compact_controls():
    html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "server" / "web" / "styles.css").read_text(encoding="utf-8")
    app_js = (ROOT / "server" / "web" / "app.js").read_text(encoding="utf-8")

    assert 'id="mappingMapToggle"' in html
    assert 'id="btnStartContinueMapping"' in html
    assert 'id="btnMappingAutoRelocate"' in html
    assert 'id="btnMappingManualRelocate"' in html
    assert 'id="mapName"' not in html
    assert "showMapSaveDialog" in app_js
    assert ".teleop-panel .dpad" in css


def test_navigation_buttons_use_requested_names_and_order():
    html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")

    labels = [
        "自动重定位",
        "手动重定位",
        "手动设置目的地",
        "停止导航",
    ]
    positions = [html.index(label) for label in labels]
    assert positions == sorted(positions)


def test_continued_mapping_uses_localize_then_mapping_node():
    map_launch = (ROOT / "launch" / "map.launch.py").read_text(encoding="utf-8")
    slam_launch = (
        ROOT / "launch" / "sub" / "slam_toolbox.launch.py"
    ).read_text(encoding="utf-8")
    manager = (ROOT / "server" / "process_manager.py").read_text(encoding="utf-8")
    relocate = (
        ROOT / "scripts" / "control" / "nav_relocate.py"
    ).read_text(encoding="utf-8")

    assert '"map_file": map_file' in map_launch
    assert 'executable="map_and_localization_slam_toolbox_node"' in slam_launch
    server_app = (ROOT / "server" / "server_app.py").read_text(encoding="utf-8")
    pages = (ROOT / "server" / "web" / "app-pages.js").read_text(encoding="utf-8")

    assert "set_continued_mapping_localization(True)" in manager
    assert "activate_continued_mapping" in manager
    assert "set_continued_mapping_localization(True)" in server_app
    assert "!navCommandsEnabled && !continuedMapping" in pages
    assert '"nav_relocate.py": "nav_relocate.yaml"' in manager
    assert "raise SystemExit(2)" in relocate
