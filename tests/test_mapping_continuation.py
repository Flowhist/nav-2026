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


def test_mapping_page_removes_continue_flow_but_keeps_core_controls():
    html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    css = (ROOT / "server" / "web" / "styles.css").read_text(encoding="utf-8")
    app_js = (ROOT / "server" / "web" / "app.js").read_text(encoding="utf-8")
    pages_js = (ROOT / "server" / "web" / "app-pages.js").read_text(encoding="utf-8")
    core_js = (ROOT / "server" / "web" / "app-core.js").read_text(encoding="utf-8")

    assert "继续建图" not in html
    assert "mappingMap" not in html
    assert "btnStartContinueMapping" not in html
    assert "btnMappingAutoRelocate" not in html
    assert "btnMappingManualRelocate" not in html
    assert "resume_mapping" not in app_js
    assert "mappingMapName" not in core_js
    assert "renderMappingMapPicker" not in pages_js
    assert 'id="btnStartMapping"' in html
    assert 'id="btnSaveMap"' in html
    assert 'id="btnStopMapping"' in html
    assert 'id="mapName"' not in html
    assert "showMapSaveDialog" in app_js
    assert ".manual-stick" in css
    assert ".teleop-panel .dpad" not in css


def test_navigation_buttons_use_current_names_and_order():
    html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")

    labels = [
        "自动定位",
        "设置机器人位置",
        "设置目标点",
        "取消当前任务",
        "关闭导航服务",
    ]
    positions = [html.index(label) for label in labels]
    assert positions == sorted(positions)


def test_web_runtime_no_longer_exposes_continue_mapping_entrypoint():
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

    assert "resume_mapping" not in server_app
    assert "resume_mapping" not in pages
    assert "map cannot be continued" not in server_app
    assert '"map_file"] = name' not in server_app
    assert '"nav_relocate.py": "nav_relocate.yaml"' in manager
    assert "raise SystemExit(2)" in relocate
