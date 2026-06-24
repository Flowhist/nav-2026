import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load_server_app():
    server_path = str(ROOT / "server")
    sys.path.insert(0, server_path)
    path = ROOT / "server" / "server_app.py"
    spec = importlib.util.spec_from_file_location("server_app_config_workflow_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    try:
        spec.loader.exec_module(module)
    finally:
        if sys.path and sys.path[0] == server_path:
            sys.path.pop(0)
        sys.modules.pop("map_utils", None)
    return module


def _load_process_manager():
    server_path = str(ROOT / "server")
    sys.path.insert(0, server_path)
    path = ROOT / "server" / "process_manager.py"
    spec = importlib.util.spec_from_file_location("process_manager_config_workflow_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    try:
        spec.loader.exec_module(module)
    finally:
        if sys.path and sys.path[0] == server_path:
            sys.path.pop(0)
    return module


class _DummyStateStore:
    def update_status(self, _payload):
        pass

    def add_event(self, *_args, **_kwargs):
        pass


def test_key_launch_files_prefer_finav_repo_dir_for_project_configs():
    launch_files = [
        "launch/map.launch.py",
        "launch/nav.launch.py",
        "launch/sub/slam_toolbox.launch.py",
        "launch/sub/joy.launch.py",
        "launch/sub/lidar.launch.py",
        "launch/sub/dm_imu.launch.py",
        "launch/sub/ekf.launch.py",
        "sim/launch/nav_sim.launch.py",
        "sim/launch/path_plan_sim.launch.py",
    ]

    for rel in launch_files:
        source = (ROOT / rel).read_text(encoding="utf-8")
        assert "FINAV_REPO_DIR" in source, rel
        assert "config_dir" in source or "_config_path" in source, rel


def test_config_impact_metadata_lists_restart_targets():
    module = _load_server_app()

    base = module.describe_config_impact("base_control.yaml")
    assert base["requires_restart"] is True
    assert base["restart_targets"] == [{"mode": "base_drive", "label": "底盘驱动"}]

    joy = module.describe_config_impact("joy.yaml")
    assert joy["requires_restart"] is True
    assert joy["restart_targets"] == [{"mode": "base_drive", "label": "底盘驱动"}]

    slam_nav = module.describe_config_impact("slam_toolbox_nav.yaml")
    assert slam_nav["requires_restart"] is True
    assert slam_nav["restart_targets"] == []
    assert "不需要" in slam_nav["message"]
    assert "colcon" in slam_nav["message"].lower()

    all_targets = [
        item["mode"]
        for name in module.CONFIG_IMPACTS
        for item in module.describe_config_impact(name)["restart_targets"]
    ]
    assert "mapping" not in all_targets
    assert "navigation" not in all_targets

    unknown = module.describe_config_impact("other.yaml")
    assert unknown["requires_restart"] is True
    assert unknown["restart_targets"] == []


def test_web_runtime_commands_use_source_launch_and_source_config():
    module = _load_process_manager()
    runtime = module.RuntimeManager(ROOT, _DummyStateStore())

    launch_cmd = runtime._build_launch_command("nav.launch.py", {"map_file": "map4"})
    assert "FINAV_REPO_DIR" in launch_cmd
    assert "ros2 launch finav nav.launch.py" in launch_cmd

    base_cmd = runtime._build_run_command("base_control.py")
    assert str(ROOT / "config" / "base_control.yaml") in base_cmd
    assert "--params-file" in base_cmd

    base_drive_cmd = runtime._build_base_drive_command()
    assert "FINAV_REPO_DIR" in base_drive_cmd
    assert "bash" in base_drive_cmd
    assert str(ROOT / "base_drive.sh") in base_drive_cmd


def test_config_overview_exposes_restart_actions_outside_editor():
    index_html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    app_pages = (ROOT / "server" / "web" / "app-pages.js").read_text(encoding="utf-8")
    styles = (ROOT / "server" / "web" / "styles.css").read_text(encoding="utf-8")

    assert 'id="configApplyPanel"' in index_html
    assert 'id="configRestartActions"' in index_html
    overview = index_html.split('id="configOverview"', 1)[1].split('id="configEditorView"', 1)[0]
    editor = index_html.split('id="configEditorView"', 1)[1]
    assert 'id="configApplyPanel"' in overview
    assert 'id="configApplyPanel"' not in editor
    assert overview.index('id="configCards"') < overview.index('id="configApplyPanel"')
    assert "renderConfigRestartActions" in app_pages
    assert "renderConfigApplyPanel" not in app_pages
    assert "restartRuntimeTarget" in app_pages
    assert 'btn.className = "danger"' in app_pages
    assert "justify-content: center;" in styles.split(".config-restart-actions", 1)[1].split("}", 1)[0]
    assert "/api/runtime/" in app_pages and "/restart" in app_pages
