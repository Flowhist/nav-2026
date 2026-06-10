import importlib.util
import subprocess
from pathlib import Path


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_scan_xy_to_base_xy_identity_for_fused_base_link_scan():
    module = _load_module(
        Path(__file__).resolve().parents[1] / "scripts" / "control" / "nav_relocate.py",
        "nav_relocate_under_test",
    )
    pts = module.scan_xy_to_base_xy(
        [[1.0, 0.0], [0.0, 1.0]],
        scan_to_base_x_m=0.0,
        scan_to_base_y_m=0.0,
        scan_to_base_yaw_rad=0.0,
    )
    assert pts.tolist() == [[1.0, 0.0], [0.0, 1.0]]


def test_detect_running_map_file_can_ignore_locations_file(tmp_path, monkeypatch):
    maps_dir = tmp_path / "maps"
    map_dir = maps_dir / "longcorridor"
    map_dir.mkdir(parents=True)
    (map_dir / "longcorridor.yaml").write_text("image: longcorridor.pgm\n", encoding="utf-8")
    (map_dir / "longcorridor.pgm").write_text("P2\n1 1\n255\n0\n", encoding="utf-8")

    def fake_run(cmd, check, stdout, stderr, text, timeout):
        if cmd == ["ros2", "param", "get", "/slam_toolbox", "map_file_name"]:
            return subprocess.CompletedProcess(cmd, 0, stdout="String value is: longcorridor\n")
        return subprocess.CompletedProcess(cmd, 1, stdout="")

    monkeypatch.setattr(subprocess, "run", fake_run)
    map_utils = _load_module(
        Path(__file__).resolve().parents[1] / "scripts" / "map_annotate" / "map_utils.py",
        "map_utils_under_test",
    )

    assert map_utils.detect_running_map_file(str(maps_dir), require_locations=False) == "longcorridor"
