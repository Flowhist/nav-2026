import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


server_map_utils = _load_module(ROOT / "server" / "map_utils.py", "server_map_utils_for_test")


def _load_script_map_utils():
    return _load_module(
        ROOT / "scripts" / "map_annotate" / "map_utils.py",
        "script_map_utils_for_server_test",
    )


def test_save_map_locations_uses_existing_locations_yaml_contract(tmp_path):
    maps_dir = tmp_path / "maps"
    map_dir = maps_dir / "office"
    map_dir.mkdir(parents=True)

    saved = server_map_utils.save_map_locations(
        maps_dir,
        "office",
        [
            {"name": "入口", "x": 1.23456, "y": -0.4, "yaw_deg": 91.24},
            {"name": "dock", "x": 0, "y": 2, "yaw_deg": -45},
        ],
    )

    assert saved == [
        {"name": "入口", "x": 1.235, "y": -0.4, "yaw_deg": 91.2},
        {"name": "dock", "x": 0.0, "y": 2.0, "yaw_deg": -45.0},
    ]
    script_map_utils = _load_script_map_utils()
    loaded = script_map_utils.load_locations(map_dir / "office.locations.yaml")
    assert loaded == {
        "入口": {"x": 1.235, "y": -0.4, "yaw_deg": 91.2},
        "dock": {"x": 0.0, "y": 2.0, "yaw_deg": -45.0},
    }
    assert server_map_utils.load_map_locations(maps_dir, "office") == saved


def test_save_map_locations_rejects_bad_entries(tmp_path):
    maps_dir = tmp_path / "maps"
    (maps_dir / "office").mkdir(parents=True)

    for payload in (
        [{"name": "", "x": 1.0, "y": 2.0, "yaw_deg": 0.0}],
        [{"name": "bad: name", "x": 1.0, "y": 2.0, "yaw_deg": 0.0}],
        [{"name": "spot", "x": "nan", "y": 2.0, "yaw_deg": 0.0}],
    ):
        try:
            server_map_utils.save_map_locations(maps_dir, "office", payload)
        except ValueError:
            pass
        else:
            raise AssertionError(f"payload should be rejected: {payload}")
