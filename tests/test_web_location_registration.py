import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "server"))

from map_utils import load_map_locations, save_map_locations  # noqa: E402


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
