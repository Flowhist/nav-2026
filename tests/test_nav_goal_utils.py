import importlib.util
import math
import sys
from pathlib import Path

from builtin_interfaces.msg import Time


def _load_module(relative_path: str, name: str):
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / "scripts" / "map_annotate"))
    path = root / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class DummyClock:
    class _Now:
        @staticmethod
        def to_msg():
            return Time()

    @staticmethod
    def now():
        return DummyClock._Now()


def test_make_map_goal_sets_position_frame_and_yaw():
    module = _load_module(
        "scripts/control/nav_voice_bridge.py",
        "nav_voice_bridge_goal_under_test",
    )

    msg = module.make_map_goal(DummyClock(), 1.2, -0.4, math.pi / 2.0)

    assert msg.header.frame_id == "map"
    assert msg.pose.position.x == 1.2
    assert msg.pose.position.y == -0.4
    assert msg.pose.position.z == 0.0
    assert msg.pose.orientation.x == 0.0
    assert msg.pose.orientation.y == 0.0
    assert math.isclose(msg.pose.orientation.z, math.sqrt(0.5), rel_tol=1e-9)
    assert math.isclose(msg.pose.orientation.w, math.sqrt(0.5), rel_tol=1e-9)


def test_relocate_set_yaw_orientation_sets_quaternion():
    module = _load_module(
        "scripts/control/nav_relocate.py",
        "nav_relocate_goal_under_test",
    )
    orientation = type("Orientation", (), {})()

    module.set_yaw_orientation(orientation, -math.pi / 2.0)

    assert orientation.x == 0.0
    assert orientation.y == 0.0
    assert math.isclose(orientation.z, -math.sqrt(0.5), rel_tol=1e-9)
    assert math.isclose(orientation.w, math.sqrt(0.5), rel_tol=1e-9)


def test_navigation_entrypoints_do_not_import_nav_goal_utils():
    root = Path(__file__).resolve().parents[1]
    for relative_path in (
        "scripts/control/nav_voice_bridge.py",
        "scripts/control/nav_relocate.py",
    ):
        source = (root / relative_path).read_text(encoding="utf-8")
        assert "nav_goal_utils" not in source
