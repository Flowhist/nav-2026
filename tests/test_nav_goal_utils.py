import importlib.util
import math
from pathlib import Path

from builtin_interfaces.msg import Time


def _load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "control" / "nav_goal_utils.py"
    spec = importlib.util.spec_from_file_location("nav_goal_utils_under_test", path)
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
    utils = _load_module()

    msg = utils.make_map_goal(DummyClock(), 1.2, -0.4, math.pi / 2.0)

    assert msg.header.frame_id == "map"
    assert msg.pose.position.x == 1.2
    assert msg.pose.position.y == -0.4
    assert msg.pose.position.z == 0.0
    assert msg.pose.orientation.x == 0.0
    assert msg.pose.orientation.y == 0.0
    assert math.isclose(msg.pose.orientation.z, math.sqrt(0.5), rel_tol=1e-9)
    assert math.isclose(msg.pose.orientation.w, math.sqrt(0.5), rel_tol=1e-9)
