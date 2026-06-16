import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "tool"))

from base_monitor import (  # noqa: E402
    CommandSnapshot,
    infer_command_source,
    render_sparkline,
    wheel_degps_to_twist,
)


def test_wheel_degps_to_twist_uses_base_control_units():
    linear, angular, left_mps, right_mps = wheel_degps_to_twist(
        left_degps=180.0,
        right_degps=360.0,
        wheel_radius=0.1,
        wheel_separation=0.5,
        wheel_velocity_sign=1.0,
    )

    assert math.isclose(left_mps, math.pi * 0.1, rel_tol=1e-6)
    assert math.isclose(right_mps, 2.0 * math.pi * 0.1, rel_tol=1e-6)
    assert math.isclose(linear, 0.15 * math.pi, rel_tol=1e-6)
    assert math.isclose(angular, (math.pi * 0.1) / 0.5, rel_tol=1e-6)


def test_render_sparkline_preserves_width_and_marks_zero():
    graph = render_sparkline(
        [-1.0, -0.5, 0.0, 0.5, 1.0],
        width=9,
        min_value=-1.0,
        max_value=1.0,
    )

    assert len(graph) == 9
    assert "0" in graph
    assert any(ch in graph for ch in "▁▂▃▄▅▆▇█")


def test_infer_command_source_prefers_fresh_matching_input():
    snapshots = {
        "joystick": CommandSnapshot(0.2, 0.0, stamp=9.9),
        "web": CommandSnapshot(0.0, 0.1, stamp=9.9),
        "nav": CommandSnapshot(0.2, 0.0, stamp=7.0),
    }

    source = infer_command_source(
        cmd=CommandSnapshot(0.2, 0.0, stamp=10.0),
        inputs=snapshots,
        now=10.0,
        timeout=0.4,
    )

    assert source == "joystick"
