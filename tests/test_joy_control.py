import math
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "control"))

from geometry_msgs.msg import Twist  # noqa: E402
from joy_control import JoyMappingConfig, joy_axes_to_twist, slew_limited_twist  # noqa: E402


def _twist(linear: float, angular: float) -> Twist:
    msg = Twist()
    msg.linear.x = float(linear)
    msg.angular.z = float(angular)
    return msg


def test_joystick_continuous_mode_scales_axes_linearly():
    cfg = JoyMappingConfig(
        linear_axis=1,
        linear_direction=-1.0,
        angular_axis=0,
        angular_direction=-1.0,
        dead_zone=0.10,
        sat_zone=1.0,
        linear_speed_high=0.75,
        angular_speed_high=math.radians(70.0),
    )

    msg = joy_axes_to_twist([-0.55, -0.55], cfg)

    expected = (0.55 - 0.10) / (1.0 - 0.10)
    assert msg.linear.x == pytest.approx(expected * 0.75)
    assert msg.angular.z == pytest.approx(expected * math.radians(70.0))


def test_joystick_continuous_mode_keeps_dead_zone_zero():
    cfg = JoyMappingConfig(
        linear_axis=1,
        linear_direction=-1.0,
        angular_axis=0,
        angular_direction=-1.0,
        dead_zone=0.10,
        sat_zone=1.0,
        linear_speed_high=0.75,
        angular_speed_high=math.radians(70.0),
    )

    msg = joy_axes_to_twist([-0.05, -0.05], cfg)

    assert msg.linear.x == 0.0
    assert msg.angular.z == 0.0


def test_slew_limiter_caps_single_publish_step():
    msg = slew_limited_twist(
        _twist(0.0, 0.0),
        _twist(0.75, math.radians(70.0)),
        dt=0.02,
        linear_accel_slew_rate=1.5,
        linear_decel_slew_rate=4.0,
        angular_accel_slew_rate=math.radians(180.0),
        angular_decel_slew_rate=math.radians(360.0),
    )

    assert msg.linear.x == pytest.approx(0.03)
    assert msg.angular.z == pytest.approx(math.radians(3.6))


def test_slew_limiter_uses_decel_rate_when_returning_to_zero():
    msg = slew_limited_twist(
        _twist(0.6, math.radians(40.0)),
        _twist(0.0, 0.0),
        dt=0.1,
        linear_accel_slew_rate=1.5,
        linear_decel_slew_rate=4.0,
        angular_accel_slew_rate=math.radians(180.0),
        angular_decel_slew_rate=math.radians(360.0),
    )

    assert msg.linear.x == pytest.approx(0.2)
    assert msg.angular.z == pytest.approx(math.radians(4.0))


def test_slew_limiter_uses_decel_rate_when_reversing_direction():
    msg = slew_limited_twist(
        _twist(0.2, 0.2),
        _twist(-0.2, -0.2),
        dt=0.05,
        linear_accel_slew_rate=1.0,
        linear_decel_slew_rate=3.0,
        angular_accel_slew_rate=1.0,
        angular_decel_slew_rate=3.0,
    )

    assert msg.linear.x == pytest.approx(0.05)
    assert msg.angular.z == pytest.approx(0.05)


def test_slew_limiter_reaches_target_without_overshoot():
    current = _twist(0.0, 0.0)
    target = _twist(0.2, -0.2)

    for _ in range(20):
        current = slew_limited_twist(
            current,
            target,
            dt=0.05,
            linear_accel_slew_rate=1.0,
            linear_decel_slew_rate=3.0,
            angular_accel_slew_rate=1.0,
            angular_decel_slew_rate=3.0,
        )

    assert current.linear.x == pytest.approx(target.linear.x)
    assert current.angular.z == pytest.approx(target.angular.z)
