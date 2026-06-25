import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "control"))

from base_control import select_wheel_acceleration  # noqa: E402


def test_acceleration_is_used_when_both_wheels_speed_up():
    assert select_wheel_acceleration(40.0, 35.0, acceleration=150, acceleration_stop=300) == 150


def test_acceleration_is_used_for_nonzero_slowdown():
    assert select_wheel_acceleration(20.0, 45.0, acceleration=150, acceleration_stop=300) == 150


def test_acceleration_stop_is_used_only_when_stopping():
    assert (
        select_wheel_acceleration(
            left_degps=0.0,
            right_degps=0.0,
            acceleration=150,
            acceleration_stop=300,
        )
        == 300
    )


def test_acceleration_is_used_when_only_one_wheel_is_zero():
    assert select_wheel_acceleration(0.0, 40.0, acceleration=150, acceleration_stop=300) == 150
