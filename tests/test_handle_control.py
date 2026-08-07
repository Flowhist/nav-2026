import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "handle"))

from handle_protocol import (  # noqa: E402
    HandleRegisters,
    command_from_registers,
    navigation_button_pressed,
)


def _command(gear: int, x: int, y: int):
    return command_from_registers(
        HandleRegisters(gear=gear, joystick_x=x, joystick_y=y, buttons=0),
        axis_minimum=0,
        axis_center=2048,
        axis_maximum=4095,
        dead_zone=50,
        linear_direction=1.0,
        angular_direction=1.0,
        max_linear_speed=0.6,
        max_angular_speed=0.5,
    )


def test_fifth_gear_uses_full_linear_and_angular_limits():
    command = _command(5, 4095, 4095)

    assert command.valid is True
    assert command.linear == pytest.approx(0.6)
    assert command.angular == pytest.approx(0.5)
    assert command.joy_axes == pytest.approx((1.0, 1.0))


def test_first_gear_scales_linear_and_angular_limits_together():
    command = _command(1, 4095, 4095)

    assert command.linear == pytest.approx(0.12)
    assert command.angular == pytest.approx(0.10)


def test_invalid_gear_returns_safe_zero_command():
    command = _command(0, 4095, 4095)

    assert command.valid is False
    assert command.linear == 0.0
    assert command.angular == 0.0


def test_axis_directions_are_applied_before_speed_scaling():
    command = command_from_registers(
        HandleRegisters(gear=5, joystick_x=4095, joystick_y=0, buttons=0),
        axis_minimum=0,
        axis_center=2048,
        axis_maximum=4095,
        dead_zone=50,
        linear_direction=-1.0,
        angular_direction=-1.0,
        max_linear_speed=0.6,
        max_angular_speed=0.5,
    )

    assert command.linear == pytest.approx(0.6)
    assert command.angular == pytest.approx(-0.5)


def test_navigation_button_only_triggers_on_rising_edge():
    navigation_mask = 1 << 2

    assert navigation_button_pressed(0, navigation_mask) is True
    assert navigation_button_pressed(navigation_mask, navigation_mask) is False
    assert navigation_button_pressed(navigation_mask, 0) is False
