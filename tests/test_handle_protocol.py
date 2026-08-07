import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "handle"))

from handle_protocol import (  # noqa: E402
    HandleRegisters,
    crc16_modbus,
    gear_scale,
    normalize_axis,
    speed_mps_to_register,
)


def test_crc16_matches_standard_modbus_vector():
    assert crc16_modbus(bytes.fromhex("01 03 00 00 00 0A")) == 0xCDC5


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        (0, -1.0),
        (1998, 0.0),
        (2048, 0.0),
        (2098, 0.0),
        (4095, 1.0),
    ],
)
def test_normalize_axis_applies_center_dead_zone_and_full_scale(raw, expected):
    value = normalize_axis(
        raw,
        minimum=0,
        center=2048,
        maximum=4095,
        dead_zone=50,
    )

    assert value == pytest.approx(expected)


def test_normalize_axis_applies_direction():
    value = normalize_axis(
        4095,
        minimum=0,
        center=2048,
        maximum=4095,
        dead_zone=50,
        direction=-1.0,
    )

    assert value == -1.0


@pytest.mark.parametrize(
    ("gear", "expected"),
    [
        (1, 0.2),
        (2, 0.4),
        (3, 0.6),
        (4, 0.8),
        (5, 1.0),
        (0, None),
        (6, None),
    ],
)
def test_gear_scale_uses_five_uniform_steps(gear, expected):
    assert gear_scale(gear) == expected


@pytest.mark.parametrize(
    ("speed_mps", "expected"),
    [
        (0.0, 0),
        (0.123, 123),
        (-0.123, 123),
        (0.6, 600),
        (100.0, 0xFFFF),
    ],
)
def test_speed_mps_to_register_uses_unsigned_millimetres_per_second(
    speed_mps, expected
):
    assert speed_mps_to_register(speed_mps) == expected


def test_handle_registers_parse_gear_axes_and_seven_button_bits():
    sample = HandleRegisters.from_words([3, 1000, 3000, 0x00FF])

    assert sample.gear == 3
    assert sample.joystick_x == 1000
    assert sample.joystick_y == 3000
    assert sample.buttons == 0x007F


def test_handle_registers_require_complete_register_block():
    with pytest.raises(ValueError, match="4 registers"):
        HandleRegisters.from_words([1, 2, 3])
