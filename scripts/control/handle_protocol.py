#!/usr/bin/env python3
"""Pure protocol helpers for the STM32 wheelchair handle."""

from dataclasses import dataclass
from typing import Optional, Sequence


DISPLAY_SPEED_REGISTER = 0x0002
HANDLE_STATE_REGISTER = 0x0003
HANDLE_STATE_REGISTER_COUNT = 4
NAVIGATION_BUTTON_MASK = 1 << 2
BUTTON_MASK = 0x007F


def crc16_modbus(data: bytes) -> int:
    """Return the standard Modbus RTU CRC-16 value for *data*."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def normalize_axis(
    raw: int,
    *,
    minimum: int,
    center: int,
    maximum: int,
    dead_zone: int,
    direction: float = 1.0,
) -> float:
    """Map an ADC axis to ``[-1, 1]`` while preserving a center dead zone."""
    if not minimum < center < maximum:
        raise ValueError("axis limits must satisfy minimum < center < maximum")
    if dead_zone < 0 or center - dead_zone <= minimum or center + dead_zone >= maximum:
        raise ValueError("dead_zone must fit inside the axis limits")

    value = max(minimum, min(maximum, int(raw)))
    lower = center - dead_zone
    upper = center + dead_zone
    if lower <= value <= upper:
        return 0.0
    if value < lower:
        normalized = (value - lower) / float(lower - minimum)
    else:
        normalized = (value - upper) / float(maximum - upper)
    return max(-1.0, min(1.0, normalized * float(direction)))


def gear_scale(gear: int) -> Optional[float]:
    """Return the uniform scale for gears 1-5, or ``None`` if invalid."""
    value = int(gear)
    if 1 <= value <= 5:
        return value / 5.0
    return None


def speed_mps_to_register(speed_mps: float) -> int:
    """Encode absolute m/s as an unsigned register in 0.001 m/s units."""
    encoded = int(round(abs(float(speed_mps)) * 1000.0))
    return max(0, min(0xFFFF, encoded))


@dataclass(frozen=True)
class HandleRegisters:
    """Decoded register block starting at ``0x0003``."""

    gear: int
    joystick_x: int
    joystick_y: int
    buttons: int

    @classmethod
    def from_words(cls, words: Sequence[int]) -> "HandleRegisters":
        if len(words) != HANDLE_STATE_REGISTER_COUNT:
            raise ValueError(
                f"expected {HANDLE_STATE_REGISTER_COUNT} registers, got {len(words)}"
            )
        return cls(
            gear=int(words[0]) & 0xFFFF,
            joystick_x=int(words[1]) & 0xFFFF,
            joystick_y=int(words[2]) & 0xFFFF,
            buttons=int(words[3]) & BUTTON_MASK,
        )


@dataclass(frozen=True)
class HandleCommand:
    """Velocity command derived from one handle register sample."""

    valid: bool
    linear: float
    angular: float
    joy_axes: tuple[float, float]


def command_from_registers(
    sample: HandleRegisters,
    *,
    axis_minimum: int,
    axis_center: int,
    axis_maximum: int,
    dead_zone: int,
    linear_direction: float,
    angular_direction: float,
    max_linear_speed: float,
    max_angular_speed: float,
) -> HandleCommand:
    """Convert a handle sample into a uniformly gear-scaled velocity command."""
    angular_axis = normalize_axis(
        sample.joystick_x,
        minimum=axis_minimum,
        center=axis_center,
        maximum=axis_maximum,
        dead_zone=dead_zone,
        direction=angular_direction,
    )
    linear_axis = normalize_axis(
        sample.joystick_y,
        minimum=axis_minimum,
        center=axis_center,
        maximum=axis_maximum,
        dead_zone=dead_zone,
        direction=linear_direction,
    )
    scale = gear_scale(sample.gear)
    axes = (angular_axis, linear_axis)
    if scale is None:
        return HandleCommand(False, 0.0, 0.0, axes)

    return HandleCommand(
        True,
        linear_axis * float(max_linear_speed) * scale,
        angular_axis * float(max_angular_speed) * scale,
        axes,
    )


def navigation_button_pressed(previous_buttons: int, current_buttons: int) -> bool:
    """Return whether the navigation button changed from released to pressed."""
    was_pressed = bool(int(previous_buttons) & NAVIGATION_BUTTON_MASK)
    is_pressed = bool(int(current_buttons) & NAVIGATION_BUTTON_MASK)
    return is_pressed and not was_pressed
