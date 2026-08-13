#!/usr/bin/env python3
"""Linux joystick input and pure command-shaping helpers for handle_control."""

from dataclasses import dataclass
import math
import os
import struct
from typing import Sequence


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
_EVENT = struct.Struct("<IhBB")


def normalize_raw_axis(value: int) -> float:
    """Normalize a Linux joystick int16 axis to [-1.0, 1.0]."""
    if value >= 0:
        return min(1.0, float(value) / 32767.0)
    return max(-1.0, float(value) / 32768.0)


def scaled_axis(
    value: float,
    *,
    direction: float = 1.0,
    dead_zone: float = 0.05,
    saturation: float = 1.0,
) -> float:
    """Apply direction, dead zone, and saturation to a normalized axis."""
    value = max(-1.0, min(1.0, float(value)))
    dead_zone = max(0.0, min(0.99, float(dead_zone)))
    saturation = max(dead_zone + 1e-6, min(1.0, float(saturation)))
    magnitude = abs(value)
    if magnitude <= dead_zone:
        return 0.0
    scaled = min(1.0, (magnitude - dead_zone) / (saturation - dead_zone))
    sign = 1.0 if value >= 0.0 else -1.0
    return sign * scaled * (1.0 if direction >= 0.0 else -1.0)


def scaled_axes(
    linear_value: float,
    angular_value: float,
    *,
    linear_direction: float = 1.0,
    angular_direction: float = 1.0,
    dead_zone: float = 0.05,
    saturation: float = 1.0,
) -> tuple[float, float]:
    """Map a joystick vector through a radial dead zone into a unit circle."""
    linear = max(-1.0, min(1.0, float(linear_value)))
    angular = max(-1.0, min(1.0, float(angular_value)))
    linear *= 1.0 if linear_direction >= 0.0 else -1.0
    angular *= 1.0 if angular_direction >= 0.0 else -1.0

    dead_zone = max(0.0, min(0.99, float(dead_zone)))
    saturation = max(dead_zone + 1e-6, min(1.0, float(saturation)))
    magnitude = math.hypot(linear, angular)
    if magnitude <= dead_zone:
        return 0.0, 0.0

    scaled_magnitude = min(
        1.0,
        (min(magnitude, saturation) - dead_zone) / (saturation - dead_zone),
    )
    vector_scale = scaled_magnitude / magnitude
    return linear * vector_scale, angular * vector_scale


def axis_value(axes: Sequence[float], index: int) -> float:
    if index < 0 or index >= len(axes):
        return 0.0
    return float(axes[index])


@dataclass(frozen=True)
class HidMappingConfig:
    linear_axis: int = 1
    angular_axis: int = 0
    linear_direction: float = 1.0
    angular_direction: float = 1.0
    dead_zone: float = 0.05
    saturation: float = 1.0
    max_linear_speed: float = 0.6
    max_angular_speed: float = 0.5


@dataclass(frozen=True)
class HidCommand:
    linear: float
    angular: float


def command_from_axes(
    axes: Sequence[float], config: HidMappingConfig, speed_scale: float
) -> HidCommand:
    """Map HID axes to a gear-scaled ROS velocity command."""
    speed_scale = max(0.0, min(1.0, float(speed_scale)))
    linear, angular = scaled_axes(
        axis_value(axes, config.linear_axis),
        axis_value(axes, config.angular_axis),
        linear_direction=config.linear_direction,
        angular_direction=config.angular_direction,
        dead_zone=config.dead_zone,
        saturation=config.saturation,
    )
    return HidCommand(
        linear=linear * max(0.0, config.max_linear_speed) * speed_scale,
        angular=angular * max(0.0, config.max_angular_speed) * speed_scale,
    )


def slew_limited_value(
    current: float,
    target: float,
    dt: float,
    acceleration_rate: float,
    deceleration_rate: float,
) -> float:
    """Move current toward target without exceeding the selected slew rate."""
    current = float(current)
    target = float(target)
    dt = max(0.0, float(dt))
    if current == target or dt == 0.0:
        return current

    same_direction = current == 0.0 or current * target > 0.0
    accelerating = same_direction and abs(target) > abs(current)
    rate = acceleration_rate if accelerating else deceleration_rate
    maximum_delta = max(0.0, float(rate)) * dt
    delta = target - current
    if abs(delta) <= maximum_delta:
        return target
    return current + (maximum_delta if delta > 0.0 else -maximum_delta)


def slew_limited_command(
    current: HidCommand,
    target: HidCommand,
    dt: float,
    linear_acceleration_rate: float,
    linear_deceleration_rate: float,
    angular_acceleration_rate: float,
    angular_deceleration_rate: float,
) -> HidCommand:
    """Move both velocity components together without distorting direction."""
    dt = max(0.0, float(dt))
    if dt == 0.0 or current == target:
        return current

    linear_rate = (
        linear_acceleration_rate
        if current.linear == 0.0
        or (
            current.linear * target.linear > 0.0
            and abs(target.linear) > abs(current.linear)
        )
        else linear_deceleration_rate
    )
    angular_rate = (
        angular_acceleration_rate
        if current.angular == 0.0
        or (
            current.angular * target.angular > 0.0
            and abs(target.angular) > abs(current.angular)
        )
        else angular_deceleration_rate
    )

    linear_delta = target.linear - current.linear
    angular_delta = target.angular - current.angular
    linear_limit = max(0.0, float(linear_rate)) * dt
    angular_limit = max(0.0, float(angular_rate)) * dt

    ratios = []
    if abs(linear_delta) > 1e-12:
        if linear_limit == 0.0:
            return current
        ratios.append(abs(linear_delta) / linear_limit)
    if abs(angular_delta) > 1e-12:
        if angular_limit == 0.0:
            return current
        ratios.append(abs(angular_delta) / angular_limit)

    scale = min(1.0, 1.0 / max(ratios, default=1.0))
    return HidCommand(
        linear=current.linear + linear_delta * scale,
        angular=current.angular + angular_delta * scale,
    )


class LinuxJoystickReader:
    """Non-blocking reader for the stable Linux /dev/input/js* ABI."""

    def __init__(self, device: str, initial_axis_count: int = 8):
        self.device = str(device)
        self.axes = [0.0] * max(0, int(initial_axis_count))
        self.buttons = []
        self._fd = None

    @property
    def is_open(self) -> bool:
        return self._fd is not None

    def open(self) -> None:
        if self._fd is not None:
            return
        self._fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        self.axes = [0.0] * len(self.axes)
        self.buttons = []

    def close(self) -> None:
        if self._fd is None:
            return
        try:
            os.close(self._fd)
        finally:
            self._fd = None

    def read_available(self) -> bool:
        """Drain queued events and return whether at least one event was read."""
        if self._fd is None:
            return False
        updated = False
        while True:
            try:
                data = os.read(self._fd, _EVENT.size)
            except BlockingIOError:
                return updated
            if not data:
                raise OSError("joystick device returned EOF")
            if len(data) != _EVENT.size:
                raise OSError("short joystick event")

            _timestamp, value, event_type, number = _EVENT.unpack(data)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS:
                if number >= len(self.axes):
                    self.axes.extend([0.0] * (number + 1 - len(self.axes)))
                self.axes[number] = normalize_raw_axis(value)
                updated = True
            elif event_type == JS_EVENT_BUTTON:
                if number >= len(self.buttons):
                    self.buttons.extend([0] * (number + 1 - len(self.buttons)))
                self.buttons[number] = 1 if value else 0
                updated = True
