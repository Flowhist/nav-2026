import importlib.util
import math
from pathlib import Path


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_joystick_discrete_mode_allows_diagonal_motion():
    module = _load(
        Path(__file__).resolve().parents[1] / "scripts" / "control" / "joy_control.py",
        "joy_control_under_test",
    )
    cfg = module.JoyMappingConfig(
        linear_axis=1,
        linear_direction=-1.0,
        angular_axis=0,
        angular_direction=-1.0,
        dead_zone=0.0,
        speed_split=0.65,
        discrete_motion_enable=True,
        linear_speed_low=0.4,
        linear_speed_high=0.75,
        angular_speed_low=math.radians(50.0),
        angular_speed_high=math.radians(70.0),
    )

    msg = module.joy_axes_to_twist([-1.0, -1.0], cfg)

    assert math.isclose(msg.linear.x, 0.75)
    assert math.isclose(msg.angular.z, math.radians(70.0))


def test_joystick_backward_diagonal_uses_driver_perspective_left_right():
    module = _load(
        Path(__file__).resolve().parents[1] / "scripts" / "control" / "joy_control.py",
        "joy_control_backward_under_test",
    )
    cfg = module.JoyMappingConfig(
        linear_axis=1,
        linear_direction=-1.0,
        angular_axis=0,
        angular_direction=-1.0,
        dead_zone=0.0,
        speed_split=0.65,
        discrete_motion_enable=True,
        linear_speed_low=0.4,
        linear_speed_high=0.75,
        angular_speed_low=math.radians(50.0),
        angular_speed_high=math.radians(70.0),
    )

    backward_left = module.joy_axes_to_twist([-1.0, 1.0], cfg)
    backward_right = module.joy_axes_to_twist([1.0, 1.0], cfg)

    assert math.isclose(backward_left.linear.x, -0.75)
    assert math.isclose(backward_left.angular.z, -math.radians(70.0))
    assert math.isclose(backward_right.linear.x, -0.75)
    assert math.isclose(backward_right.angular.z, math.radians(70.0))


def test_keyboard_resolves_single_and_diagonal_sticky_commands():
    module = _load(
        Path(__file__).resolve().parents[1] / "scripts" / "control" / "base_control_router.py",
        "router_under_test",
    )

    assert module.resolve_keyboard_command("w", 0.3, 0.5) == (0.3, 0.0)
    assert module.resolve_keyboard_command("a", 0.3, 0.5) == (0.0, 0.5)
    assert module.resolve_keyboard_command("q", 0.3, 0.5) == (0.3, 0.5)
    assert module.resolve_keyboard_command("e", 0.3, 0.5) == (0.3, -0.5)
    assert module.resolve_keyboard_command("z", 0.3, 0.5) == (-0.3, -0.5)
    assert module.resolve_keyboard_command("c", 0.3, 0.5) == (-0.3, 0.5)
    assert module.resolve_keyboard_command(" ", 0.3, 0.5) == (0.0, 0.0)
