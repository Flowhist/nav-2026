import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "handle"))

from handle_hid import (  # noqa: E402
    HidCommand,
    HidMappingConfig,
    command_from_axes,
    slew_limited_command,
)


class HandleHidContinuousTest(unittest.TestCase):
    def setUp(self):
        self.config = HidMappingConfig(
            linear_axis=1,
            angular_axis=0,
            dead_zone=0.0,
            saturation=1.0,
            max_linear_speed=0.6,
            max_angular_speed=0.5,
        )

    def test_radial_dead_zone_keeps_center_zero(self):
        config = HidMappingConfig(dead_zone=0.05)
        command = command_from_axes([0.03, 0.04], config, 1.0)
        self.assertEqual(command, HidCommand(0.0, 0.0))

    def test_arbitrary_stick_angle_is_preserved(self):
        command = command_from_axes([0.8, 0.6], self.config, 1.0)
        self.assertAlmostEqual(command.linear, 0.36)
        self.assertAlmostEqual(command.angular, 0.4)

    def test_diagonal_corner_is_limited_to_unit_circle(self):
        command = command_from_axes([1.0, 1.0], self.config, 1.0)
        normalized = math.hypot(
            command.linear / self.config.max_linear_speed,
            command.angular / self.config.max_angular_speed,
        )
        self.assertAlmostEqual(normalized, 1.0)

    def test_gear_scales_both_components_together(self):
        command = command_from_axes([0.8, 0.6], self.config, 0.4)
        self.assertAlmostEqual(command.linear, 0.144)
        self.assertAlmostEqual(command.angular, 0.16)

    def test_coupled_slew_preserves_target_direction(self):
        command = slew_limited_command(
            HidCommand(0.0, 0.0),
            HidCommand(0.6, 0.5),
            dt=0.02,
            linear_acceleration_rate=1.5,
            linear_deceleration_rate=6.0,
            angular_acceleration_rate=math.pi,
            angular_deceleration_rate=2.0 * math.pi,
        )
        self.assertAlmostEqual(command.linear, 0.03)
        self.assertAlmostEqual(command.angular, 0.025)

    def test_reversal_uses_deceleration_without_overshoot(self):
        command = slew_limited_command(
            HidCommand(0.2, 0.2),
            HidCommand(-0.2, -0.2),
            dt=0.05,
            linear_acceleration_rate=1.0,
            linear_deceleration_rate=3.0,
            angular_acceleration_rate=1.0,
            angular_deceleration_rate=3.0,
        )
        self.assertAlmostEqual(command.linear, 0.05)
        self.assertAlmostEqual(command.angular, 0.05)

    def test_coupled_slew_reaches_target(self):
        current = HidCommand(0.0, 0.0)
        target = HidCommand(0.3, -0.25)
        for _ in range(50):
            current = slew_limited_command(
                current,
                target,
                dt=0.02,
                linear_acceleration_rate=1.5,
                linear_deceleration_rate=6.0,
                angular_acceleration_rate=math.pi,
                angular_deceleration_rate=2.0 * math.pi,
            )
        self.assertAlmostEqual(current.linear, target.linear)
        self.assertAlmostEqual(current.angular, target.angular)


if __name__ == "__main__":
    unittest.main()
