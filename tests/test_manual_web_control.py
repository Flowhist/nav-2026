import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts" / "control"))
sys.path.insert(0, str(ROOT / "server"))

from geometry_msgs.msg import Twist  # noqa: E402

from base_control_router import BaseControlRouter  # noqa: E402
from ros_bridge import load_handle_mapping, manual_drive_command  # noqa: E402


class _Logger:
    def warn(self, _message):
        pass


class _RouterHarness:
    _nonzero = staticmethod(BaseControlRouter._nonzero)
    _on_web_cmd = BaseControlRouter._on_web_cmd

    def __init__(self):
        self._joystick_stop_latched = False
        self._web_reset_required = False
        self._web_cmd = Twist()
        self._web_time = 0.0
        self._last_lock_msg_time = 0.0
        self.statuses = []

    def _warn_reset_required(self, _reason):
        pass

    def _publish_status(self, status):
        self.statuses.append(status)


def test_web_drive_uses_handle_mapping_and_current_gear():
    mapping = load_handle_mapping(ROOT / "config" / "handle.yaml")

    full_speed = manual_drive_command(1.0, 0.0, 5, mapping)
    third_gear = manual_drive_command(1.0, 0.0, 3, mapping)
    diagonal = manual_drive_command(1.0, 1.0, 5, mapping)

    assert full_speed is not None and math.isclose(full_speed.linear, 0.6)
    assert third_gear is not None and math.isclose(third_gear.linear, 0.36)
    assert diagonal is not None
    assert math.isclose(diagonal.linear, 0.6 / math.sqrt(2), rel_tol=1e-6)
    assert math.isclose(diagonal.angular, 0.5 / math.sqrt(2), rel_tol=1e-6)
    assert manual_drive_command(1.0, 0.0, None, mapping) is None


def test_web_command_must_return_neutral_after_joystick_interrupt():
    harness = _RouterHarness()
    harness._web_reset_required = True
    moving = Twist()
    moving.linear.x = 0.5

    harness._on_web_cmd(moving)
    assert harness._web_time == 0.0
    assert harness._web_reset_required

    harness._on_web_cmd(Twist())
    assert not harness._web_reset_required
    assert harness.statuses == ["web_reset"]


def test_neutral_received_during_joystick_lock_is_remembered():
    harness = _RouterHarness()
    harness._joystick_stop_latched = True
    harness._web_reset_required = True

    harness._on_web_cmd(Twist())

    assert not harness._web_reset_required
    assert harness._web_time == 0.0


def test_nonzero_web_command_during_joystick_lock_requires_a_new_neutral():
    harness = _RouterHarness()
    harness._joystick_stop_latched = True
    moving = Twist()
    moving.angular.z = 0.5

    harness._on_web_cmd(moving)

    assert harness._web_reset_required
    assert harness._web_time == 0.0


def test_manual_control_ui_is_shared_by_mapping_and_navigation():
    html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    script = (ROOT / "server" / "web" / "app-manual-control.js").read_text(encoding="utf-8")

    assert html.count('class="canvas-shell live-canvas-shell"') == 2
    assert html.count('id="manualDrive"') == 1
    assert "mappingCanvas" in script and "navigationCanvas" in script
    assert '"arrowup"' in script and '"arrowleft"' in script
    assert "setPointerCapture" in script
    assert "gear_online" in script
