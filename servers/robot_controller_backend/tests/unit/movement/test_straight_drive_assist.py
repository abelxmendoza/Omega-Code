import math
import os
import sys

import pytest

BACKEND_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
if BACKEND_ROOT not in sys.path:
    sys.path.insert(0, BACKEND_ROOT)

from movement.straight_drive_assist import StraightDriveAssist


class FakeMotor:
    MAX_PWM = 4095

    def __init__(self):
        self.calls = []
        self.left_trim = None
        self.right_trim = None

    def set_trim(self, left=0, right=0):
        self.left_trim = left
        self.right_trim = right
        self.calls.append((left, right))


def test_initialisation_applies_zero_trim():
    motor = FakeMotor()
    assist = StraightDriveAssist(motor)

    status = assist.status()
    assert status["enabled"] is True
    assert status["leftTrim"] == 0
    assert status["rightTrim"] == 0
    assert motor.calls[-1] == (0, 0)
    assert math.isclose(status["lastApplied"], assist.status()["lastApplied"], rel_tol=1e-6)


def test_trim_configuration_and_enable_toggle():
    motor = FakeMotor()
    assist = StraightDriveAssist(motor)

    assist.set_trim(left=150, right=20)
    assert motor.calls[-1] == (150, 20)

    assist.set_enabled(False)
    # When disabled, trim is applied as zeros but stored internally
    assert motor.calls[-1] == (0, 0)

    assist.set_enabled(True)
    assert motor.calls[-1] == (150, 20)
    status = assist.status()
    assert status["leftTrim"] == 150
    assert status["rightTrim"] == 20


def test_nudge_logic_and_relaxation():
    motor = FakeMotor()
    assist = StraightDriveAssist(motor, step=50, max_trim=100)

    status = assist.nudge("left")
    assert status["rightTrim"] == 50
    assert status["leftTrim"] == 0

    status = assist.nudge("right", amount=40)
    assert status["leftTrim"] == 40
    assert status["rightTrim"] == 50

    status = assist.nudge("center", amount=30)
    assert status["leftTrim"] == 10
    assert status["rightTrim"] == 20

    status = assist.nudge("reset")
    assert status["leftTrim"] == 0
    assert status["rightTrim"] == 0


def test_max_trim_clamps_values():
    motor = FakeMotor()
    assist = StraightDriveAssist(motor)

    assist.set_trim(left=2000, right=2000)
    status = assist.set_max_trim(100)
    assert status["leftTrim"] == 100
    assert status["rightTrim"] == 100


def test_invalid_direction_raises_value_error():
    motor = FakeMotor()
    assist = StraightDriveAssist(motor)

    with pytest.raises(ValueError):
        assist.nudge("diagonal")


def test_set_step_updates_step_size():
    motor = FakeMotor()
    assist = StraightDriveAssist(motor)
    assist.set_step(80)
    assert assist.status()["step"] == 80

    # Negative values should be normalised to positive
    assist.set_step(-30)
    assert assist.status()["step"] == 30
