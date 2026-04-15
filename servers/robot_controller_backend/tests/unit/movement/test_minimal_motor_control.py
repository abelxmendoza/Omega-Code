"""
Unit tests for movement/minimal_motor_control.py.

The Motor class delegates to the hardware abstraction layer (get_motor_driver).
We mock that layer at the module boundary to verify trim/stop/direction logic
without touching real hardware.
"""

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

BACKEND_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
if BACKEND_ROOT not in sys.path:
    sys.path.insert(0, BACKEND_ROOT)


def _make_motor():
    """Return (motor, mock_driver) with get_motor_driver patched in."""
    mock_driver = MagicMock()
    mock_driver.MAX_PWM = 4095
    mock_driver.trim_left = 0
    mock_driver.trim_right = 0

    def _set_trim(left, right):
        mock_driver.trim_left = left
        mock_driver.trim_right = right
    mock_driver.set_trim.side_effect = _set_trim

    import movement.minimal_motor_control as mmc
    with patch.object(mmc, 'get_motor_driver', return_value=mock_driver):
        import importlib
        importlib.reload(mmc)
        motor = mmc.Motor()
        # Re-inject the mock after reload (reload replaces the reference)
        motor.driver = mock_driver
    return motor, mock_driver


def test_forward_calls_set_pwm():
    motor, driver = _make_motor()
    motor.forward(1500)
    driver.set_pwm.assert_called_once_with(1500, 1500)


def test_backward_calls_set_pwm_negative():
    motor, driver = _make_motor()
    motor.backward(1000)
    driver.set_pwm.assert_called_once_with(-1000, -1000)


def test_stop_calls_driver_stop():
    motor, driver = _make_motor()
    motor.stop()
    driver.stop.assert_called_once()


def test_set_trim_clamps_to_max_pwm():
    motor, driver = _make_motor()
    motor.set_trim(left=99999, right=50)
    assert motor.left_trim == motor.MAX_PWM


def test_get_trim_returns_current_values():
    motor, driver = _make_motor()
    motor.set_trim(left=100, right=200)
    trim = motor.get_trim()
    assert trim['left'] == 100
    assert trim['right'] == 200
