"""Unit tests exercising the advanced PID controllers."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from controllers.advanced_pid import AdvancedPIDController, PIDConfig


def test_pid_controller_converges_to_setpoint() -> None:
    controller = AdvancedPIDController(
        PIDConfig(kp=1.4, ki=0.8, kd=0.2, output_limits=(-10.0, 10.0))
    )
    controller.set_setpoint(5.0)

    measurement = 0.0
    dt = 0.05
    for _ in range(200):
        output = controller.update(measurement, dt)
        measurement += output * dt * 0.3

    state = controller.get_state()
    assert math.isclose(measurement, 5.0, rel_tol=0.05)
    assert abs(state.last_error) < 0.3
    assert state.last_output <= 10.0


def test_pid_respects_output_limits_and_anti_windup() -> None:
    controller = AdvancedPIDController(
        PIDConfig(kp=0.5, ki=2.0, kd=0.0, output_limits=(-1.0, 1.0))
    )
    controller.set_setpoint(100.0)

    measurement = 0.0
    dt = 0.1
    for _ in range(50):
        controller.update(measurement, dt)

    state = controller.get_state()
    assert abs(state.integral) < controller.config.integral_limits[1]
    assert math.isclose(state.last_output, 1.0, rel_tol=1e-6)


def test_pid_derivative_filter_smooths_noise() -> None:
    noisy_controller = AdvancedPIDController(
        PIDConfig(kp=0.0, ki=0.0, kd=1.0, derivative_filter=0.0, output_limits=(-5.0, 5.0))
    )
    filtered_controller = AdvancedPIDController(
        PIDConfig(kp=0.0, ki=0.0, kd=1.0, derivative_filter=0.8, output_limits=(-5.0, 5.0))
    )

    measurements = [0.0, 1.0, -1.0, 1.0, -1.0]
    dt = 0.05

    noisy_outputs = [noisy_controller.update(value, dt) for value in measurements]
    filtered_outputs = [filtered_controller.update(value, dt) for value in measurements]

    assert max(abs(val) for val in filtered_outputs) < max(abs(val) for val in noisy_outputs)


def test_feedforward_defaults_to_setpoint_when_enabled() -> None:
    controller = AdvancedPIDController(
        PIDConfig(kp=0.0, ki=0.0, kd=0.0, kf=0.5, setpoint=3.0, output_limits=(-10.0, 10.0))
    )

    output = controller.update(0.0, dt=0.1)
    assert math.isclose(output, 1.5, rel_tol=1e-6)


def test_pid_requires_positive_dt() -> None:
    controller = AdvancedPIDController(PIDConfig(kp=1.0, ki=0.0, kd=0.0))
    with pytest.raises(ValueError):
        controller.update(0.0, 0.0)
