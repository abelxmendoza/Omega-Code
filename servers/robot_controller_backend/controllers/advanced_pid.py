"""Advanced PID controller tailored for the rover's throttle motor."""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional, Tuple
import math


@dataclass
class PIDConfig:
    """Configuration values for the throttle controller."""

    kp: float
    ki: float
    kd: float
    kf: float = 0.0
    setpoint: float = 0.0
    output_limits: Tuple[float, float] = (-1.0, 1.0)
    integral_limits: Tuple[float, float] = (-1.0, 1.0)
    derivative_filter: float = 0.2
    integral_separation: float = math.inf
    anti_windup: bool = True


@dataclass
class PIDState:
    """Holds the evolving controller state for introspection and testing."""

    integral: float = 0.0
    filtered_derivative: float = 0.0
    last_error: float = 0.0
    last_measurement: float = 0.0
    last_output: float = 0.0


def _clamp(value: float, limits: Tuple[float, float]) -> float:
    lower, upper = limits
    return max(lower, min(upper, value))


class AdvancedPIDController:
    """PID controller with anti-windup, derivative filtering and feed-forward."""

    def __init__(self, config: PIDConfig) -> None:
        self.config = config
        self._state = PIDState()
        self._setpoint = config.setpoint

    def reset(self) -> None:
        self._state = PIDState()

    def set_setpoint(self, setpoint: float) -> None:
        self._setpoint = setpoint

    def update(self, measurement: float, dt: float, *, feedforward: Optional[float] = None) -> float:
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        error = self._setpoint - measurement
        proportional = self.config.kp * error

        if abs(error) <= self.config.integral_separation:
            new_integral = self._state.integral + error * dt
            new_integral = _clamp(new_integral, self.config.integral_limits)
        else:
            new_integral = self._state.integral

        derivative = 0.0
        if dt > 0:
            derivative = (measurement - self._state.last_measurement) / dt
        alpha = _clamp(self.config.derivative_filter, (0.0, 0.999))
        filtered_derivative = alpha * self._state.filtered_derivative + (1.0 - alpha) * derivative
        derivative_term = -self.config.kd * filtered_derivative

        ff_term = 0.0
        if feedforward is not None:
            ff_term = self.config.kf * feedforward
        elif self.config.kf:
            ff_term = self.config.kf * self._setpoint

        output = proportional + self.config.ki * new_integral + derivative_term + ff_term
        clamped_output = _clamp(output, self.config.output_limits)

        if self.config.anti_windup and clamped_output != output:
            new_integral = self._state.integral
            output = proportional + self.config.ki * new_integral + derivative_term + ff_term
            clamped_output = _clamp(output, self.config.output_limits)

        self._state = PIDState(
            integral=new_integral,
            filtered_derivative=filtered_derivative,
            last_error=error,
            last_measurement=measurement,
            last_output=clamped_output,
        )
        return clamped_output

    def get_state(self) -> PIDState:
        return replace(self._state)


