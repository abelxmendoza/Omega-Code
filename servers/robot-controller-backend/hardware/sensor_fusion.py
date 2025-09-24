"""Sensor fusion utilities for combining motion hints and ultrasonic data.

The module provides a light-weight Kalman filter tailored for the rover's
primary localisation problem: estimating forward displacement using available
motion cues and the ultrasonic range finder as the absolute distance sensor.
When IMU acceleration data is available it can be supplied to the manager, but
the fusion pipeline also works when only ultrasonic measurements are present.
In that case the filter relies on the constant-acceleration model to propagate
state between measurements which still provides a smoothed position estimate
and an approximate velocity derived from the measurement history.

The public entry point is :class:`SensorFusionManager` which keeps track of the
latest fused state, measurement quality and residuals.  The helper class
:class:`KalmanFilter1D` performs the actual predict/update maths.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Deque, Dict, Optional
from collections import deque
import math

import numpy as np


@dataclass
class KalmanConfig:
    """Configuration for the 1D Kalman filter used by the rover.

    Parameters
    ----------
    dt:
        Default integration period in seconds.  When sensor timestamps are
        provided the filter adapts the timestep dynamically.
    process_variance:
        Variance of the motion model noise.  Higher values make the prediction
        more responsive to acceleration updates.
    measurement_variance:
        Variance associated with the ultrasonic measurements.
    acceleration_variance:
        Variance applied to the IMU acceleration control input.
    min_quality:
        Quality threshold (0-1) below which ultrasonic measurements are ignored.
    history_size:
        Number of fused states stored for introspection and debugging.
    """

    dt: float = 0.05
    process_variance: float = 1e-3
    measurement_variance: float = 0.05
    acceleration_variance: float = 0.1
    min_quality: float = 0.2
    history_size: int = 120


@dataclass
class FusedState:
    """Container describing the fused pose estimate."""

    position: float
    velocity: float
    covariance: np.ndarray
    residual: float
    timestamp: float
    quality: float


class KalmanFilter1D:
    """Constant-acceleration Kalman filter for 1D position estimates."""

    def __init__(self, config: KalmanConfig) -> None:
        self.config = config
        self.state = np.zeros((2, 1), dtype=float)  # [position, velocity]
        self.covariance = np.eye(2, dtype=float)
        self._dt = config.dt
        self._residual = 0.0

    @staticmethod
    def _system_matrices(dt: float, process_variance: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return state transition, control and process noise matrices."""

        f = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
        b = np.array([[0.5 * dt ** 2], [dt]], dtype=float)
        q = np.array(
            [
                [dt ** 4 / 4.0, dt ** 3 / 2.0],
                [dt ** 3 / 2.0, dt ** 2],
            ],
            dtype=float,
        ) * process_variance
        return f, b, q

    def predict(self, acceleration: float = 0.0, *, dt: Optional[float] = None) -> None:
        """Propagate the state using the provided acceleration."""

        step = float(dt) if dt is not None and dt > 0.0 else self.config.dt
        f, b, q = self._system_matrices(step, self.config.process_variance)

        control = np.array([[acceleration]], dtype=float)
        self.state = f @ self.state + b @ control
        self.covariance = f @ self.covariance @ f.T + q
        self._dt = step

    def update(self, distance: float, *, variance: Optional[float] = None) -> None:
        """Correct the prediction with an ultrasonic measurement."""

        h = np.array([[1.0, 0.0]], dtype=float)
        r = float(variance) if variance is not None else self.config.measurement_variance
        r_matrix = np.array([[max(r, 1e-9)]], dtype=float)

        innovation = distance - float((h @ self.state)[0, 0])
        innovation_covariance = float((h @ self.covariance @ h.T + r_matrix)[0, 0])
        kalman_gain = (self.covariance @ h.T) / innovation_covariance

        self.state = self.state + kalman_gain * innovation
        identity = np.eye(2, dtype=float)
        self.covariance = (identity - kalman_gain @ h) @ self.covariance
        self._residual = innovation

    @property
    def residual(self) -> float:
        return float(self._residual)

    @property
    def position(self) -> float:
        return float(self.state[0, 0])

    @property
    def velocity(self) -> float:
        return float(self.state[1, 0])


class SensorFusionManager:
    """High level helper blending motion hints and ultrasonic readings."""

    def __init__(self, config: Optional[KalmanConfig] = None) -> None:
        self.config = config or KalmanConfig()
        self.filter = KalmanFilter1D(self.config)
        self._history: Deque[FusedState] = deque(maxlen=self.config.history_size)
        self._last_timestamp: Optional[float] = None
        self._confidence = 0.0
        self._missed_ultrasonic = 0
        self._pending_acceleration = 0.0

    def reset(self) -> None:
        """Reset the fusion filter and internal caches."""

        self.filter = KalmanFilter1D(self.config)
        self._history.clear()
        self._last_timestamp = None
        self._confidence = 0.0
        self._missed_ultrasonic = 0
        self._pending_acceleration = 0.0

    def _compute_dt(self, timestamp: Optional[float]) -> Optional[float]:
        """Determine the integration step based on the provided timestamp."""

        if timestamp is None:
            return self.config.dt

        if self._last_timestamp is None:
            return self.config.dt

        delta = timestamp - self._last_timestamp
        if delta <= 1e-6:
            return None

        return max(delta, 1e-3)

    def _advance_time(self, timestamp: Optional[float], dt: Optional[float]) -> None:
        """Update the cached timestamp after a prediction step."""

        if timestamp is not None:
            self._last_timestamp = timestamp
        elif dt is not None:
            base = self._last_timestamp or 0.0
            self._last_timestamp = base + dt

    def ingest_imu(self, *, acceleration: float, timestamp: Optional[float] = None, variance: Optional[float] = None) -> None:
        """Process a new IMU reading.

        Parameters
        ----------
        acceleration:
            Linear acceleration along the rover's forward axis in metres per
            second squared.
        timestamp:
            Optional monotonic timestamp for the reading.  When supplied the
            integration step is derived from the delta compared to the previous
            update.
        variance:
            Optional variance of the acceleration.  If provided it is blended
            with the configured process noise.
        """

        dt = self._compute_dt(timestamp)

        if dt is not None:
            if variance is not None:
                original_variance = self.config.process_variance
                self.config.process_variance = max(variance, 1e-6)
                self.filter.predict(acceleration, dt=dt)
                self.config.process_variance = original_variance
            else:
                self.filter.predict(acceleration, dt=dt)
            self._advance_time(timestamp, dt)

        self._pending_acceleration = acceleration

        self._update_confidence()
        self._record_state(timestamp)

    def ingest_ultrasonic(
        self,
        *,
        distance: float,
        quality: float = 1.0,
        timestamp: Optional[float] = None,
        variance: Optional[float] = None,
    ) -> None:
        """Process a new ultrasonic reading."""

        dt = self._compute_dt(timestamp)
        if dt is not None:
            self.filter.predict(self._pending_acceleration, dt=dt)
            self._advance_time(timestamp, dt)

        if quality < self.config.min_quality:
            self._missed_ultrasonic += 1
            self._update_confidence()
            self._record_state(timestamp)
            return

        self._missed_ultrasonic = 0
        self.filter.update(distance, variance=variance)

        if timestamp is not None:
            self._last_timestamp = timestamp

        self._update_confidence(boost=True)
        self._record_state(timestamp)

    def _update_confidence(self, *, boost: bool = False) -> None:
        """Update the confidence metric based on covariance and measurement health."""

        position_variance = float(self.filter.covariance[0, 0])
        covariance_score = 1.0 / (1.0 + position_variance)
        covariance_score = max(0.0, min(1.0, covariance_score))

        if self._missed_ultrasonic:
            decay = math.exp(-0.5 * self._missed_ultrasonic)
            covariance_score *= decay

        if boost:
            covariance_score = min(1.0, covariance_score + 0.1)

        self._confidence = covariance_score

    def _record_state(self, timestamp: Optional[float]) -> None:
        state = FusedState(
            position=self.filter.position,
            velocity=self.filter.velocity,
            covariance=self.filter.covariance.copy(),
            residual=self.filter.residual,
            timestamp=timestamp if timestamp is not None else (self._last_timestamp or 0.0),
            quality=self._confidence,
        )
        self._history.append(state)

    def get_state(self) -> Dict[str, float | np.ndarray]:
        """Return the most recent fused state as a dictionary."""

        return {
            "position": self.filter.position,
            "velocity": self.filter.velocity,
            "covariance": self.filter.covariance.copy(),
            "residual": self.filter.residual,
            "quality": self._confidence,
        }

    def history(self) -> Deque[FusedState]:
        """Return a reference to the fused state history for inspection."""

        return self._history
