"""Tests for the sensor fusion module."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from hardware.sensor_fusion import KalmanConfig, SensorFusionManager


def test_sensor_fusion_converges_on_constant_acceleration() -> None:
    config = KalmanConfig(dt=0.1, process_variance=5e-3, measurement_variance=0.02)
    fusion = SensorFusionManager(config)

    acceleration = 0.8
    time = 0.0
    for _ in range(20):
        time += config.dt
        distance = 0.5 * acceleration * time ** 2
        fusion.ingest_imu(acceleration=acceleration, timestamp=time)
        fusion.ingest_ultrasonic(distance=distance, quality=1.0, timestamp=time)

    state = fusion.get_state()
    assert math.isclose(state["position"], distance, rel_tol=0.05)
    assert math.isclose(state["velocity"], acceleration * time, rel_tol=0.05)
    assert state["quality"] > 0.7


def test_sensor_fusion_rejects_low_quality_measurements() -> None:
    fusion = SensorFusionManager(KalmanConfig(min_quality=0.5))

    # Seed the filter with a valid measurement
    fusion.ingest_ultrasonic(distance=1.5, quality=1.0, timestamp=0.0)

    # Attempt to apply a low quality measurement which should be ignored
    fusion.ingest_ultrasonic(distance=10.0, quality=0.2, timestamp=0.1)

    state = fusion.get_state()
    assert math.isclose(state["position"], 1.5, rel_tol=0.2)
    assert state["quality"] < 1.0  # Confidence decays without good readings


def test_sensor_fusion_history_tracks_recent_states() -> None:
    config = KalmanConfig(history_size=5)
    fusion = SensorFusionManager(config)

    current_time = 0.0
    for idx in range(7):
        fusion.ingest_ultrasonic(distance=float(idx), quality=1.0, timestamp=current_time)
        current_time += config.dt

    history = fusion.history()
    assert len(history) == config.history_size
    latest = history[-1]
    assert math.isclose(latest.position, fusion.get_state()["position"], rel_tol=1e-6)
    assert isinstance(latest.covariance, np.ndarray)


def test_sensor_fusion_ultrasonic_only_sequence_estimates_velocity() -> None:
    config = KalmanConfig(dt=0.1, process_variance=1e-2, measurement_variance=0.05)
    fusion = SensorFusionManager(config)

    distances = [2.0, 1.8, 1.6, 1.4, 1.2]
    current_time = 0.0
    for distance in distances:
        fusion.ingest_ultrasonic(distance=distance, quality=1.0, timestamp=current_time)
        current_time += config.dt

    state = fusion.get_state()
    assert state["position"] < distances[0]
    assert state["velocity"] < 0.0
    assert state["quality"] > 0.3
