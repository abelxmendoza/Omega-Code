"""Unit tests for movement/pose_ekf.py (SE2PoseFilter).

Tests cover: predict kinematics, covariance growth, ArUco update gating,
EKF correction direction, quality scoring, reset, thread safety.
"""

from __future__ import annotations

import math
import sys
import os
import threading
import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

BACKEND_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
if BACKEND_ROOT not in sys.path:
    sys.path.insert(0, BACKEND_ROOT)

from movement.pose_ekf import MarkerPose, SE2Pose, SE2PoseFilter


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_filter(**kwargs) -> SE2PoseFilter:
    """Return a fresh filter with tight noise (easier to assert exact moves)."""
    Q = kwargs.pop("Q", np.diag([1e-6, 1e-6, 1e-6]))
    R = kwargs.pop("R", np.diag([1e-6, 1e-6, 1e-6]))
    return SE2PoseFilter(Q=Q, R=R, **kwargs)


# ---------------------------------------------------------------------------
# predict() — kinematics
# ---------------------------------------------------------------------------

class TestPredict:
    def test_forward_increments_x(self):
        filt = _make_filter(initial_pose=(0.0, 0.0, 0.0))
        filt.predict(v=1.0, omega=0.0, dt=0.5)
        pose = filt.get_pose()
        assert pose.x == pytest.approx(0.5, abs=1e-4)
        assert pose.y == pytest.approx(0.0, abs=1e-4)

    def test_backward_decrements_x(self):
        filt = _make_filter(initial_pose=(1.0, 0.0, 0.0))
        filt.predict(v=-0.4, omega=0.0, dt=1.0)
        pose = filt.get_pose()
        assert pose.x == pytest.approx(0.6, abs=1e-4)

    def test_rotate_left_increments_theta(self):
        filt = _make_filter(initial_pose=(0.0, 0.0, 0.0))
        filt.predict(v=0.0, omega=math.pi, dt=0.5)
        pose = filt.get_pose()
        assert pose.theta == pytest.approx(math.pi / 2, abs=1e-4)

    def test_rotate_right_decrements_theta(self):
        filt = _make_filter(initial_pose=(0.0, 0.0, 0.0))
        filt.predict(v=0.0, omega=-math.pi, dt=0.5)
        pose = filt.get_pose()
        assert pose.theta == pytest.approx(-math.pi / 2, abs=1e-4)

    def test_heading_90deg_moves_along_y(self):
        filt = _make_filter(initial_pose=(0.0, 0.0, math.pi / 2))
        filt.predict(v=1.0, omega=0.0, dt=1.0)
        pose = filt.get_pose()
        assert pose.x == pytest.approx(0.0, abs=1e-4)
        assert pose.y == pytest.approx(1.0, abs=1e-4)

    def test_zero_dt_is_noop(self):
        filt = _make_filter(initial_pose=(3.0, 4.0, 0.5))
        filt.predict(v=99.0, omega=99.0, dt=0.0)
        pose = filt.get_pose()
        assert pose.x == pytest.approx(3.0, abs=1e-9)
        assert pose.y == pytest.approx(4.0, abs=1e-9)
        assert pose.theta == pytest.approx(0.5, abs=1e-9)

    def test_negative_dt_is_noop(self):
        filt = _make_filter(initial_pose=(1.0, 2.0, 0.0))
        filt.predict(v=5.0, omega=5.0, dt=-1.0)
        pose = filt.get_pose()
        assert pose.x == pytest.approx(1.0, abs=1e-9)

    def test_theta_wraps_past_pi(self):
        filt = _make_filter(initial_pose=(0.0, 0.0, math.pi - 0.01))
        filt.predict(v=0.0, omega=1.0, dt=0.1)
        pose = filt.get_pose()
        # Should wrap into [-π, π]
        assert -math.pi <= pose.theta <= math.pi

    def test_covariance_grows_without_correction(self):
        """Repeated predicts without any update should increase covariance trace."""
        filt = SE2PoseFilter()   # default Q/R
        trace_before = float(np.trace(np.array(filt.get_pose().covariance)))
        for _ in range(20):
            filt.predict(v=0.1, omega=0.0, dt=0.05)
        trace_after = float(np.trace(np.array(filt.get_pose().covariance)))
        assert trace_after > trace_before


# ---------------------------------------------------------------------------
# update_from_aruco() — gating and correction direction
# ---------------------------------------------------------------------------

class TestUpdateFromAruco:
    def test_unknown_marker_returns_false(self):
        filt = SE2PoseFilter(marker_map={})
        dummy_rvec = np.zeros(3)
        dummy_tvec = np.array([0.0, 0.0, 1.0])
        result = filt.update_from_aruco(99, dummy_rvec, dummy_tvec)
        assert result is False

    def test_unknown_marker_does_not_change_pose(self):
        filt = SE2PoseFilter(marker_map={}, initial_pose=(1.0, 2.0, 0.5))
        filt.update_from_aruco(99, np.zeros(3), np.zeros(3))
        pose = filt.get_pose()
        assert pose.x == pytest.approx(1.0, abs=1e-6)
        assert pose.y == pytest.approx(2.0, abs=1e-6)

    def test_known_marker_returns_true_when_cv2_available(self):
        """Requires OpenCV. Skipped gracefully if not installed."""
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not installed")

        marker_map = {7: MarkerPose(x=1.0, y=0.0, alpha=math.pi)}
        filt = SE2PoseFilter(marker_map=marker_map, initial_pose=(0.0, 0.0, 0.0))
        # rvec pointing straight (identity rotation), tvec ~1m in Z
        rvec = np.zeros(3)
        tvec = np.array([0.0, 0.0, 1.0])
        result = filt.update_from_aruco(7, rvec, tvec)
        assert result is True

    def test_known_marker_increments_correction_count(self):
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not installed")

        marker_map = {5: MarkerPose(x=0.0, y=0.0, alpha=0.0)}
        filt = SE2PoseFilter(marker_map=marker_map)
        assert filt.correction_count == 0
        filt.update_from_aruco(5, np.zeros(3), np.array([0.0, 0.0, 1.0]))
        assert filt.correction_count == 1

    def test_known_marker_sets_last_marker_seen(self):
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not installed")

        marker_map = {42: MarkerPose(x=0.0, y=0.0, alpha=0.0)}
        filt = SE2PoseFilter(marker_map=marker_map)
        assert filt.last_marker_seen is None
        filt.update_from_aruco(42, np.zeros(3), np.array([0.0, 0.0, 0.5]))
        assert filt.last_marker_seen == 42

    def test_correction_reduces_covariance_trace(self):
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not installed")

        marker_map = {1: MarkerPose(x=0.0, y=0.0, alpha=0.0)}
        filt = SE2PoseFilter(marker_map=marker_map)
        # Inflate covariance with many predicts
        for _ in range(50):
            filt.predict(v=0.0, omega=0.0, dt=0.05)
        trace_before = float(np.trace(np.array(filt.get_pose().covariance)))
        filt.update_from_aruco(1, np.zeros(3), np.array([0.0, 0.0, 1.0]))
        trace_after = float(np.trace(np.array(filt.get_pose().covariance)))
        assert trace_after < trace_before

    def test_bad_rvec_shape_returns_false(self):
        """Malformed rvec → exception caught, returns False."""
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not installed")

        marker_map = {3: MarkerPose(x=0.0, y=0.0, alpha=0.0)}
        filt = SE2PoseFilter(marker_map=marker_map)
        # Pass a 2-element vector to trigger cv2.Rodrigues failure
        result = filt.update_from_aruco(3, np.zeros(2), np.zeros(3))
        assert result is False


# ---------------------------------------------------------------------------
# get_pose() — return shape and quality
# ---------------------------------------------------------------------------

class TestGetPose:
    def test_returns_se2pose(self):
        filt = SE2PoseFilter()
        pose = filt.get_pose()
        assert isinstance(pose, SE2Pose)

    def test_initial_pose_matches_constructor(self):
        filt = SE2PoseFilter(initial_pose=(1.5, -2.0, 0.3))
        pose = filt.get_pose()
        assert pose.x == pytest.approx(1.5, abs=1e-6)
        assert pose.y == pytest.approx(-2.0, abs=1e-6)
        assert pose.theta == pytest.approx(0.3, abs=1e-6)

    def test_covariance_is_3x3_list(self):
        filt = SE2PoseFilter()
        cov = filt.get_pose().covariance
        assert len(cov) == 3
        assert all(len(row) == 3 for row in cov)

    def test_quality_between_0_and_1(self):
        filt = SE2PoseFilter()
        quality = filt.get_pose().quality
        assert 0.0 <= quality <= 1.0

    def test_quality_capped_at_half_before_aruco_fix(self):
        """Without any ArUco correction, quality must not exceed 0.5."""
        filt = SE2PoseFilter()
        quality = filt.get_pose().quality
        assert quality <= 0.5

    def test_timestamp_is_recent(self):
        before = time.monotonic()
        filt = SE2PoseFilter()
        pose = filt.get_pose()
        after = time.monotonic()
        assert before <= pose.timestamp <= after + 0.1


# ---------------------------------------------------------------------------
# reset()
# ---------------------------------------------------------------------------

class TestReset:
    def test_reset_clears_state(self):
        filt = _make_filter(initial_pose=(5.0, 5.0, 1.0))
        filt.predict(v=1.0, omega=0.5, dt=0.1)
        filt.reset(x=0.0, y=0.0, theta=0.0)
        pose = filt.get_pose()
        assert pose.x == pytest.approx(0.0, abs=1e-6)
        assert pose.y == pytest.approx(0.0, abs=1e-6)
        assert pose.theta == pytest.approx(0.0, abs=1e-6)

    def test_reset_with_nonzero_pose(self):
        filt = SE2PoseFilter()
        filt.reset(x=3.0, y=-1.0, theta=math.pi / 4)
        pose = filt.get_pose()
        assert pose.x == pytest.approx(3.0, abs=1e-6)
        assert pose.y == pytest.approx(-1.0, abs=1e-6)

    def test_reset_clears_correction_count(self):
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not installed")

        marker_map = {1: MarkerPose(x=0.0, y=0.0, alpha=0.0)}
        filt = SE2PoseFilter(marker_map=marker_map)
        filt.update_from_aruco(1, np.zeros(3), np.array([0.0, 0.0, 1.0]))
        assert filt.correction_count == 1
        filt.reset()
        assert filt.correction_count == 0


# ---------------------------------------------------------------------------
# set_marker_map()
# ---------------------------------------------------------------------------

class TestSetMarkerMap:
    def test_set_marker_map_replaces_existing(self):
        filt = SE2PoseFilter(marker_map={1: MarkerPose(0, 0, 0)})
        filt.set_marker_map({2: MarkerPose(1.0, 2.0, 0.5)})
        # Marker 1 should now be unknown
        result = filt.update_from_aruco(1, np.zeros(3), np.zeros(3))
        assert result is False

    def test_set_marker_map_empty_disables_all(self):
        filt = SE2PoseFilter(marker_map={7: MarkerPose(1.0, 0.0, 0.0)})
        filt.set_marker_map({})
        result = filt.update_from_aruco(7, np.zeros(3), np.zeros(3))
        assert result is False


# ---------------------------------------------------------------------------
# Thread safety
# ---------------------------------------------------------------------------

class TestThreadSafety:
    def test_concurrent_predict_and_get_pose_do_not_raise(self):
        """Hammer predict() and get_pose() from multiple threads simultaneously."""
        filt = SE2PoseFilter()
        errors: list = []

        def _predict_loop():
            try:
                for _ in range(100):
                    filt.predict(v=0.1, omega=0.05, dt=0.05)
            except Exception as exc:
                errors.append(exc)

        def _read_loop():
            try:
                for _ in range(100):
                    _ = filt.get_pose()
            except Exception as exc:
                errors.append(exc)

        threads = [
            threading.Thread(target=_predict_loop) for _ in range(4)
        ] + [
            threading.Thread(target=_read_loop) for _ in range(2)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)

        assert errors == [], f"Exceptions raised in threads: {errors}"

    def test_concurrent_reset_and_predict_do_not_corrupt_state(self):
        """Reset mid-predict should not leave NaN in state."""
        filt = SE2PoseFilter()
        stop_event = threading.Event()

        def _predict():
            while not stop_event.is_set():
                filt.predict(v=0.3, omega=0.1, dt=0.05)

        def _reset():
            for _ in range(20):
                filt.reset(x=0.0, y=0.0, theta=0.0)
                time.sleep(0.001)

        t1 = threading.Thread(target=_predict)
        t2 = threading.Thread(target=_reset)
        t1.start(); t2.start()
        t2.join(timeout=2.0)
        stop_event.set()
        t1.join(timeout=2.0)

        pose = filt.get_pose()
        assert not math.isnan(pose.x)
        assert not math.isnan(pose.y)
        assert not math.isnan(pose.theta)
