"""Integration tests for api/localization_routes.py.

Uses FastAPI TestClient (no live server needed).  The SE2PoseFilter is
real (no mock) so each test exercises the full request → filter → response
path without touching hardware.
"""

from __future__ import annotations

import math
import pathlib
import sys
from typing import Any, Dict

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fastapi import FastAPI
from fastapi.testclient import TestClient

# Import the router (this also instantiates the singleton _filter)
import api.localization_routes as loc_mod
from api.localization_routes import router, _filter
from movement.pose_ekf import MarkerPose


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def _reset_filter_and_commands():
    """Reset the singleton filter and velocity state before every test."""
    _filter.reset(x=0.0, y=0.0, theta=0.0)
    loc_mod._last_v = 0.0
    loc_mod._last_omega = 0.0
    loc_mod._last_speed_fraction = 0.0
    yield
    _filter.reset(x=0.0, y=0.0, theta=0.0)


@pytest.fixture(scope="module")
def client():
    app = FastAPI()
    app.include_router(router)
    return TestClient(app)


# ---------------------------------------------------------------------------
# GET /localization/pose
# ---------------------------------------------------------------------------

class TestGetPose:
    def test_returns_200(self, client):
        resp = client.get("/localization/pose")
        assert resp.status_code == 200

    def test_response_has_required_fields(self, client):
        data = client.get("/localization/pose").json()
        for key in ("x", "y", "theta_rad", "theta_deg", "covariance", "quality",
                    "correction_count", "last_marker_seen", "ts"):
            assert key in data, f"Missing key: {key}"

    def test_initial_pose_is_origin(self, client):
        data = client.get("/localization/pose").json()
        assert data["x"] == pytest.approx(0.0, abs=1e-6)
        assert data["y"] == pytest.approx(0.0, abs=1e-6)
        assert data["theta_rad"] == pytest.approx(0.0, abs=1e-6)

    def test_theta_deg_consistent_with_theta_rad(self, client):
        data = client.get("/localization/pose").json()
        expected_deg = math.degrees(data["theta_rad"])
        assert data["theta_deg"] == pytest.approx(expected_deg, abs=1e-4)

    def test_quality_between_0_and_1(self, client):
        data = client.get("/localization/pose").json()
        assert 0.0 <= data["quality"] <= 1.0

    def test_covariance_is_3x3(self, client):
        cov = client.get("/localization/pose").json()["covariance"]
        assert len(cov) == 3
        assert all(len(row) == 3 for row in cov)

    def test_correction_count_starts_at_zero(self, client):
        data = client.get("/localization/pose").json()
        assert data["correction_count"] == 0

    def test_last_marker_seen_starts_null(self, client):
        data = client.get("/localization/pose").json()
        assert data["last_marker_seen"] is None


# ---------------------------------------------------------------------------
# POST /localization/reset
# ---------------------------------------------------------------------------

class TestResetPose:
    def test_reset_to_origin(self, client):
        resp = client.post("/localization/reset", json={"x": 0.0, "y": 0.0, "theta": 0.0})
        assert resp.status_code == 200
        data = resp.json()
        assert data["ok"] is True
        assert data["pose"]["x"] == pytest.approx(0.0)
        assert data["pose"]["y"] == pytest.approx(0.0)
        assert data["pose"]["theta_rad"] == pytest.approx(0.0)

    def test_reset_to_nonzero_pose(self, client):
        resp = client.post("/localization/reset",
                           json={"x": 3.5, "y": -1.2, "theta": 1.57})
        assert resp.status_code == 200
        data = resp.json()
        assert data["pose"]["x"] == pytest.approx(3.5, abs=1e-4)
        assert data["pose"]["y"] == pytest.approx(-1.2, abs=1e-4)
        assert data["pose"]["theta_rad"] == pytest.approx(1.57, abs=1e-4)

    def test_reset_is_reflected_in_get_pose(self, client):
        client.post("/localization/reset", json={"x": 5.0, "y": 0.0, "theta": 0.0})
        data = client.get("/localization/pose").json()
        assert data["x"] == pytest.approx(5.0, abs=1e-4)

    def test_reset_default_values(self, client):
        """POST with empty body uses defaults x=y=theta=0."""
        resp = client.post("/localization/reset", json={})
        assert resp.status_code == 200
        data = resp.json()
        assert data["pose"]["x"] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# POST /localization/command
# ---------------------------------------------------------------------------

class TestCommandUpdate:
    def test_forward_sets_positive_v(self, client):
        resp = client.post("/localization/command",
                           json={"command": "forward", "speed_fraction": 1.0})
        assert resp.status_code == 200
        data = resp.json()
        assert data["ok"] is True
        assert data["v"] > 0
        assert data["omega"] == pytest.approx(0.0, abs=1e-9)

    def test_backward_sets_negative_v(self, client):
        data = client.post("/localization/command",
                           json={"command": "backward", "speed_fraction": 1.0}).json()
        assert data["v"] < 0

    def test_left_sets_positive_omega(self, client):
        data = client.post("/localization/command",
                           json={"command": "left", "speed_fraction": 1.0}).json()
        assert data["omega"] > 0
        assert data["v"] == pytest.approx(0.0, abs=1e-9)

    def test_right_sets_negative_omega(self, client):
        data = client.post("/localization/command",
                           json={"command": "right", "speed_fraction": 1.0}).json()
        assert data["omega"] < 0

    def test_stop_zeros_velocity(self, client):
        client.post("/localization/command",
                    json={"command": "forward", "speed_fraction": 1.0})
        data = client.post("/localization/command",
                           json={"command": "stop", "speed_fraction": 1.0}).json()
        assert data["v"] == pytest.approx(0.0, abs=1e-9)
        assert data["omega"] == pytest.approx(0.0, abs=1e-9)

    def test_unknown_command_treated_as_stop(self, client):
        data = client.post("/localization/command",
                           json={"command": "moonwalk", "speed_fraction": 1.0}).json()
        assert data["v"] == pytest.approx(0.0, abs=1e-9)
        assert data["omega"] == pytest.approx(0.0, abs=1e-9)

    def test_speed_fraction_scales_velocity(self, client):
        half = client.post("/localization/command",
                           json={"command": "forward", "speed_fraction": 0.5}).json()
        full = client.post("/localization/command",
                           json={"command": "forward", "speed_fraction": 1.0}).json()
        assert full["v"] == pytest.approx(half["v"] * 2.0, abs=1e-6)

    def test_zero_speed_fraction_gives_zero_output(self, client):
        data = client.post("/localization/command",
                           json={"command": "forward", "speed_fraction": 0.0}).json()
        assert data["v"] == pytest.approx(0.0, abs=1e-9)

    def test_command_case_insensitive(self, client):
        data = client.post("/localization/command",
                           json={"command": "FORWARD", "speed_fraction": 1.0}).json()
        assert data["v"] > 0

    def test_move_aliases_work(self, client):
        """'move-up' should map the same as 'forward'."""
        fwd = client.post("/localization/command",
                          json={"command": "forward", "speed_fraction": 1.0}).json()
        up = client.post("/localization/command",
                         json={"command": "move-up", "speed_fraction": 1.0}).json()
        assert fwd["v"] == pytest.approx(up["v"], abs=1e-9)


# ---------------------------------------------------------------------------
# POST /localization/aruco_update
# ---------------------------------------------------------------------------

class TestArucoUpdate:
    def test_unknown_marker_returns_update_applied_false(self, client):
        _filter.set_marker_map({})
        resp = client.post("/localization/aruco_update",
                           json={"marker_id": 99,
                                 "rvec": [0.0, 0.0, 0.0],
                                 "tvec": [0.0, 0.0, 1.0]})
        assert resp.status_code == 200
        data = resp.json()
        assert data["ok"] is True
        assert data["update_applied"] is False
        assert data["reason"] == "marker_not_in_map"

    def test_response_always_includes_pose(self, client):
        resp = client.post("/localization/aruco_update",
                           json={"marker_id": 999,
                                 "rvec": [0.0, 0.0, 0.0],
                                 "tvec": [0.0, 0.0, 1.0]})
        data = resp.json()
        assert "pose" in data
        assert "x" in data["pose"]
        assert "quality" in data["pose"]

    def test_known_marker_returns_update_applied_true(self, client):
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not installed")

        _filter.set_marker_map({7: MarkerPose(x=1.0, y=0.0, alpha=math.pi)})
        resp = client.post("/localization/aruco_update",
                           json={"marker_id": 7,
                                 "rvec": [0.0, 0.0, 0.0],
                                 "tvec": [0.0, 0.0, 1.0]})
        data = resp.json()
        assert data["update_applied"] is True
        assert data["reason"] == "ok"
        assert data["correction_count"] >= 1

    def test_correction_count_increments(self, client):
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not installed")

        _filter.set_marker_map({5: MarkerPose(x=0.0, y=0.0, alpha=0.0)})
        payload = {"marker_id": 5, "rvec": [0.0, 0.0, 0.0], "tvec": [0.0, 0.0, 0.5]}
        r1 = client.post("/localization/aruco_update", json=payload).json()
        r2 = client.post("/localization/aruco_update", json=payload).json()
        assert r2["correction_count"] == r1["correction_count"] + 1

    def test_bad_rvec_length_returns_422(self, client):
        """Pydantic should reject rvec with wrong length."""
        resp = client.post("/localization/aruco_update",
                           json={"marker_id": 1,
                                 "rvec": [0.0, 0.0],  # only 2 elements
                                 "tvec": [0.0, 0.0, 1.0]})
        assert resp.status_code == 422

    def test_optional_timestamp_accepted(self, client):
        resp = client.post("/localization/aruco_update",
                           json={"marker_id": 99,
                                 "rvec": [0.0, 0.0, 0.0],
                                 "tvec": [0.0, 0.0, 1.0],
                                 "timestamp": 1234567890.0})
        assert resp.status_code == 200


# ---------------------------------------------------------------------------
# POST /localization/marker_map
# ---------------------------------------------------------------------------

class TestMarkerMap:
    def test_update_marker_map_returns_ok(self, client):
        resp = client.post("/localization/marker_map",
                           json={"7": {"x": 1.0, "y": 0.0, "alpha": 3.14159}})
        assert resp.status_code == 200
        data = resp.json()
        assert data["ok"] is True
        assert data["marker_count"] == 1

    def test_update_multiple_markers(self, client):
        resp = client.post("/localization/marker_map",
                           json={
                               "7":  {"x": 1.0, "y": 0.0, "alpha": 3.14159},
                               "12": {"x": 0.0, "y": 2.0, "alpha": 1.5708},
                           })
        data = resp.json()
        assert data["marker_count"] == 2

    def test_empty_map_disables_corrections(self, client):
        client.post("/localization/marker_map", json={})
        resp = client.post("/localization/aruco_update",
                           json={"marker_id": 7,
                                 "rvec": [0.0, 0.0, 0.0],
                                 "tvec": [0.0, 0.0, 1.0]})
        data = resp.json()
        assert data["update_applied"] is False

    def test_update_marker_map_enables_subsequent_update(self, client):
        try:
            import cv2
        except ImportError:
            pytest.skip("OpenCV not installed")

        client.post("/localization/marker_map",
                    json={"3": {"x": 0.0, "y": 0.0, "alpha": 0.0}})
        resp = client.post("/localization/aruco_update",
                           json={"marker_id": 3,
                                 "rvec": [0.0, 0.0, 0.0],
                                 "tvec": [0.0, 0.0, 1.0]})
        data = resp.json()
        assert data["update_applied"] is True


# ---------------------------------------------------------------------------
# GET /localization/status
# ---------------------------------------------------------------------------

class TestLocalizationStatus:
    def test_returns_200(self, client):
        assert client.get("/localization/status").status_code == 200

    def test_has_required_fields(self, client):
        data = client.get("/localization/status").json()
        for key in ("filter_active", "predict_hz", "last_v", "last_omega",
                    "correction_count", "last_marker_seen",
                    "pose_quality", "covariance_trace"):
            assert key in data, f"Missing key: {key}"

    def test_filter_active_is_true(self, client):
        data = client.get("/localization/status").json()
        assert data["filter_active"] is True

    def test_predict_hz_matches_module_constant(self, client):
        data = client.get("/localization/status").json()
        assert data["predict_hz"] == loc_mod.PREDICT_HZ

    def test_covariance_trace_positive(self, client):
        data = client.get("/localization/status").json()
        assert data["covariance_trace"] > 0.0
