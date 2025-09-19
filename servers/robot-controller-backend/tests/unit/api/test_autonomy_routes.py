from __future__ import annotations

import asyncio
import sys
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient


PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from api.autonomy_routes import controller, router


@pytest.fixture(autouse=True)
def reset_autonomy_controller():
    """Ensure each test runs with a clean AutonomyController state."""

    asyncio.run(controller.stop())
    yield
    asyncio.run(controller.stop())


@pytest.fixture()
def client() -> TestClient:
    app = FastAPI()
    app.include_router(router)
    with TestClient(app) as test_client:
        yield test_client


def test_status_idle(client: TestClient):
    res = client.get("/autonomy/status")
    assert res.status_code == 200
    body = res.json()
    assert body["status"] == "ok"
    assert body["autonomy"]["active"] is False
    assert body["autonomy"]["mode"] == "idle"


def test_start_update_stop_cycle(client: TestClient):
    res = client.post("/autonomy/start", json={
        "mode": "patrol",
        "params": {"speedPct": 40, "obstacleAvoidance": True},
    })
    assert res.status_code == 200
    body = res.json()
    assert body["autonomy"]["active"] is True
    assert body["autonomy"]["mode"] == "patrol"
    assert body["autonomy"]["params"]["speedPct"] == 40

    res = client.post("/autonomy/update", json={"params": {"speedPct": 55}})
    assert res.status_code == 200
    body = res.json()
    assert body["autonomy"]["params"]["speedPct"] == 55

    res = client.post("/autonomy/stop")
    assert res.status_code == 200
    body = res.json()
    assert body["autonomy"]["active"] is False


def test_set_waypoint(client: TestClient):
    client.post("/autonomy/start", json={"mode": "waypoints", "params": {}})

    res = client.post("/autonomy/set_waypoint", json={
        "label": "Home",
        "lat": 12.345,
        "lon": -98.765,
    })
    assert res.status_code == 200
    waypoint = res.json()["autonomy"]["params"]["lastWaypoint"]
    assert waypoint["label"] == "Home"
    assert waypoint["lat"] == pytest.approx(12.345)
    assert waypoint["lon"] == pytest.approx(-98.765)


def test_dock_switches_mode(client: TestClient):
    client.post("/autonomy/start", json={"mode": "patrol", "params": {}})

    res = client.post("/autonomy/dock")
    assert res.status_code == 200
    body = res.json()
    assert body["autonomy"]["mode"] == "dock"
    assert body["autonomy"]["active"] is True


def test_update_without_session_returns_400(client: TestClient):
    res = client.post("/autonomy/update", json={"params": {"speedPct": 10}})
    assert res.status_code == 400
    detail = res.json()["detail"]
    assert "no autonomy mode" in detail


def test_invalid_mode_returns_400(client: TestClient):
    res = client.post("/autonomy/start", json={"mode": "", "params": {}})
    assert res.status_code == 400
    assert res.json()["detail"]


def test_start_accepts_legacy_line_track_name(client: TestClient):
    res = client.post("/autonomy/start", json={"mode": "line_track", "params": {}})
    assert res.status_code == 200
    body = res.json()
    assert body["status"] == "ok"
    assert body["autonomy"]["mode"] == "line_follow"


def test_modal_parameters_roundtrip(client: TestClient):
    modal_params = {
        "speedPct": 37,
        "aggressiveness": 82,
        "obstacleAvoidance": True,
        "laneKeeping": False,
        "headlights": True,
        "avoidStopDistM": 0.35,
        "line_kP": 1.25,
        "searchYawRate": 45,
        "hsvLow": [5, 120, 110],
        "hsvHigh": [15, 240, 250],
        "minArea": 450,
        "maxArea": 18000,
        "priorities": ["safety", "vision", "waypoints"],
        "gridEnabled": True,
        "gridCellSizeM": 0.2,
        "gridDecay": 0.05,
        "batteryMinPct": 18,
    }

    res = client.post("/autonomy/start", json={"mode": "patrol", "params": modal_params})
    assert res.status_code == 200
    body = res.json()
    params = body["autonomy"]["params"]

    assert params["speedPct"] == modal_params["speedPct"]
    assert params["aggressiveness"] == modal_params["aggressiveness"]
    assert params["obstacleAvoidance"] is modal_params["obstacleAvoidance"]
    assert params["laneKeeping"] is modal_params["laneKeeping"]
    assert params["headlights"] is modal_params["headlights"]
    assert params["priorities"] == modal_params["priorities"]
    assert params["hsvLow"] == modal_params["hsvLow"]
    assert params["hsvHigh"] == modal_params["hsvHigh"]
    assert params["minArea"] == modal_params["minArea"]
    assert params["maxArea"] == modal_params["maxArea"]
    assert params["gridEnabled"] is True
    assert params["batteryMinPct"] == modal_params["batteryMinPct"]
    assert params["gridCellSizeM"] == pytest.approx(modal_params["gridCellSizeM"])
    assert params["gridDecay"] == pytest.approx(modal_params["gridDecay"])
    assert params["avoidStopDistM"] == pytest.approx(modal_params["avoidStopDistM"])
    assert params["line_kP"] == pytest.approx(modal_params["line_kP"])
    assert params["searchYawRate"] == modal_params["searchYawRate"]

    res = client.post("/autonomy/update", json={
        "params": {
            "gridEnabled": False,
            "priorities": ["vision", "edge", "safety"],
            "hsvLow": [10, 200, 210],
        },
    })
    assert res.status_code == 200
    updated = res.json()["autonomy"]["params"]
    assert updated["gridEnabled"] is False
    assert updated["priorities"] == ["vision", "edge", "safety"]
    assert updated["hsvLow"] == [10, 200, 210]
    assert updated["speedPct"] == modal_params["speedPct"]
