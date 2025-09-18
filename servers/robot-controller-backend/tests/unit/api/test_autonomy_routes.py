from __future__ import annotations

import asyncio

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

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
