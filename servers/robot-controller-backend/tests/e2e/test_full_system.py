"""End-to-end tests covering the public HTTP APIs."""

from __future__ import annotations

import asyncio
from typing import Iterator, List

import pytest
from fastapi.testclient import TestClient

from api import autonomy_routes, lighting_routes
from autonomy import build_default_controller
from main_api import app as backend_app
from servers.gateway_api import app as gateway_app


@pytest.fixture
def api_client(monkeypatch) -> Iterator[TestClient]:
    """Provide a FastAPI test client wired up with a fresh controller."""

    controller = build_default_controller()
    monkeypatch.setattr(autonomy_routes, "controller", controller)
    with TestClient(backend_app) as client:
        yield client
    asyncio.run(controller.stop())


@pytest.fixture
def gateway_client() -> Iterator[TestClient]:
    with TestClient(gateway_app) as client:
        yield client


def test_autonomy_flow_end_to_end(api_client: TestClient) -> None:
    status = api_client.get("/autonomy/status")
    assert status.status_code == 200
    payload = status.json()
    assert payload["status"] == "ok"
    autonomy = payload["autonomy"]
    assert autonomy["active"] is False
    assert "patrol" in autonomy["availableModes"]

    start = api_client.post("/autonomy/start", json={"mode": "patrol", "params": {"speed": 1}})
    assert start.status_code == 200
    autonomy = start.json()["autonomy"]
    assert autonomy["active"] is True
    assert autonomy["mode"] == "patrol"
    assert autonomy["params"]["speed"] == 1

    updated = api_client.post("/autonomy/update", json={"params": {"speed": 3}})
    assert updated.status_code == 200
    assert updated.json()["autonomy"]["params"]["speed"] == 3

    docked = api_client.post("/autonomy/dock")
    assert docked.status_code == 200
    assert docked.json()["autonomy"]["mode"] == "dock"

    stopped = api_client.post("/autonomy/stop")
    assert stopped.status_code == 200
    stop_state = stopped.json()["autonomy"]
    assert stop_state["active"] is False
    assert stop_state["mode"] == "idle"


def test_autonomy_waypoint_round_trip(api_client: TestClient) -> None:
    start = api_client.post("/autonomy/start", json={"mode": "waypoints", "params": {"speed": 0.5}})
    assert start.status_code == 200

    response = api_client.post(
        "/autonomy/set_waypoint",
        json={"label": "Base", "lat": 41.5, "lon": -71.2},
    )
    assert response.status_code == 200
    waypoint = response.json()["autonomy"]["params"]["lastWaypoint"]
    assert waypoint["label"] == "Base"
    assert waypoint["lat"] == pytest.approx(41.5)
    assert waypoint["lon"] == pytest.approx(-71.2)
    assert waypoint["ts"] >= response.json()["autonomy"]["startedAt"]


def test_lighting_routes_invoke_subprocess(api_client: TestClient, monkeypatch) -> None:
    calls: List[List[str]] = []

    def fake_run(cmd: List[str], check: bool = True) -> None:
        calls.append(cmd)

    monkeypatch.setattr(lighting_routes.subprocess, "run", fake_run)

    on = api_client.get("/lighting/light/on")
    off = api_client.get("/lighting/light/off")

    assert on.status_code == 200
    assert off.status_code == 200
    assert on.json()["status"] == "success"
    assert off.json()["status"] == "success"
    assert calls[0][:2] == ["python3", "controllers/lighting/led_control.py"]
    assert calls[0][-1] == "--on"
    assert calls[1][-1] == "--off"


def test_gateway_health_and_network_endpoints(gateway_client: TestClient) -> None:
    health = gateway_client.get("/health")
    assert health.status_code == 200
    assert health.text == "ok"

    summary = gateway_client.get("/api/net/summary")
    assert summary.status_code == 200
    payload = summary.json()
    assert payload["online"] is True
    assert payload["ssid"]

    wifi = gateway_client.post("/api/net/wifi/connect", json={"ssid": "TestNet"})
    assert wifi.status_code == 200
    assert "TestNet" in wifi.json()["message"]

    pan = gateway_client.post("/api/net/pan/connect", json={})
    assert pan.status_code == 200
    assert "PAN" in pan.json()["message"]


def test_gateway_video_feed_stream_produces_chunks(gateway_client: TestClient) -> None:
    with gateway_client.stream("GET", "/video_feed") as response:
        assert response.status_code == 200
        assert "multipart/x-mixed-replace" in response.headers["content-type"]
        chunk = next(response.iter_bytes())
        assert b"--frame" in chunk
