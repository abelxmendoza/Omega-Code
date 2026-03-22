"""
Unit tests for api/movement_routes.py — WebSocket movement protocol.

Uses FastAPI's built-in TestClient WebSocket support so no real network
or ROS bridge is required.  The ROS bridge is replaced with a lightweight
spy that records every call made to it.
"""

from __future__ import annotations

import pathlib
import sys
import time
from unittest.mock import MagicMock

ROOT = pathlib.Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from api.movement_routes import router


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

class _BridgeSpy:
    """Minimal ROS bridge spy — records calls, no real ROS needed."""

    def __init__(self, active: bool = True) -> None:
        self.is_active = active
        self.commands: list[tuple[str, float]] = []
        self.twists: list[tuple[float, float]] = []
        self.stopped = False

    def send_command(self, cmd: str, speed: float = 0.5) -> bool:
        self.commands.append((cmd, speed))
        return self.is_active

    def send_twist(self, linear_x: float, angular_z: float) -> bool:
        self.twists.append((linear_x, angular_z))
        return self.is_active

    def stop(self) -> None:
        self.stopped = True


@pytest.fixture()
def app_with_bridge():
    """Return (FastAPI app, bridge spy) with bridge on app.state."""
    spy = _BridgeSpy(active=True)
    application = FastAPI()
    application.include_router(router)
    application.state.ros_bridge = spy
    return application, spy


@pytest.fixture()
def client(app_with_bridge):
    application, spy = app_with_bridge
    with TestClient(application) as tc:
        yield tc, spy


@pytest.fixture()
def client_no_bridge():
    """App with no bridge on state — simulates disabled ROS."""
    application = FastAPI()
    application.include_router(router)
    # Deliberately do NOT set app.state.ros_bridge
    with TestClient(application) as tc:
        yield tc


# ---------------------------------------------------------------------------
# Welcome message
# ---------------------------------------------------------------------------

class TestWelcomeMessage:
    def test_receives_welcome_on_connect(self, client):
        tc, _ = client
        with tc.websocket_connect("/ws/movement") as ws:
            msg = ws.receive_json()
            assert msg["type"] == "welcome"
            assert msg["service"] == "movement"
            assert msg["status"] == "connected"

    def test_welcome_includes_protocol(self, client):
        tc, _ = client
        with tc.websocket_connect("/ws/movement") as ws:
            msg = ws.receive_json()
            assert "protocol" in msg


# ---------------------------------------------------------------------------
# Heartbeat (ping / pong)
# ---------------------------------------------------------------------------

class TestPingPong:
    def test_ping_returns_pong(self, client):
        tc, _ = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()  # consume welcome
            ws.send_json({"type": "ping", "ts": 1_700_000_000_000})
            pong = ws.receive_json()
            assert pong["type"] == "pong"
            assert pong["ts"] == 1_700_000_000_000

    def test_ping_echoes_ts_exactly(self, client):
        tc, _ = client
        ts = int(time.time() * 1000)
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"type": "ping", "ts": ts})
            pong = ws.receive_json()
            assert pong["ts"] == ts


# ---------------------------------------------------------------------------
# Status query
# ---------------------------------------------------------------------------

class TestStatusCommand:
    def test_status_command_returns_status(self, client):
        tc, _ = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": "status"})
            resp = ws.receive_json()
            assert resp["type"] == "status"
            assert "movementV2" in resp
            assert resp["movementV2"]["enabled"] is True

    def test_status_has_timestamp(self, client):
        tc, _ = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": "status"})
            resp = ws.receive_json()
            assert "ts" in resp
            assert isinstance(resp["ts"], int)


# ---------------------------------------------------------------------------
# Discrete movement commands
# ---------------------------------------------------------------------------

class TestMovementCommands:
    @pytest.mark.parametrize("ui_cmd,expected_bridge_cmd", [
        ("move-up",    "forward"),
        ("move-down",  "backward"),
        ("move-left",  "left"),
        ("move-right", "right"),
        ("move-stop",  "stop"),
        ("stop",       "stop"),
    ])
    def test_known_command_acked_ok(self, client, ui_cmd, expected_bridge_cmd):
        tc, spy = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": ui_cmd})
            ack = ws.receive_json()
            assert ack["type"] == "ack"
            assert ack["action"] == ui_cmd
            assert ack["status"] == "ok"

    @pytest.mark.parametrize("ui_cmd,expected_bridge_cmd", [
        ("move-up",    "forward"),
        ("move-down",  "backward"),
        ("move-left",  "left"),
        ("move-right", "right"),
        ("stop",       "stop"),
    ])
    def test_bridge_receives_correct_command(self, client, ui_cmd, expected_bridge_cmd):
        tc, spy = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": ui_cmd, "speed": 0.8})
            ws.receive_json()
        assert any(cmd == expected_bridge_cmd for cmd, _ in spy.commands)

    def test_speed_clamped_to_zero_one(self, client):
        tc, spy = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": "move-up", "speed": 999.0})
            ws.receive_json()
        _, recorded_speed = spy.commands[-1]
        assert recorded_speed <= 1.0

    def test_ack_contains_ros_sent_flag(self, client):
        tc, spy = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": "move-up"})
            ack = ws.receive_json()
        assert "ros_sent" in ack
        assert ack["ros_sent"] is True  # spy.is_active == True

    def test_ros_sent_false_when_bridge_inactive(self, app_with_bridge):
        application, spy = app_with_bridge
        spy.is_active = False
        with TestClient(application) as tc:
            with tc.websocket_connect("/ws/movement") as ws:
                ws.receive_json()
                ws.send_json({"command": "move-up"})
                ack = ws.receive_json()
        assert ack["ros_sent"] is False

    def test_unknown_command_acked_error(self, client):
        tc, _ = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": "fly"})
            ack = ws.receive_json()
            assert ack["type"] == "ack"
            assert ack["status"] == "error"
            assert "unknown command" in ack["error"]

    def test_no_bridge_still_acks(self, client_no_bridge):
        with client_no_bridge.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": "move-up"})
            ack = ws.receive_json()
            assert ack["type"] == "ack"
            assert ack["status"] == "ok"
            assert ack["ros_sent"] is False


# ---------------------------------------------------------------------------
# Twist (proportional velocity from gamepad)
# ---------------------------------------------------------------------------

class TestTwistCommand:
    def test_twist_acked(self, client):
        tc, spy = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": "twist", "linear_x": 0.5, "angular_z": -0.3})
            ack = ws.receive_json()
            assert ack["type"] == "ack"
            assert ack["action"] == "twist"
            assert ack["status"] == "ok"

    def test_twist_forwarded_to_bridge(self, client):
        tc, spy = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": "twist", "linear_x": 0.7, "angular_z": 0.2})
            ws.receive_json()
        assert len(spy.twists) == 1
        linear_x, angular_z = spy.twists[0]
        assert linear_x == pytest.approx(0.7)
        assert angular_z == pytest.approx(0.2)

    def test_twist_clamped_to_minus_one_plus_one(self, client):
        tc, spy = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": "twist", "linear_x": 5.0, "angular_z": -10.0})
            ws.receive_json()
        linear_x, angular_z = spy.twists[0]
        assert -1.0 <= linear_x <= 1.0
        assert -1.0 <= angular_z <= 1.0

    def test_twist_defaults_zeros_when_omitted(self, client):
        tc, spy = client
        with tc.websocket_connect("/ws/movement") as ws:
            ws.receive_json()
            ws.send_json({"command": "twist"})
            ws.receive_json()
        linear_x, angular_z = spy.twists[0]
        assert linear_x == 0.0
        assert angular_z == 0.0
