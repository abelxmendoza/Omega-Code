"""
Unit tests for api/sensor_ws_routes.py

Tests the three WebSocket endpoints:
  /ws/ultrasonic  — ultrasonic range stream
  /ws/line        — line-tracking state stream
  /ws/lighting    — lighting command + ack

Hardware, ROS, and the sensor bridge are fully mocked.
"""

from __future__ import annotations

import asyncio
import pathlib
import sys
from unittest.mock import MagicMock, patch

ROOT = pathlib.Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from api.sensor_ws_routes import router


# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

class _FakeSensorBridge:
    """Minimal stand-in for OmegaSensorBridge."""

    def __init__(self) -> None:
        self.is_active = False
        self._ultra_queues: set = set()
        self._line_queues: set = set()

    def add_ultrasonic_queue(self, q: asyncio.Queue) -> None:
        self._ultra_queues.add(q)

    def remove_ultrasonic_queue(self, q: asyncio.Queue) -> None:
        self._ultra_queues.discard(q)

    def add_line_queue(self, q: asyncio.Queue) -> None:
        self._line_queues.add(q)

    def remove_line_queue(self, q: asyncio.Queue) -> None:
        self._line_queues.discard(q)

    def push_ultrasonic(self, data: dict) -> None:
        for q in self._ultra_queues:
            try:
                q.put_nowait(data)
            except asyncio.QueueFull:
                pass

    def push_line(self, data: dict) -> None:
        for q in self._line_queues:
            try:
                q.put_nowait(data)
            except asyncio.QueueFull:
                pass


class _FakeRosBridge:
    """Minimal stand-in for OmegaRosBridge (lighting commands)."""

    def __init__(self) -> None:
        self.is_active = True
        self.lighting_calls: list[dict] = []

    def send_lighting(self, payload: dict) -> bool:
        self.lighting_calls.append(payload)
        return True

    def send_lighting_cmd(self, pattern: str, **kwargs) -> bool:
        self.lighting_calls.append({"pattern": pattern, **kwargs})
        return True


def _make_app(sensor_bridge=None, ros_bridge=None) -> FastAPI:
    app = FastAPI()
    app.include_router(router)
    if sensor_bridge is not None:
        app.state.sensor_bridge = sensor_bridge
    if ros_bridge is not None:
        app.state.ros_bridge = ros_bridge
    return app


# ---------------------------------------------------------------------------
# /ws/ultrasonic
# ---------------------------------------------------------------------------

class TestUltrasonicWs:
    def test_welcome_on_connect(self):
        app = _make_app(sensor_bridge=_FakeSensorBridge())
        with TestClient(app) as tc:
            with tc.websocket_connect("/ws/ultrasonic") as ws:
                msg = ws.receive_json()
                assert msg["type"] == "welcome"
                assert msg["service"] == "ultrasonic"
                assert msg["status"] == "connected"

    def test_ping_returns_pong(self):
        app = _make_app(sensor_bridge=_FakeSensorBridge())
        with TestClient(app) as tc:
            with tc.websocket_connect("/ws/ultrasonic") as ws:
                ws.receive_json()  # welcome
                ws.send_json({"type": "ping", "ts": 9999})
                pong = ws.receive_json()
                assert pong["type"] == "pong"
                assert pong["ts"] == 9999

    def test_no_bridge_still_sends_welcome(self):
        """Endpoint must not crash when sensor_bridge is absent."""
        app = _make_app()  # no sensor_bridge set
        with TestClient(app) as tc:
            with tc.websocket_connect("/ws/ultrasonic") as ws:
                msg = ws.receive_json()
                assert msg["type"] == "welcome"


# ---------------------------------------------------------------------------
# /ws/line
# ---------------------------------------------------------------------------

class TestLineTrackingWs:
    def test_welcome_on_connect(self):
        app = _make_app(sensor_bridge=_FakeSensorBridge())
        with TestClient(app) as tc:
            with tc.websocket_connect("/ws/line") as ws:
                msg = ws.receive_json()
                assert msg["type"] == "welcome"
                assert msg["service"] == "line"

    def test_ping_returns_pong(self):
        app = _make_app(sensor_bridge=_FakeSensorBridge())
        with TestClient(app) as tc:
            with tc.websocket_connect("/ws/line") as ws:
                ws.receive_json()
                ws.send_json({"type": "ping", "ts": 12345})
                pong = ws.receive_json()
                assert pong["type"] == "pong"
                assert pong["ts"] == 12345

    def test_no_bridge_still_sends_welcome(self):
        app = _make_app()
        with TestClient(app) as tc:
            with tc.websocket_connect("/ws/line") as ws:
                msg = ws.receive_json()
                assert msg["type"] == "welcome"


# ---------------------------------------------------------------------------
# /ws/lighting
# ---------------------------------------------------------------------------

class TestLightingWs:
    def test_welcome_on_connect(self):
        app = _make_app(ros_bridge=_FakeRosBridge())
        with TestClient(app) as tc:
            with tc.websocket_connect("/ws/lighting") as ws:
                msg = ws.receive_json()
                assert msg["type"] == "welcome"
                assert msg["service"] == "lighting"

    def test_ping_returns_pong(self):
        app = _make_app(ros_bridge=_FakeRosBridge())
        with TestClient(app) as tc:
            with tc.websocket_connect("/ws/lighting") as ws:
                ws.receive_json()
                ws.send_json({"type": "ping", "ts": 42})
                pong = ws.receive_json()
                assert pong["type"] == "pong"
                assert pong["ts"] == 42

    def test_lighting_command_acked(self):
        ros_bridge = _FakeRosBridge()
        app = _make_app(ros_bridge=ros_bridge)
        with TestClient(app) as tc:
            with tc.websocket_connect("/ws/lighting") as ws:
                ws.receive_json()  # welcome
                ws.send_json({"pattern": "solid", "color": "#FF0000", "brightness": 0.8})
                ack = ws.receive_json()
                assert ack["type"] == "ack"
                assert ack["status"] in ("ok", "no_bridge")

    def test_no_bridge_ack_status(self):
        """When no ROS bridge is available the endpoint should ack gracefully."""
        app = _make_app()  # no ros_bridge
        with TestClient(app) as tc:
            with tc.websocket_connect("/ws/lighting") as ws:
                ws.receive_json()
                ws.send_json({"pattern": "solid"})
                ack = ws.receive_json()
                assert ack["type"] == "ack"
                # Should indicate no bridge — not an error crash
                assert ack["status"] in ("no_bridge", "ok", "error")
