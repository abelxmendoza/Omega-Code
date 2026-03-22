"""
Integration tests for api/lighting_routes.py

Mocks the LED hardware so tests run anywhere.  Verifies the REST
contract: correct HTTP status codes, response shapes, and that the
LED controller is actually called with the right arguments.
"""

from __future__ import annotations

import pathlib
import sys
from unittest.mock import MagicMock, patch

ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient


# ---------------------------------------------------------------------------
# Module-level mocks (must happen before importing the routes)
# ---------------------------------------------------------------------------

# rpi_ws281x is Pi-only; stub it out before the router is imported
import types
import sys as _sys

if "rpi_ws281x" not in _sys.modules:
    _mock_ws = types.ModuleType("rpi_ws281x")
    _mock_ws.Color = lambda r, g, b: (r << 16) | (g << 8) | b
    _mock_ws.Adafruit_NeoPixel = MagicMock()
    _mock_ws.WS2811_STRIP_GRB = 0
    _sys.modules["rpi_ws281x"] = _mock_ws


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture()
def mock_led_controller():
    """Patch the led_controller instance directly so no real GPIO is touched."""
    led = MagicMock()
    led.color_wipe = MagicMock(return_value=None)
    led.clear_strip = MagicMock(return_value=None)
    with patch("api.lighting_routes.led_controller", led):
        yield led


@pytest.fixture()
def client(mock_led_controller):
    from api.lighting_routes import router

    app = FastAPI()
    app.include_router(router, prefix="/lighting")
    with TestClient(app) as tc:
        yield tc, mock_led_controller


@pytest.fixture()
def client_no_led():
    """App where LED controller initialisation fails — simulates missing hardware."""
    with patch("api.lighting_routes.LedController", side_effect=RuntimeError("no GPIO")):
        from importlib import reload
        import api.lighting_routes as lr_mod
        reload(lr_mod)
        app = FastAPI()
        app.include_router(lr_mod.router, prefix="/lighting")
        with TestClient(app) as tc:
            yield tc
        reload(lr_mod)  # restore for other tests


# ---------------------------------------------------------------------------
# GET /lighting/light/on
# ---------------------------------------------------------------------------

class TestLightOn:
    def test_returns_200(self, client):
        tc, _ = client
        res = tc.get("/lighting/light/on")
        assert res.status_code == 200

    def test_response_has_success_status(self, client):
        tc, _ = client
        body = tc.get("/lighting/light/on").json()
        assert body["status"] == "success"

    def test_calls_color_wipe_with_white(self, client):
        tc, led = client
        tc.get("/lighting/light/on")
        led.color_wipe.assert_called_once()
        args = led.color_wipe.call_args[0]
        # First argument is the Color value for white (255,255,255) → 0xFFFFFF
        assert args[0] == (255 << 16) | (255 << 8) | 255

    def test_no_led_returns_503(self):
        """When LED controller is unavailable the endpoint must return 503."""
        import types as _types
        import api.lighting_routes as lr

        original = lr.led_controller
        lr.led_controller = None
        try:
            app = FastAPI()
            app.include_router(lr.router, prefix="/lighting")
            with TestClient(app) as tc:
                res = tc.get("/lighting/light/on")
            assert res.status_code == 503
        finally:
            lr.led_controller = original


# ---------------------------------------------------------------------------
# GET /lighting/light/off
# ---------------------------------------------------------------------------

class TestLightOff:
    def test_returns_200(self, client):
        tc, _ = client
        res = tc.get("/lighting/light/off")
        assert res.status_code == 200

    def test_response_has_success_status(self, client):
        tc, _ = client
        body = tc.get("/lighting/light/off").json()
        assert body["status"] == "success"

    def test_calls_clear_strip(self, client):
        tc, led = client
        tc.get("/lighting/light/off")
        led.clear_strip.assert_called_once()

    def test_no_led_returns_503(self):
        import api.lighting_routes as lr

        original = lr.led_controller
        lr.led_controller = None
        try:
            app = FastAPI()
            app.include_router(lr.router, prefix="/lighting")
            with TestClient(app) as tc:
                res = tc.get("/lighting/light/off")
            assert res.status_code == 503
        finally:
            lr.led_controller = original

    def test_color_wipe_exception_returns_500(self, client):
        tc, led = client
        led.clear_strip.side_effect = RuntimeError("GPIO error")
        res = tc.get("/lighting/light/off")
        assert res.status_code == 500
