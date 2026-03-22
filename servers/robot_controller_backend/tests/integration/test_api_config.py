"""
Integration tests for api/config_routes.py

Uses the real ConfigManager against a temporary YAML directory so no
Pi hardware is needed and the real config.yaml is never mutated.
"""

from __future__ import annotations

import copy
import pathlib
import shutil
import sys
import tempfile
from typing import Generator

ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pytest
import yaml
from fastapi import FastAPI
from fastapi.testclient import TestClient

# ---------------------------------------------------------------------------
# Minimal config YAML fixtures written to a temp directory
# ---------------------------------------------------------------------------

_BASE_CONFIG = {
    "robot":    {"name": "TestBot", "profile": "pi4b", "version": "1.0.0", "serial_number": ""},
    "network":  {"default_mode": "client", "ap": {}, "client": {}, "tailscale": {"enabled": False}},
    "services": {"autostart": [], "restart_policies": {}},
    "camera":   {"backend": "gstreamer", "width": 640, "height": 480, "fps": 30},
    "movement": {"default_profile": "smooth", "max_speed": 4095, "min_speed": 0},
    "lighting": {"default_pattern": "omega_signature", "default_brightness": 0.5, "led_count": 16, "gpio_pin": 18},
    "logging":  {"level": "INFO", "directory": "/tmp/omega_logs"},
    "security": {"api_auth_enabled": False, "api_key": "", "allowed_origins": [], "rate_limit_enabled": False, "csrf_enabled": False},
    "telemetry": {"enabled": True, "update_interval_ms": 1000},
}

_BASE_PROFILE = {
    "profiles": {
        "pi4b": {
            "name": "Raspberry Pi 4B",
            "capabilities": {"ml_capable": False},
            "recommended_settings": {"camera_backend": "gstreamer"},
        }
    },
    "active_profile": "pi4b",
}


@pytest.fixture()
def config_dir(tmp_path: pathlib.Path) -> Generator[pathlib.Path, None, None]:
    """Write minimal YAML files to a temp dir and return its path."""
    (tmp_path / "config.yaml").write_text(yaml.dump(_BASE_CONFIG))
    (tmp_path / "robot_profile.yaml").write_text(yaml.dump(_BASE_PROFILE))
    yield tmp_path


@pytest.fixture()
def client(config_dir: pathlib.Path) -> Generator[TestClient, None, None]:
    """Build a TestClient pointing at the temp config dir."""
    from unittest.mock import patch
    import omega_config.config_manager as cm_mod

    # Patch CONFIG_FILE / PROFILE_FILE / STATE_FILE inside config_manager
    with (
        patch.object(cm_mod, "CONFIG_FILE",  config_dir / "config.yaml"),
        patch.object(cm_mod, "PROFILE_FILE", config_dir / "robot_profile.yaml"),
        patch.object(cm_mod, "STATE_FILE",   config_dir / "state.json"),
        patch.object(cm_mod, "BACKUP_DIR",   config_dir / "backups"),
        patch.object(cm_mod, "_config_manager", None),
    ):
        (config_dir / "backups").mkdir(exist_ok=True)

        from api.config_routes import router
        app = FastAPI()
        app.include_router(router)
        with TestClient(app) as tc:
            yield tc


# ---------------------------------------------------------------------------
# GET /api/config/  — retrieve full config
# ---------------------------------------------------------------------------

class TestGetConfig:
    def test_returns_200(self, client):
        res = client.get("/api/config/")
        assert res.status_code == 200

    def test_contains_robot_section(self, client):
        body = client.get("/api/config/").json()
        # Sections may be nested under a "config" wrapper key
        sections = body.get("config", body)
        assert "robot" in sections

    def test_robot_name_matches_fixture(self, client):
        body = client.get("/api/config/").json()
        sections = body.get("config", body)
        assert sections["robot"]["name"] == "TestBot"

    def test_camera_backend_is_gstreamer(self, client):
        body = client.get("/api/config/").json()
        sections = body.get("config", body)
        assert sections["camera"]["backend"] == "gstreamer"


# ---------------------------------------------------------------------------
# POST /api/config/update  — update a config section
# ---------------------------------------------------------------------------

class TestUpdateConfig:
    def test_updates_robot_name(self, client):
        # POST /{section} with section data as body
        res = client.post("/api/config/robot", json={"name": "NewBot"})
        assert res.status_code == 200
        # Verify the change persisted
        body = client.get("/api/config/").json()
        sections = body.get("config", body)
        assert sections["robot"]["name"] == "NewBot"

    def test_updates_camera_fps(self, client):
        res = client.post("/api/config/camera", json={"fps": 60})
        assert res.status_code == 200
        body = client.get("/api/config/").json()
        sections = body.get("config", body)
        assert sections["camera"]["fps"] == 60

    def test_unknown_section_returns_error(self, client):
        res = client.post(
            "/api/config/nonexistent_section",
            json={"key": "val"},
        )
        # Should return 4xx (bad request) or 500 — not succeed silently
        assert res.status_code >= 400

    def test_invalid_payload_returns_422(self, client):
        # Sending non-JSON body to a route that expects JSON should give 422
        res = client.post(
            "/api/config/robot",
            content=b"not-json",
            headers={"Content-Type": "application/json"},
        )
        assert res.status_code == 422


# ---------------------------------------------------------------------------
# GET /api/config/section/{section}
# ---------------------------------------------------------------------------

class TestGetConfigSection:
    def test_get_camera_section(self, client):
        # GET /{section} — section name in URL path
        res = client.get("/api/config/camera")
        assert res.status_code == 200
        body = res.json()
        # The response should contain camera fields (may be nested under "data")
        data = body.get("data", body)
        assert "backend" in data or "camera" in str(body).lower()

    def test_unknown_section_404_or_400(self, client):
        res = client.get("/api/config/does_not_exist")
        assert res.status_code in (400, 404, 500)


# ---------------------------------------------------------------------------
# GET /api/config/validate
# ---------------------------------------------------------------------------

class TestValidateConfig:
    def test_validate_returns_200(self, client):
        res = client.get("/api/config/validate")
        # Valid config should validate without error
        assert res.status_code == 200

    def test_validate_response_has_valid_key(self, client):
        body = client.get("/api/config/validate").json()
        # Must include some kind of validity indicator
        assert "valid" in body or "status" in body or "ok" in body
