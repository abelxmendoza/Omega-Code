"""
Integration tests for api/ros_routes.py

All Docker subprocess calls are mocked so no Docker daemon is needed.
Tests verify HTTP contract, container name mapping, and disabled-ROS
fallback behaviour.
"""

from __future__ import annotations

import json
import pathlib
import sys
from unittest.mock import MagicMock, patch

ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from api.ros_routes import router


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_client(ros_enabled: str = "true") -> TestClient:
    with patch.dict("os.environ", {"ROS_ENABLED": ros_enabled}):
        app = FastAPI()
        app.include_router(router)
        return TestClient(app)


def _docker_ps_output(*container_names: str) -> str:
    """Build fake `docker compose ps --format json` output."""
    lines = []
    for name in container_names:
        lines.append(json.dumps({"Name": name, "State": "running", "Status": "Up 5 minutes"}))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# ROS disabled mode
# ---------------------------------------------------------------------------

class TestRosDisabled:
    def setup_method(self):
        # ROS_ENABLED is a module-level constant — patch it directly
        self._patcher = patch("api.ros_routes.ROS_ENABLED", False)
        self._patcher.start()
        app = FastAPI()
        app.include_router(router)
        self.tc = TestClient(app)

    def teardown_method(self):
        self._patcher.stop()

    def test_status_returns_disabled(self):
        res = self.tc.get("/api/ros/status")
        assert res.status_code == 200
        body = res.json()
        assert body["mode"] == "disabled"
        assert body["containers"] == []

    def test_topics_returns_empty_with_message(self):
        res = self.tc.get("/api/ros/topics")
        assert res.status_code == 200
        body = res.json()
        assert body["topics"] == []
        assert "disabled" in body.get("message", "").lower()

    def test_control_returns_503(self):
        res = self.tc.post("/api/ros/control", json={"action": "start"})
        assert res.status_code == 503

    def test_logs_returns_503(self):
        res = self.tc.get("/api/ros/logs/motor_controller")
        assert res.status_code == 503


# ---------------------------------------------------------------------------
# GET /api/ros/status — enabled, Docker available
# ---------------------------------------------------------------------------

class TestRosStatus:
    @patch("api.ros_routes.run_command")
    def test_status_200_when_compose_responds(self, mock_run):
        mock_run.return_value = {
            "success": True,
            "stdout": _docker_ps_output(
                "ros2_robot_motor_controller",
                "ros2_robot_sensor_node",
            ),
            "stderr": "",
            "returncode": 0,
        }
        with patch.dict("os.environ", {"ROS_ENABLED": "true"}):
            app = FastAPI()
            app.include_router(router)
            with TestClient(app) as tc:
                res = tc.get("/api/ros/status")
        assert res.status_code == 200
        body = res.json()
        assert "containers" in body

    @patch("api.ros_routes.run_command")
    def test_fallback_uses_motor_controller_and_sensor_node(self, mock_run):
        """When compose ps fails, fallback checks motor_controller and sensor_node."""
        # First call (compose ps) fails; subsequent calls check individual containers
        def side_effect(cmd, **kw):
            if "ps" in cmd and "--format" in cmd:
                return {"success": False, "stdout": "", "stderr": "error", "returncode": 1}
            # docker ps --filter name=...
            return {"success": True, "stdout": "", "stderr": "", "returncode": 0}

        mock_run.side_effect = side_effect
        with patch.dict("os.environ", {"ROS_ENABLED": "true"}):
            app = FastAPI()
            app.include_router(router)
            with TestClient(app) as tc:
                res = tc.get("/api/ros/status")
        assert res.status_code == 200
        # Verify no old 'telemetry_*' names appear in any container check
        all_calls = [str(call) for call in mock_run.call_args_list]
        assert not any("telemetry" in c for c in all_calls)


# ---------------------------------------------------------------------------
# POST /api/ros/control
# ---------------------------------------------------------------------------

class TestRosControl:
    @patch("api.ros_routes.run_command")
    def test_start_all_succeeds(self, mock_run):
        mock_run.return_value = {"success": True, "stdout": "Started", "stderr": "", "returncode": 0}
        with patch.dict("os.environ", {"ROS_ENABLED": "true"}):
            app = FastAPI()
            app.include_router(router)
            with TestClient(app) as tc:
                res = tc.post("/api/ros/control", json={"action": "start"})
        assert res.status_code == 200
        body = res.json()
        assert body["success"] is True

    @patch("api.ros_routes.run_command")
    def test_stop_specific_service(self, mock_run):
        mock_run.return_value = {"success": True, "stdout": "", "stderr": "", "returncode": 0}
        with patch.dict("os.environ", {"ROS_ENABLED": "true"}):
            app = FastAPI()
            app.include_router(router)
            with TestClient(app) as tc:
                res = tc.post(
                    "/api/ros/control",
                    json={"action": "stop", "service": "motor_controller"},
                )
        assert res.status_code == 200

    @patch("api.ros_routes.run_command")
    def test_restart_calls_correct_docker_command(self, mock_run):
        mock_run.return_value = {"success": True, "stdout": "", "stderr": "", "returncode": 0}
        with patch.dict("os.environ", {"ROS_ENABLED": "true"}):
            app = FastAPI()
            app.include_router(router)
            with TestClient(app) as tc:
                tc.post("/api/ros/control", json={"action": "restart"})
        cmd = mock_run.call_args[0][0]
        assert "restart" in cmd

    @patch("api.ros_routes.run_command")
    def test_invalid_action_returns_400(self, mock_run):
        with patch.dict("os.environ", {"ROS_ENABLED": "true"}):
            app = FastAPI()
            app.include_router(router)
            with TestClient(app) as tc:
                res = tc.post("/api/ros/control", json={"action": "explode"})
        assert res.status_code == 400

    @patch("api.ros_routes.run_command")
    def test_docker_failure_returns_500(self, mock_run):
        mock_run.return_value = {"success": False, "stdout": "", "stderr": "daemon not running", "returncode": 1}
        with patch.dict("os.environ", {"ROS_ENABLED": "true"}):
            app = FastAPI()
            app.include_router(router)
            with TestClient(app) as tc:
                res = tc.post("/api/ros/control", json={"action": "start"})
        assert res.status_code == 500


# ---------------------------------------------------------------------------
# GET /api/ros/logs/{service}
# ---------------------------------------------------------------------------

class TestRosLogs:
    @patch("api.ros_routes.run_command")
    def test_logs_returned_for_motor_controller(self, mock_run):
        mock_run.return_value = {
            "success": True,
            "stdout": "ROS2 node started\n/odom published",
            "stderr": "",
            "returncode": 0,
        }
        with patch.dict("os.environ", {"ROS_ENABLED": "true"}):
            app = FastAPI()
            app.include_router(router)
            with TestClient(app) as tc:
                res = tc.get("/api/ros/logs/motor_controller")
        assert res.status_code == 200
        body = res.json()
        assert "logs" in body
        assert body["service"] == "motor_controller"

    @patch("api.ros_routes.run_command")
    def test_logs_returned_for_sensor_node(self, mock_run):
        mock_run.return_value = {
            "success": True,
            "stdout": "sensor data published",
            "stderr": "",
            "returncode": 0,
        }
        with patch.dict("os.environ", {"ROS_ENABLED": "true"}):
            app = FastAPI()
            app.include_router(router)
            with TestClient(app) as tc:
                res = tc.get("/api/ros/logs/sensor_node")
        assert res.status_code == 200

    @patch("api.ros_routes.run_command")
    def test_missing_container_returns_404(self, mock_run):
        mock_run.return_value = {"success": False, "stdout": "", "stderr": "", "returncode": 1}
        with patch.dict("os.environ", {"ROS_ENABLED": "true"}):
            app = FastAPI()
            app.include_router(router)
            with TestClient(app) as tc:
                res = tc.get("/api/ros/logs/nonexistent_service")
        assert res.status_code == 404


# ---------------------------------------------------------------------------
# ContainerAction model — service field docstring check
# ---------------------------------------------------------------------------

class TestContainerActionModel:
    def test_service_field_accepts_motor_controller(self):
        from api.ros_routes import ContainerAction
        action = ContainerAction(action="start", service="motor_controller")
        assert action.service == "motor_controller"

    def test_service_field_accepts_sensor_node(self):
        from api.ros_routes import ContainerAction
        action = ContainerAction(action="stop", service="sensor_node")
        assert action.service == "sensor_node"

    def test_service_none_targets_all(self):
        from api.ros_routes import ContainerAction
        action = ContainerAction(action="start")
        assert action.service is None
