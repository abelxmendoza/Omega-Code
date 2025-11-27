"""
System Mode API Routes Unit Tests
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))

from api.system_mode_routes import router
from fastapi import FastAPI

app = FastAPI()
app.include_router(router)  # Router already has prefix="/system/mode"

client = TestClient(app)


@pytest.fixture
def mock_hybrid_system_manager():
    """Mock hybrid system manager."""
    with patch('api.system_mode_routes.get_hybrid_system_manager') as mock:
        manager = Mock()
        manager.get_effective_mode.return_value = None
        
        # Create a proper mock for SystemMode enum
        system_mode_mock = Mock()
        system_mode_mock.value = 'pi_only'
        manager.get_system_mode.return_value = system_mode_mock
        
        manager.orin_available = False
        
        # Mock thermal monitor
        thermal_monitor = Mock()
        thermal_monitor.get_temperature.return_value = 50.0
        thermal_monitor.should_throttle.return_value = False
        manager.thermal_monitor = thermal_monitor
        
        # Mock CPU monitor
        cpu_monitor = Mock()
        cpu_monitor.get_load.return_value = 30.0
        cpu_monitor.should_throttle.return_value = False
        manager.cpu_monitor = cpu_monitor
        
        manager.should_throttle_modules.return_value = False
        manager.set_manual_mode.return_value = None
        
        mock.return_value = manager
        yield manager


@pytest.fixture
def mock_system_state():
    """Mock system state."""
    with patch('api.system_mode_routes.get_system_state') as mock:
        state = Mock()
        state.get_current_mode.return_value = 0
        state.set_mode.return_value = True
        state.get_status.return_value = {
            "mode": 0,
            "description": "Camera Only",
            "manual_override": False,
            "mode_name": "CAMERA_ONLY"
        }
        state.list_modes.return_value = {
            i: {"mode": i, "name": f"MODE_{i}", "description": f"Mode {i}"}
            for i in range(8)
        }
        mock.return_value = state
        yield state


class TestSystemModeRoutes:
    """Test system mode API routes."""

    def test_list_system_modes(self, mock_hybrid_system_manager, mock_system_state):
        """Test listing all system modes."""
        response = client.get("/system/mode/list")
        assert response.status_code == 200
        data = response.json()
        assert data['ok'] is True
        assert 'modes' in data
        assert isinstance(data['modes'], dict)
        assert len(data['modes']) == 8  # Modes 0-7

    def test_get_system_mode_status(self, mock_hybrid_system_manager, mock_system_state):
        """Test getting system mode status."""
        response = client.get("/system/mode/status")
        assert response.status_code == 200
        data = response.json()
        assert data['ok'] is True
        assert 'mode' in data
        assert 'description' in data
        assert 'mode_name' in data
        # Hybrid info may or may not be present depending on availability

    def test_set_system_mode_valid(self, mock_hybrid_system_manager, mock_system_state):
        """Test setting a valid system mode."""
        response = client.post("/system/mode/set", json={"mode": 3})
        assert response.status_code == 200
        data = response.json()
        assert data['ok'] is True
        assert 'message' in data
        mock_system_state.set_mode.assert_called_once_with(3, manual=True)
        mock_hybrid_system_manager.set_manual_mode.assert_called_once_with(3)

    def test_set_system_mode_invalid_low(self, mock_hybrid_system_manager, mock_system_state):
        """Test setting an invalid mode (too low)."""
        # Pydantic validation returns 422 for invalid field values
        response = client.post("/system/mode/set", json={"mode": -1})
        assert response.status_code == 422  # Pydantic validation error
        data = response.json()
        assert 'detail' in data

    def test_set_system_mode_invalid_high(self, mock_hybrid_system_manager, mock_system_state):
        """Test setting an invalid mode (too high)."""
        # Pydantic validation returns 422 for invalid field values
        response = client.post("/system/mode/set", json={"mode": 8})
        assert response.status_code == 422  # Pydantic validation error
        data = response.json()
        assert 'detail' in data

    def test_set_system_mode_missing_mode(self, mock_hybrid_system_manager, mock_system_state):
        """Test setting mode without providing mode parameter."""
        # Pydantic validation returns 422 for missing required fields
        response = client.post("/system/mode/set", json={})
        assert response.status_code == 422  # Pydantic validation error

    def test_set_system_mode_with_throttling(self, mock_hybrid_system_manager, mock_system_state):
        """Test status with thermal throttling active."""
        mock_hybrid_system_manager.thermal_monitor.get_temperature.return_value = 75.0
        mock_hybrid_system_manager.thermal_monitor.should_throttle.return_value = True
        mock_hybrid_system_manager.should_throttle_modules.return_value = True
        
        response = client.get("/system/mode/status")
        assert response.status_code == 200
        data = response.json()
        assert data['ok'] is True
        assert 'throttling' in data or 'thermal_temp' in data

    def test_set_system_mode_with_cpu_throttling(self, mock_hybrid_system_manager, mock_system_state):
        """Test status with CPU throttling active."""
        mock_hybrid_system_manager.cpu_monitor.get_load.return_value = 85.0
        mock_hybrid_system_manager.cpu_monitor.should_throttle.return_value = True
        mock_hybrid_system_manager.should_throttle_modules.return_value = True
        
        response = client.get("/system/mode/status")
        assert response.status_code == 200
        data = response.json()
        assert data['ok'] is True
        assert 'throttling' in data or 'cpu_load' in data

    def test_set_system_mode_with_manual_override(self, mock_hybrid_system_manager, mock_system_state):
        """Test status with manual mode override."""
        mock_system_state.get_status.return_value = {
            "mode": 5,
            "description": "Recording Only",
            "manual_override": True,
            "mode_name": "RECORDING_ONLY"
        }
        
        response = client.get("/system/mode/status")
        assert response.status_code == 200
        data = response.json()
        assert data['ok'] is True
        assert data['mode'] == 5
        assert data['manual_override'] is True

