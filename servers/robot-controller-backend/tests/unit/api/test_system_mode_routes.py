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
app.include_router(router, prefix="/api/system/mode")

client = TestClient(app)


@pytest.fixture
def mock_hybrid_system_manager():
    """Mock hybrid system manager."""
    with patch('api.system_mode_routes.get_hybrid_system_manager') as mock:
        manager = Mock()
        manager.get_effective_mode.return_value = None
        manager.get_system_mode.return_value = Mock(value='pi_only')
        manager.is_orin_available.return_value = False
        manager.thermal_monitor.is_throttling_active.return_value = False
        manager.cpu_monitor.is_throttling_active.return_value = False
        mock.return_value = manager
        yield manager


@pytest.fixture
def mock_system_state():
    """Mock system state."""
    with patch('api.system_mode_routes.get_system_state') as mock:
        state = Mock()
        state.get_current_mode.return_value = 0
        state.set_current_mode.return_value = None
        mock.return_value = state
        yield state


class TestSystemModeRoutes:
    """Test system mode API routes."""

    def test_list_system_modes(self, mock_hybrid_system_manager, mock_system_state):
        """Test listing all system modes."""
        response = client.get("/api/system/mode/list")
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)
        assert len(data) == 8  # Modes 0-7
        assert all('id' in mode for mode in data)
        assert all('name' in mode for mode in data)
        assert all('description' in mode for mode in data)

    def test_get_system_mode_status(self, mock_hybrid_system_manager, mock_system_state):
        """Test getting system mode status."""
        response = client.get("/api/system/mode/status")
        assert response.status_code == 200
        data = response.json()
        assert data['ok'] is True
        assert 'current_mode' in data
        assert 'effective_mode' in data
        assert 'hybrid_system_status' in data
        assert 'orin_available' in data

    def test_set_system_mode_valid(self, mock_hybrid_system_manager, mock_system_state):
        """Test setting a valid system mode."""
        response = client.post("/api/system/mode/set", json={"mode": 3})
        assert response.status_code == 200
        data = response.json()
        assert data['ok'] is True
        assert 'message' in data
        mock_system_state.set_current_mode.assert_called_once_with(3)
        mock_hybrid_system_manager.set_manual_mode.assert_called_once_with(3)

    def test_set_system_mode_invalid_low(self, mock_hybrid_system_manager, mock_system_state):
        """Test setting an invalid mode (too low)."""
        response = client.post("/api/system/mode/set", json={"mode": -1})
        assert response.status_code == 400
        data = response.json()
        assert 'detail' in data
        assert 'Invalid mode' in data['detail']

    def test_set_system_mode_invalid_high(self, mock_hybrid_system_manager, mock_system_state):
        """Test setting an invalid mode (too high)."""
        response = client.post("/api/system/mode/set", json={"mode": 8})
        assert response.status_code == 400
        data = response.json()
        assert 'detail' in data
        assert 'Invalid mode' in data['detail']

    def test_set_system_mode_missing_mode(self, mock_hybrid_system_manager, mock_system_state):
        """Test setting mode without providing mode parameter."""
        response = client.post("/api/system/mode/set", json={})
        assert response.status_code == 400

    def test_set_system_mode_with_throttling(self, mock_hybrid_system_manager, mock_system_state):
        """Test status with thermal throttling active."""
        mock_hybrid_system_manager.thermal_monitor.is_throttling_active.return_value = True
        mock_hybrid_system_manager.thermal_monitor.get_temperature.return_value = 75.0
        
        response = client.get("/api/system/mode/status")
        assert response.status_code == 200
        data = response.json()
        assert data['thermal_throttle_active'] is True

    def test_set_system_mode_with_cpu_throttling(self, mock_hybrid_system_manager, mock_system_state):
        """Test status with CPU throttling active."""
        mock_hybrid_system_manager.cpu_monitor.is_throttling_active.return_value = True
        mock_hybrid_system_manager.cpu_monitor.get_load.return_value = 85.0
        
        response = client.get("/api/system/mode/status")
        assert response.status_code == 200
        data = response.json()
        assert data['cpu_throttle_active'] is True

    def test_set_system_mode_with_manual_override(self, mock_hybrid_system_manager, mock_system_state):
        """Test status with manual mode override."""
        mock_hybrid_system_manager.get_effective_mode.return_value = 5
        
        response = client.get("/api/system/mode/status")
        assert response.status_code == 200
        data = response.json()
        assert data['effective_mode'] == 5

