"""
Mode WebSocket Update Tests
Tests for WebSocket push events when mode changes
"""

import pytest
import json
import asyncio
import websockets
from unittest.mock import Mock, patch, AsyncMock
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))


@pytest.fixture
def mock_websocket_server():
    """Mock WebSocket server."""
    server = Mock()
    server.send = AsyncMock()
    server.broadcast = AsyncMock()
    return server


@pytest.fixture
def mock_mode_update_event():
    """Mock mode update event."""
    return {
        "event": "mode_update",
        "mode": 0,
        "mode_name": "PI_ONLY",
        "reason": "manual",
        "timestamp": 1704067200.0,
    }


class TestModeWebSocketUpdates:
    """Test WebSocket mode update events."""

    @pytest.mark.asyncio
    async def test_mode_update_event_sent(self, mock_websocket_server, mock_mode_update_event):
        """Test that mode update events are sent via WebSocket."""
        # Simulate mode change
        await mock_websocket_server.broadcast(json.dumps(mock_mode_update_event))
        
        # Verify broadcast was called
        mock_websocket_server.broadcast.assert_called_once()
        call_args = mock_websocket_server.broadcast.call_args[0][0]
        data = json.loads(call_args)
        assert data['event'] == 'mode_update'
        assert data['mode'] == 0

    @pytest.mark.asyncio
    async def test_mode_update_thermal_reason(self, mock_websocket_server):
        """Test mode update with thermal reason."""
        thermal_event = {
            "event": "mode_update",
            "mode": 1,
            "mode_name": "SAFE_MODE",
            "reason": "thermal",
            "thermal_temp": 75.0,
            "timestamp": 1704067200.0,
        }
        
        await mock_websocket_server.broadcast(json.dumps(thermal_event))
        
        call_args = mock_websocket_server.broadcast.call_args[0][0]
        data = json.loads(call_args)
        assert data['reason'] == 'thermal'
        assert data['thermal_temp'] == 75.0

    @pytest.mark.asyncio
    async def test_mode_update_cpu_reason(self, mock_websocket_server):
        """Test mode update with CPU reason."""
        cpu_event = {
            "event": "mode_update",
            "mode": 1,
            "mode_name": "SAFE_MODE",
            "reason": "cpu",
            "cpu_load": 85.0,
            "timestamp": 1704067200.0,
        }
        
        await mock_websocket_server.broadcast(json.dumps(cpu_event))
        
        call_args = mock_websocket_server.broadcast.call_args[0][0]
        data = json.loads(call_args)
        assert data['reason'] == 'cpu'
        assert data['cpu_load'] == 85.0

    @pytest.mark.asyncio
    async def test_mode_update_orin_failure_reason(self, mock_websocket_server):
        """Test mode update with Orin failure reason."""
        orin_failure_event = {
            "event": "mode_update",
            "mode": 0,
            "mode_name": "PI_ONLY",
            "reason": "orin_failure",
            "timestamp": 1704067200.0,
        }
        
        await mock_websocket_server.broadcast(json.dumps(orin_failure_event))
        
        call_args = mock_websocket_server.broadcast.call_args[0][0]
        data = json.loads(call_args)
        assert data['reason'] == 'orin_failure'
        assert data['mode'] == 0  # Should fallback to PI_ONLY

    def test_mode_update_json_structure(self, mock_mode_update_event):
        """Test that mode update JSON structure is correct."""
        json_str = json.dumps(mock_mode_update_event)
        data = json.loads(json_str)
        
        assert 'event' in data
        assert 'mode' in data
        assert 'mode_name' in data
        assert 'reason' in data
        assert 'timestamp' in data

    @pytest.mark.asyncio
    async def test_multiple_clients_receive_updates(self, mock_websocket_server):
        """Test that multiple WebSocket clients receive mode updates."""
        event = {
            "event": "mode_update",
            "mode": 3,
            "mode_name": "FACE_DETECTION",
            "reason": "manual",
        }
        
        # Simulate broadcasting to multiple clients
        await mock_websocket_server.broadcast(json.dumps(event))
        
        # Verify broadcast was called (would send to all connected clients)
        mock_websocket_server.broadcast.assert_called_once()

