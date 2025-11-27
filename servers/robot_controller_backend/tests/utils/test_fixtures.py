"""
Test Fixtures and Mock Data
Common fixtures for backend tests
"""

import pytest
import json
from unittest.mock import Mock, MagicMock
from typing import Dict, Any


@pytest.fixture
def mock_frame():
    """Generate a mock video frame."""
    import numpy as np
    return np.zeros((480, 640, 3), dtype=np.uint8)


@pytest.fixture
def mock_system_mode_response():
    """Mock system mode API response."""
    return {
        "ok": True,
        "mode": 0,
        "description": "Camera Only",
        "manual_override": False,
        "mode_name": "CAMERA_ONLY",
        "hybrid_mode": "pi_only",
        "orin_available": False,
        "thermal_temp": 45.0,
        "cpu_load": 25.0,
        "throttling": False,
    }


@pytest.fixture
def mock_latency_response():
    """Mock latency API response."""
    return {
        "ok": True,
        "type": "pi_only",
        "timestamps_ns": {
            "capture_timestamp_ns": 1704067200123456789,
            "encode_start_ns": 1704067200123457890,
            "encode_end_ns": 1704067200123458901,
        },
        "latencies_ms": {
            "capture_to_encode_ms": 1.10,
            "encode_duration_ms": 1.01,
            "total_processing_ms": 2.11,
        },
        "ts": 1704067200123,
    }


@pytest.fixture
def mock_hybrid_latency_response():
    """Mock hybrid latency API response."""
    return {
        "ok": True,
        "type": "hybrid",
        "round_trip_ms": {
            "min": 45.2,
            "max": 125.8,
            "avg": 78.5,
            "count": 150,
        },
        "inference_ms": {
            "min": 12.3,
            "max": 45.6,
            "avg": 25.4,
            "count": 150,
        },
        "ts": 1704067200123,
    }


@pytest.fixture
def mock_hybrid_system_manager():
    """Mock hybrid system manager."""
    manager = Mock()
    manager.get_system_mode.return_value = Mock(value='pi_only')
    manager.is_orin_available.return_value = False
    manager.is_hybrid_mode.return_value = False
    manager.get_effective_mode.return_value = None
    manager.set_manual_mode = Mock()
    manager.clear_manual_mode = Mock()
    manager.thermal_monitor = Mock()
    manager.thermal_monitor.is_throttling_active.return_value = False
    manager.thermal_monitor.get_temperature.return_value = 45.0
    manager.cpu_monitor = Mock()
    manager.cpu_monitor.is_throttling_active.return_value = False
    manager.cpu_monitor.get_load.return_value = 25.0
    manager.pi_sensor_hub = Mock()
    manager.pi_sensor_hub.get_latency_stats.return_value = {
        "ok": True,
        "round_trip_ms": {"avg": 50.0, "min": 40.0, "max": 60.0, "count": 100},
    }
    return manager


@pytest.fixture
def mock_system_state():
    """Mock system state."""
    state = Mock()
    state.get_current_mode.return_value = 0
    state.set_current_mode = Mock()
    state.get_mode_history.return_value = []
    return state


@pytest.fixture
def mock_video_server():
    """Mock video server."""
    server = Mock()
    server.frame_overlay = Mock()
    server.frame_overlay.get_latency_metrics.return_value = {
        "capture_timestamp_ns": 1704067200123456789,
        "encode_start_ns": 1704067200123457890,
        "encode_end_ns": 1704067200123458901,
    }
    return server


@pytest.fixture
def mock_ros2_node():
    """Mock ROS2 node."""
    node = Mock()
    node.get_clock.return_value = Mock()
    node.get_clock().now.return_value = Mock()
    node.get_clock().now().to_msg.return_value = Mock()
    return node


@pytest.fixture
def sample_detection_result():
    """Sample detection result for testing."""
    return {
        "detections": [
            {
                "class": "person",
                "confidence": 0.95,
                "bbox": [100, 100, 200, 300],
            }
        ],
        "frame_uuid": "test-uuid-123",
        "inference_duration_ms": 25.4,
    }


@pytest.fixture
def sample_aruco_markers():
    """Sample ArUco markers for testing."""
    return [
        {
            "id": 0,
            "corners": [[100, 100], [200, 100], [200, 200], [100, 200]],
            "pose": {"x": 0.5, "y": 0.3, "z": 1.0},
        }
    ]


@pytest.fixture
def mock_websocket_message():
    """Mock WebSocket message."""
    return json.dumps({
        "command": "move-up",
        "speed": 50,
        "duration": 1000,
    })


@pytest.fixture
def mock_motor_telemetry():
    """Mock motor telemetry data."""
    return {
        "frontLeft": {"speed": 100.0, "power": 20.0, "pwm": 1500},
        "frontRight": {"speed": 100.0, "power": 20.0, "pwm": 1500},
        "rearLeft": {"speed": 100.0, "power": 20.0, "pwm": 1500},
        "rearRight": {"speed": 100.0, "power": 20.0, "pwm": 1500},
    }


@pytest.fixture
def mock_servo_telemetry():
    """Mock servo telemetry data."""
    return {
        "horizontal": 90,
        "vertical": 45,
        "min": 0,
        "max": 180,
    }


@pytest.fixture
def mock_sensor_data():
    """Mock sensor data."""
    return {
        "ultrasonic": {"distance": 50.0, "unit": "cm"},
        "lineTracking": {"left": 0, "center": 1, "right": 0},
        "battery": {"voltage": 12.6, "percentage": 75},
    }

