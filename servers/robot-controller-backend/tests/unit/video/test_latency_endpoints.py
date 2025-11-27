"""
Latency Endpoints Unit Tests
Tests for /latency and /latency/hybrid endpoints
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from flask import Flask
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))


@pytest.fixture
def mock_video_server_app():
    """Create a mock Flask app for video server."""
    app = Flask(__name__)
    return app


@pytest.fixture
def mock_frame_overlay():
    """Mock frame overlay with latency methods."""
    overlay = Mock()
    overlay.get_capture_timestamp_ns.return_value = 1704067200123456789
    overlay.get_encode_start_timestamp_ns.return_value = 1704067200123457890
    overlay.get_encode_end_timestamp_ns.return_value = 1704067200123458901
    overlay.get_latency_metrics.return_value = {
        "capture_timestamp_ns": 1704067200123456789,
        "encode_start_ns": 1704067200123457890,
        "encode_end_ns": 1704067200123458901,
    }
    return overlay


@pytest.fixture
def mock_hybrid_system_manager():
    """Mock hybrid system manager."""
    manager = Mock()
    manager.pi_sensor_hub = Mock()
    manager.pi_sensor_hub.get_latency_stats.return_value = {
        "ok": True,
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
    }
    return manager


class TestLatencyEndpoints:
    """Test latency endpoint functionality."""

    def test_latency_endpoint_returns_metrics(self, mock_frame_overlay):
        """Test that /latency endpoint returns metrics."""
        with patch('video.video_server.frame_overlay', mock_frame_overlay):
            from video.video_server import app
            
            with app.test_client() as client:
                response = client.get('/latency')
                assert response.status_code == 200
                data = response.get_json()
                assert data['ok'] is True
                assert data['type'] == 'pi_only'
                assert 'latencies_ms' in data or 'timestamps_ns' in data

    def test_latency_endpoint_handles_missing_overlay(self):
        """Test /latency endpoint handles missing frame overlay."""
        with patch('video.video_server.frame_overlay', None):
            from video.video_server import app
            
            with app.test_client() as client:
                response = client.get('/latency')
                assert response.status_code == 503
                data = response.get_json()
                assert data['ok'] is False
                assert 'error' in data

    def test_hybrid_latency_endpoint_returns_metrics(self, mock_hybrid_system_manager):
        """Test that /latency/hybrid endpoint returns metrics."""
        with patch('video.video_server.hybrid_system_manager', mock_hybrid_system_manager):
            from video.video_server import app
            
            with app.test_client() as client:
                response = client.get('/latency/hybrid')
                assert response.status_code == 200
                data = response.get_json()
                assert data['ok'] is True
                assert data['type'] == 'hybrid'
                assert 'round_trip_ms' in data

    def test_hybrid_latency_endpoint_handles_unavailable(self):
        """Test /latency/hybrid endpoint handles unavailable hybrid system."""
        with patch('video.video_server.hybrid_system_manager', None):
            from video.video_server import app
            
            with app.test_client() as client:
                response = client.get('/latency/hybrid')
                assert response.status_code == 503
                data = response.get_json()
                assert data['ok'] is False
                assert 'error' in data

    def test_latency_endpoint_calculates_durations(self, mock_frame_overlay):
        """Test that latency endpoint calculates durations correctly."""
        with patch('video.video_server.frame_overlay', mock_frame_overlay):
            from video.video_server import app
            
            with app.test_client() as client:
                response = client.get('/latency')
                assert response.status_code == 200
                data = response.get_json()
                
                if 'latencies_ms' in data:
                    latencies = data['latencies_ms']
                    # Verify calculations
                    if 'capture_to_encode_ms' in latencies:
                        assert latencies['capture_to_encode_ms'] > 0
                    if 'encode_duration_ms' in latencies:
                        assert latencies['encode_duration_ms'] > 0
                    if 'total_processing_ms' in latencies:
                        assert latencies['total_processing_ms'] > 0

    def test_latency_endpoint_includes_timestamps(self, mock_frame_overlay):
        """Test that latency endpoint includes timestamps."""
        with patch('video.video_server.frame_overlay', mock_frame_overlay):
            from video.video_server import app
            
            with app.test_client() as client:
                response = client.get('/latency')
                assert response.status_code == 200
                data = response.get_json()
                
                if 'timestamps_ns' in data:
                    timestamps = data['timestamps_ns']
                    assert 'capture_timestamp_ns' in timestamps
                    assert 'encode_start_ns' in timestamps
                    assert 'encode_end_ns' in timestamps

