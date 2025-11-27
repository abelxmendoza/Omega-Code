"""
Complete Workflow E2E Tests
Tests for complete user workflows from start to finish
"""

import pytest
import requests
import time
from unittest.mock import patch


@pytest.mark.e2e
class TestCompleteWorkflow:
    """Complete workflow E2E tests."""

    @pytest.fixture
    def gateway_url(self):
        """Gateway base URL."""
        return "http://localhost:7070"

    @pytest.fixture
    def video_url(self):
        """Video server URL."""
        return "http://localhost:5000"

    def test_system_mode_switching_workflow(self, gateway_url):
        """Test complete system mode switching workflow."""
        try:
            # Step 1: Get initial state
            response = requests.get(f"{gateway_url}/api/system/mode/status", timeout=2)
            assert response.status_code == 200
            initial_state = response.json()
            initial_mode = initial_state.get('current_mode', 0)

            # Step 2: List available modes
            response = requests.get(f"{gateway_url}/api/system/mode/list", timeout=2)
            assert response.status_code == 200
            modes = response.json()
            assert len(modes) == 8

            # Step 3: Switch through modes 0-3
            for mode in range(4):
                response = requests.post(
                    f"{gateway_url}/api/system/mode/set",
                    json={"mode": mode},
                    timeout=5
                )
                assert response.status_code == 200
                assert response.json()['ok'] is True
                time.sleep(0.5)  # Allow state to propagate

            # Step 4: Reset to initial mode
            requests.post(
                f"{gateway_url}/api/system/mode/set",
                json={"mode": initial_mode},
                timeout=5
            )

        except requests.exceptions.ConnectionError:
            pytest.skip("Gateway not running")

    def test_latency_monitoring_workflow(self, video_url):
        """Test latency monitoring workflow."""
        try:
            # Step 1: Get Pi-only latency
            response = requests.get(f"{video_url}/latency", timeout=5)
            assert response.status_code in [200, 503]
            
            if response.status_code == 200:
                data = response.json()
                assert data['type'] == 'pi_only'

            # Step 2: Get hybrid latency (may not be available)
            response = requests.get(f"{video_url}/latency/hybrid", timeout=5)
            assert response.status_code in [200, 503]

            # Step 3: Verify latency metrics structure
            if response.status_code == 200:
                data = response.json()
                assert 'round_trip_ms' in data or 'inference_ms' in data

        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

    def test_mode_switching_with_latency_tracking(self, gateway_url, video_url):
        """Test mode switching while tracking latency."""
        try:
            # Switch to mode 1
            response = requests.post(
                f"{gateway_url}/api/system/mode/set",
                json={"mode": 1},
                timeout=5
            )
            assert response.status_code == 200

            # Wait for mode to take effect
            time.sleep(2)

            # Check latency
            try:
                response = requests.get(f"{video_url}/latency", timeout=5)
                if response.status_code == 200:
                    data = response.json()
                    assert 'latencies_ms' in data or 'timestamps_ns' in data
            except:
                pass

            # Reset
            requests.post(
                f"{gateway_url}/api/system/mode/set",
                json={"mode": 0},
                timeout=5
            )

        except requests.exceptions.ConnectionError:
            pytest.skip("Services not running")

    def test_error_recovery_workflow(self, gateway_url):
        """Test error recovery workflow."""
        try:
            # Try invalid mode
            response = requests.post(
                f"{gateway_url}/api/system/mode/set",
                json={"mode": 99},
                timeout=5
            )
            assert response.status_code == 400

            # System should still be functional
            response = requests.get(f"{gateway_url}/api/system/mode/status", timeout=2)
            assert response.status_code == 200

        except requests.exceptions.ConnectionError:
            pytest.skip("Gateway not running")

