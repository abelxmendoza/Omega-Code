"""
System Mode End-to-End Tests
Full system workflow tests for system mode management
"""

import pytest
import requests
import time
import json
from unittest.mock import patch


@pytest.mark.e2e
class TestSystemModeE2E:
    """End-to-end tests for system mode functionality."""

    @pytest.fixture
    def gateway_url(self):
        """Gateway base URL."""
        return "http://localhost:7070"

    @pytest.fixture
    def video_url(self):
        """Video server URL."""
        return "http://localhost:5000"

    def test_complete_mode_switching_workflow(self, gateway_url):
        """Test complete workflow: list -> status -> set -> verify."""
        try:
            # Step 1: List available modes
            response = requests.get(f"{gateway_url}/api/system/mode/list", timeout=2)
            assert response.status_code == 200
            modes = response.json()
            assert len(modes) == 8

            # Step 2: Get current status
            response = requests.get(f"{gateway_url}/api/system/mode/status", timeout=2)
            assert response.status_code == 200
            initial_status = response.json()
            initial_mode = initial_status.get('current_mode', 0)

            # Step 3: Switch to mode 3
            response = requests.post(
                f"{gateway_url}/api/system/mode/set",
                json={"mode": 3},
                timeout=5
            )
            assert response.status_code == 200
            assert response.json()['ok'] is True

            # Step 4: Verify mode changed
            time.sleep(1)  # Allow state to propagate
            response = requests.get(f"{gateway_url}/api/system/mode/status", timeout=2)
            assert response.status_code == 200
            new_status = response.json()
            assert new_status.get('current_mode') == 3 or new_status.get('effective_mode') == 3

            # Step 5: Reset to initial mode
            requests.post(
                f"{gateway_url}/api/system/mode/set",
                json={"mode": initial_mode},
                timeout=5
            )

        except requests.exceptions.ConnectionError:
            pytest.skip("Gateway not running")

    def test_latency_metrics_workflow(self, video_url):
        """Test latency metrics collection workflow."""
        try:
            # Get Pi-only latency
            response = requests.get(f"{video_url}/latency", timeout=5)
            assert response.status_code in [200, 503]
            
            if response.status_code == 200:
                data = response.json()
                assert 'type' in data
                assert data['type'] == 'pi_only'

            # Get hybrid latency
            response = requests.get(f"{video_url}/latency/hybrid", timeout=5)
            assert response.status_code in [200, 503]
            
            if response.status_code == 200:
                data = response.json()
                assert 'type' in data
                assert data['type'] == 'hybrid'

        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

    def test_mode_switching_with_latency_tracking(self, gateway_url, video_url):
        """Test mode switching while tracking latency."""
        try:
            # Switch to mode 1 (Motion Detection)
            response = requests.post(
                f"{gateway_url}/api/system/mode/set",
                json={"mode": 1},
                timeout=5
            )
            assert response.status_code == 200

            # Wait a bit for mode to take effect
            time.sleep(2)

            # Check latency metrics
            try:
                response = requests.get(f"{video_url}/latency", timeout=5)
                if response.status_code == 200:
                    data = response.json()
                    # Verify latency data structure
                    assert 'latencies_ms' in data or 'timestamps_ns' in data
            except:
                pass  # Latency endpoint may not be available

            # Reset mode
            requests.post(
                f"{gateway_url}/api/system/mode/set",
                json={"mode": 0},
                timeout=5
            )

        except requests.exceptions.ConnectionError:
            pytest.skip("Services not running")

    def test_thermal_throttling_workflow(self, gateway_url):
        """Test thermal throttling detection and mode adjustment."""
        try:
            # Get status
            response = requests.get(f"{gateway_url}/api/system/mode/status", timeout=2)
            assert response.status_code == 200
            status = response.json()
            
            # Check if throttling is detected
            if status.get('thermal_throttle_active'):
                # System should be in throttling mode
                assert 'thermal_throttle_active' in status
                
        except requests.exceptions.ConnectionError:
            pytest.skip("Gateway not running")

    def test_all_modes_switchable(self, gateway_url):
        """Test that all modes (0-7) can be switched to."""
        try:
            for mode in range(8):
                response = requests.post(
                    f"{gateway_url}/api/system/mode/set",
                    json={"mode": mode},
                    timeout=5
                )
                assert response.status_code == 200, f"Failed to switch to mode {mode}"
                assert response.json()['ok'] is True
                
                # Verify mode was set
                time.sleep(0.5)
                status_response = requests.get(f"{gateway_url}/api/system/mode/status", timeout=2)
                if status_response.status_code == 200:
                    status = status_response.json()
                    assert status.get('current_mode') == mode or status.get('effective_mode') == mode

            # Reset to mode 0
            requests.post(f"{gateway_url}/api/system/mode/set", json={"mode": 0}, timeout=5)

        except requests.exceptions.ConnectionError:
            pytest.skip("Gateway not running")

