"""
System Mode API Integration Tests
Tests the full integration of system mode API with backend services
"""

import pytest
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

try:
    import requests
except ImportError:
    requests = None

import time
from unittest.mock import patch, Mock

# Skip all tests if requests not available
if requests is None:
    pytestmark = pytest.mark.skip(reason="requests library not available")


@pytest.fixture
def api_base_url():
    """Base URL for API."""
    return "http://localhost:8000/api/system/mode"


@pytest.fixture
def gateway_base_url():
    """Base URL for gateway."""
    return "http://localhost:7070/api/system/mode"


@pytest.mark.integration
class TestSystemModeIntegration:
    """Integration tests for system mode API."""

    def test_full_mode_switch_workflow(self, gateway_base_url):
        """Test complete mode switching workflow."""
        # Skip if gateway not running
        try:
            response = requests.get(f"{gateway_base_url}/status", timeout=2)
        except requests.exceptions.ConnectionError:
            pytest.skip("Gateway not running")

        # Get initial status
        response = requests.get(f"{gateway_base_url}/status")
        assert response.status_code == 200
        initial_mode = response.json().get('current_mode', 0)

        # Switch to mode 3
        response = requests.post(
            f"{gateway_base_url}/set",
            json={"mode": 3},
            timeout=5
        )
        assert response.status_code == 200
        assert response.json()['ok'] is True

        # Verify mode changed
        time.sleep(0.5)  # Allow state to update
        response = requests.get(f"{gateway_base_url}/status")
        assert response.status_code == 200
        data = response.json()
        assert data['current_mode'] == 3 or data['effective_mode'] == 3

        # Reset to initial mode
        requests.post(f"{gateway_base_url}/set", json={"mode": initial_mode})

    def test_mode_list_endpoint(self, gateway_base_url):
        """Test mode list endpoint."""
        try:
            response = requests.get(f"{gateway_base_url}/list", timeout=2)
        except requests.exceptions.ConnectionError:
            pytest.skip("Gateway not running")

        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)
        assert len(data) == 8

    def test_mode_status_with_hybrid_system(self, gateway_base_url):
        """Test status endpoint with hybrid system detection."""
        try:
            response = requests.get(f"{gateway_base_url}/status", timeout=2)
        except requests.exceptions.ConnectionError:
            pytest.skip("Gateway not running")

        assert response.status_code == 200
        data = response.json()
        assert 'hybrid_system_status' in data
        assert 'orin_available' in data

    def test_concurrent_mode_switches(self, gateway_base_url):
        """Test handling of concurrent mode switch requests."""
        try:
            requests.get(f"{gateway_base_url}/status", timeout=2)
        except requests.exceptions.ConnectionError:
            pytest.skip("Gateway not running")

        import concurrent.futures

        def switch_mode(mode):
            response = requests.post(
                f"{gateway_base_url}/set",
                json={"mode": mode},
                timeout=5
            )
            return response.status_code == 200

        # Try switching to different modes concurrently
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            futures = [
                executor.submit(switch_mode, 1),
                executor.submit(switch_mode, 2),
                executor.submit(switch_mode, 3),
            ]
            results = [f.result() for f in concurrent.futures.as_completed(futures)]

        # At least one should succeed
        assert any(results)

