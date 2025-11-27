"""
Latency Integration Tests
Integration tests for latency measurement system
"""

import pytest
import requests
import time


@pytest.mark.integration
class TestLatencyIntegration:
    """Integration tests for latency endpoints."""

    @pytest.fixture
    def video_server_url(self):
        """Video server URL."""
        return "http://localhost:5000"

    def test_latency_endpoint_available(self, video_server_url):
        """Test that latency endpoint is available."""
        try:
            response = requests.get(f"{video_server_url}/latency", timeout=5)
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        assert response.status_code in [200, 503]
        
        if response.status_code == 200:
            data = response.json()
            assert data['ok'] is True
            assert data['type'] == 'pi_only'

    def test_latency_endpoint_structure(self, video_server_url):
        """Test latency endpoint response structure."""
        try:
            response = requests.get(f"{video_server_url}/latency", timeout=5)
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        if response.status_code == 200:
            data = response.json()
            assert 'ok' in data
            assert 'type' in data
            assert 'ts' in data
            assert 'latencies_ms' in data or 'timestamps_ns' in data

    def test_hybrid_latency_endpoint(self, video_server_url):
        """Test hybrid latency endpoint."""
        try:
            response = requests.get(f"{video_server_url}/latency/hybrid", timeout=5)
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        assert response.status_code in [200, 503]
        
        if response.status_code == 200:
            data = response.json()
            assert data['ok'] is True
            assert data['type'] == 'hybrid'
            assert 'round_trip_ms' in data

    def test_latency_metrics_consistency(self, video_server_url):
        """Test that latency metrics are consistent across requests."""
        try:
            response1 = requests.get(f"{video_server_url}/latency", timeout=5)
            time.sleep(0.5)
            response2 = requests.get(f"{video_server_url}/latency", timeout=5)
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        if response1.status_code == 200 and response2.status_code == 200:
            data1 = response1.json()
            data2 = response2.json()
            
            # Both should have same structure
            assert data1['type'] == data2['type']
            assert ('latencies_ms' in data1) == ('latencies_ms' in data2)

    def test_latency_endpoint_performance(self, video_server_url):
        """Test latency endpoint performance."""
        try:
            start = time.time()
            response = requests.get(f"{video_server_url}/latency", timeout=5)
            elapsed = time.time() - start
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        # Should respond quickly
        assert elapsed < 1.0

