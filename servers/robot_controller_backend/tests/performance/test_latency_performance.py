"""
Latency Performance Tests
Tests for latency endpoint performance
"""

import pytest
import sys
import os
import time
import statistics
from concurrent.futures import ThreadPoolExecutor

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

try:
    import requests
except ImportError:
    requests = None

# Skip all tests if requests not available
if requests is None:
    pytestmark = pytest.mark.skip(reason="requests library not available")


@pytest.mark.performance
class TestLatencyPerformance:
    """Performance tests for latency endpoints."""

    @pytest.fixture
    def video_server_url(self):
        """Video server URL."""
        return "http://localhost:5000"

    def test_latency_endpoint_response_time(self, video_server_url):
        """Test that latency endpoint responds quickly."""
        try:
            start = time.time()
            response = requests.get(f"{video_server_url}/latency", timeout=5)
            elapsed = time.time() - start
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        assert response.status_code in [200, 503]
        # Should respond within 100ms
        assert elapsed < 0.1

    def test_latency_endpoint_throughput(self, video_server_url):
        """Test latency endpoint can handle multiple concurrent requests."""
        try:
            requests.get(f"{video_server_url}/latency", timeout=2)
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        def make_request():
            start = time.time()
            try:
                response = requests.get(f"{video_server_url}/latency", timeout=5)
                elapsed = time.time() - start
                return elapsed, response.status_code
            except Exception as e:
                return None, None

        # Make 50 concurrent requests
        with ThreadPoolExecutor(max_workers=50) as executor:
            futures = [executor.submit(make_request) for _ in range(50)]
            results = [f.result() for f in futures]

        # Filter successful requests
        successful = [r[0] for r in results if r[0] is not None]
        
        if successful:
            avg_time = statistics.mean(successful)
            max_time = max(successful)
            
            # Average should be under 200ms
            assert avg_time < 0.2, f"Average response time {avg_time:.3f}s exceeds 200ms"
            # Max should be under 1s
            assert max_time < 1.0, f"Max response time {max_time:.3f}s exceeds 1s"

    def test_hybrid_latency_endpoint_performance(self, video_server_url):
        """Test hybrid latency endpoint performance."""
        try:
            start = time.time()
            response = requests.get(f"{video_server_url}/latency/hybrid", timeout=5)
            elapsed = time.time() - start
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        assert response.status_code in [200, 503]
        # Should respond within 200ms
        assert elapsed < 0.2

    def test_latency_metrics_accuracy(self, video_server_url):
        """Test that latency metrics are accurate."""
        try:
            response = requests.get(f"{video_server_url}/latency", timeout=5)
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        if response.status_code == 200:
            data = response.json()
            assert 'latencies_ms' in data or 'timestamps_ns' in data
            
            # If latencies are present, verify they're reasonable
            if 'latencies_ms' in data:
                latencies = data['latencies_ms']
                if 'total_processing_ms' in latencies:
                    total = latencies['total_processing_ms']
                    # Should be positive and reasonable (< 1000ms)
                    assert 0 < total < 1000

    def test_concurrent_latency_requests(self, video_server_url):
        """Test handling of concurrent latency requests."""
        try:
            requests.get(f"{video_server_url}/latency", timeout=2)
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        def make_request():
            try:
                return requests.get(f"{video_server_url}/latency", timeout=5).status_code
            except:
                return None

        # Make 20 concurrent requests
        with ThreadPoolExecutor(max_workers=20) as executor:
            futures = [executor.submit(make_request) for _ in range(20)]
            results = [f.result() for f in futures]

        # All should succeed (200) or be unavailable (503)
        assert all(r in [200, 503, None] for r in results)

