"""
Load Testing
Tests for system performance under load
"""

import pytest
import requests
import time
import statistics
from concurrent.futures import ThreadPoolExecutor, as_completed


@pytest.mark.performance
class TestLoadTesting:
    """Load testing for API endpoints."""

    def test_system_mode_endpoint_load(self):
        """Test system mode endpoint under load."""
        try:
            requests.get("http://localhost:8000/api/system/mode/status", timeout=2)
        except requests.exceptions.ConnectionError:
            pytest.skip("API server not running")

        def make_request():
            start = time.time()
            try:
                response = requests.get(
                    "http://localhost:8000/api/system/mode/status",
                    timeout=5
                )
                elapsed = time.time() - start
                return elapsed, response.status_code
            except Exception as e:
                return None, None

        # Make 100 concurrent requests
        with ThreadPoolExecutor(max_workers=50) as executor:
            futures = [executor.submit(make_request) for _ in range(100)]
            results = [f.result() for f in as_completed(futures)]

        # Analyze results
        successful = [r[0] for r in results if r[0] is not None and r[1] == 200]
        
        if successful:
            avg_time = statistics.mean(successful)
            p95_time = statistics.quantiles(successful, n=20)[18] if len(successful) > 20 else max(successful)
            
            # Average should be under 500ms
            assert avg_time < 0.5, f"Average response time {avg_time:.3f}s exceeds 500ms"
            # P95 should be under 1s
            assert p95_time < 1.0, f"P95 response time {p95_time:.3f}s exceeds 1s"

    def test_latency_endpoint_load(self):
        """Test latency endpoint under load."""
        try:
            requests.get("http://localhost:5000/latency", timeout=2)
        except requests.exceptions.ConnectionError:
            pytest.skip("Video server not running")

        def make_request():
            start = time.time()
            try:
                response = requests.get("http://localhost:5000/latency", timeout=5)
                elapsed = time.time() - start
                return elapsed, response.status_code
            except Exception as e:
                return None, None

        # Make 50 concurrent requests
        with ThreadPoolExecutor(max_workers=25) as executor:
            futures = [executor.submit(make_request) for _ in range(50)]
            results = [f.result() for f in as_completed(futures)]

        successful = [r[0] for r in results if r[0] is not None and r[1] == 200]
        
        if successful:
            avg_time = statistics.mean(successful)
            assert avg_time < 0.2, f"Average latency response time {avg_time:.3f}s exceeds 200ms"

    def test_sustained_load(self):
        """Test system under sustained load."""
        try:
            requests.get("http://localhost:8000/api/system/mode/status", timeout=2)
        except requests.exceptions.ConnectionError:
            pytest.skip("API server not running")

        def make_request():
            try:
                return requests.get(
                    "http://localhost:8000/api/system/mode/status",
                    timeout=5
                ).status_code
            except:
                return None

        # Sustained load: 10 requests/second for 30 seconds
        start_time = time.time()
        request_count = 0
        
        with ThreadPoolExecutor(max_workers=10) as executor:
            while time.time() - start_time < 30:
                futures = [executor.submit(make_request) for _ in range(10)]
                results = [f.result() for f in as_completed(futures)]
                request_count += len([r for r in results if r == 200])
                time.sleep(1)

        # Should handle at least 200 requests in 30 seconds
        assert request_count >= 200, f"Only handled {request_count} requests in 30 seconds"

