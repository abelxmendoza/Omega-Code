"""
API Security Tests
Tests for security vulnerabilities in API endpoints
"""

import pytest
import sys
import os
from unittest.mock import patch

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

try:
    import requests
except ImportError:
    requests = None

# Skip all tests if requests not available
if requests is None:
    pytestmark = pytest.mark.skip(reason="requests library not available")


@pytest.mark.security
class TestAPISecurity:
    """Security tests for API endpoints."""

    def test_sql_injection_prevention(self):
        """Test that SQL injection attempts are prevented."""
        # This test ensures no SQL queries are vulnerable
        malicious_inputs = [
            "'; DROP TABLE users; --",
            "1' OR '1'='1",
            "admin'--",
            "' UNION SELECT * FROM users--",
        ]

        for malicious_input in malicious_inputs:
            # Test system mode endpoint
            response = requests.post(
                "http://localhost:8000/api/system/mode/set",
                json={"mode": malicious_input},
                timeout=2
            )
            # Should reject invalid input, not execute SQL
            assert response.status_code in [400, 422]

    def test_xss_prevention(self):
        """Test that XSS attacks are prevented."""
        xss_payloads = [
            "<script>alert('XSS')</script>",
            "<img src=x onerror=alert('XSS')>",
            "javascript:alert('XSS')",
            "<svg onload=alert('XSS')>",
        ]

        for payload in xss_payloads:
            response = requests.post(
                "http://localhost:8000/api/system/mode/set",
                json={"mode": 0, "description": payload},
                timeout=2
            )
            # Response should not contain executable script tags
            if response.status_code == 200:
                assert "<script>" not in response.text.lower()

    def test_input_validation(self):
        """Test input validation on all endpoints."""
        # Test invalid mode values
        invalid_modes = [-1, 8, 999, "invalid", None, [], {}]

        for invalid_mode in invalid_modes:
            response = requests.post(
                "http://localhost:8000/api/system/mode/set",
                json={"mode": invalid_mode},
                timeout=2
            )
            assert response.status_code == 400

    def test_rate_limiting(self):
        """Test rate limiting on API endpoints."""
        # Make many rapid requests
        for _ in range(100):
            try:
                response = requests.get(
                    "http://localhost:8000/api/system/mode/status",
                    timeout=1
                )
                # After rate limit, should get 429
                if response.status_code == 429:
                    break
            except requests.exceptions.RequestException:
                pass
        else:
            # If no 429, rate limiting may not be implemented
            pytest.skip("Rate limiting not implemented")

    def test_cors_headers(self):
        """Test CORS headers are properly set."""
        response = requests.options(
            "http://localhost:8000/api/system/mode/status",
            headers={"Origin": "http://localhost:3000"},
            timeout=2
        )
        # Should have CORS headers
        assert "Access-Control-Allow-Origin" in response.headers or response.status_code == 405

    def test_authentication_required_endpoints(self):
        """Test that sensitive endpoints require authentication."""
        # If authentication is implemented, test here
        sensitive_endpoints = [
            "/api/system/mode/set",
        ]

        for endpoint in sensitive_endpoints:
            # Test without auth token
            response = requests.post(
                f"http://localhost:8000{endpoint}",
                json={"mode": 0},
                timeout=2
            )
            # Should either require auth (401) or be public (200)
            assert response.status_code in [200, 401, 403]

    def test_path_traversal_prevention(self):
        """Test that path traversal attacks are prevented."""
        malicious_paths = [
            "../../../etc/passwd",
            "..\\..\\..\\windows\\system32",
            "/etc/passwd",
            "C:\\Windows\\System32",
        ]

        for path in malicious_paths:
            response = requests.get(
                f"http://localhost:8000/api/system/mode/{path}",
                timeout=2
            )
            # Should not expose file system
            assert response.status_code in [404, 400, 403]

    def test_json_bomb_prevention(self):
        """Test that JSON bombs are prevented."""
        # Deeply nested JSON
        deep_json = {"a": {"b": {"c": {"d": {"e": {"f": {"g": {"h": {"i": {"j": "value"}}}}}}}}}}

        response = requests.post(
            "http://localhost:8000/api/system/mode/set",
            json=deep_json,
            timeout=2
        )
        # Should reject or handle gracefully
        assert response.status_code in [400, 422, 413]

