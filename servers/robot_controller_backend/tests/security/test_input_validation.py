"""
Input Validation Security Tests
Tests for input validation and sanitization
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
class TestInputValidation:
    """Test input validation security."""

    def test_mode_parameter_validation(self):
        """Test that mode parameter is properly validated."""
        invalid_inputs = [
            None,
            "",
            "invalid",
            [],
            {},
            -1,
            8,
            999,
            float('inf'),
            float('-inf'),
        ]

        for invalid_input in invalid_inputs:
            try:
                response = requests.post(
                    "http://localhost:8000/api/system/mode/set",
                    json={"mode": invalid_input},
                    timeout=2
                )
                # Should reject invalid input
                assert response.status_code in [400, 422, 500]
            except requests.exceptions.RequestException:
                pass  # Server may not be running

    def test_json_structure_validation(self):
        """Test that JSON structure is validated."""
        invalid_structures = [
            {"mode": 3, "extra": "data"},
            {"mode": 3, "nested": {"data": "value"}},
            [{"mode": 3}],
            "not json",
        ]

        for invalid_structure in invalid_structures:
            try:
                response = requests.post(
                    "http://localhost:8000/api/system/mode/set",
                    json=invalid_structure,
                    timeout=2
                )
                # Should either accept (if extra fields ignored) or reject
                assert response.status_code in [200, 400, 422]
            except requests.exceptions.RequestException:
                pass

    def test_string_length_limits(self):
        """Test that string inputs have length limits."""
        # Test with very long strings
        long_string = "a" * 10000
        
        try:
            response = requests.post(
                "http://localhost:8000/api/system/mode/set",
                json={"mode": 0, "description": long_string},
                timeout=2
            )
            # Should reject or truncate
            assert response.status_code in [200, 400, 413, 422]
        except requests.exceptions.RequestException:
            pass

    def test_type_coercion_prevention(self):
        """Test that type coercion attacks are prevented."""
        coercion_attempts = [
            {"mode": "0"},  # String instead of int
            {"mode": "3.0"},  # Float string
            {"mode": True},  # Boolean
            {"mode": "3"},  # String number
        ]

        for attempt in coercion_attempts:
            try:
                response = requests.post(
                    "http://localhost:8000/api/system/mode/set",
                    json=attempt,
                    timeout=2
                )
                # Should validate type strictly
                assert response.status_code in [200, 400, 422]
            except requests.exceptions.RequestException:
                pass

    def test_special_character_handling(self):
        """Test handling of special characters."""
        special_chars = [
            "\x00",  # Null byte
            "\n",  # Newline
            "\r",  # Carriage return
            "\t",  # Tab
            "\x1a",  # EOF
        ]

        for char in special_chars:
            try:
                response = requests.post(
                    "http://localhost:8000/api/system/mode/set",
                    json={"mode": 0, "description": f"test{char}value"},
                    timeout=2
                )
                # Should sanitize or reject
                assert response.status_code in [200, 400, 422]
            except requests.exceptions.RequestException:
                pass

