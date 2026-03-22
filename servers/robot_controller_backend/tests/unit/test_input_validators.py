"""
Unit tests for api/input_validators.py

Every public function is covered: happy-path, boundary values, and
security / malformed inputs.  No hardware or network required.
"""

from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pytest
from api.input_validators import (
    validate_robot_name,
    validate_network_ssid,
    validate_network_password,
    validate_ip_address,
    validate_port,
    validate_color_hex,
    validate_brightness,
    validate_speed,
    sanitize_json_input,
    validate_path_traversal,
    validate_and_sanitize_input,
)


# ---------------------------------------------------------------------------
# validate_robot_name
# ---------------------------------------------------------------------------

class TestValidateRobotName:
    def test_simple_name(self):
        assert validate_robot_name("Omega-1") == "Omega-1"

    def test_strips_whitespace(self):
        assert validate_robot_name("  Omega  ") == "Omega"

    def test_allows_underscores_and_hyphens(self):
        assert validate_robot_name("omega_robot-v2") == "omega_robot-v2"

    def test_allows_spaces(self):
        assert validate_robot_name("Omega Robot") == "Omega Robot"

    def test_single_character(self):
        assert validate_robot_name("A") == "A"

    def test_max_length_50(self):
        name = "A" * 50
        assert validate_robot_name(name) == name

    def test_too_long_raises(self):
        with pytest.raises(ValueError, match="between 1 and 50"):
            validate_robot_name("A" * 51)

    def test_empty_raises(self):
        with pytest.raises(ValueError):
            validate_robot_name("")

    def test_non_string_raises(self):
        with pytest.raises(ValueError, match="must be a string"):
            validate_robot_name(123)  # type: ignore

    def test_special_chars_raise(self):
        with pytest.raises(ValueError, match="invalid characters"):
            validate_robot_name("omega<script>")

    def test_slash_raises(self):
        with pytest.raises(ValueError, match="invalid characters"):
            validate_robot_name("omega/robot")


# ---------------------------------------------------------------------------
# validate_network_ssid
# ---------------------------------------------------------------------------

class TestValidateNetworkSsid:
    def test_normal_ssid(self):
        assert validate_network_ssid("HomeNetwork") == "HomeNetwork"

    def test_max_length_32(self):
        ssid = "A" * 32
        assert validate_network_ssid(ssid) == ssid

    def test_too_long_raises(self):
        with pytest.raises(ValueError, match="between 1 and 32"):
            validate_network_ssid("A" * 33)

    def test_empty_raises(self):
        with pytest.raises(ValueError):
            validate_network_ssid("")

    def test_non_string_raises(self):
        with pytest.raises(ValueError, match="must be a string"):
            validate_network_ssid(None)  # type: ignore

    def test_strips_control_chars(self):
        # Control characters (ord < 32) should be removed
        result = validate_network_ssid("Net\x00work")
        assert "\x00" not in result


# ---------------------------------------------------------------------------
# validate_network_password
# ---------------------------------------------------------------------------

class TestValidateNetworkPassword:
    def test_valid_password(self):
        pw = "securePass1"
        assert validate_network_password(pw) == pw

    def test_min_length_8(self):
        assert validate_network_password("12345678") == "12345678"

    def test_too_short_raises(self):
        with pytest.raises(ValueError, match="between 8 and 63"):
            validate_network_password("short")

    def test_max_length_63(self):
        pw = "A" * 63
        assert validate_network_password(pw) == pw

    def test_too_long_raises(self):
        with pytest.raises(ValueError, match="between 8 and 63"):
            validate_network_password("A" * 64)

    def test_strips_null_bytes(self):
        result = validate_network_password("pass\x00word1")
        assert "\x00" not in result

    def test_non_string_raises(self):
        with pytest.raises(ValueError, match="must be a string"):
            validate_network_password(12345678)  # type: ignore


# ---------------------------------------------------------------------------
# validate_ip_address
# ---------------------------------------------------------------------------

class TestValidateIpAddress:
    def test_valid_ipv4(self):
        assert validate_ip_address("192.168.1.1") == "192.168.1.1"

    def test_loopback(self):
        assert validate_ip_address("127.0.0.1") == "127.0.0.1"

    def test_broadcast(self):
        assert validate_ip_address("255.255.255.255") == "255.255.255.255"

    def test_zeros(self):
        assert validate_ip_address("0.0.0.0") == "0.0.0.0"

    def test_too_few_octets_raises(self):
        with pytest.raises(ValueError, match="Invalid IP"):
            validate_ip_address("192.168.1")

    def test_too_many_octets_raises(self):
        with pytest.raises(ValueError, match="Invalid IP"):
            validate_ip_address("192.168.1.1.1")

    def test_out_of_range_octet_raises(self):
        with pytest.raises(ValueError, match="out of range"):
            validate_ip_address("192.168.1.256")

    def test_negative_octet_raises(self):
        with pytest.raises(ValueError):
            validate_ip_address("192.168.1.-1")

    def test_non_numeric_raises(self):
        with pytest.raises(ValueError):
            validate_ip_address("192.168.1.abc")

    def test_non_string_raises(self):
        with pytest.raises(ValueError, match="must be a string"):
            validate_ip_address(192168)  # type: ignore


# ---------------------------------------------------------------------------
# validate_port
# ---------------------------------------------------------------------------

class TestValidatePort:
    def test_valid_port(self):
        assert validate_port(8080) == 8080

    def test_string_port(self):
        assert validate_port("3000") == 3000

    def test_min_port_1(self):
        assert validate_port(1) == 1

    def test_max_port_65535(self):
        assert validate_port(65535) == 65535

    def test_zero_raises(self):
        with pytest.raises(ValueError, match="between 1 and 65535"):
            validate_port(0)

    def test_too_high_raises(self):
        with pytest.raises(ValueError, match="between 1 and 65535"):
            validate_port(65536)

    def test_non_numeric_raises(self):
        with pytest.raises(ValueError, match="must be a number"):
            validate_port("not_a_port")

    def test_none_raises(self):
        with pytest.raises(ValueError):
            validate_port(None)  # type: ignore


# ---------------------------------------------------------------------------
# validate_color_hex
# ---------------------------------------------------------------------------

class TestValidateColorHex:
    def test_with_hash(self):
        assert validate_color_hex("#ff0000") == "#FF0000"

    def test_without_hash(self):
        assert validate_color_hex("00FF00") == "#00FF00"

    def test_normalises_to_uppercase(self):
        assert validate_color_hex("aabbcc") == "#AABBCC"

    def test_white(self):
        assert validate_color_hex("FFFFFF") == "#FFFFFF"

    def test_black(self):
        assert validate_color_hex("000000") == "#000000"

    def test_too_short_raises(self):
        with pytest.raises(ValueError, match="6 hex characters"):
            validate_color_hex("FFF")

    def test_too_long_raises(self):
        with pytest.raises(ValueError, match="6 hex characters"):
            validate_color_hex("FFFFFFF")

    def test_invalid_hex_chars_raise(self):
        with pytest.raises(ValueError, match="invalid hex"):
            validate_color_hex("GGHHII")

    def test_non_string_raises(self):
        with pytest.raises(ValueError, match="must be a string"):
            validate_color_hex(0xFF0000)  # type: ignore


# ---------------------------------------------------------------------------
# validate_brightness
# ---------------------------------------------------------------------------

class TestValidateBrightness:
    def test_midpoint(self):
        assert validate_brightness(0.5) == 0.5

    def test_zero(self):
        assert validate_brightness(0.0) == 0.0

    def test_one(self):
        assert validate_brightness(1.0) == 1.0

    def test_integer_input(self):
        assert validate_brightness(1) == 1.0

    def test_string_numeric(self):
        assert validate_brightness("0.75") == 0.75

    def test_above_one_raises(self):
        with pytest.raises(ValueError, match="between 0.0 and 1.0"):
            validate_brightness(1.1)

    def test_negative_raises(self):
        with pytest.raises(ValueError, match="between 0.0 and 1.0"):
            validate_brightness(-0.1)

    def test_non_numeric_raises(self):
        with pytest.raises(ValueError, match="must be a number"):
            validate_brightness("bright")


# ---------------------------------------------------------------------------
# validate_speed
# ---------------------------------------------------------------------------

class TestValidateSpeed:
    def test_zero(self):
        assert validate_speed(0) == 0

    def test_max_4095(self):
        assert validate_speed(4095) == 4095

    def test_midpoint(self):
        assert validate_speed(2048) == 2048

    def test_string_input(self):
        assert validate_speed("1000") == 1000

    def test_float_truncated(self):
        assert validate_speed(512.9) == 512

    def test_negative_raises(self):
        with pytest.raises(ValueError, match="between 0 and 4095"):
            validate_speed(-1)

    def test_above_max_raises(self):
        with pytest.raises(ValueError, match="between 0 and 4095"):
            validate_speed(4096)

    def test_non_numeric_raises(self):
        with pytest.raises(ValueError, match="must be a number"):
            validate_speed("fast")


# ---------------------------------------------------------------------------
# sanitize_json_input
# ---------------------------------------------------------------------------

class TestSanitizeJsonInput:
    def test_flat_dict_passes(self):
        data = {"key": "value", "num": 42}
        assert sanitize_json_input(data) == data

    def test_nested_within_limit_passes(self):
        data = {"a": {"b": {"c": {"d": "leaf"}}}}
        result = sanitize_json_input(data, max_depth=10)
        assert result == data

    def test_exceeds_depth_raises(self):
        # Build a dict nested 5 levels deep
        data: dict = {}
        node = data
        for _ in range(5):
            node["child"] = {}
            node = node["child"]
        with pytest.raises(ValueError, match="nesting depth"):
            sanitize_json_input(data, max_depth=3)

    def test_list_values_checked(self):
        data = {"items": [{"nested": {"deep": {"too_deep": "x"}}}]}
        with pytest.raises(ValueError, match="nesting depth"):
            sanitize_json_input(data, max_depth=2)

    def test_empty_dict_passes(self):
        assert sanitize_json_input({}) == {}


# ---------------------------------------------------------------------------
# validate_path_traversal
# ---------------------------------------------------------------------------

class TestValidatePathTraversal:
    def test_simple_filename(self):
        assert validate_path_traversal("config.yaml") == "config.yaml"

    def test_simple_relative_path(self):
        assert validate_path_traversal("subdir/file.txt") == "subdir/file.txt"

    def test_dotdot_raises(self):
        with pytest.raises(ValueError, match="path traversal"):
            validate_path_traversal("../../etc/passwd")

    def test_absolute_path_raises(self):
        with pytest.raises(ValueError, match="path traversal"):
            validate_path_traversal("/etc/passwd")

    def test_semicolon_raises(self):
        with pytest.raises(ValueError, match="dangerous character"):
            validate_path_traversal("file;rm -rf /")

    def test_pipe_raises(self):
        with pytest.raises(ValueError, match="dangerous character"):
            validate_path_traversal("file|cat /etc/passwd")

    def test_backtick_raises(self):
        with pytest.raises(ValueError, match="dangerous character"):
            validate_path_traversal("file`whoami`")

    def test_dollar_sign_raises(self):
        with pytest.raises(ValueError, match="dangerous character"):
            validate_path_traversal("file$HOME")

    def test_non_string_raises(self):
        with pytest.raises(ValueError, match="must be a string"):
            validate_path_traversal(123)  # type: ignore


# ---------------------------------------------------------------------------
# validate_and_sanitize_input
# ---------------------------------------------------------------------------

class TestValidateAndSanitizeInput:
    def test_string_passthrough(self):
        result = validate_and_sanitize_input("hello", str, "name")
        assert result == "hello"

    def test_type_coercion(self):
        # int value coerced to str
        result = validate_and_sanitize_input(42, str, "label")
        assert result == "42"

    def test_none_required_raises(self):
        with pytest.raises(ValueError, match="required"):
            validate_and_sanitize_input(None, str, "name", allow_none=False)

    def test_none_allowed_returns_none(self):
        result = validate_and_sanitize_input(None, str, "name", allow_none=True)
        assert result is None

    def test_max_length_truncates(self):
        result = validate_and_sanitize_input("hello world", str, "msg", max_length=5)
        assert len(result) == 5
        assert result == "hello"

    def test_strips_null_bytes(self):
        result = validate_and_sanitize_input("hel\x00lo", str, "name")
        assert "\x00" not in result

    def test_strips_control_chars(self):
        # \x01 is a control character, should be stripped
        result = validate_and_sanitize_input("ab\x01cd", str, "field")
        assert "\x01" not in result

    def test_preserves_newline_and_tab(self):
        result = validate_and_sanitize_input("line1\nline2\t", str, "text")
        assert "\n" in result
        assert "\t" in result
