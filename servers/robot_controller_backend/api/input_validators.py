"""
Input Validation Utilities

Provides validation and sanitization functions for API inputs.
Ensures security while maintaining functionality.
"""

from typing import Any, Dict, List, Optional, Union
from pydantic import BaseModel, validator, Field
import re
import logging

logger = logging.getLogger(__name__)


class ValidatedString(str):
    """String type with validation"""
    pass


def validate_robot_name(name: str) -> str:
    """
    Validate robot name input.
    
    Rules:
    - 1-50 characters
    - Alphanumeric, spaces, hyphens, underscores only
    - No special characters that could cause issues
    """
    if not isinstance(name, str):
        raise ValueError("Robot name must be a string")
    
    if len(name) < 1 or len(name) > 50:
        raise ValueError("Robot name must be between 1 and 50 characters")
    
    # Allow alphanumeric, spaces, hyphens, underscores
    if not re.match(r'^[a-zA-Z0-9\s\-_]+$', name):
        raise ValueError("Robot name contains invalid characters")
    
    # Remove leading/trailing whitespace
    name = name.strip()
    
    return name


def validate_network_ssid(ssid: str) -> str:
    """
    Validate Wi-Fi SSID.
    
    Rules:
    - 1-32 characters (Wi-Fi standard)
    - No control characters
    """
    if not isinstance(ssid, str):
        raise ValueError("SSID must be a string")
    
    if len(ssid) < 1 or len(ssid) > 32:
        raise ValueError("SSID must be between 1 and 32 characters")
    
    # Remove control characters
    ssid = "".join(char for char in ssid if ord(char) >= 32)
    
    return ssid


def validate_network_password(password: str) -> str:
    """
    Validate Wi-Fi password.
    
    Rules:
    - 8-63 characters (WPA2 standard)
    - No null bytes
    """
    if not isinstance(password, str):
        raise ValueError("Password must be a string")
    
    if len(password) < 8 or len(password) > 63:
        raise ValueError("Password must be between 8 and 63 characters")
    
    # Remove null bytes
    password = password.replace("\x00", "")
    
    return password


def validate_ip_address(ip: str) -> str:
    """
    Validate IP address format.
    
    Rules:
    - Valid IPv4 format (xxx.xxx.xxx.xxx)
    """
    if not isinstance(ip, str):
        raise ValueError("IP address must be a string")
    
    # Basic IPv4 validation
    parts = ip.split(".")
    if len(parts) != 4:
        raise ValueError("Invalid IP address format")
    
    try:
        for part in parts:
            num = int(part)
            if num < 0 or num > 255:
                raise ValueError("Invalid IP address: octet out of range")
    except ValueError:
        raise ValueError("Invalid IP address: non-numeric octet")
    
    return ip


def validate_port(port: Union[int, str]) -> int:
    """
    Validate port number.
    
    Rules:
    - 1-65535
    """
    try:
        port_int = int(port)
    except (ValueError, TypeError):
        raise ValueError("Port must be a number")
    
    if port_int < 1 or port_int > 65535:
        raise ValueError("Port must be between 1 and 65535")
    
    return port_int


def validate_color_hex(color: str) -> str:
    """
    Validate hex color code.
    
    Rules:
    - Format: #RRGGBB or RRGGBB
    - Valid hex characters only
    """
    if not isinstance(color, str):
        raise ValueError("Color must be a string")
    
    # Remove # if present
    color = color.lstrip("#")
    
    # Must be 6 hex characters
    if len(color) != 6:
        raise ValueError("Color must be 6 hex characters")
    
    if not re.match(r'^[0-9A-Fa-f]{6}$', color):
        raise ValueError("Color contains invalid hex characters")
    
    return f"#{color.upper()}"


def validate_brightness(brightness: Union[float, int]) -> float:
    """
    Validate brightness value.
    
    Rules:
    - 0.0 to 1.0
    """
    try:
        brightness_float = float(brightness)
    except (ValueError, TypeError):
        raise ValueError("Brightness must be a number")
    
    if brightness_float < 0.0 or brightness_float > 1.0:
        raise ValueError("Brightness must be between 0.0 and 1.0")
    
    return brightness_float


def validate_speed(speed: Union[int, float]) -> int:
    """
    Validate motor speed value.
    
    Rules:
    - 0 to 4095 (PWM range)
    """
    try:
        speed_int = int(speed)
    except (ValueError, TypeError):
        raise ValueError("Speed must be a number")
    
    if speed_int < 0 or speed_int > 4095:
        raise ValueError("Speed must be between 0 and 4095")
    
    return speed_int


def sanitize_json_input(data: Dict[str, Any], max_depth: int = 10) -> Dict[str, Any]:
    """
    Sanitize JSON input to prevent JSON bomb attacks.
    
    Args:
        data: JSON data to sanitize
        max_depth: Maximum nesting depth
    
    Returns:
        Sanitized data
    
    Raises:
        ValueError: If data exceeds max depth
    """
    def check_depth(obj: Any, depth: int = 0) -> None:
        if depth > max_depth:
            raise ValueError(f"JSON nesting depth exceeds maximum ({max_depth})")
        
        if isinstance(obj, dict):
            for value in obj.values():
                check_depth(value, depth + 1)
        elif isinstance(obj, list):
            for item in obj:
                check_depth(item, depth + 1)
    
    check_depth(data)
    return data


def validate_path_traversal(path: str) -> str:
    """
    Validate file path to prevent path traversal attacks.
    
    Args:
        path: Path to validate
    
    Returns:
        Validated path
    
    Raises:
        ValueError: If path contains traversal attempts
    """
    if not isinstance(path, str):
        raise ValueError("Path must be a string")
    
    # Check for path traversal attempts
    if ".." in path or path.startswith("/"):
        raise ValueError("Invalid path: path traversal detected")
    
    # Remove dangerous characters
    dangerous = ["<", ">", "|", "&", ";", "`", "$", "(", ")", "{", "}"]
    for char in dangerous:
        if char in path:
            raise ValueError(f"Invalid path: dangerous character '{char}' detected")
    
    return path


class BaseRequestModel(BaseModel):
    """Base request model with common validation"""
    
    class Config:
        # Prevent extra fields by default (can be overridden)
        extra = "forbid"
        # Validate assignment
        validate_assignment = True


def validate_and_sanitize_input(
    value: Any,
    field_type: type,
    field_name: str,
    max_length: Optional[int] = None,
    allow_none: bool = False
) -> Any:
    """
    Generic input validation and sanitization.
    
    Args:
        value: Value to validate
        field_type: Expected type
        field_name: Field name for error messages
        max_length: Maximum length (for strings)
        allow_none: Whether None is allowed
    
    Returns:
        Validated and sanitized value
    
    Raises:
        ValueError: If validation fails
    """
    if value is None:
        if allow_none:
            return None
        raise ValueError(f"{field_name} is required")
    
    # Type checking
    if not isinstance(value, field_type):
        try:
            value = field_type(value)
        except (ValueError, TypeError):
            raise ValueError(f"{field_name} must be of type {field_type.__name__}")
    
    # String-specific validation
    if isinstance(value, str):
        # Remove null bytes
        value = value.replace("\x00", "")
        
        # Remove control characters (except newline, tab, carriage return)
        value = "".join(
            char for char in value
            if ord(char) >= 32 or char in "\n\r\t"
        )
        
        # Length check
        if max_length and len(value) > max_length:
            logger.warning(f"{field_name} truncated from {len(value)} to {max_length} characters")
            value = value[:max_length]
    
    return value

