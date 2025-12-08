"""
WebSocket Security Enhancements

Provides security features for WebSocket connections:
- Origin validation
- Rate limiting per connection
- Input sanitization
- Connection limits
- Message size limits
"""

from typing import Set, Optional, Dict, Any
from collections import defaultdict
import time
import json
import logging
from functools import lru_cache

logger = logging.getLogger(__name__)

# WebSocket security configuration
MAX_WS_CONNECTIONS = 10
MAX_WS_MESSAGE_SIZE = 1024 * 1024  # 1MB
MAX_WS_MESSAGES_PER_MINUTE = 100
WS_RATE_LIMIT_WINDOW = 60  # seconds

# Rate limiting storage per connection
_ws_rate_limits: Dict[str, list] = defaultdict(list)
_ws_last_cleanup = time.time()
_ws_cleanup_interval = 300  # 5 minutes


@lru_cache(maxsize=128)
def is_allowed_origin(origin: str, allowed_origins: tuple) -> bool:
    """
    Check if WebSocket origin is allowed.
    
    Args:
        origin: Origin header value
        allowed_origins: Tuple of allowed origins (for caching)
    
    Returns:
        True if origin is allowed
    """
    if not origin:
        return False
    
    origin_lower = origin.lower().strip()
    
    for allowed in allowed_origins:
        allowed_lower = allowed.lower().strip()
        # Exact match
        if origin_lower == allowed_lower:
            return True
        # Wildcard subdomain match
        if allowed_lower.startswith("*."):
            domain = allowed_lower[2:]
            if origin_lower.endswith(domain):
                return True
    
    return False


def validate_websocket_origin(
    origin: Optional[str],
    allowed_origins: Set[str],
    allow_no_origin: bool = False
) -> bool:
    """
    Validate WebSocket origin header.
    
    Args:
        origin: Origin header value
        allowed_origins: Set of allowed origins
        allow_no_origin: Whether to allow requests without origin (CLI tools)
    
    Returns:
        True if origin is valid
    """
    # Allow requests without origin if configured (for CLI tools)
    if not origin:
        return allow_no_origin
    
    # Check against allowed origins
    return is_allowed_origin(origin, tuple(allowed_origins))


def check_websocket_rate_limit(connection_id: str) -> tuple[bool, Optional[str]]:
    """
    Check if WebSocket connection has exceeded rate limit.
    
    Args:
        connection_id: Unique connection identifier
    
    Returns:
        Tuple of (allowed, error_message)
    """
    now = time.time()
    window_start = now - WS_RATE_LIMIT_WINDOW
    
    # Clean up old entries
    global _ws_last_cleanup
    if now - _ws_last_cleanup > _ws_cleanup_interval:
        _cleanup_ws_rate_limits(window_start)
        _ws_last_cleanup = now
    
    # Get messages in current window
    messages = _ws_rate_limits[connection_id]
    messages[:] = [msg_time for msg_time in messages if msg_time > window_start]
    
    # Check limit
    if len(messages) >= MAX_WS_MESSAGES_PER_MINUTE:
        logger.warning(f"WebSocket rate limit exceeded for {connection_id}")
        return False, "Rate limit exceeded. Please slow down."
    
    # Add current message
    messages.append(now)
    
    return True, None


def validate_websocket_message_size(message: str) -> tuple[bool, Optional[str]]:
    """
    Validate WebSocket message size.
    
    Args:
        message: Message string
    
    Returns:
        Tuple of (valid, error_message)
    """
    size = len(message.encode('utf-8'))
    
    if size > MAX_WS_MESSAGE_SIZE:
        logger.warning(f"WebSocket message too large: {size} bytes")
        return False, f"Message too large. Maximum size: {MAX_WS_MESSAGE_SIZE / 1024:.0f}KB"
    
    return True, None


def sanitize_websocket_message(message: str) -> tuple[bool, Optional[str], Optional[dict]]:
    """
    Sanitize and parse WebSocket message.
    
    Args:
        message: Raw message string
    
    Returns:
        Tuple of (valid, error_message, parsed_data)
    """
    # Check size
    valid, error = validate_websocket_message_size(message)
    if not valid:
        return False, error, None
    
    # Parse JSON
    try:
        data = json.loads(message)
    except json.JSONDecodeError as e:
        logger.warning(f"Invalid JSON in WebSocket message: {e}")
        return False, "Invalid JSON format", None
    
    # Check JSON depth (prevent JSON bombs)
    if not _validate_json_depth(data, max_depth=10):
        logger.warning("JSON depth exceeds limit")
        return False, "Message structure too complex", None
    
    # Sanitize string values
    sanitized = _sanitize_json_data(data)
    
    return True, None, sanitized


def _validate_json_depth(obj: Any, max_depth: int = 10, current_depth: int = 0) -> bool:
    """Validate JSON object depth"""
    if current_depth > max_depth:
        return False
    
    if isinstance(obj, dict):
        return all(
            _validate_json_depth(value, max_depth, current_depth + 1)
            for value in obj.values()
        )
    elif isinstance(obj, list):
        return all(
            _validate_json_depth(item, max_depth, current_depth + 1)
            for item in obj
        )
    
    return True


def _sanitize_json_data(data: Any) -> Any:
    """Sanitize JSON data recursively"""
    if isinstance(data, dict):
        return {
            key: _sanitize_json_data(value)
            for key, value in data.items()
        }
    elif isinstance(data, list):
        return [_sanitize_json_data(item) for item in data]
    elif isinstance(data, str):
        # Remove null bytes and control characters
        sanitized = data.replace("\x00", "")
        sanitized = "".join(
            char for char in sanitized
            if ord(char) >= 32 or char in "\n\r\t"
        )
        # Limit length
        if len(sanitized) > 10000:
            sanitized = sanitized[:10000]
        return sanitized
    else:
        return data


def _cleanup_ws_rate_limits(window_start: float):
    """Clean up old rate limit entries"""
    for connection_id in list(_ws_rate_limits.keys()):
        _ws_rate_limits[connection_id] = [
            msg_time for msg_time in _ws_rate_limits[connection_id]
            if msg_time > window_start
        ]
        if not _ws_rate_limits[connection_id]:
            del _ws_rate_limits[connection_id]


def check_connection_limit(current_connections: int) -> tuple[bool, Optional[str]]:
    """
    Check if connection limit is exceeded.
    
    Args:
        current_connections: Current number of connections
    
    Returns:
        Tuple of (allowed, error_message)
    """
    if current_connections >= MAX_WS_CONNECTIONS:
        logger.warning(f"Connection limit exceeded: {current_connections}/{MAX_WS_CONNECTIONS}")
        return False, "Server at capacity. Please try again later."
    
    return True, None

