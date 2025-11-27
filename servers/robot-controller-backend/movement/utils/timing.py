"""
Movement V2 - Timing Utilities

Helper functions for timing, rate limiting, and time-based calculations.
Optimized for performance using monotonic time.
"""

import time
from typing import Optional, Callable

# Performance: Use monotonic time (faster, not affected by system clock changes)
_monotonic = time.monotonic


class RateLimiter:
    """Optimized rate limiter with __slots__ for memory efficiency"""
    __slots__ = ('rate_hz', 'min_interval', 'last_update')
    
    def __init__(self, rate_hz: float = 10.0):
        """
        Initialize rate limiter.
        
        Args:
            rate_hz: Maximum update rate (Hz)
        """
        self.rate_hz = max(0.1, rate_hz)
        self.min_interval = 1.0 / self.rate_hz
        self.last_update = 0.0
    
    def should_update(self) -> bool:
        """
        Check if enough time has passed for next update (optimized).
        
        Returns:
            True if update should proceed
        """
        # Performance: Use monotonic time
        current_time = _monotonic()
        if current_time - self.last_update >= self.min_interval:
            self.last_update = current_time
            return True
        return False
    
    def set_rate(self, rate_hz: float) -> None:
        """Update rate limit"""
        self.rate_hz = max(0.1, rate_hz)
        self.min_interval = 1.0 / self.rate_hz


class Timer:
    """Optimized timer with __slots__ for memory efficiency"""
    __slots__ = ('start_time',)
    
    def __init__(self):
        """Initialize timer"""
        self.start_time = _monotonic()  # Use monotonic time
    
    def elapsed(self) -> float:
        """Get elapsed time in seconds"""
        return _monotonic() - self.start_time
    
    def reset(self) -> None:
        """Reset timer"""
        self.start_time = _monotonic()  # Use monotonic time


def get_timestamp_ms() -> int:
    """Get current timestamp in milliseconds (uses time.time() for compatibility)"""
    return int(time.time() * 1000)


def get_timestamp_ns() -> int:
    """Get current timestamp in nanoseconds (uses time.time() for compatibility)"""
    return int(time.time() * 1e9)


def elapsed_ms(start_time: float) -> int:
    """Get elapsed time in milliseconds from start_time (monotonic)"""
    return int((_monotonic() - start_time) * 1000)


def now_ns() -> int:
    """Get current monotonic time in nanoseconds"""
    return int(_monotonic() * 1e9)


def sleep_until(target_time: float) -> None:
    """
    Sleep until target time.
    
    Args:
        target_time: Target time (seconds since epoch)
    """
    current_time = time.time()
    sleep_duration = target_time - current_time
    if sleep_duration > 0:
        time.sleep(sleep_duration)


def debounce(delay: float) -> Callable:
    """
    Create a debounced function decorator.
    
    Args:
        delay: Debounce delay in seconds
    
    Usage:
        @debounce(0.5)
        def my_function():
            pass
    """
    def decorator(func: Callable) -> Callable:
        last_call_time = [0.0]
        
        def wrapper(*args, **kwargs):
            current_time = time.time()
            if current_time - last_call_time[0] >= delay:
                last_call_time[0] = current_time
                return func(*args, **kwargs)
            return None
        
        return wrapper
    return decorator


__all__ = [
    "RateLimiter",
    "Timer",
    "get_timestamp_ms",
    "get_timestamp_ns",
    "elapsed_ms",
    "now_ns",
    "sleep_until",
    "debounce"
]

