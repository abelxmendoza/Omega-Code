"""
Movement V2 - Watchdog Timer Module

Automatically stops motors if no command is received within timeout period.
Prevents runaway robot scenarios.
"""

import time
import logging
from typing import Optional, Callable
from enum import Enum

# Performance: Use monotonic time (faster, not affected by system clock changes)
_monotonic = time.monotonic

logger = logging.getLogger(__name__)


class WatchdogState(Enum):
    """Watchdog state"""
    ACTIVE = "active"
    TRIGGERED = "triggered"
    DISABLED = "disabled"


class MovementWatchdog:
    """Optimized watchdog with __slots__ for memory efficiency"""
    __slots__ = (
        'timeout', 'stop_callback', 'enabled', 'last_update', 'state',
        'trigger_count', 'last_trigger_time', 'grace_period_end'
    )
    
    def __init__(
        self,
        timeout_sec: float = 2.0,
        stop_callback: Optional[Callable[[], None]] = None,
        enabled: bool = True
    ):
        """
        Initialize movement watchdog.
        
        Args:
            timeout_sec: Timeout in seconds before triggering stop
            stop_callback: Callback function to call when watchdog triggers
            enabled: Whether watchdog is enabled
        """
        self.timeout = max(0.1, float(timeout_sec))
        self.stop_callback = stop_callback
        self.enabled = enabled
        
        # Use monotonic time for better performance
        now = _monotonic()
        self.last_update = now
        self.state = WatchdogState.ACTIVE if enabled else WatchdogState.DISABLED
        self.trigger_count = 0
        self.last_trigger_time = 0.0
        
        # Grace period: don't trigger immediately after enable
        self.grace_period_end = now + 0.5
    
    def kick(self) -> None:
        """
        Reset watchdog timer (call on every movement command).
        
        This should be called whenever a valid movement command is received.
        """
        if not self.enabled:
            return
        
        # Use monotonic time
        self.last_update = _monotonic()
        if self.state == WatchdogState.TRIGGERED:
            self.state = WatchdogState.ACTIVE
            logger.info("Watchdog reset after trigger")
    
    def refresh(self) -> None:
        """Refresh watchdog timer (alias for kick)"""
        self.kick()
    
    def should_stop(self) -> bool:
        """
        Check if watchdog should trigger stop.
        
        Returns:
            True if timeout exceeded and stop should be triggered
        """
        if not self.enabled or self.state == WatchdogState.DISABLED:
            return False
        
        # Performance: Use monotonic time and cache current time
        now = _monotonic()
        
        # Grace period check
        if now < self.grace_period_end:
            return False
        
        elapsed = now - self.last_update
        
        # Performance: Cache timeout comparison
        if elapsed > self.timeout:
            if self.state != WatchdogState.TRIGGERED:
                self.state = WatchdogState.TRIGGERED
                self.trigger_count += 1
                self.last_trigger_time = now
                logger.warning(
                    f"Watchdog triggered: no command for {elapsed:.2f}s "
                    f"(timeout={self.timeout:.2f}s)"
                )
                
                # Call stop callback if provided
                if self.stop_callback:
                    try:
                        self.stop_callback()
                    except Exception as e:
                        logger.error(f"Watchdog stop callback error: {e}")
            
            return True
        
        return False
    
    def get_time_until_trigger(self) -> float:
        """
        Get time remaining until watchdog triggers.
        
        Returns:
            Seconds until trigger (0 if already triggered)
        """
        if not self.enabled:
            return float('inf')
        
        # Use monotonic time
        elapsed = _monotonic() - self.last_update
        remaining = self.timeout - elapsed
        return max(0.0, remaining)
    
    def elapsed(self) -> float:
        """
        Get elapsed time since last refresh/kick.
        
        Returns:
            Seconds since last refresh
        """
        return _monotonic() - self.last_update
    
    def is_timed_out(self) -> bool:
        """Check if watchdog has timed out (alias for should_stop)"""
        return self.should_stop()
    
    def enable(self) -> None:
        """Enable watchdog"""
        self.enabled = True
        self.state = WatchdogState.ACTIVE
        now = _monotonic()
        self.grace_period_end = now + 0.5  # 0.5s grace period
        self.last_update = now
        logger.info(f"Watchdog enabled (timeout={self.timeout:.2f}s)")
    
    def disable(self) -> None:
        """Disable watchdog"""
        self.enabled = False
        self.state = WatchdogState.DISABLED
        logger.info("Watchdog disabled")
    
    def set_timeout(self, timeout_sec: float) -> None:
        """Update watchdog timeout"""
        self.timeout = max(0.1, float(timeout_sec))
        logger.info(f"Watchdog timeout set to {self.timeout:.2f}s")
    
    def reset(self) -> None:
        """Reset watchdog state and trigger count"""
        now = _monotonic()
        self.last_update = now
        self.state = WatchdogState.ACTIVE
        self.trigger_count = 0
        self.grace_period_end = now + 0.5
    
    def get_status(self) -> dict:
        """Get watchdog status"""
        return {
            "enabled": self.enabled,
            "state": self.state.value,
            "timeout": self.timeout,
            "time_until_trigger": self.get_time_until_trigger(),
            "trigger_count": self.trigger_count,
            "last_trigger_time": self.last_trigger_time,
            "last_update": self.last_update,
            "elapsed_since_update": _monotonic() - self.last_update
        }


__all__ = ["MovementWatchdog", "WatchdogState"]

