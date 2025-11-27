"""
Movement V2 - Thermal and Current Safety Module

Monitors motor temperature and current, automatically throttling or stopping
motors when limits are exceeded.
"""

import time
import logging
from typing import Dict, Any, Optional
from enum import Enum
from dataclasses import dataclass

# Performance: Use monotonic time
_monotonic = time.monotonic

# Performance: Pre-define motor names tuple (faster iteration than list)
_MOTOR_NAMES = ('frontLeft', 'frontRight', 'rearLeft', 'rearRight')

logger = logging.getLogger(__name__)


class SafetyState(Enum):
    """Thermal safety states"""
    OK = "ok"
    WARNING = "warning"
    THROTTLE = "throttle"
    KILL = "kill"


@dataclass
class ThermalLimits:
    """Thermal and current safety limits"""
    max_temp: float = 75.0        # Maximum temperature (°C)
    warning_temp: float = 60.0    # Warning temperature (°C)
    max_current: float = 2.5      # Maximum current per motor (A)
    warning_current: float = 2.0  # Warning current (A)
    cooldown_temp: float = 50.0   # Temperature to resume operation (°C)
    throttle_factor: float = 0.5   # Speed reduction factor when throttling


class ThermalSafety:
    """Optimized thermal safety with __slots__ for memory efficiency"""
    __slots__ = (
        'limits', 'enabled', 'state', 'last_check_time', 'throttle_start_time',
        'kill_start_time', 'max_temp_seen', 'max_current_seen', 'warning_count',
        'throttle_count', 'kill_count'
    )
    
    def __init__(
        self,
        limits: Optional[ThermalLimits] = None,
        enabled: bool = True
    ):
        """
        Initialize thermal safety system.
        
        Args:
            limits: Thermal and current limits
            enabled: Whether thermal safety is enabled
        """
        self.limits = limits or ThermalLimits()
        self.enabled = enabled
        
        self.state = SafetyState.OK
        self.last_check_time = _monotonic()  # Use monotonic time
        self.throttle_start_time = 0.0
        self.kill_start_time = 0.0
        
        # State tracking
        self.max_temp_seen = 0.0
        self.max_current_seen = 0.0
        self.warning_count = 0
        self.throttle_count = 0
        self.kill_count = 0
    
    def check(self, telemetry: Dict[str, Any]) -> SafetyState:
        """
        Check telemetry and return safety state.
        
        Args:
            telemetry: Motor telemetry dict from MotorTelemetryController
        
        Returns:
            SafetyState: ok, warning, throttle, or kill
        """
        if not self.enabled:
            return SafetyState.OK
        
        self.last_check_time = _monotonic()  # Use monotonic time
        
        # Performance: Use tuple iteration and optimize dict lookups
        max_temp = 0.0
        max_current = 0.0
        
        # Performance: Direct dict access (faster than .get() with defaults)
        for motor_name in _MOTOR_NAMES:
            motor_data = telemetry.get(motor_name)
            if motor_data:
                # Performance: Direct access with fallback (faster than lambda)
                temp = motor_data.get('temperature', 25.0)
                current = motor_data.get('current', 0.0)
                
                # Performance: Direct comparison (faster than max())
                if temp > max_temp:
                    max_temp = temp
                if current > max_current:
                    max_current = current
        
        # Update max values seen
        self.max_temp_seen = max(self.max_temp_seen, max_temp)
        self.max_current_seen = max(self.max_current_seen, max_current)
        
        # Check limits and determine state
        new_state = self._evaluate_state(max_temp, max_current)
        
        # State transition logging
        if new_state != self.state:
            self._log_state_transition(new_state, max_temp, max_current)
            self.state = new_state
            
            # Use monotonic time
            now = _monotonic()
            if new_state == SafetyState.THROTTLE:
                self.throttle_start_time = now
                self.throttle_count += 1
            elif new_state == SafetyState.KILL:
                self.kill_start_time = now
                self.kill_count += 1
            elif new_state == SafetyState.WARNING:
                self.warning_count += 1
        
        return self.state
    
    def _evaluate_state(self, max_temp: float, max_current: float) -> SafetyState:
        """Evaluate safety state based on temperature and current (optimized)"""
        # Performance: Cache limit access
        limits = self.limits
        max_temp_limit = limits.max_temp
        max_current_limit = limits.max_current
        warning_temp = limits.warning_temp
        warning_current = limits.warning_current
        
        # Kill conditions (highest priority) - check most restrictive first
        if max_temp >= max_temp_limit or max_current >= max_current_limit:
            return SafetyState.KILL
        
        # Throttle conditions
        if max_temp >= warning_temp or max_current >= warning_current:
            return SafetyState.THROTTLE
        
        # Performance: Pre-compute warning thresholds
        warning_temp_threshold = warning_temp * 0.8
        warning_current_threshold = warning_current * 0.7
        
        # Warning conditions
        if max_temp >= warning_temp_threshold or max_current >= (warning_current * 0.8):
            return SafetyState.WARNING
        
        # Recovery from throttle/kill (optimized: use tuple for 'in' check)
        current_state = self.state
        if current_state in (SafetyState.THROTTLE, SafetyState.KILL):
            if max_temp <= limits.cooldown_temp and max_current <= warning_current_threshold:
                return SafetyState.OK
            return current_state  # Maintain state
        
        return SafetyState.OK
    
    def _log_state_transition(self, new_state: SafetyState, temp: float, current: float) -> None:
        """Log state transition"""
        if new_state == SafetyState.KILL:
            logger.error(
                f"THERMAL SAFETY: KILL - Temp: {temp:.1f}°C, Current: {current:.2f}A "
                f"(limits: {self.limits.max_temp}°C, {self.limits.max_current}A)"
            )
        elif new_state == SafetyState.THROTTLE:
            logger.warning(
                f"THERMAL SAFETY: THROTTLE - Temp: {temp:.1f}°C, Current: {current:.2f}A"
            )
        elif new_state == SafetyState.WARNING:
            logger.info(
                f"THERMAL SAFETY: WARNING - Temp: {temp:.1f}°C, Current: {current:.2f}A"
            )
        elif new_state == SafetyState.OK:
            logger.info("THERMAL SAFETY: OK - Motors within safe limits")
    
    def get_throttle_factor(self) -> float:
        """
        Get speed reduction factor when throttling.
        
        Returns:
            Multiplier for PWM (0.0-1.0)
        """
        if self.state == SafetyState.THROTTLE:
            return self.limits.throttle_factor
        elif self.state == SafetyState.WARNING:
            return 0.75  # Slight reduction on warning
        return 1.0  # No reduction
    
    def apply_throttle(self, pwm: float) -> float:
        """
        Apply thermal throttling to PWM value.
        
        Args:
            pwm: Input PWM value
        
        Returns:
            Throttled PWM value
        """
        if self.state == SafetyState.KILL:
            return 0.0  # Kill: stop motors
        
        factor = self.get_throttle_factor()
        return float(pwm) * factor
    
    def apply_limits(self, pwm: float, telemetry: Dict[str, Any]) -> float:
        """
        Apply thermal limits to PWM value based on telemetry.
        
        This is the main API method that checks telemetry and applies limits.
        
        Args:
            pwm: Input PWM value
            telemetry: Motor telemetry dict from MotorTelemetryController
        
        Returns:
            Safe PWM value (throttled or killed if needed)
        """
        # Check telemetry and update state
        state = self.check(telemetry)
        
        # Apply throttling based on state
        return self.apply_throttle(pwm)
    
    def is_safe(self) -> bool:
        """Check if motors are safe to operate"""
        return self.state == SafetyState.OK
    
    def enable(self) -> None:
        """Enable thermal safety"""
        self.enabled = True
        logger.info("Thermal safety enabled")
    
    def disable(self) -> None:
        """Disable thermal safety (not recommended)"""
        self.enabled = False
        logger.warning("Thermal safety DISABLED - motors may overheat!")
    
    def update_limits(self, limits: ThermalLimits) -> None:
        """Update thermal and current limits"""
        self.limits = limits
        logger.info(f"Thermal limits updated: max_temp={limits.max_temp}°C, max_current={limits.max_current}A")
    
    def get_status(self) -> Dict[str, Any]:
        """Get thermal safety status"""
        return {
            "enabled": self.enabled,
            "state": self.state.value,
            "max_temp_seen": self.max_temp_seen,
            "max_current_seen": self.max_current_seen,
            "warning_count": self.warning_count,
            "throttle_count": self.throttle_count,
            "kill_count": self.kill_count,
            "limits": {
                "max_temp": self.limits.max_temp,
                "warning_temp": self.limits.warning_temp,
                "max_current": self.limits.max_current,
                "warning_current": self.limits.warning_current,
                "cooldown_temp": self.limits.cooldown_temp,
                "throttle_factor": self.limits.throttle_factor
            }
        }


__all__ = ["ThermalSafety", "SafetyState", "ThermalLimits"]

