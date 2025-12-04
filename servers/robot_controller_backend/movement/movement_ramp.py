"""
Movement V2 - Speed Ramping Module

Handles smooth acceleration and deceleration curves to replace instant PWM changes.
Supports linear, exponential, and S-curve ramping profiles.
"""

import time
import math
from typing import Optional
from enum import Enum

# Performance: Use monotonic time (faster, not affected by system clock changes)
_monotonic = time.monotonic

# Performance: Pre-compute constants
_TOLERANCE = 0.1
_TWO_PI = 2.0 * math.pi


class RampType(Enum):
    """Ramping curve types"""
    LINEAR = "linear"
    EXPONENTIAL = "exponential"
    S_CURVE = "s_curve"


class MovementRamp:
    """Optimized movement ramp with __slots__ for memory efficiency"""
    __slots__ = (
        'current_pwm', 'target_pwm', 'accel_rate', 'decel_rate', 'ramp_type',
        'min_pwm', 'max_pwm', '_last_update_time', '_is_ramping',
        '_ramp_start_pwm', '_ramp_start_time', '_s_curve_tension'
    )
    """
    Handles speed ramping: smooth acceleration/deceleration.
    Replaces instant PWM jumps with controlled transitions.
    
    Usage:
        ramp = MovementRamp(accel_rate=150, decel_rate=200)
        ramp.set_target(2000)  # Target PWM
        while not ramp.is_at_target():
            current_pwm = ramp.update(dt)  # Call every control loop
            motor.setMotors(current_pwm)
    """
    
    def __init__(
        self,
        accel_rate: float = 150.0,  # PWM units per second
        decel_rate: float = 200.0,  # PWM units per second
        ramp_type: RampType = RampType.LINEAR,
        min_pwm: int = 0,
        max_pwm: int = 4095
    ):
        """
        Initialize movement ramp controller.
        
        Args:
            accel_rate: Acceleration rate (PWM units per second)
            decel_rate: Deceleration rate (PWM units per second)
            ramp_type: Type of ramping curve (linear/exponential/s_curve)
            min_pwm: Minimum PWM value
            max_pwm: Maximum PWM value
        """
        self.current_pwm = 0.0
        self.target_pwm = 0.0
        self.accel_rate = accel_rate
        self.decel_rate = decel_rate
        self.ramp_type = ramp_type
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm
        
        # State tracking (use monotonic time for better performance)
        self._last_update_time = _monotonic()
        self._is_ramping = False
        self._ramp_start_pwm = 0.0
        self._ramp_start_time = 0.0
        
        # S-curve parameters
        self._s_curve_tension = 0.5  # 0.0 = linear, 1.0 = very curved
    
    def set_target(self, pwm: float) -> None:
        """
        Set target PWM value to ramp to.
        
        Args:
            pwm: Target PWM value (will be clamped to min_pwm..max_pwm)
        """
        pwm = max(self.min_pwm, min(self.max_pwm, float(pwm)))
        
        # Performance: Cache tolerance check
        if abs(pwm - self.target_pwm) < _TOLERANCE:
            return  # Already at target
        
        self.target_pwm = pwm
        self._ramp_start_pwm = self.current_pwm
        self._ramp_start_time = _monotonic()  # Use monotonic time
        self._is_ramping = True
        print(f"⤴️ [RAMPING] Starting ramp: current={self.current_pwm:.0f}, target={self.target_pwm:.0f}")
    
    def update(self, dt: Optional[float] = None) -> float:
        """
        Update ramp state and return current PWM value.
        
        Args:
            dt: Time delta in seconds. If None, calculates automatically.
        
        Returns:
            Current PWM value after ramping
        """
        if dt is None:
            current_time = _monotonic()  # Use monotonic time
            dt = current_time - self._last_update_time
            self._last_update_time = current_time
        
        if dt <= 0:
            return self.current_pwm
        
        # Performance: Cache tolerance and diff calculations
        diff = self.current_pwm - self.target_pwm
        abs_diff = abs(diff)
        
        # Check if we're at target
        if abs_diff < _TOLERANCE:
            self.current_pwm = self.target_pwm
            self._is_ramping = False
            return self.current_pwm
        
        # Determine if accelerating or decelerating (optimized)
        is_accelerating = abs(self.target_pwm) > abs(self.current_pwm)
        rate = self.accel_rate if is_accelerating else self.decel_rate
        
        # Calculate new PWM based on ramp type
        if self.ramp_type == RampType.LINEAR:
            self.current_pwm = self._linear_ramp(dt, rate)
        elif self.ramp_type == RampType.EXPONENTIAL:
            self.current_pwm = self._exponential_ramp(dt, rate)
        elif self.ramp_type == RampType.S_CURVE:
            self.current_pwm = self._s_curve_ramp(dt, rate)
        else:
            self.current_pwm = self._linear_ramp(dt, rate)
        
        # Clamp to limits (optimized: reuse min/max)
        self.current_pwm = max(self.min_pwm, min(self.max_pwm, self.current_pwm))
        
        # Snap to target if very close (reuse abs_diff calculation)
        if abs(self.current_pwm - self.target_pwm) < _TOLERANCE:
            self.current_pwm = self.target_pwm
            self._is_ramping = False
        
        return self.current_pwm
    
    def _linear_ramp(self, dt: float, rate: float) -> float:
        """Linear ramping: constant rate"""
        delta = rate * dt
        if self.target_pwm > self.current_pwm:
            return min(self.current_pwm + delta, self.target_pwm)
        else:
            return max(self.current_pwm - delta, self.target_pwm)
    
    def _exponential_ramp(self, dt: float, rate: float) -> float:
        """Exponential ramping: fast start, slow finish (optimized)"""
        # Use monotonic time
        elapsed = _monotonic() - self._ramp_start_time
        if elapsed <= 0:
            return self.current_pwm
        
        diff = self.target_pwm - self._ramp_start_pwm
        abs_diff = abs(diff)
        if abs_diff < _TOLERANCE:
            return self.target_pwm
        
        # Performance: Cache division result
        if abs_diff > 0:
            rate_normalized = rate * elapsed / abs_diff
        else:
            rate_normalized = rate * elapsed
        
        # Exponential decay factor (optimized)
        factor = 1.0 - math.exp(-rate_normalized)
        return self._ramp_start_pwm + diff * factor
    
    def _s_curve_ramp(self, dt: float, rate: float) -> float:
        """S-curve ramping: slow start, fast middle, slow finish (optimized)"""
        # Use monotonic time
        elapsed = _monotonic() - self._ramp_start_time
        if elapsed <= 0:
            return self.current_pwm
        
        diff = self.target_pwm - self._ramp_start_pwm
        abs_diff = abs(diff)
        if abs_diff < _TOLERANCE:
            return self.target_pwm
        
        # Performance: Optimize time normalization
        if rate > 0:
            total_time = abs_diff / rate
            t = min(elapsed / total_time, 1.0) if total_time > 0 else 1.0
        else:
            t = 1.0
        
        # S-curve: smoothstep function (optimized: cache t²)
        t_sq = t * t
        s = 3.0 * t_sq - 2.0 * t_sq * t  # 3t² - 2t³
        
        return self._ramp_start_pwm + diff * s
    
    def is_at_target(self, tolerance: float = _TOLERANCE) -> bool:
        """Check if current PWM is at target within tolerance"""
        return abs(self.current_pwm - self.target_pwm) < tolerance
    
    def is_ramping(self) -> bool:
        """Check if currently ramping"""
        return self._is_ramping and not self.is_at_target()
    
    def get_current(self) -> float:
        """Get current PWM value"""
        return self.current_pwm
    
    def get_target(self) -> float:
        """Get target PWM value"""
        return self.target_pwm
    
    def stop(self) -> None:
        """Immediately stop ramping and set to zero"""
        self.set_target(0)
        self.current_pwm = 0.0
        self._is_ramping = False
    
    def reset(self) -> None:
        """Reset ramp state to zero (alias for stop)"""
        self.stop()
    
    def is_active(self) -> bool:
        """Check if ramp is currently active (alias for is_ramping)"""
        return self.is_ramping()
    
    def set_ramp_type(self, ramp_type: RampType) -> None:
        """Change ramping curve type"""
        self.ramp_type = ramp_type
    
    def set_rates(self, accel_rate: Optional[float] = None, decel_rate: Optional[float] = None) -> None:
        """Update acceleration/deceleration rates"""
        if accel_rate is not None:
            self.accel_rate = max(1.0, accel_rate)
        if decel_rate is not None:
            self.decel_rate = max(1.0, decel_rate)


__all__ = ["MovementRamp", "RampType"]

