"""
Movement V2 - PID Control Module

Wraps advanced PID controller for motor speed stabilization and control.
Provides PID-based speed regulation, straight-drive correction, and servo smoothing.
"""

import time
import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass

try:
    from controllers.advanced_pid import AdvancedPIDController, PIDConfig
    PID_AVAILABLE = True
except ImportError:
    PID_AVAILABLE = False
    logging.warning("AdvancedPIDController not available - PID features disabled")

logger = logging.getLogger(__name__)


@dataclass
class PIDTuning:
    """PID tuning parameters"""
    kp: float = 0.3   # Proportional gain
    ki: float = 0.05  # Integral gain
    kd: float = 0.01  # Derivative gain
    kf: float = 0.0   # Feed-forward gain


class SpeedPID:
    """Optimized PID controller with __slots__ for memory efficiency"""
    __slots__ = (
        'tuning', 'max_rpm', 'max_pwm_correction', 'target_rpm', 'enabled', 'pid'
    )
    
    def __init__(
        self,
        tuning: Optional[PIDTuning] = None,
        max_rpm: float = 300.0,
        max_pwm_correction: float = 500.0
    ):
        """
        Initialize speed PID controller.
        
        Args:
            tuning: PID tuning parameters (default: moderate tuning)
            max_rpm: Maximum motor RPM (for normalization)
            max_pwm_correction: Maximum PWM correction (±PWM units)
        """
        self.tuning = tuning or PIDTuning()
        self.max_rpm = max_rpm
        self.max_pwm_correction = max_pwm_correction
        self.target_rpm = 0.0
        self.enabled = False
        
        if PID_AVAILABLE:
            # Create PID config
            config = PIDConfig(
                kp=self.tuning.kp,
                ki=self.tuning.ki,
                kd=self.tuning.kd,
                kf=self.tuning.kf,
                setpoint=0.0,
                output_limits=(-max_pwm_correction, max_pwm_correction),
                anti_windup=True
            )
            self.pid = AdvancedPIDController(config)
            logger.info("Speed PID controller initialized")
        else:
            self.pid = None
            logger.warning("PID controller unavailable - using pass-through")
    
    def set_target_rpm(self, rpm: float) -> None:
        """Set target RPM for PID control"""
        self.target_rpm = max(0.0, min(self.max_rpm, float(rpm)))
        if self.pid:
            # Normalize RPM to 0-1 range for PID
            normalized = self.target_rpm / self.max_rpm if self.max_rpm > 0 else 0.0
            self.pid.set_setpoint(normalized)
        self.enabled = True
    
    def update(
        self,
        measured_speed: float,
        dt: float = 0.1
    ) -> float:
        """
        Update PID controller with measured speed and return corrected PWM.
        
        This is the main API method matching the blueprint.
        
        Args:
            measured_speed: Measured RPM from telemetry
            dt: Time delta in seconds
        
        Returns:
            Corrected PWM adjustment (±max_pwm_correction)
        """
        return self.compute(current_rpm=measured_speed, dt=dt)
    
    def set_target_speed(self, rpm: float) -> None:
        """Set target speed (alias for set_target_rpm)"""
        self.set_target_rpm(rpm)
    
    def compute(
        self,
        target_rpm: Optional[float] = None,
        current_rpm: float = 0.0,
        dt: float = 0.1
    ) -> float:
        """
        Compute PID correction for PWM adjustment.
        
        Args:
            target_rpm: Target RPM (uses set_target_rpm if None)
            current_rpm: Current RPM from telemetry
            dt: Time delta in seconds
        
        Returns:
            PWM correction value (±max_pwm_correction)
        """
        if not self.enabled or not self.pid:
            return 0.0
        
        if target_rpm is not None:
            self.set_target_rpm(target_rpm)
        
        # Normalize RPM values for PID (0.0 to 1.0)
        target_norm = self.target_rpm / self.max_rpm if self.max_rpm > 0 else 0.0
        current_norm = current_rpm / self.max_rpm if self.max_rpm > 0 else 0.0
        
        # Compute PID output
        try:
            correction = self.pid.update(current_norm, dt)
            # Denormalize back to PWM units
            pwm_correction = correction * self.max_pwm_correction
            return pwm_correction
        except Exception as e:
            logger.error(f"PID compute error: {e}")
            return 0.0
    
    def reset(self) -> None:
        """Reset PID state (clear integral term)"""
        if self.pid:
            self.pid.reset()
    
    def disable(self) -> None:
        """Disable PID control"""
        self.enabled = False
    
    def enable(self) -> None:
        """Enable PID control"""
        self.enabled = True
    
    def update_tuning(self, tuning: PIDTuning) -> None:
        """Update PID tuning parameters"""
        self.tuning = tuning
        if self.pid:
            self.pid.config.kp = tuning.kp
            self.pid.config.ki = tuning.ki
            self.pid.config.kd = tuning.kd
            self.pid.config.kf = tuning.kf


class StraightDrivePID:
    """
    PID controller for straight-drive correction.
    
    Uses trim adjustment to maintain straight-line driving.
    Integrates with existing trim system.
    """
    
    def __init__(self, max_trim: float = 800.0):
        """
        Initialize straight-drive PID controller.
        
        Args:
            max_trim: Maximum trim adjustment (±PWM units)
        """
        self.max_trim = max_trim
        self.enabled = False
        
        if PID_AVAILABLE:
            config = PIDConfig(
                kp=0.5,   # Higher P for responsive correction
                ki=0.1,   # Moderate I for steady-state error
                kd=0.05,  # Low D to avoid oscillation
                setpoint=0.0,  # Target: zero drift
                output_limits=(-max_trim, max_trim),
                anti_windup=True
            )
            self.pid = AdvancedPIDController(config)
        else:
            self.pid = None
    
    def compute_correction(
        self,
        drift_error: float,  # Positive = veering right, Negative = veering left
        dt: float = 0.1
    ) -> tuple[float, float]:
        """
        Compute trim correction for left/right motors.
        
        Args:
            drift_error: Drift error (can be from IMU, vision, or odometry)
            dt: Time delta
        
        Returns:
            Tuple of (left_trim_adjustment, right_trim_adjustment)
        """
        if not self.enabled or not self.pid:
            return (0.0, 0.0)
        
        try:
            correction = self.pid.update(drift_error, dt)
            # Apply correction: if veering right, increase left trim
            left_adjust = correction
            right_adjust = -correction
            return (left_adjust, right_adjust)
        except Exception as e:
            logger.error(f"Straight-drive PID error: {e}")
            return (0.0, 0.0)
    
    def reset(self) -> None:
        """Reset PID state"""
        if self.pid:
            self.pid.reset()
    
    def enable(self) -> None:
        """Enable straight-drive PID"""
        self.enabled = True
    
    def disable(self) -> None:
        """Disable straight-drive PID"""
        self.enabled = False


class ServoSmoothPID:
    """
    PID controller for smooth servo movement.
    
    Provides smooth transitions between servo positions.
    """
    
    def __init__(self, max_rate: float = 5.0):  # degrees per second
        """
        Initialize servo smoothing PID.
        
        Args:
            max_rate: Maximum servo movement rate (degrees/second)
        """
        self.max_rate = max_rate
        self.enabled = False
        
        if PID_AVAILABLE:
            config = PIDConfig(
                kp=2.0,   # High P for responsive movement
                ki=0.0,   # No I (servos don't need integral)
                kd=0.5,   # D for smooth deceleration
                setpoint=0.0,
                output_limits=(-max_rate, max_rate),
                anti_windup=False
            )
            self.pid = AdvancedPIDController(config)
        else:
            self.pid = None
    
    def compute_rate(
        self,
        target_angle: float,
        current_angle: float,
        dt: float = 0.1
    ) -> float:
        """
        Compute servo movement rate for smooth transition.
        
        Args:
            target_angle: Target angle (degrees)
            current_angle: Current angle (degrees)
            dt: Time delta
        
        Returns:
            Movement rate (degrees/second)
        """
        if not self.enabled or not self.pid:
            # Fallback: direct movement
            return (target_angle - current_angle) / dt if dt > 0 else 0.0
        
        error = target_angle - current_angle
        if abs(error) < 0.1:
            return 0.0
        
        try:
            rate = self.pid.update(current_angle, dt, feedforward=error)
            return rate
        except Exception as e:
            logger.error(f"Servo PID error: {e}")
            return (target_angle - current_angle) / dt if dt > 0 else 0.0
    
    def enable(self) -> None:
        """Enable servo smoothing"""
        self.enabled = True
    
    def disable(self) -> None:
        """Disable servo smoothing"""
        self.enabled = False


__all__ = ["SpeedPID", "StraightDrivePID", "ServoSmoothPID", "PIDTuning", "PID_AVAILABLE"]

