# /Omega-Code/servers/robot_controller_backend/movement/minimal_motor_control.py
"""Minimal motor controller with basic trim support.

Now uses the multi-platform hardware abstraction layer for automatic
platform detection and driver selection.
"""

import time
from typing import Dict

# Use the new hardware abstraction layer
from servers.robot_controller_backend.movement.hardware import get_motor_driver
from servers.robot_controller_backend.movement.hardware.base_motor_driver import BaseMotorDriver

MAX_PWM = 4095

class Motor:
    """
    Optimized motor controller using multi-platform hardware abstraction.
    
    Uses __slots__ for memory efficiency and automatic platform detection.
    Wraps the platform-specific driver with a unified interface.
    """
    __slots__ = ("driver", "left_trim", "right_trim")
    
    MAX_PWM = MAX_PWM
    
    def __init__(self, trim_left: int = 0, trim_right: int = 0):
        """
        Initialize motor controller with platform-specific driver.
        
        Args:
            trim_left: Initial trim for left motors
            trim_right: Initial trim for right motors
        """
        # Get platform-specific driver (auto-detected)
        self.driver: BaseMotorDriver = get_motor_driver(trim_left=trim_left, trim_right=trim_right)
        self.left_trim = trim_left
        self.right_trim = trim_right

    def setMotors(self, duty: int) -> None:
        """
        All 4 wheels same direction (tank forward/back).
        
        Args:
            duty: PWM duty cycle (-4095 to 4095, negative = backward)
        """
        # Apply trim before converting to unsigned
        left_duty = self._apply_trim(duty, self.left_trim)
        right_duty = self._apply_trim(duty, self.right_trim)
        # Use low-level helper which handles signed duty
        self._set_lr(left_duty, right_duty)

    # Straight-drive helpers -------------------------------------------------

    def set_trim(self, left: int | float = 0, right: int | float = 0) -> Dict[str, int]:
        """
        Configure compensation offsets for the left and right wheel pairs.
        
        Args:
            left: Trim value for left motors
            right: Trim value for right motors
        
        Returns:
            Dict with current trim values
        """
        self.left_trim = self._clamp_trim(int(left))
        self.right_trim = self._clamp_trim(int(right))
        # Update driver trim as well
        self.driver.set_trim(self.left_trim, self.right_trim)
        return self.get_trim()

    def get_trim(self) -> Dict[str, int]:
        """Return the currently configured trim values."""
        return {"left": self.left_trim, "right": self.right_trim}

    def forward(self, speed=2000):
        print("Moving forward")
        self.setMotors(speed)

    def backward(self, speed=2000):
        print("Moving backward")
        self.setMotors(-speed)

    def stop(self) -> None:
        """Stop all motors immediately."""
        print("Stopping")
        self.driver.stop()

    # --- NEW: turning helpers ---

    def pivot_left(self, speed=1500):
        """Left side backward, right side forward (in-place turn)."""
        print("Pivot left")
        L = -abs(speed)
        R = +abs(speed)
        self._set_lr(L, R)

    def pivot_right(self, speed=1500):
        """Right side backward, left side forward (in-place turn)."""
        print("Pivot right")
        L = +abs(speed)
        R = -abs(speed)
        self._set_lr(L, R)

    def left(self, speed=1500, ratio=0.5):
        """Gentle left: slow left side, full right side forward."""
        print("Turn left")
        L = int(abs(speed) * ratio)
        R = +abs(speed)
        self._set_lr(L, R)

    def right(self, speed=1500, ratio=0.5):
        """Gentle right: slow right side, full left side forward."""
        print("Turn right")
        L = +abs(speed)
        R = int(abs(speed) * ratio)
        self._set_lr(L, R)

    # Low-level helper: set left/right sides with signed duty
    def _set_lr(self, left_duty: int, right_duty: int) -> None:
        """
        Set left and right motor PWM values.
        
        Args:
            left_duty: PWM value for left motors (-4095 to 4095, signed)
            right_duty: PWM value for right motors (-4095 to 4095, signed)
        """
        # Clamp signed duty values
        left_duty = self._clamp_duty(left_duty)
        right_duty = self._clamp_duty(right_duty)
        
        # Convert signed duty to unsigned PWM
        left_pwm = abs(left_duty) if left_duty != 0 else 0
        right_pwm = abs(right_duty) if right_duty != 0 else 0
        
        # Check if driver supports reverse parameter (PiMotorDriver)
        if hasattr(self.driver, 'set_pwm') and 'left_reverse' in self.driver.set_pwm.__code__.co_varnames:
            # PiMotorDriver supports reverse parameter
            left_reverse = left_duty < 0
            right_reverse = right_duty < 0
            self.driver.set_pwm(left_pwm, right_pwm, left_reverse=left_reverse, right_reverse=right_reverse)
        else:
            # Other drivers (Mac, Linux, Sim) just use unsigned PWM
            self.driver.set_pwm(left_pwm, right_pwm)

    # Internal helpers -------------------------------------------------------

    def _apply_trim(self, duty: int, trim: int) -> int:
        if duty == 0 or trim == 0:
            return duty
        return duty + (trim if duty > 0 else -trim)

    def _clamp_duty(self, duty: int) -> int:
        return max(-MAX_PWM, min(MAX_PWM, int(duty)))

    def _clamp_trim(self, trim: int) -> int:
        return max(-MAX_PWM, min(MAX_PWM, trim))

if __name__ == "__main__":
    m = Motor()
    try:
        m.forward(1800); time.sleep(1)
        m.left(1600);    time.sleep(1)
        m.right(1600);   time.sleep(1)
        m.pivot_left(1500);  time.sleep(0.8)
        m.pivot_right(1500); time.sleep(0.8)
        m.backward(1800); time.sleep(1)
    finally:
        m.stop()
