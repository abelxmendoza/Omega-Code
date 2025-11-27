"""
macOS Motor Driver

Simulated motor driver for macOS development.
"""

from .base_motor_driver import BaseMotorDriver


class MacMotorDriver(BaseMotorDriver):
    """Simulated motor driver for macOS development."""
    
    __slots__ = ()
    
    def set_pwm(self, left_pwm: int, right_pwm: int) -> None:
        """
        Simulate PWM setting (prints to console).
        
        Args:
            left_pwm: PWM value for left motors (0-4095)
            right_pwm: PWM value for right motors (0-4095)
        """
        print(f"[MAC_SIM] L={left_pwm} R={right_pwm}")
    
    def stop(self) -> None:
        """Simulate stop."""
        print("[MAC_SIM] STOP")

