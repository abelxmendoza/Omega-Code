"""
Linux Motor Driver

Simulated or partial hardware for Linux dev machines (ThinkPad, Ubuntu, etc.).
"""

from .base_motor_driver import BaseMotorDriver


class LinuxMotorDriver(BaseMotorDriver):
    """Simulated or partial hardware for Linux dev machines."""
    
    __slots__ = ()
    
    def set_pwm(self, left_pwm: int, right_pwm: int) -> None:
        """
        Simulate PWM setting (prints to console).
        
        Args:
            left_pwm: PWM value for left motors (0-4095)
            right_pwm: PWM value for right motors (0-4095)
        """
        print(f"[LINUX_SIM] L={left_pwm} R={right_pwm}")
    
    def stop(self) -> None:
        """Simulate stop."""
        print("[LINUX_SIM] STOP")

