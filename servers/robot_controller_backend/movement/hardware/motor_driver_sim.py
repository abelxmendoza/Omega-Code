"""
Simulation Motor Driver

Pure simulation driver used when ROBOT_SIM=1.
"""

from .base_motor_driver import BaseMotorDriver


class SimMotorDriver(BaseMotorDriver):
    """Simulation — used when ROBOT_SIM=1."""
    
    __slots__ = ()
    
    def set_pwm(self, left_pwm: int, right_pwm: int) -> None:
        """
        Simulate PWM setting (prints to console).
        
        Args:
            left_pwm: PWM value for left motors (0-4095)
            right_pwm: PWM value for right motors (0-4095)
        """
        print(f"[SIM] L={left_pwm} R={right_pwm}")
    
    def stop(self) -> None:
        """Simulate stop."""
        print("[SIM] STOP")

