"""
Noop Motor Driver

Absolute last resort driver when nothing else works.
"""

from .base_motor_driver import BaseMotorDriver


class NoopMotorDriver(BaseMotorDriver):
    """Used when NOTHING else works."""
    
    __slots__ = ()
    
    def set_pwm(self, left_pwm: int, right_pwm: int) -> None:
        """
        No-op implementation (does nothing).
        
        Args:
            left_pwm: PWM value for left motors (ignored)
            right_pwm: PWM value for right motors (ignored)
        """
        pass
    
    def stop(self) -> None:
        """No-op stop."""
        pass

