"""
Base Motor Driver - Abstract Hardware Interface

All platform-specific motor drivers must inherit from this base class
and implement the required methods.
"""

from abc import ABC, abstractmethod
from typing import Tuple


class BaseMotorDriver(ABC):
    """
    Abstract hardware driver interface.
    
    All platforms must implement:
        set_pwm(left, right)
        stop()
        cleanup()
    """
    
    __slots__ = ("trim_left", "trim_right")
    
    def __init__(self, trim_left: int = 0, trim_right: int = 0):
        """
        Initialize motor driver with trim values.
        
        Args:
            trim_left: Trim offset for left motors (PWM units)
            trim_right: Trim offset for right motors (PWM units)
        """
        self.trim_left = trim_left
        self.trim_right = trim_right
    
    @abstractmethod
    def set_pwm(self, left_pwm: int, right_pwm: int) -> None:
        """
        Set PWM values for left and right motors.
        
        Args:
            left_pwm: PWM value for left motors (0-4095)
            right_pwm: PWM value for right motors (0-4095)
        """
        raise NotImplementedError
    
    @abstractmethod
    def stop(self) -> None:
        """Stop all motors immediately."""
        raise NotImplementedError
    
    def cleanup(self) -> None:
        """Cleanup resources (override in subclasses if needed)."""
        pass
    
    def set_trim(self, trim_left: int = 0, trim_right: int = 0) -> Tuple[int, int]:
        """
        Update trim values.
        
        Args:
            trim_left: New trim for left motors
            trim_right: New trim for right motors
        
        Returns:
            Tuple of (trim_left, trim_right)
        """
        self.trim_left = trim_left
        self.trim_right = trim_right
        return (self.trim_left, self.trim_right)
    
    def get_trim(self) -> Tuple[int, int]:
        """
        Get current trim values.
        
        Returns:
            Tuple of (trim_left, trim_right)
        """
        return (self.trim_left, self.trim_right)

