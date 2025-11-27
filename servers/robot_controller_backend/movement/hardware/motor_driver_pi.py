"""
Raspberry Pi Motor Driver

Real PCA9685 motor control on Raspberry Pi hardware.
"""

from .base_motor_driver import BaseMotorDriver
from .pca9685_real import PCA9685
from servers.robot_controller_backend.hardware.pwm_singleton import get_pca


class PiMotorDriver(BaseMotorDriver):
    """Real PCA9685 motor control on Raspberry Pi."""
    
    __slots__ = ("pca",)
    
    def __init__(self, trim_left: int = 0, trim_right: int = 0):
        """
        Initialize Raspberry Pi motor driver.
        
        Args:
            trim_left: Trim offset for left motors
            trim_right: Trim offset for right motors
        """
        super().__init__(trim_left, trim_right)
        # Use singleton PCA9685 instance (shared across all components)
        self.pca = get_pca(PCA9685, 0x40, debug=True)
        self.pca.setPWMFreq(50)
    
    def set_pwm(self, left_pwm: int, right_pwm: int, left_reverse: bool = False, right_reverse: bool = False) -> None:
        """
        Set PWM values for left and right motors with direction control.
        
        Args:
            left_pwm: PWM value for left motors (0-4095, unsigned)
            right_pwm: PWM value for right motors (0-4095, unsigned)
            left_reverse: If True, reverse left motors (for backward motion)
            right_reverse: If True, reverse right motors (for backward motion)
        """
        # Apply trim
        left = max(0, min(4095, left_pwm + self.trim_left))
        right = max(0, min(4095, right_pwm + self.trim_right))
        
        # Left motors: forward channels 0/4, reverse channels 1/5
        if left_reverse:
            self.pca.setMotorPwm(0, 0)
            self.pca.setMotorPwm(4, 0)
            self.pca.setMotorPwm(1, left)
            self.pca.setMotorPwm(5, left)
        else:
            self.pca.setMotorPwm(0, left)
            self.pca.setMotorPwm(4, left)
            self.pca.setMotorPwm(1, 0)
            self.pca.setMotorPwm(5, 0)
        
        # Right motors: forward channels 2/6, reverse channels 3/7
        if right_reverse:
            self.pca.setMotorPwm(2, 0)
            self.pca.setMotorPwm(6, 0)
            self.pca.setMotorPwm(3, right)
            self.pca.setMotorPwm(7, right)
        else:
            self.pca.setMotorPwm(2, right)
            self.pca.setMotorPwm(6, right)
            self.pca.setMotorPwm(3, 0)
            self.pca.setMotorPwm(7, 0)
    
    def stop(self) -> None:
        """Stop all motors immediately."""
        # Stop all channels (forward and reverse)
        self.pca.setMotorPwm(0, 0)
        self.pca.setMotorPwm(1, 0)
        self.pca.setMotorPwm(2, 0)
        self.pca.setMotorPwm(3, 0)
        self.pca.setMotorPwm(4, 0)
        self.pca.setMotorPwm(5, 0)
        self.pca.setMotorPwm(6, 0)
        self.pca.setMotorPwm(7, 0)
    
    def cleanup(self) -> None:
        """Cleanup resources."""
        self.stop()

