"""
Jetson Orin Nano Motor Driver

PWM motor control using Jetson GPIO.
"""

from .base_motor_driver import BaseMotorDriver

try:
    import Jetson.GPIO as GPIO
    JETSON_GPIO_AVAILABLE = True
except ImportError:
    JETSON_GPIO_AVAILABLE = False


class OrinMotorDriver(BaseMotorDriver):
    """Jetson Orin Nano PWM motor control."""
    
    __slots__ = ("left_pin", "right_pin", "left_pwm", "right_pwm", "_initialized")
    
    def __init__(self, trim_left: int = 0, trim_right: int = 0):
        """
        Initialize Jetson Orin Nano motor driver.
        
        Args:
            trim_left: Trim offset for left motors
            trim_right: Trim offset for right motors
        """
        super().__init__(trim_left, trim_right)
        
        if not JETSON_GPIO_AVAILABLE:
            raise ImportError("Jetson.GPIO not available. Install Jetson GPIO library.")
        
        self.left_pin = 33
        self.right_pin = 32
        self._initialized = False
        
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.left_pin, GPIO.OUT)
            GPIO.setup(self.right_pin, GPIO.OUT)
            
            self.left_pwm = GPIO.PWM(self.left_pin, 1000)
            self.right_pwm = GPIO.PWM(self.right_pin, 1000)
            
            self.left_pwm.start(0)
            self.right_pwm.start(0)
            self._initialized = True
        except Exception as e:
            raise RuntimeError(f"Failed to initialize Jetson GPIO: {e}")
    
    def set_pwm(self, left_pwm: int, right_pwm: int) -> None:
        """
        Set PWM values for left and right motors.
        
        Args:
            left_pwm: PWM value for left motors (0-4095)
            right_pwm: PWM value for right motors (0-4095)
        """
        if not self._initialized:
            return
        
        # Convert 0-4095 to 0-100 duty cycle
        l = max(0, min(100, int((left_pwm + self.trim_left) / 40.95)))
        r = max(0, min(100, int((right_pwm + self.trim_right) / 40.95)))
        
        self.left_pwm.ChangeDutyCycle(l)
        self.right_pwm.ChangeDutyCycle(r)
    
    def stop(self) -> None:
        """Stop all motors immediately."""
        if self._initialized:
            self.left_pwm.ChangeDutyCycle(0)
            self.right_pwm.ChangeDutyCycle(0)
    
    def cleanup(self) -> None:
        """Cleanup resources."""
        self.stop()
        if self._initialized:
            try:
                self.left_pwm.stop()
                self.right_pwm.stop()
                GPIO.cleanup()
                self._initialized = False
            except Exception:
                pass

