"""
Mock PCA9685 Driver

Simulation implementation for testing without hardware.
"""


class PCA9685:
    """Mock PCA9685 that simulates hardware behavior."""
    
    def __init__(self, address: int = 0x40, debug: bool = False):
        """
        Initialize mock PCA9685.
        
        Args:
            address: I2C address (ignored in mock)
            debug: Enable debug output
        """
        self.address = address
        self.debug = debug
        self.channels = [0] * 16  # Track PWM values for each channel
    
    def setPWMFreq(self, freq: int) -> None:
        """Mock PWM frequency setting."""
        if self.debug:
            print(f"[MOCK PCA9685] Set PWM frequency to {freq}Hz")
    
    def setPWM(self, channel: int, on: int, off: int) -> None:
        """Mock PWM setting."""
        if 0 <= channel < 16:
            self.channels[channel] = off
            if self.debug:
                print(f"[MOCK PCA9685] Channel {channel}: on={on}, off={off}")
    
    def setMotorPwm(self, channel: int, duty: int) -> None:
        """Mock motor PWM setting."""
        self.setPWM(channel, 0, duty)
        if self.debug:
            print(f"[MOCK PCA9685] Motor channel {channel}: duty={duty}")
    
    def setServoPulse(self, channel: int, pulse: int) -> None:
        """Mock servo pulse setting."""
        pulse_pwm = pulse * 4096 / 20000
        self.setPWM(channel, 0, int(pulse_pwm))
        if self.debug:
            print(f"[MOCK PCA9685] Servo channel {channel}: pulse={pulse}us -> PWM={int(pulse_pwm)}")

