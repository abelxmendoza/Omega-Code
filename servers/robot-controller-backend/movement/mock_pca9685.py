# File: /Omega-Code/servers/robot-controller-backend/movement/mock_pca9685.py
"""
Mock PCA9685 for testing motor telemetry without hardware dependencies.
"""

class MockPCA9685:
    """Mock PCA9685 that simulates hardware behavior"""
    
    def __init__(self, address=0x40, debug=False):
        self.address = address
        self.debug = debug
        self.channels = [0] * 16  # Track PWM values for each channel
        
    def setPWMFreq(self, freq):
        """Mock PWM frequency setting"""
        if self.debug:
            print(f"[MOCK PCA9685] Set PWM frequency to {freq}Hz")
    
    def setPWM(self, channel, on, off):
        """Mock PWM setting"""
        if 0 <= channel < 16:
            self.channels[channel] = off
            if self.debug:
                print(f"[MOCK PCA9685] Channel {channel}: on={on}, off={off}")
    
    def setMotorPwm(self, channel, duty):
        """Mock motor PWM setting"""
        self.setPWM(channel, 0, duty)
        if self.debug:
            print(f"[MOCK PCA9685] Motor channel {channel}: duty={duty}")
    
    def setServoPulse(self, channel, pulse):
        """Mock servo pulse setting"""
        pulse_pwm = pulse * 4096 / 20000
        self.setPWM(channel, 0, int(pulse_pwm))
        if self.debug:
            print(f"[MOCK PCA9685] Servo channel {channel}: pulse={pulse}us -> PWM={int(pulse_pwm)}")

# Create a mock PCA9685 module
import sys
sys.modules['PCA9685'] = MockPCA9685
