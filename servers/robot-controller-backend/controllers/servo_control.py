# File: /Omega-Code/servers/robot-controller-backend/servo_control.py

"""
This script controls the servo motors using the PCA9685 module.
If running on a non-Raspberry Pi system (e.g., macOS), it uses a mock PCA9685 class for testing purposes.

Classes:
    Servo -- A class to control servo motors using the PCA9685 driver.

Usage:
    Run this script with two arguments: <servo-type> and <angle>.
    Example: python3 servo_control.py horizontal 10
    Example: python3 servo_control.py 0 90
"""

import sys
import os

# --- FIXED: Add parent dir to sys.path so "utils" is importable anywhere ---
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Try to import PCA9685. If it fails, use mock class for testing on macOS.
try:
    from utils.pca9685 import PCA9685  # Updated import path
except ImportError:
    from utils.mock_pca9685 import PCA9685  # Updated import path

class Servo:
    def __init__(self):
        """
        Initialize the Servo class and set default servo positions.
        """
        self.PwmServo = PCA9685(0x40, debug=True)
        self.PwmServo.setPWMFreq(50)
        self.PwmServo.setServoPulse(8, 1500)
        self.PwmServo.setServoPulse(9, 1500)

    def setServoPwm(self, channel, angle, error=10):
        """
        Set the PWM for the specified servo channel.

        Parameters:
        channel (str): The servo channel ('horizontal', 'vertical', '0', '1').
        angle (int): The angle to set the servo to.
        error (int): The error correction value.
        """
        angle = int(angle)
        if channel in ('horizontal', '0'):
            print(f"[SERVO] Setting HORIZONTAL/0 to angle {angle}")
            self.PwmServo.setServoPulse(8, 2500 - int((angle + error) / 0.09))
        elif channel in ('vertical', '1'):
            print(f"[SERVO] Setting VERTICAL/1 to angle {angle}")
            self.PwmServo.setServoPulse(9, 500 + int((angle + error) / 0.09))
        else:
            print(f"[SERVO] Unknown channel: {channel}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 servo_control.py <servo-type> <angle>")
        print("   <servo-type>: horizontal, vertical, 0, or 1")
        sys.exit(1)

    servo_type = sys.argv[1]
    angle = int(sys.argv[2])

    try:
        servo = Servo()
        servo.setServoPwm(servo_type, angle)
    except Exception as e:
        print(f"Error controlling servo: {e}")
        sys.exit(1)
