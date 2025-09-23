# File: /Omega-Code/servers/robot-controller-backend/controllers/servo_control.py

"""
This script controls the servo motors using the PCA9685 module.

Usage:
    python3 servo_control.py horizontal 90
    python3 servo_control.py 0 45

Arguments:
    <servo-type>: horizontal, vertical, 0, or 1
    <angle>: Integer angle in degrees
"""

import sys
import os
import logging
import traceback

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Add project root to sys.path so "utils" is importable
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Error handling
class ServoError(Exception):
    """Servo control specific errors"""
    pass

class HardwareError(ServoError):
    """Hardware communication errors"""
    pass

# Import only the real PCA9685 driver
try:
    from utils.pca9685 import PCA9685
    logger.info("Using real PCA9685 for servo control")
except ImportError:
    logger.error("PCA9685 driver not available - servo control will not work")
    raise ImportError("PCA9685 driver not available")

class Servo:
    def __init__(self):
        """
        Initialize the Servo class and set default servo positions.
        """
        try:
            self.PwmServo = PCA9685(0x40, debug=True)
            self.PwmServo.setPWMFreq(50)
            self.PwmServo.setServoPulse(8, 1500)  # Horizontal servo
            self.PwmServo.setServoPulse(9, 1500)  # Vertical servo
            self.error_count = 0
            self.max_errors = 10
            logger.info("Servo controller initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize servo controller: {e}")
            raise ServoError(f"Servo initialization failed: {e}")

    def setServoPwm(self, channel, angle, error=10):
        """
        Set the PWM for the specified servo channel.

        Parameters:
        channel (str): The servo channel ('horizontal', 'vertical', '0', or '1')
        angle (int): Desired angle in degrees
        error (int): Error correction offset
        """
        try:
            # Validate inputs
            if not isinstance(angle, (int, float)):
                raise ValueError(f"Invalid angle type: {type(angle)}")
            
            angle = int(angle)
            if angle < 0 or angle > 180:
                logger.warning(f"Angle {angle} out of range [0, 180], clamping")
                angle = max(0, min(180, angle))
            
            if channel in ('horizontal', '0'):
                logger.info(f"Setting HORIZONTAL/0 to angle {angle}")
                pulse_width = 2500 - int((angle + error) / 0.09)
                self.PwmServo.setServoPulse(8, pulse_width)
            elif channel in ('vertical', '1'):
                logger.info(f"Setting VERTICAL/1 to angle {angle}")
                pulse_width = 500 + int((angle + error) / 0.09)
                self.PwmServo.setServoPulse(9, pulse_width)
            else:
                raise ValueError(f"Unknown channel: {channel}")
                
        except Exception as e:
            self.error_count += 1
            logger.error(f"Error in setServoPwm: {e}")
            if self.error_count >= self.max_errors:
                raise ServoError(f"Too many errors ({self.error_count}), stopping servo control")
            # Continue with degraded performance

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 servo_control.py <servo-type> <angle>")
        print("   <servo-type>: horizontal, vertical, 0, or 1")
        sys.exit(1)

    servo_type = sys.argv[1]
    
    try:
        angle = int(sys.argv[2])
    except ValueError:
        logger.error(f"Invalid angle: {sys.argv[2]}")
        print(f"Error: Invalid angle '{sys.argv[2]}'. Must be an integer.")
        sys.exit(1)

    try:
        servo = Servo()
        servo.setServoPwm(servo_type, angle)
        logger.info(f"Successfully set {servo_type} servo to {angle} degrees")
    except ServoError as e:
        logger.error(f"Servo error: {e}")
        print(f"Servo Error: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        print(f"Unexpected Error: {e}")
        sys.exit(1)
