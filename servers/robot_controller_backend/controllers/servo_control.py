# File: /Omega-Code/servers/robot_controller_backend/controllers/servo_control.py

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
import time
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

# Import only the real PCA9685 driver - use same import logic as movement subsystem
# Check SIM_MODE first to prevent using mock in production
import os
SIM_MODE = os.getenv('ROBOT_SIM', '0') == '1' or os.getenv('SIM_MODE', '0') == '1'

if SIM_MODE:
    logger.warning("SIM_MODE=True - servo control will use mock/NOOP")
    # In SIM_MODE, servo operations will fail gracefully
    PCA9685 = None
else:
    # Try real hardware imports ONLY when not in SIM_MODE
    try:
        from servers.robot_controller_backend.movement.PCA9685 import PCA9685
        logger.info("Using real PCA9685 for servo control (absolute import)")
    except ImportError:
        try:
            from movement.PCA9685 import PCA9685
            logger.info("Using real PCA9685 for servo control (relative import)")
        except ImportError:
            try:
                # Try importing from movement directory
                import sys
                movement_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'movement'))
                if movement_dir not in sys.path:
                    sys.path.insert(0, movement_dir)
                from PCA9685 import PCA9685
                logger.info("Using real PCA9685 for servo control (local import)")
            except ImportError:
                logger.error("PCA9685 hardware driver not available. Install smbus2: pip install smbus2")
                raise ImportError("PCA9685 hardware driver not available")

# PCA9685 PRESCALE register and the expected value for exactly 50 Hz.
# Formula: floor(25_000_000 / 4096 / 50 − 1 + 0.5) = 121
# Any other value means a different process changed the global PWM clock,
# which produces invalid servo pulses (e.g. 500 Hz → 147 µs instead of 1498 µs).
_PCA9685_PRESCALE_REG   = 0xFE
_EXPECTED_PRESCALE_50HZ = 121
_PRESCALE_TOLERANCE     = 2    # ±2 counts ≈ ±0.8 Hz at 50 Hz

# Hardware-safe pulse range for the servos on this chassis.
# Exceeding these limits risks stripping the servo gears.
_SERVO_PULSE_MIN_US = 1200
_SERVO_PULSE_MAX_US = 1800


class Servo:
    def __init__(self):
        """
        Initialize the Servo class and set default servo positions.
        """
        if PCA9685 is None:
            if SIM_MODE:
                logger.warning("SIM_MODE=True - Servo controller using NOOP mode")
                self.PwmServo = None
                self.error_count = 0
                self.max_errors = 10
                return
            else:
                raise ServoError("PCA9685 hardware driver not available and SIM_MODE=False")
        
        try:
            self.PwmServo = PCA9685(0x40, debug=True)
            self.PwmServo.setPWMFreq(50)
            time.sleep(0.5)              # let PCA9685 stabilize after freq change
            self._assert_50hz()          # SAFETY: abort if another process changed the clock
            self.PwmServo.setServoPulse(8, 1500)  # Horizontal servo - neutral
            self.PwmServo.setServoPulse(9, 1500)  # Vertical servo - neutral
            time.sleep(0.1)              # hold neutral before releasing
            self.error_count = 0
            self.max_errors = 10
            logger.info("Servo controller initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize servo controller: {e}")
            raise ServoError(f"Servo initialization failed: {e}")

    def _assert_50hz(self):
        """
        Read the PRESCALE register and verify the PCA9685 is running at 50 Hz.
        If another process has changed the global PWM frequency (e.g. to 500 Hz),
        servo channels receive invalid pulses and oscillate destructively.
        On mismatch: log ERROR and force-reset to 50 Hz + re-center both channels.
        """
        if self.PwmServo is None:
            return
        try:
            prescale = self.PwmServo.read(_PCA9685_PRESCALE_REG)
            actual_hz = 25_000_000.0 / 4096.0 / (prescale + 1)
            if abs(prescale - _EXPECTED_PRESCALE_50HZ) > _PRESCALE_TOLERANCE:
                logger.error(
                    "SERVO_SAFETY: PCA9685 frequency is %.1f Hz (prescale=%d), "
                    "expected 50 Hz (prescale=%d). "
                    "Another process changed the global PWM clock — forcing reset.",
                    actual_hz, prescale, _EXPECTED_PRESCALE_50HZ,
                )
                self.PwmServo.setPWMFreq(50)
                time.sleep(0.1)
                self.PwmServo.setServoPulse(8, 1500)
                self.PwmServo.setServoPulse(9, 1500)
                logger.warning(
                    "SERVO_SAFETY: Frequency reset to 50 Hz; both servo channels "
                    "re-centered to 1500 µs."
                )
            else:
                logger.info(
                    "SERVO_SAFETY: PCA9685 frequency verified — %.1f Hz (prescale=%d).",
                    actual_hz, prescale,
                )
        except Exception as exc:
            logger.error("SERVO_SAFETY: Could not read PRESCALE register: %s", exc)

    def setServoPwm(self, channel, angle, error=10):
        """
        Set the PWM for the specified servo channel.

        Parameters:
        channel (str): The servo channel ('horizontal', 'vertical', '0', or '1')
        angle (int): Desired angle in degrees
        error (int): Error correction offset
        """
        if self.PwmServo is None:
            logger.warning(f"SIM_MODE: Servo {channel} would be set to {angle}° (NOOP)")
            return
        
        try:
            # Validate inputs
            if not isinstance(angle, (int, float)):
                raise ValueError(f"Invalid angle type: {type(angle)}")
            
            angle = int(angle)
            if angle < 0 or angle > 180:
                logger.warning(f"Angle {angle} out of range [0, 180], clamping")
                angle = max(0, min(180, angle))
            
            if channel in ('horizontal', '0'):
                pulse_width = 2500 - int((angle + error) / 0.09)
                pulse_width = max(_SERVO_PULSE_MIN_US, min(_SERVO_PULSE_MAX_US, pulse_width))
                logger.info("Setting HORIZONTAL/0 to angle %d → %d µs", angle, pulse_width)
                self.PwmServo.setServoPulse(8, pulse_width)
            elif channel in ('vertical', '1'):
                pulse_width = 500 + int((angle + error) / 0.09)
                pulse_width = max(_SERVO_PULSE_MIN_US, min(_SERVO_PULSE_MAX_US, pulse_width))
                logger.info("Setting VERTICAL/1 to angle %d → %d µs", angle, pulse_width)
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
