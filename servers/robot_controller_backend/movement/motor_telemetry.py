# File: /Omega-Code/servers/robot_controller_backend/movement/motor_telemetry.py
"""
Enhanced motor controller with telemetry capabilities.
Provides individual motor data for frontend telemetry display.
"""

import time
import math
import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass

# Configure logging
logger = logging.getLogger(__name__)

# Optimized imports: use cached hardware flags and singleton PCA
from servers.robot_controller_backend.hardware.hardware_flags import is_sim
from servers.robot_controller_backend.hardware.pwm_singleton import get_pca

# Import PCA9685 class (not instance) - singleton pattern handles instance creation
# MotorTelemetryController uses the SAME singleton PCA instance as Motor
try:
    if not is_sim():
        from servers.robot_controller_backend.movement.PCA9685 import PCA9685
        logger.info("Using real PCA9685 for motor telemetry")
    else:
        raise ImportError("SIM_MODE enabled")
except ImportError:
    try:
        from .PCA9685 import PCA9685
        logger.info("Using real PCA9685 for motor telemetry (relative import)")
    except ImportError:
        # Fallback to mock only in SIM_MODE
        if is_sim():
            from .mock_pca9685 import MockPCA9685 as PCA9685
            logger.warning("Using MockPCA9685 for motor telemetry (SIM_MODE=True)")
        else:
            logger.error("PCA9685 hardware driver not available. Install smbus2: pip install smbus2")
            raise ImportError("PCA9685 hardware driver not available")

try:
    from .minimal_motor_control import Motor
except ImportError:
    try:
        from servers.robot_controller_backend.movement.minimal_motor_control import Motor
    except ImportError:
        try:
            from minimal_motor_control import Motor
        except ImportError:
            logger.error("minimal_motor_control not available")
            raise ImportError("minimal_motor_control module not found")

# Error handling
class MotorTelemetryError(Exception):
    """Motor telemetry specific errors"""
    pass

class HardwareError(MotorTelemetryError):
    """Hardware communication errors"""
    pass

@dataclass
class MotorTelemetry:
    """Individual motor telemetry data"""
    speed: float  # RPM
    power: float  # Watts
    pwm: int      # PWM value
    current: float # Amperes (simulated)
    temperature: float # Celsius (simulated)

class MotorTelemetryController:
    """
    Enhanced motor controller with telemetry tracking.
    
    Uses singleton PCA9685 instance (shared with Motor) and __slots__ for memory efficiency.
    """
    __slots__ = ("motor", "pca", "telemetry", "last_update", "error_count", "max_errors", "motor_constants")
    
    def __init__(self):
        try:
            self.motor = Motor()
            # Use the SAME singleton PCA instance as Motor (shared across all components)
            self.pca = get_pca(PCA9685, 0x40, debug=True)
            logger.info("Motor controller initialized successfully")
        except ImportError as e:
            print("❌ [MOTOR_TELEMETRY] Missing dependency smbus2 or Adafruit-PCA9685")
            print(f"   Install with: pip install smbus2 adafruit-circuitpython-pca9685")
            print(f"   Exception: {repr(e)}")
            logger.error(f"Import error: {e}")
            raise MotorTelemetryError(f"Missing dependency: {e}")
        except OSError as e:
            print("❌ [MOTOR_TELEMETRY] I2C communication failed")
            print(f"   Check wiring, power, SDA/SCL, and PCA9685 address")
            print(f"   Exception: {repr(e)}")
            logger.error(f"I2C error: {e}")
            raise MotorTelemetryError(f"I2C communication failed: {e}")
        except Exception as e:
            print(f"🔥 [MOTOR_TELEMETRY][ERROR] Failed to initialize motor controller")
            print(f"   Exception: {repr(e)}")
            print(f"   Type: {type(e).__name__}")
            logger.error(f"Failed to initialize motor controller: {e}")
            raise MotorTelemetryError(f"Motor initialization failed: {e}")
        
        self.telemetry = {
            'frontLeft': MotorTelemetry(0, 0, 0, 0, 25),
            'frontRight': MotorTelemetry(0, 0, 0, 0, 25),
            'rearLeft': MotorTelemetry(0, 0, 0, 0, 25),
            'rearRight': MotorTelemetry(0, 0, 0, 0, 25)
        }
        self.last_update = time.time()
        self.error_count = 0
        self.max_errors = 10
        
        # Motor characteristics (simulated)
        self.motor_constants = {
            'max_rpm': 300,      # Maximum RPM
            'max_power': 50,     # Maximum power in watts
            'max_current': 5,    # Maximum current in amps
            'efficiency': 0.85,  # Motor efficiency
            'thermal_mass': 0.1  # Thermal response time
        }
    
    def setMotors(self, duty: int):
        """Set motor PWM and update telemetry"""
        try:
            # Validate duty cycle
            if not isinstance(duty, (int, float)):
                raise ValueError(f"Invalid duty cycle type: {type(duty)}")
            
            duty = int(duty)
            if duty < -4095 or duty > 4095:
                logger.warning(f"Duty cycle {duty} out of range [-4095, 4095]")
                duty = max(-4095, min(4095, duty))
            
            # Apply trim and set motors
            left_duty = self._apply_trim(duty, self.motor.left_trim)
            right_duty = self._apply_trim(duty, self.motor.right_trim)
            
            # Update telemetry for all motors
            self._update_telemetry(left_duty, right_duty)
            
            # Set actual motor PWM
            self.motor.setMotors(duty)
            
        except Exception as e:
            self.error_count += 1
            logger.error(f"Error in setMotors: {e}")
            if self.error_count >= self.max_errors:
                raise MotorTelemetryError(f"Too many errors ({self.error_count}), stopping motor control")
            # Continue with degraded performance
    
    def _update_telemetry(self, left_duty: int, right_duty: int):
        """Update motor telemetry based on PWM values"""
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time
        
        # Calculate telemetry for left motors (front and rear)
        left_telemetry = self._calculate_motor_telemetry(left_duty, dt)
        self.telemetry['frontLeft'] = left_telemetry
        self.telemetry['rearLeft'] = left_telemetry
        
        # Calculate telemetry for right motors (front and rear)
        right_telemetry = self._calculate_motor_telemetry(right_duty, dt)
        self.telemetry['frontRight'] = right_telemetry
        self.telemetry['rearRight'] = right_telemetry
    
    def _calculate_motor_telemetry(self, duty: int, dt: float) -> MotorTelemetry:
        """Calculate motor telemetry from PWM duty cycle"""
        # Normalize duty to 0-1 range
        duty_normalized = abs(duty) / self.motor.MAX_PWM
        
        # Calculate RPM (simulated based on PWM)
        speed_rpm = duty_normalized * self.motor_constants['max_rpm']
        
        # Calculate power (simulated)
        power_watts = duty_normalized * self.motor_constants['max_power']
        
        # Calculate current (simulated)
        current_amps = duty_normalized * self.motor_constants['max_current']
        
        # Calculate temperature (simulated thermal model)
        target_temp = 25 + (power_watts * 0.5)  # Base temp + power heating
        current_temp = self.telemetry['frontLeft'].temperature  # Use existing temp as reference
        temp_change = (target_temp - current_temp) * self.motor_constants['thermal_mass'] * dt
        new_temp = max(25, min(80, current_temp + temp_change))  # Clamp between 25-80°C
        
        return MotorTelemetry(
            speed=speed_rpm,
            power=power_watts,
            pwm=abs(duty),
            current=current_amps,
            temperature=new_temp
        )
    
    def _apply_trim(self, duty: int, trim: int) -> int:
        """Apply trim to duty cycle"""
        if duty == 0 or trim == 0:
            return duty
        return duty + (trim if duty > 0 else -trim)
    
    def get_telemetry(self) -> Dict[str, Any]:
        """Get current motor telemetry data"""
        try:
            return {
                'frontLeft': {
                    'speed': round(self.telemetry['frontLeft'].speed, 1),
                    'power': round(self.telemetry['frontLeft'].power, 1),
                    'pwm': self.telemetry['frontLeft'].pwm,
                    'current': round(self.telemetry['frontLeft'].current, 2),
                    'temperature': round(self.telemetry['frontLeft'].temperature, 1)
                },
                'frontRight': {
                    'speed': round(self.telemetry['frontRight'].speed, 1),
                    'power': round(self.telemetry['frontRight'].power, 1),
                    'pwm': self.telemetry['frontRight'].pwm,
                    'current': round(self.telemetry['frontRight'].current, 2),
                    'temperature': round(self.telemetry['frontRight'].temperature, 1)
                },
                'rearLeft': {
                    'speed': round(self.telemetry['rearLeft'].speed, 1),
                    'power': round(self.telemetry['rearLeft'].power, 1),
                    'pwm': self.telemetry['rearLeft'].pwm,
                    'current': round(self.telemetry['rearLeft'].current, 2),
                    'temperature': round(self.telemetry['rearLeft'].temperature, 1)
                },
                'rearRight': {
                    'speed': round(self.telemetry['rearRight'].speed, 1),
                    'power': round(self.telemetry['rearRight'].power, 1),
                    'pwm': self.telemetry['rearRight'].pwm,
                    'current': round(self.telemetry['rearRight'].current, 2),
                    'temperature': round(self.telemetry['rearRight'].temperature, 1)
                }
            }
        except Exception as e:
            logger.error(f"Error getting telemetry: {e}")
            # Return safe default values
            default_telemetry = {
                'speed': 0.0,
                'power': 0.0,
                'pwm': 0,
                'current': 0.0,
                'temperature': 25.0
            }
            return {
                'frontLeft': default_telemetry.copy(),
                'frontRight': default_telemetry.copy(),
                'rearLeft': default_telemetry.copy(),
                'rearRight': default_telemetry.copy()
            }
    
    def stop(self):
        """Stop all motors and reset telemetry"""
        self.motor.stop()
        # Reset telemetry to zero
        for motor_name in self.telemetry:
            self.telemetry[motor_name] = MotorTelemetry(0, 0, 0, 0, 25)
    
    def forward(self, speed=2000):
        """Move forward with telemetry tracking"""
        self.setMotors(speed)
    
    def backward(self, speed=2000):
        """Move backward with telemetry tracking"""
        self.setMotors(-speed)
    
    def left(self, speed=1500, ratio=0.5):
        """Turn left with telemetry tracking"""
        left_duty = int(abs(speed) * ratio)
        right_duty = abs(speed)
        self._update_telemetry(left_duty, right_duty)
        self.motor.left(speed, ratio)
    
    def right(self, speed=1500, ratio=0.5):
        """Turn right with telemetry tracking"""
        left_duty = abs(speed)
        right_duty = int(abs(speed) * ratio)
        self._update_telemetry(left_duty, right_duty)
        self.motor.right(speed, ratio)
    
    def pivot_left(self, speed=1500):
        """Pivot left with telemetry tracking"""
        left_duty = -abs(speed)
        right_duty = abs(speed)
        self._update_telemetry(left_duty, right_duty)
        self.motor.pivot_left(speed)
    
    def pivot_right(self, speed=1500):
        """Pivot right with telemetry tracking"""
        left_duty = abs(speed)
        right_duty = -abs(speed)
        self._update_telemetry(left_duty, right_duty)
        self.motor.pivot_right(speed)

if __name__ == "__main__":
    # Test the telemetry controller
    controller = MotorTelemetryController()
    
    print("Testing Motor Telemetry Controller")
    print("=" * 40)
    
    # Test forward movement
    print("\n1. Forward movement:")
    controller.forward(2000)
    time.sleep(1)
    telemetry = controller.get_telemetry()
    for motor, data in telemetry.items():
        print(f"  {motor}: {data['speed']} RPM, {data['power']}W, PWM {data['pwm']}")
    
    # Test turning
    print("\n2. Left turn:")
    controller.left(1500, 0.3)
    time.sleep(1)
    telemetry = controller.get_telemetry()
    for motor, data in telemetry.items():
        print(f"  {motor}: {data['speed']} RPM, {data['power']}W, PWM {data['pwm']}")
    
    # Test stop
    print("\n3. Stop:")
    controller.stop()
    telemetry = controller.get_telemetry()
    for motor, data in telemetry.items():
        print(f"  {motor}: {data['speed']} RPM, {data['power']}W, PWM {data['pwm']}")
    
    print("\nTest completed!")
