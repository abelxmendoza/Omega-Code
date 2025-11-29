# File: /Omega-Code/servers/robot_controller_backend/movement/test_motor_telemetry.py
"""
Simple test script for motor telemetry functionality.
"""

import time
import math
from typing import Dict, Any
from dataclasses import dataclass

@dataclass
class MotorTelemetry:
    """Individual motor telemetry data"""
    speed: float  # RPM
    power: float  # Watts
    pwm: int      # PWM value
    current: float # Amperes (simulated)
    temperature: float # Celsius (simulated)

class SimpleMotorTelemetryController:
    """Simple motor controller with telemetry tracking for testing"""
    
    def __init__(self):
        self.telemetry = {
            'frontLeft': MotorTelemetry(0, 0, 0, 0, 25),
            'frontRight': MotorTelemetry(0, 0, 0, 0, 25),
            'rearLeft': MotorTelemetry(0, 0, 0, 0, 25),
            'rearRight': MotorTelemetry(0, 0, 0, 0, 25)
        }
        self.last_update = time.time()
        self.left_trim = 0
        self.right_trim = 0
        self.MAX_PWM = 4095
        
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
        # Apply trim and set motors
        left_duty = self._apply_trim(duty, self.left_trim)
        right_duty = self._apply_trim(duty, self.right_trim)
        
        # Update telemetry for all motors
        self._update_telemetry(left_duty, right_duty)
        
        print(f"[MOTOR] Setting motors: left={left_duty}, right={right_duty}")
    
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
        duty_normalized = abs(duty) / self.MAX_PWM
        
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
    
    def stop(self):
        """Stop all motors and reset telemetry"""
        print("[MOTOR] Stopping all motors")
        # Reset telemetry to zero
        for motor_name in self.telemetry:
            self.telemetry[motor_name] = MotorTelemetry(0, 0, 0, 0, 25)
    
    def forward(self, speed=2000):
        """Move forward with telemetry tracking"""
        print(f"[MOTOR] Moving forward at speed {speed}")
        self.setMotors(speed)
    
    def backward(self, speed=2000):
        """Move backward with telemetry tracking"""
        print(f"[MOTOR] Moving backward at speed {speed}")
        self.setMotors(-speed)
    
    def left(self, speed=1500, ratio=0.5):
        """Turn left with telemetry tracking"""
        print(f"[MOTOR] Turning left: speed={speed}, ratio={ratio}")
        left_duty = int(abs(speed) * ratio)
        right_duty = abs(speed)
        self._update_telemetry(left_duty, right_duty)
    
    def right(self, speed=1500, ratio=0.5):
        """Turn right with telemetry tracking"""
        print(f"[MOTOR] Turning right: speed={speed}, ratio={ratio}")
        left_duty = abs(speed)
        right_duty = int(abs(speed) * ratio)
        self._update_telemetry(left_duty, right_duty)

if __name__ == "__main__":
    # Test the telemetry controller
    controller = SimpleMotorTelemetryController()
    
    print("Testing Motor Telemetry Controller")
    print("=" * 40)
    
    # Test forward movement
    print("\n1. Forward movement:")
    controller.forward(2000)
    time.sleep(0.1)
    telemetry = controller.get_telemetry()
    for motor, data in telemetry.items():
        print(f"  {motor}: {data['speed']} RPM, {data['power']}W, PWM {data['pwm']}")
    
    # Test turning
    print("\n2. Left turn:")
    controller.left(1500, 0.3)
    time.sleep(0.1)
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
