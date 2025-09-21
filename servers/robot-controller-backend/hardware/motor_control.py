"""
Advanced Motor Control Module
Supports DC motors, stepper motors, and servo motors
"""

import asyncio
import logging
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import math

logger = logging.getLogger(__name__)

class MotorType(Enum):
    DC_MOTOR = "dc_motor"
    STEPPER_MOTOR = "stepper_motor"
    SERVO_MOTOR = "servo_motor"

class MotorState(Enum):
    STOPPED = "stopped"
    FORWARD = "forward"
    BACKWARD = "backward"
    BRAKING = "braking"

@dataclass
class DCMotorConfig:
    """Configuration for DC motor"""
    enable_pin: int
    in1_pin: int
    in2_pin: int
    pwm_frequency: int = 1000
    max_speed: int = 100
    acceleration: float = 10.0  # steps per second
    deceleration: float = 20.0  # steps per second

@dataclass
class StepperMotorConfig:
    """Configuration for stepper motor"""
    step_pin: int
    dir_pin: int
    enable_pin: int
    steps_per_revolution: int = 200
    microsteps: int = 1
    max_speed: int = 1000  # steps per second
    acceleration: int = 500  # steps per second squared

@dataclass
class ServoMotorConfig:
    """Configuration for servo motor"""
    control_pin: int
    min_angle: int = 0
    max_angle: int = 180
    min_pulse_width: int = 500  # microseconds
    max_pulse_width: int = 2500  # microseconds
    frequency: int = 50  # Hz

class MotorController:
    """Base class for motor controllers"""
    
    def __init__(self, motor_id: str, motor_type: MotorType):
        self.motor_id = motor_id
        self.motor_type = motor_type
        self.current_speed = 0
        self.target_speed = 0
        self.current_state = MotorState.STOPPED
        self.initialized = False
    
    async def initialize(self) -> bool:
        """Initialize motor controller"""
        raise NotImplementedError
    
    async def cleanup(self) -> None:
        """Cleanup motor resources"""
        raise NotImplementedError
    
    async def set_speed(self, speed: int) -> bool:
        """Set motor speed (0-100)"""
        raise NotImplementedError
    
    async def set_direction(self, direction: str) -> bool:
        """Set motor direction"""
        raise NotImplementedError
    
    async def stop(self) -> bool:
        """Stop motor immediately"""
        raise NotImplementedError
    
    async def brake(self) -> bool:
        """Apply motor brake"""
        raise NotImplementedError

class DCMotorController(MotorController):
    """DC Motor Controller with PWM and direction control"""
    
    def __init__(self, motor_id: str, config: DCMotorConfig):
        super().__init__(motor_id, MotorType.DC_MOTOR)
        self.config = config
        self.pwm = None
        self.gpio = None
        
        # Import GPIO library
        try:
            import RPi.GPIO as GPIO
            self.gpio = GPIO
        except ImportError:
            logger.error("RPi.GPIO not available for DC motor control")
            raise
    
    async def initialize(self) -> bool:
        """Initialize DC motor GPIO pins"""
        try:
            self.gpio.setmode(self.gpio.BCM)
            self.gpio.setwarnings(False)
            
            # Setup GPIO pins
            self.gpio.setup(self.config.enable_pin, self.gpio.OUT)
            self.gpio.setup(self.config.in1_pin, self.gpio.OUT)
            self.gpio.setup(self.config.in2_pin, self.gpio.OUT)
            
            # Setup PWM
            self.pwm = self.gpio.PWM(self.config.enable_pin, self.config.pwm_frequency)
            self.pwm.start(0)
            
            self.initialized = True
            logger.info(f"DC Motor {self.motor_id} initialized")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize DC motor {self.motor_id}: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup DC motor resources"""
        try:
            if self.pwm:
                self.pwm.stop()
            
            # Set all pins low
            self.gpio.output(self.config.enable_pin, self.gpio.LOW)
            self.gpio.output(self.config.in1_pin, self.gpio.LOW)
            self.gpio.output(self.config.in2_pin, self.gpio.LOW)
            
            self.initialized = False
            logger.info(f"DC Motor {self.motor_id} cleaned up")
            
        except Exception as e:
            logger.error(f"Error cleaning up DC motor {self.motor_id}: {e}")
    
    async def set_speed(self, speed: int) -> bool:
        """Set DC motor speed with acceleration"""
        if not self.initialized:
            return False
        
        try:
            speed = max(0, min(100, speed))
            self.target_speed = speed
            
            # Gradual acceleration/deceleration
            while self.current_speed != self.target_speed:
                if self.current_speed < self.target_speed:
                    self.current_speed = min(self.target_speed, 
                                          self.current_speed + self.config.acceleration)
                else:
                    self.current_speed = max(self.target_speed, 
                                          self.current_speed - self.config.deceleration)
                
                duty_cycle = (self.current_speed / 100) * 100
                self.pwm.ChangeDutyCycle(duty_cycle)
                
                await asyncio.sleep(0.1)  # 100ms update interval
            
            logger.info(f"DC Motor {self.motor_id} speed set to {self.current_speed}%")
            return True
            
        except Exception as e:
            logger.error(f"Error setting DC motor {self.motor_id} speed: {e}")
            return False
    
    async def set_direction(self, direction: str) -> bool:
        """Set DC motor direction"""
        if not self.initialized:
            return False
        
        try:
            if direction == "forward":
                self.gpio.output(self.config.in1_pin, self.gpio.HIGH)
                self.gpio.output(self.config.in2_pin, self.gpio.LOW)
                self.current_state = MotorState.FORWARD
            elif direction == "backward":
                self.gpio.output(self.config.in1_pin, self.gpio.LOW)
                self.gpio.output(self.config.in2_pin, self.gpio.HIGH)
                self.current_state = MotorState.BACKWARD
            elif direction == "stop":
                self.gpio.output(self.config.in1_pin, self.gpio.LOW)
                self.gpio.output(self.config.in2_pin, self.gpio.LOW)
                self.current_state = MotorState.STOPPED
            else:
                logger.warning(f"Unknown direction: {direction}")
                return False
            
            logger.info(f"DC Motor {self.motor_id} direction set to {direction}")
            return True
            
        except Exception as e:
            logger.error(f"Error setting DC motor {self.motor_id} direction: {e}")
            return False
    
    async def stop(self) -> bool:
        """Stop DC motor immediately"""
        return await self.set_direction("stop")
    
    async def brake(self) -> bool:
        """Apply DC motor brake (short circuit)"""
        if not self.initialized:
            return False
        
        try:
            # Short circuit both terminals for braking
            self.gpio.output(self.config.in1_pin, self.gpio.HIGH)
            self.gpio.output(self.config.in2_pin, self.gpio.HIGH)
            self.current_state = MotorState.BRAKING
            
            logger.info(f"DC Motor {self.motor_id} brake applied")
            return True
            
        except Exception as e:
            logger.error(f"Error applying brake to DC motor {self.motor_id}: {e}")
            return False

class ServoMotorController(MotorController):
    """Servo Motor Controller with precise angle control"""
    
    def __init__(self, motor_id: str, config: ServoMotorConfig):
        super().__init__(motor_id, MotorType.SERVO_MOTOR)
        self.config = config
        self.current_angle = 90  # Start at center position
        self.target_angle = 90
        self.pwm = None
        self.gpio = None
        
        # Import GPIO library
        try:
            import RPi.GPIO as GPIO
            self.gpio = GPIO
        except ImportError:
            logger.error("RPi.GPIO not available for servo control")
            raise
    
    async def initialize(self) -> bool:
        """Initialize servo motor GPIO pin"""
        try:
            self.gpio.setmode(self.gpio.BCM)
            self.gpio.setwarnings(False)
            
            # Setup GPIO pin
            self.gpio.setup(self.config.control_pin, self.gpio.OUT)
            
            # Setup PWM for servo control
            self.pwm = self.gpio.PWM(self.config.control_pin, self.config.frequency)
            self.pwm.start(0)
            
            # Set initial position
            await self.set_angle(self.current_angle)
            
            self.initialized = True
            logger.info(f"Servo Motor {self.motor_id} initialized")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize servo motor {self.motor_id}: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup servo motor resources"""
        try:
            if self.pwm:
                self.pwm.stop()
            
            self.gpio.output(self.config.control_pin, self.gpio.LOW)
            self.initialized = False
            logger.info(f"Servo Motor {self.motor_id} cleaned up")
            
        except Exception as e:
            logger.error(f"Error cleaning up servo motor {self.motor_id}: {e}")
    
    async def set_angle(self, angle: int) -> bool:
        """Set servo motor angle (0-180)"""
        if not self.initialized:
            return False
        
        try:
            angle = max(self.config.min_angle, min(self.config.max_angle, angle))
            self.target_angle = angle
            
            # Calculate pulse width
            pulse_width = self.config.min_pulse_width + \
                         (angle - self.config.min_angle) * \
                         (self.config.max_pulse_width - self.config.min_pulse_width) / \
                         (self.config.max_angle - self.config.min_angle)
            
            # Convert to duty cycle
            duty_cycle = (pulse_width / 1000000) * self.config.frequency * 100
            
            self.pwm.ChangeDutyCycle(duty_cycle)
            self.current_angle = angle
            
            logger.info(f"Servo Motor {self.motor_id} angle set to {angle}°")
            return True
            
        except Exception as e:
            logger.error(f"Error setting servo motor {self.motor_id} angle: {e}")
            return False
    
    async def set_speed(self, speed: int) -> bool:
        """Servo motors don't have variable speed, only position"""
        logger.warning("Servo motors don't support variable speed")
        return False
    
    async def set_direction(self, direction: str) -> bool:
        """Set servo motor direction (angle-based)"""
        if not self.initialized:
            return False
        
        try:
            if direction == "forward" or direction == "up":
                new_angle = min(self.config.max_angle, self.current_angle + 10)
            elif direction == "backward" or direction == "down":
                new_angle = max(self.config.min_angle, self.current_angle - 10)
            elif direction == "left":
                new_angle = max(self.config.min_angle, self.current_angle - 10)
            elif direction == "right":
                new_angle = min(self.config.max_angle, self.current_angle + 10)
            elif direction == "stop":
                new_angle = 90  # Center position
            else:
                logger.warning(f"Unknown direction: {direction}")
                return False
            
            return await self.set_angle(new_angle)
            
        except Exception as e:
            logger.error(f"Error setting servo motor {self.motor_id} direction: {e}")
            return False
    
    async def stop(self) -> bool:
        """Stop servo motor (center position)"""
        return await self.set_angle(90)
    
    async def brake(self) -> bool:
        """Servo motors don't have brakes"""
        logger.warning("Servo motors don't support braking")
        return False

class RobotMotorSystem:
    """Complete robot motor system with multiple motors"""
    
    def __init__(self):
        self.motors: Dict[str, MotorController] = {}
        self.initialized = False
    
    async def add_motor(self, motor_id: str, motor_type: MotorType, config) -> bool:
        """Add a motor to the system"""
        try:
            if motor_type == MotorType.DC_MOTOR:
                motor = DCMotorController(motor_id, config)
            elif motor_type == MotorType.SERVO_MOTOR:
                motor = ServoMotorController(motor_id, config)
            else:
                logger.error(f"Unsupported motor type: {motor_type}")
                return False
            
            success = await motor.initialize()
            if success:
                self.motors[motor_id] = motor
                logger.info(f"Motor {motor_id} added to system")
            
            return success
            
        except Exception as e:
            logger.error(f"Failed to add motor {motor_id}: {e}")
            return False
    
    async def initialize(self) -> bool:
        """Initialize all motors"""
        try:
            self.initialized = True
            logger.info("Robot motor system initialized")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize motor system: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup all motors"""
        try:
            for motor in self.motors.values():
                await motor.cleanup()
            
            self.motors.clear()
            self.initialized = False
            logger.info("Robot motor system cleaned up")
            
        except Exception as e:
            logger.error(f"Error cleaning up motor system: {e}")
    
    async def move_forward(self, speed: int = 50) -> bool:
        """Move robot forward"""
        if not self.initialized:
            return False
        
        try:
            results = []
            for motor_id, motor in self.motors.items():
                if motor.motor_type == MotorType.DC_MOTOR:
                    results.append(await motor.set_direction("forward"))
                    results.append(await motor.set_speed(speed))
                elif motor.motor_type == MotorType.SERVO_MOTOR:
                    results.append(await motor.set_direction("forward"))
            
            return all(results)
            
        except Exception as e:
            logger.error(f"Error moving forward: {e}")
            return False
    
    async def move_backward(self, speed: int = 50) -> bool:
        """Move robot backward"""
        if not self.initialized:
            return False
        
        try:
            results = []
            for motor_id, motor in self.motors.items():
                if motor.motor_type == MotorType.DC_MOTOR:
                    results.append(await motor.set_direction("backward"))
                    results.append(await motor.set_speed(speed))
                elif motor.motor_type == MotorType.SERVO_MOTOR:
                    results.append(await motor.set_direction("backward"))
            
            return all(results)
            
        except Exception as e:
            logger.error(f"Error moving backward: {e}")
            return False
    
    async def turn_left(self, speed: int = 50) -> bool:
        """Turn robot left"""
        if not self.initialized:
            return False
        
        try:
            results = []
            for motor_id, motor in self.motors.items():
                if motor.motor_type == MotorType.DC_MOTOR:
                    results.append(await motor.set_direction("left"))
                    results.append(await motor.set_speed(speed))
                elif motor.motor_type == MotorType.SERVO_MOTOR:
                    results.append(await motor.set_direction("left"))
            
            return all(results)
            
        except Exception as e:
            logger.error(f"Error turning left: {e}")
            return False
    
    async def turn_right(self, speed: int = 50) -> bool:
        """Turn robot right"""
        if not self.initialized:
            return False
        
        try:
            results = []
            for motor_id, motor in self.motors.items():
                if motor.motor_type == MotorType.DC_MOTOR:
                    results.append(await motor.set_direction("right"))
                    results.append(await motor.set_speed(speed))
                elif motor.motor_type == MotorType.SERVO_MOTOR:
                    results.append(await motor.set_direction("right"))
            
            return all(results)
            
        except Exception as e:
            logger.error(f"Error turning right: {e}")
            return False
    
    async def stop_all(self) -> bool:
        """Stop all motors"""
        if not self.initialized:
            return False
        
        try:
            results = []
            for motor_id, motor in self.motors.items():
                results.append(await motor.stop())
            
            return all(results)
            
        except Exception as e:
            logger.error(f"Error stopping motors: {e}")
            return False
    
    async def emergency_stop(self) -> bool:
        """Emergency stop with braking"""
        if not self.initialized:
            return False
        
        try:
            results = []
            for motor_id, motor in self.motors.items():
                if hasattr(motor, 'brake'):
                    results.append(await motor.brake())
                else:
                    results.append(await motor.stop())
            
            logger.warning("Emergency stop activated")
            return all(results)
            
        except Exception as e:
            logger.error(f"Error in emergency stop: {e}")
            return False

# Global motor system instance
motor_system = RobotMotorSystem()
