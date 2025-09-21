"""
Hardware Abstraction Layer for Robot Controller
Supports multiple robot platforms: Raspberry Pi, Arduino, Jetson Nano
"""

import asyncio
import logging
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
from enum import Enum
import json

logger = logging.getLogger(__name__)

class MotorDirection(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    STOP = "stop"

class SensorType(Enum):
    ULTRASONIC = "ultrasonic"
    TEMPERATURE = "temperature"
    HUMIDITY = "humidity"
    LIGHT = "light"
    GYROSCOPE = "gyroscope"
    ACCELEROMETER = "accelerometer"

@dataclass
class MotorConfig:
    left_motor_pins: List[int]
    right_motor_pins: List[int]
    pwm_frequency: int = 1000
    max_speed: int = 100

@dataclass
class SensorConfig:
    sensor_type: SensorType
    pin: int
    i2c_address: Optional[int] = None
    calibration_factor: float = 1.0

@dataclass
class CameraConfig:
    device_id: int = 0
    resolution: tuple = (640, 480)
    fps: int = 30

class HardwareInterface(ABC):
    """Abstract base class for hardware interfaces"""
    
    @abstractmethod
    async def initialize(self) -> bool:
        """Initialize hardware interface"""
        pass
    
    @abstractmethod
    async def cleanup(self) -> None:
        """Cleanup hardware resources"""
        pass
    
    @abstractmethod
    async def set_motor_speed(self, motor: str, speed: int) -> bool:
        """Set motor speed (0-100)"""
        pass
    
    @abstractmethod
    async def set_motor_direction(self, direction: MotorDirection) -> bool:
        """Set motor direction"""
        pass
    
    @abstractmethod
    async def read_sensor(self, sensor_type: SensorType) -> float:
        """Read sensor value"""
        pass
    
    @abstractmethod
    async def set_led_color(self, r: int, g: int, b: int) -> bool:
        """Set LED color (0-255)"""
        pass
    
    @abstractmethod
    async def set_servo_angle(self, servo_id: int, angle: int) -> bool:
        """Set servo angle (0-180)"""
        pass

class RaspberryPiInterface(HardwareInterface):
    """Raspberry Pi hardware interface using RPi.GPIO"""
    
    def __init__(self, motor_config: MotorConfig, sensor_configs: List[SensorConfig]):
        self.motor_config = motor_config
        self.sensor_configs = sensor_configs
        self.initialized = False
        
        # Import GPIO library
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
        except ImportError:
            logger.error("RPi.GPIO not available. Install with: pip install RPi.GPIO")
            raise
        
        # Import additional libraries
        try:
            import board
            import busio
            import adafruit_dht
            import adafruit_hcsr04
            self.board = board
            self.busio = busio
            self.adafruit_dht = adafruit_dht
            self.adafruit_hcsr04 = adafruit_hcsr04
        except ImportError:
            logger.warning("Some sensor libraries not available. Install with: pip install adafruit-circuitpython-dht adafruit-circuitpython-hcsr04")
    
    async def initialize(self) -> bool:
        """Initialize Raspberry Pi GPIO"""
        try:
            self.GPIO.setmode(self.GPIO.BCM)
            self.GPIO.setwarnings(False)
            
            # Setup motor pins
            for pin in self.motor_config.left_motor_pins + self.motor_config.right_motor_pins:
                self.GPIO.setup(pin, self.GPIO.OUT)
            
            # Setup PWM for motor speed control
            self.left_pwm = self.GPIO.PWM(self.motor_config.left_motor_pins[0], self.motor_config.pwm_frequency)
            self.right_pwm = self.GPIO.PWM(self.motor_config.right_motor_pins[0], self.motor_config.pwm_frequency)
            
            self.left_pwm.start(0)
            self.right_pwm.start(0)
            
            # Initialize sensors
            self.sensors = {}
            for config in self.sensor_configs:
                if config.sensor_type == SensorType.ULTRASONIC:
                    self.sensors[config.sensor_type] = self.adafruit_hcsr04.HCSR04(
                        trigger_pin=config.pin,
                        echo_pin=config.pin + 1
                    )
                elif config.sensor_type == SensorType.TEMPERATURE:
                    self.sensors[config.sensor_type] = self.adafruit_dht.DHT22(config.pin)
            
            self.initialized = True
            logger.info("Raspberry Pi hardware initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize Raspberry Pi hardware: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup GPIO resources"""
        try:
            if hasattr(self, 'left_pwm'):
                self.left_pwm.stop()
            if hasattr(self, 'right_pwm'):
                self.right_pwm.stop()
            
            self.GPIO.cleanup()
            self.initialized = False
            logger.info("Raspberry Pi hardware cleaned up")
        except Exception as e:
            logger.error(f"Error cleaning up Raspberry Pi hardware: {e}")
    
    async def set_motor_speed(self, motor: str, speed: int) -> bool:
        """Set motor speed (0-100)"""
        if not self.initialized:
            return False
        
        try:
            speed = max(0, min(100, speed))
            duty_cycle = (speed / 100) * 100
            
            if motor == "left":
                self.left_pwm.ChangeDutyCycle(duty_cycle)
            elif motor == "right":
                self.right_pwm.ChangeDutyCycle(duty_cycle)
            
            return True
        except Exception as e:
            logger.error(f"Error setting motor speed: {e}")
            return False
    
    async def set_motor_direction(self, direction: MotorDirection) -> bool:
        """Set motor direction"""
        if not self.initialized:
            return False
        
        try:
            if direction == MotorDirection.FORWARD:
                self.GPIO.output(self.motor_config.left_motor_pins[1], self.GPIO.HIGH)
                self.GPIO.output(self.motor_config.left_motor_pins[2], self.GPIO.LOW)
                self.GPIO.output(self.motor_config.right_motor_pins[1], self.GPIO.HIGH)
                self.GPIO.output(self.motor_config.right_motor_pins[2], self.GPIO.LOW)
            elif direction == MotorDirection.BACKWARD:
                self.GPIO.output(self.motor_config.left_motor_pins[1], self.GPIO.LOW)
                self.GPIO.output(self.motor_config.left_motor_pins[2], self.GPIO.HIGH)
                self.GPIO.output(self.motor_config.right_motor_pins[1], self.GPIO.LOW)
                self.GPIO.output(self.motor_config.right_motor_pins[2], self.GPIO.HIGH)
            elif direction == MotorDirection.LEFT:
                self.GPIO.output(self.motor_config.left_motor_pins[1], self.GPIO.LOW)
                self.GPIO.output(self.motor_config.left_motor_pins[2], self.GPIO.HIGH)
                self.GPIO.output(self.motor_config.right_motor_pins[1], self.GPIO.HIGH)
                self.GPIO.output(self.motor_config.right_motor_pins[2], self.GPIO.LOW)
            elif direction == MotorDirection.RIGHT:
                self.GPIO.output(self.motor_config.left_motor_pins[1], self.GPIO.HIGH)
                self.GPIO.output(self.motor_config.left_motor_pins[2], self.GPIO.LOW)
                self.GPIO.output(self.motor_config.right_motor_pins[1], self.GPIO.LOW)
                self.GPIO.output(self.motor_config.right_motor_pins[2], self.GPIO.HIGH)
            elif direction == MotorDirection.STOP:
                self.GPIO.output(self.motor_config.left_motor_pins[1], self.GPIO.LOW)
                self.GPIO.output(self.motor_config.left_motor_pins[2], self.GPIO.LOW)
                self.GPIO.output(self.motor_config.right_motor_pins[1], self.GPIO.LOW)
                self.GPIO.output(self.motor_config.right_motor_pins[2], self.GPIO.LOW)
            
            return True
        except Exception as e:
            logger.error(f"Error setting motor direction: {e}")
            return False
    
    async def read_sensor(self, sensor_type: SensorType) -> float:
        """Read sensor value"""
        if not self.initialized or sensor_type not in self.sensors:
            return 0.0
        
        try:
            if sensor_type == SensorType.ULTRASONIC:
                return self.sensors[sensor_type].distance
            elif sensor_type == SensorType.TEMPERATURE:
                return self.sensors[sensor_type].temperature
            elif sensor_type == SensorType.HUMIDITY:
                return self.sensors[sensor_type].humidity
            else:
                return 0.0
        except Exception as e:
            logger.error(f"Error reading sensor {sensor_type}: {e}")
            return 0.0
    
    async def set_led_color(self, r: int, g: int, b: int) -> bool:
        """Set LED color using PWM"""
        if not self.initialized:
            return False
        
        try:
            # This would control RGB LED strip
            # Implementation depends on specific LED hardware
            logger.info(f"Setting LED color to RGB({r}, {g}, {b})")
            return True
        except Exception as e:
            logger.error(f"Error setting LED color: {e}")
            return False
    
    async def set_servo_angle(self, servo_id: int, angle: int) -> bool:
        """Set servo angle (0-180)"""
        if not self.initialized:
            return False
        
        try:
            # This would control servo motors for camera
            logger.info(f"Setting servo {servo_id} to angle {angle}")
            return True
        except Exception as e:
            logger.error(f"Error setting servo angle: {e}")
            return False

class ArduinoInterface(HardwareInterface):
    """Arduino hardware interface using serial communication"""
    
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        self.initialized = False
    
    async def initialize(self) -> bool:
        """Initialize Arduino serial connection"""
        try:
            import serial
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
            await asyncio.sleep(2)  # Wait for Arduino to initialize
            
            # Send initialization command
            self.serial_connection.write(b"INIT\n")
            response = self.serial_connection.readline().decode().strip()
            
            if response == "READY":
                self.initialized = True
                logger.info("Arduino hardware initialized successfully")
                return True
            else:
                logger.error(f"Arduino initialization failed: {response}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to initialize Arduino hardware: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Close serial connection"""
        try:
            if self.serial_connection:
                self.serial_connection.close()
            self.initialized = False
            logger.info("Arduino hardware cleaned up")
        except Exception as e:
            logger.error(f"Error cleaning up Arduino hardware: {e}")
    
    async def _send_command(self, command: str) -> str:
        """Send command to Arduino and get response"""
        if not self.initialized or not self.serial_connection:
            return ""
        
        try:
            self.serial_connection.write(f"{command}\n".encode())
            response = self.serial_connection.readline().decode().strip()
            return response
        except Exception as e:
            logger.error(f"Error sending command to Arduino: {e}")
            return ""
    
    async def set_motor_speed(self, motor: str, speed: int) -> bool:
        """Set motor speed via Arduino"""
        response = await self._send_command(f"MOTOR_SPEED {motor} {speed}")
        return response == "OK"
    
    async def set_motor_direction(self, direction: MotorDirection) -> bool:
        """Set motor direction via Arduino"""
        response = await self._send_command(f"MOTOR_DIR {direction.value}")
        return response == "OK"
    
    async def read_sensor(self, sensor_type: SensorType) -> float:
        """Read sensor value via Arduino"""
        response = await self._send_command(f"READ_SENSOR {sensor_type.value}")
        try:
            return float(response)
        except ValueError:
            return 0.0
    
    async def set_led_color(self, r: int, g: int, b: int) -> bool:
        """Set LED color via Arduino"""
        response = await self._send_command(f"LED_COLOR {r} {g} {b}")
        return response == "OK"
    
    async def set_servo_angle(self, servo_id: int, angle: int) -> bool:
        """Set servo angle via Arduino"""
        response = await self._send_command(f"SERVO_ANGLE {servo_id} {angle}")
        return response == "OK"

class MockInterface(HardwareInterface):
    """Mock hardware interface for testing"""
    
    def __init__(self):
        self.initialized = False
        self.motor_speeds = {"left": 0, "right": 0}
        self.current_direction = MotorDirection.STOP
        self.sensor_values = {
            SensorType.ULTRASONIC: 25.0,
            SensorType.TEMPERATURE: 22.5,
            SensorType.HUMIDITY: 45.0,
            SensorType.LIGHT: 850.0
        }
    
    async def initialize(self) -> bool:
        """Initialize mock hardware"""
        self.initialized = True
        logger.info("Mock hardware initialized")
        return True
    
    async def cleanup(self) -> None:
        """Cleanup mock hardware"""
        self.initialized = False
        logger.info("Mock hardware cleaned up")
    
    async def set_motor_speed(self, motor: str, speed: int) -> bool:
        """Set mock motor speed"""
        self.motor_speeds[motor] = max(0, min(100, speed))
        logger.info(f"Mock motor {motor} speed set to {speed}%")
        return True
    
    async def set_motor_direction(self, direction: MotorDirection) -> bool:
        """Set mock motor direction"""
        self.current_direction = direction
        logger.info(f"Mock motor direction set to {direction.value}")
        return True
    
    async def read_sensor(self, sensor_type: SensorType) -> float:
        """Read mock sensor value"""
        # Simulate sensor noise
        import random
        base_value = self.sensor_values[sensor_type]
        noise = random.uniform(-0.1, 0.1) * base_value
        return base_value + noise
    
    async def set_led_color(self, r: int, g: int, b: int) -> bool:
        """Set mock LED color"""
        logger.info(f"Mock LED color set to RGB({r}, {g}, {b})")
        return True
    
    async def set_servo_angle(self, servo_id: int, angle: int) -> bool:
        """Set mock servo angle"""
        logger.info(f"Mock servo {servo_id} angle set to {angle}")
        return True

class HardwareManager:
    """Manages hardware interfaces and provides unified API"""
    
    def __init__(self):
        self.interface: Optional[HardwareInterface] = None
        self.initialized = False
    
    async def initialize(self, platform: str = "mock", **kwargs) -> bool:
        """Initialize hardware interface based on platform"""
        try:
            if platform == "raspberry_pi":
                motor_config = MotorConfig(
                    left_motor_pins=[18, 23, 24],  # PWM, IN1, IN2
                    right_motor_pins=[19, 25, 8]   # PWM, IN1, IN2
                )
                sensor_configs = [
                    SensorConfig(SensorType.ULTRASONIC, 20),
                    SensorConfig(SensorType.TEMPERATURE, 21)
                ]
                self.interface = RaspberryPiInterface(motor_config, sensor_configs)
            elif platform == "arduino":
                self.interface = ArduinoInterface(**kwargs)
            elif platform == "mock":
                self.interface = MockInterface()
            else:
                raise ValueError(f"Unsupported platform: {platform}")
            
            success = await self.interface.initialize()
            if success:
                self.initialized = True
                logger.info(f"Hardware manager initialized with {platform} interface")
            return success
            
        except Exception as e:
            logger.error(f"Failed to initialize hardware manager: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup hardware resources"""
        if self.interface:
            await self.interface.cleanup()
        self.initialized = False
    
    async def execute_command(self, command: str, data: Dict[str, Any] = None) -> bool:
        """Execute robot command"""
        if not self.initialized or not self.interface:
            return False
        
        try:
            if command == "forward":
                return await self.interface.set_motor_direction(MotorDirection.FORWARD)
            elif command == "backward":
                return await self.interface.set_motor_direction(MotorDirection.BACKWARD)
            elif command == "left":
                return await self.interface.set_motor_direction(MotorDirection.LEFT)
            elif command == "right":
                return await self.interface.set_motor_direction(MotorDirection.RIGHT)
            elif command == "stop":
                return await self.interface.set_motor_direction(MotorDirection.STOP)
            elif command == "speed_up" and data:
                current_speed = data.get("current_speed", 50)
                new_speed = min(100, current_speed + 10)
                return await self.interface.set_motor_speed("left", new_speed) and \
                       await self.interface.set_motor_speed("right", new_speed)
            elif command == "speed_down" and data:
                current_speed = data.get("current_speed", 50)
                new_speed = max(0, current_speed - 10)
                return await self.interface.set_motor_speed("left", new_speed) and \
                       await self.interface.set_motor_speed("right", new_speed)
            elif command == "camera_up" and data:
                return await self.interface.set_servo_angle(0, data.get("tilt", 0) + 5)
            elif command == "camera_down" and data:
                return await self.interface.set_servo_angle(0, data.get("tilt", 0) - 5)
            elif command == "camera_left" and data:
                return await self.interface.set_servo_angle(1, data.get("pan", 0) + 5)
            elif command == "camera_right" and data:
                return await self.interface.set_servo_angle(1, data.get("pan", 0) - 5)
            elif command == "led_color" and data:
                color = data.get("color", "#ff0000")
                r = int(color[1:3], 16)
                g = int(color[3:5], 16)
                b = int(color[5:7], 16)
                return await self.interface.set_led_color(r, g, b)
            else:
                logger.warning(f"Unknown command: {command}")
                return False
                
        except Exception as e:
            logger.error(f"Error executing command {command}: {e}")
            return False
    
    async def read_all_sensors(self) -> Dict[str, float]:
        """Read all available sensors"""
        if not self.initialized or not self.interface:
            return {}
        
        sensors = {}
        for sensor_type in SensorType:
            try:
                value = await self.interface.read_sensor(sensor_type)
                sensors[sensor_type.value] = value
            except Exception as e:
                logger.error(f"Error reading sensor {sensor_type}: {e}")
        
        return sensors

# Global hardware manager instance
hardware_manager = HardwareManager()
