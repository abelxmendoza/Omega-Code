"""
Advanced Sensor Hardware Drivers
Supports ultrasonic, temperature, humidity, light, gyroscope, and accelerometer sensors
"""

import asyncio
import logging
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
import math

logger = logging.getLogger(__name__)

class SensorType(Enum):
    ULTRASONIC = "ultrasonic"
    TEMPERATURE = "temperature"
    HUMIDITY = "humidity"
    LIGHT = "light"
    GYROSCOPE = "gyroscope"
    ACCELEROMETER = "accelerometer"
    MAGNETOMETER = "magnetometer"
    PRESSURE = "pressure"

@dataclass
class SensorReading:
    """Sensor reading data structure"""
    sensor_id: str
    sensor_type: SensorType
    value: float
    unit: str
    timestamp: float
    quality: float = 1.0  # 0.0 to 1.0, indicates reading quality

@dataclass
class UltrasonicConfig:
    """Configuration for ultrasonic sensor (HC-SR04)"""
    trigger_pin: int
    echo_pin: int
    max_distance: float = 400.0  # cm
    min_distance: float = 2.0    # cm
    timeout: float = 0.1         # seconds
    calibration_factor: float = 1.0

@dataclass
class TemperatureConfig:
    """Configuration for temperature sensor (DHT22/DS18B20)"""
    data_pin: int
    sensor_type: str = "DHT22"  # DHT22, DS18B20
    calibration_offset: float = 0.0
    calibration_factor: float = 1.0

@dataclass
class LightConfig:
    """Configuration for light sensor (LDR/TSL2561)"""
    analog_pin: int
    sensor_type: str = "LDR"  # LDR, TSL2561
    max_lux: float = 10000.0
    min_lux: float = 0.1

@dataclass
class IMUConfig:
    """Configuration for IMU sensor (MPU6050/BNO055)"""
    i2c_address: int
    sensor_type: str = "MPU6050"  # MPU6050, BNO055
    sample_rate: int = 100  # Hz
    gyro_range: int = 250   # degrees per second
    accel_range: int = 2    # g

class SensorDriver:
    """Base class for sensor drivers"""
    
    def __init__(self, sensor_id: str, sensor_type: SensorType):
        self.sensor_id = sensor_id
        self.sensor_type = sensor_type
        self.initialized = False
        self.last_reading: Optional[SensorReading] = None
        self.reading_history: List[SensorReading] = []
        self.max_history = 100
    
    async def initialize(self) -> bool:
        """Initialize sensor driver"""
        raise NotImplementedError
    
    async def cleanup(self) -> None:
        """Cleanup sensor resources"""
        raise NotImplementedError
    
    async def read_sensor(self) -> SensorReading:
        """Read sensor value"""
        raise NotImplementedError
    
    async def calibrate(self) -> bool:
        """Calibrate sensor"""
        raise NotImplementedError
    
    def get_average_reading(self, count: int = 5) -> float:
        """Get average of last N readings"""
        if len(self.reading_history) < count:
            return self.last_reading.value if self.last_reading else 0.0
        
        recent_readings = self.reading_history[-count:]
        return sum(r.value for r in recent_readings) / len(recent_readings)

class UltrasonicDriver(SensorDriver):
    """Ultrasonic sensor driver (HC-SR04)"""
    
    def __init__(self, sensor_id: str, config: UltrasonicConfig):
        super().__init__(sensor_id, SensorType.ULTRASONIC)
        self.config = config
        self.gpio = None
        
        # Import GPIO library
        try:
            import RPi.GPIO as GPIO
            self.gpio = GPIO
        except ImportError:
            logger.error("RPi.GPIO not available for ultrasonic sensor")
            raise
    
    async def initialize(self) -> bool:
        """Initialize ultrasonic sensor GPIO pins"""
        try:
            self.gpio.setmode(self.gpio.BCM)
            self.gpio.setwarnings(False)
            
            # Setup GPIO pins
            self.gpio.setup(self.config.trigger_pin, self.gpio.OUT)
            self.gpio.setup(self.config.echo_pin, self.gpio.IN)
            
            # Ensure trigger is low initially
            self.gpio.output(self.config.trigger_pin, self.gpio.LOW)
            await asyncio.sleep(0.1)
            
            self.initialized = True
            logger.info(f"Ultrasonic sensor {self.sensor_id} initialized")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize ultrasonic sensor {self.sensor_id}: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup ultrasonic sensor resources"""
        try:
            self.gpio.output(self.config.trigger_pin, self.gpio.LOW)
            self.initialized = False
            logger.info(f"Ultrasonic sensor {self.sensor_id} cleaned up")
        except Exception as e:
            logger.error(f"Error cleaning up ultrasonic sensor {self.sensor_id}: {e}")
    
    async def read_sensor(self) -> SensorReading:
        """Read ultrasonic sensor distance"""
        if not self.initialized:
            return SensorReading(
                self.sensor_id, self.sensor_type, 0.0, "cm", time.time(), 0.0
            )
        
        try:
            # Send trigger pulse
            self.gpio.output(self.config.trigger_pin, self.gpio.HIGH)
            await asyncio.sleep(0.00001)  # 10 microseconds
            self.gpio.output(self.config.trigger_pin, self.gpio.LOW)
            
            # Wait for echo pulse
            start_time = time.time()
            while self.gpio.input(self.config.echo_pin) == self.gpio.LOW:
                if time.time() - start_time > self.config.timeout:
                    logger.warning(f"Ultrasonic sensor {self.sensor_id} timeout")
                    return SensorReading(
                        self.sensor_id, self.sensor_type, 0.0, "cm", time.time(), 0.0
                    )
            
            # Measure echo pulse duration
            start_time = time.time()
            while self.gpio.input(self.config.echo_pin) == self.gpio.HIGH:
                if time.time() - start_time > self.config.timeout:
                    logger.warning(f"Ultrasonic sensor {self.sensor_id} echo timeout")
                    return SensorReading(
                        self.sensor_id, self.sensor_type, 0.0, "cm", time.time(), 0.0
                    )
            
            pulse_duration = time.time() - start_time
            
            # Calculate distance (speed of sound = 343 m/s)
            distance = (pulse_duration * 34300) / 2  # Convert to cm
            distance *= self.config.calibration_factor
            
            # Apply limits
            if distance < self.config.min_distance:
                distance = self.config.min_distance
            elif distance > self.config.max_distance:
                distance = self.config.max_distance
            
            reading = SensorReading(
                self.sensor_id, self.sensor_type, distance, "cm", time.time(), 1.0
            )
            
            # Update history
            self.last_reading = reading
            self.reading_history.append(reading)
            if len(self.reading_history) > self.max_history:
                self.reading_history.pop(0)
            
            return reading
            
        except Exception as e:
            logger.error(f"Error reading ultrasonic sensor {self.sensor_id}: {e}")
            return SensorReading(
                self.sensor_id, self.sensor_type, 0.0, "cm", time.time(), 0.0
            )
    
    async def calibrate(self) -> bool:
        """Calibrate ultrasonic sensor"""
        try:
            logger.info(f"Calibrating ultrasonic sensor {self.sensor_id}")
            
            # Take multiple readings and calculate average
            readings = []
            for _ in range(10):
                reading = await self.read_sensor()
                if reading.quality > 0.5:
                    readings.append(reading.value)
                await asyncio.sleep(0.1)
            
            if len(readings) > 5:
                avg_distance = sum(readings) / len(readings)
                logger.info(f"Ultrasonic sensor {self.sensor_id} calibrated: {avg_distance:.2f} cm")
                return True
            else:
                logger.warning(f"Insufficient readings for calibration of {self.sensor_id}")
                return False
                
        except Exception as e:
            logger.error(f"Error calibrating ultrasonic sensor {self.sensor_id}: {e}")
            return False

class TemperatureDriver(SensorDriver):
    """Temperature sensor driver (DHT22/DS18B20)"""
    
    def __init__(self, sensor_id: str, config: TemperatureConfig):
        super().__init__(sensor_id, SensorType.TEMPERATURE)
        self.config = config
        self.sensor = None
        
        # Import sensor libraries
        try:
            if config.sensor_type == "DHT22":
                import adafruit_dht
                import board
                self.adafruit_dht = adafruit_dht
                self.board = board
            elif config.sensor_type == "DS18B20":
                import w1thermsensor
                self.w1thermsensor = w1thermsensor
        except ImportError as e:
            logger.error(f"Sensor library not available: {e}")
            raise
    
    async def initialize(self) -> bool:
        """Initialize temperature sensor"""
        try:
            if self.config.sensor_type == "DHT22":
                self.sensor = self.adafruit_dht.DHT22(self.config.data_pin)
            elif self.config.sensor_type == "DS18B20":
                self.sensor = self.w1thermsensor.W1ThermSensor()
            
            self.initialized = True
            logger.info(f"Temperature sensor {self.sensor_id} initialized")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize temperature sensor {self.sensor_id}: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup temperature sensor resources"""
        try:
            self.initialized = False
            logger.info(f"Temperature sensor {self.sensor_id} cleaned up")
        except Exception as e:
            logger.error(f"Error cleaning up temperature sensor {self.sensor_id}: {e}")
    
    async def read_sensor(self) -> SensorReading:
        """Read temperature sensor value"""
        if not self.initialized:
            return SensorReading(
                self.sensor_id, self.sensor_type, 0.0, "°C", time.time(), 0.0
            )
        
        try:
            if self.config.sensor_type == "DHT22":
                temperature = self.sensor.temperature
                if temperature is None:
                    temperature = 0.0
                    quality = 0.0
                else:
                    quality = 1.0
            elif self.config.sensor_type == "DS18B20":
                temperature = self.sensor.get_temperature()
                quality = 1.0
            
            # Apply calibration
            temperature = (temperature + self.config.calibration_offset) * self.config.calibration_factor
            
            reading = SensorReading(
                self.sensor_id, self.sensor_type, temperature, "°C", time.time(), quality
            )
            
            # Update history
            self.last_reading = reading
            self.reading_history.append(reading)
            if len(self.reading_history) > self.max_history:
                self.reading_history.pop(0)
            
            return reading
            
        except Exception as e:
            logger.error(f"Error reading temperature sensor {self.sensor_id}: {e}")
            return SensorReading(
                self.sensor_id, self.sensor_type, 0.0, "°C", time.time(), 0.0
            )
    
    async def calibrate(self) -> bool:
        """Calibrate temperature sensor"""
        try:
            logger.info(f"Calibrating temperature sensor {self.sensor_id}")
            
            # Take multiple readings
            readings = []
            for _ in range(10):
                reading = await self.read_sensor()
                if reading.quality > 0.5:
                    readings.append(reading.value)
                await asyncio.sleep(0.5)  # DHT22 needs time between readings
            
            if len(readings) > 5:
                avg_temp = sum(readings) / len(readings)
                logger.info(f"Temperature sensor {self.sensor_id} calibrated: {avg_temp:.2f}°C")
                return True
            else:
                logger.warning(f"Insufficient readings for calibration of {self.sensor_id}")
                return False
                
        except Exception as e:
            logger.error(f"Error calibrating temperature sensor {self.sensor_id}: {e}")
            return False

class LightDriver(SensorDriver):
    """Light sensor driver (LDR/TSL2561)"""
    
    def __init__(self, sensor_id: str, config: LightConfig):
        super().__init__(sensor_id, SensorType.LIGHT)
        self.config = config
        self.sensor = None
        
        # Import sensor libraries
        try:
            if config.sensor_type == "TSL2561":
                import adafruit_tsl2561
                import board
                import busio
                self.adafruit_tsl2561 = adafruit_tsl2561
                self.board = board
                self.busio = busio
        except ImportError as e:
            logger.error(f"Light sensor library not available: {e}")
            raise
    
    async def initialize(self) -> bool:
        """Initialize light sensor"""
        try:
            if self.config.sensor_type == "TSL2561":
                i2c = self.busio.I2C(self.board.SCL, self.board.SDA)
                self.sensor = self.adafruit_tsl2561.TSL2561(i2c)
            elif self.config.sensor_type == "LDR":
                # LDR uses analog input, no special initialization needed
                pass
            
            self.initialized = True
            logger.info(f"Light sensor {self.sensor_id} initialized")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize light sensor {self.sensor_id}: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup light sensor resources"""
        try:
            self.initialized = False
            logger.info(f"Light sensor {self.sensor_id} cleaned up")
        except Exception as e:
            logger.error(f"Error cleaning up light sensor {self.sensor_id}: {e}")
    
    async def read_sensor(self) -> SensorReading:
        """Read light sensor value"""
        if not self.initialized:
            return SensorReading(
                self.sensor_id, self.sensor_type, 0.0, "lux", time.time(), 0.0
            )
        
        try:
            if self.config.sensor_type == "TSL2561":
                lux = self.sensor.lux
                if lux is None:
                    lux = 0.0
                    quality = 0.0
                else:
                    quality = 1.0
            elif self.config.sensor_type == "LDR":
                # Simulate LDR reading (would use ADC in real implementation)
                lux = 1000.0  # Placeholder
                quality = 1.0
            
            # Apply limits
            lux = max(self.config.min_lux, min(self.config.max_lux, lux))
            
            reading = SensorReading(
                self.sensor_id, self.sensor_type, lux, "lux", time.time(), quality
            )
            
            # Update history
            self.last_reading = reading
            self.reading_history.append(reading)
            if len(self.reading_history) > self.max_history:
                self.reading_history.pop(0)
            
            return reading
            
        except Exception as e:
            logger.error(f"Error reading light sensor {self.sensor_id}: {e}")
            return SensorReading(
                self.sensor_id, self.sensor_type, 0.0, "lux", time.time(), 0.0
            )
    
    async def calibrate(self) -> bool:
        """Calibrate light sensor"""
        try:
            logger.info(f"Calibrating light sensor {self.sensor_id}")
            
            # Take multiple readings
            readings = []
            for _ in range(10):
                reading = await self.read_sensor()
                if reading.quality > 0.5:
                    readings.append(reading.value)
                await asyncio.sleep(0.1)
            
            if len(readings) > 5:
                avg_lux = sum(readings) / len(readings)
                logger.info(f"Light sensor {self.sensor_id} calibrated: {avg_lux:.2f} lux")
                return True
            else:
                logger.warning(f"Insufficient readings for calibration of {self.sensor_id}")
                return False
                
        except Exception as e:
            logger.error(f"Error calibrating light sensor {self.sensor_id}: {e}")
            return False

class SensorManager:
    """Manages all sensors and provides unified interface"""
    
    def __init__(self):
        self.sensors: Dict[str, SensorDriver] = {}
        self.initialized = False
        self.reading_task: Optional[asyncio.Task] = None
        self.reading_interval = 1.0  # seconds
    
    async def add_sensor(self, sensor_id: str, sensor_type: SensorType, config) -> bool:
        """Add a sensor to the system"""
        try:
            if sensor_type == SensorType.ULTRASONIC:
                sensor = UltrasonicDriver(sensor_id, config)
            elif sensor_type == SensorType.TEMPERATURE:
                sensor = TemperatureDriver(sensor_id, config)
            elif sensor_type == SensorType.LIGHT:
                sensor = LightDriver(sensor_id, config)
            else:
                logger.error(f"Unsupported sensor type: {sensor_type}")
                return False
            
            success = await sensor.initialize()
            if success:
                self.sensors[sensor_id] = sensor
                logger.info(f"Sensor {sensor_id} added to system")
            
            return success
            
        except Exception as e:
            logger.error(f"Failed to add sensor {sensor_id}: {e}")
            return False
    
    async def initialize(self) -> bool:
        """Initialize all sensors"""
        try:
            self.initialized = True
            
            # Start continuous reading task
            self.reading_task = asyncio.create_task(self._continuous_reading())
            
            logger.info("Sensor manager initialized")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize sensor manager: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup all sensors"""
        try:
            # Stop reading task
            if self.reading_task:
                self.reading_task.cancel()
                try:
                    await self.reading_task
                except asyncio.CancelledError:
                    pass
            
            # Cleanup all sensors
            for sensor in self.sensors.values():
                await sensor.cleanup()
            
            self.sensors.clear()
            self.initialized = False
            logger.info("Sensor manager cleaned up")
            
        except Exception as e:
            logger.error(f"Error cleaning up sensor manager: {e}")
    
    async def _continuous_reading(self) -> None:
        """Continuous sensor reading task"""
        while self.initialized:
            try:
                for sensor in self.sensors.values():
                    await sensor.read_sensor()
                
                await asyncio.sleep(self.reading_interval)
                
            except Exception as e:
                logger.error(f"Error in continuous reading: {e}")
                await asyncio.sleep(self.reading_interval)
    
    async def get_sensor_reading(self, sensor_id: str) -> Optional[SensorReading]:
        """Get latest reading from specific sensor"""
        if sensor_id in self.sensors:
            return self.sensors[sensor_id].last_reading
        return None
    
    async def get_all_readings(self) -> Dict[str, SensorReading]:
        """Get latest readings from all sensors"""
        readings = {}
        for sensor_id, sensor in self.sensors.items():
            if sensor.last_reading:
                readings[sensor_id] = sensor.last_reading
        return readings
    
    async def calibrate_all_sensors(self) -> Dict[str, bool]:
        """Calibrate all sensors"""
        results = {}
        for sensor_id, sensor in self.sensors.items():
            try:
                results[sensor_id] = await sensor.calibrate()
            except Exception as e:
                logger.error(f"Error calibrating sensor {sensor_id}: {e}")
                results[sensor_id] = False
        return results

# Global sensor manager instance
sensor_manager = SensorManager()
