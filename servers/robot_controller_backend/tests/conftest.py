# Backend Test Configuration
# This file contains test configurations and utilities for the robot controller backend

import pytest
import asyncio
import sys
import types
from unittest.mock import Mock, patch, MagicMock
import logging

# Configure logging for tests
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ============================================================================
# Hardware Module Mocking (for non-Pi environments)
# ============================================================================

# Mock rpi_ws281x (LED strip library)
class MockWS281x:
    def __init__(self, *args, **kwargs):
        pass
    def begin(self):
        pass
    def numPixels(self):
        return 8
    def setPixelColor(self, i, color):
        pass
    def show(self):
        pass

if 'rpi_ws281x' not in sys.modules:
    mock_rpi_ws281x = types.ModuleType("rpi_ws281x")
    mock_rpi_ws281x.Adafruit_NeoPixel = MockWS281x
    mock_rpi_ws281x.Color = lambda r, g, b: (r << 16) | (g << 8) | b
    mock_rpi_ws281x.WS2811_STRIP_GRB = 0
    sys.modules["rpi_ws281x"] = mock_rpi_ws281x

# Mock PCA9685 (PWM controller)
class MockPCA9685:
    def __init__(self, address=0x40, debug=False):
        self.address = address
        self.debug = debug
    def setPWMFreq(self, freq):
        pass
    def setPWM(self, channel, on, off):
        pass
    def setMotorPwm(self, channel, pwm):
        pass
    def setServoPulse(self, channel, pulse):
        pass

if 'PCA9685' not in sys.modules:
    mock_pca9685_module = types.ModuleType("PCA9685")
    mock_pca9685_module.PCA9685 = MockPCA9685
    sys.modules["PCA9685"] = mock_pca9685_module

# Mock RPi.GPIO
class MockGPIO:
    BCM = 11
    OUT = 0
    IN = 1
    HIGH = 1
    LOW = 0
    
    @staticmethod
    def setmode(mode):
        pass
    
    @staticmethod
    def setup(pin, mode):
        pass
    
    @staticmethod
    def output(pin, value):
        pass
    
    @staticmethod
    def input(pin):
        return 0
    
    @staticmethod
    def cleanup():
        pass

if 'RPi' not in sys.modules:
    mock_rpi = types.ModuleType("RPi")
    mock_rpi.GPIO = MockGPIO
    sys.modules["RPi"] = mock_rpi
    sys.modules["RPi.GPIO"] = MockGPIO

# Mock picamera2
class MockPicamera2:
    def __init__(self, *args, **kwargs):
        pass
    def start(self):
        pass
    def stop(self):
        pass
    def capture_array(self):
        import numpy as np
        return np.zeros((480, 640, 3), dtype=np.uint8)

if 'picamera2' not in sys.modules:
    mock_picamera2 = types.ModuleType("picamera2")
    mock_picamera2.Picamera2 = MockPicamera2
    sys.modules["picamera2"] = mock_picamera2

# Test configuration
TEST_CONFIG = {
    "MOVEMENT_WS_PORT": 8081,
    "SENSOR_WS_PORT": 8090,
    "API_PORT": 8080,
    "TEST_TIMEOUT": 10,
    "MOCK_HARDWARE": True
}

# Test fixtures
@pytest.fixture
def event_loop():
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()

@pytest.fixture
def mock_pca9685():
    """Mock PCA9685 for testing without hardware."""
    return MockPCA9685()

@pytest.fixture
def mock_led_control():
    """Mock LED control for testing."""
    with patch('controllers.lighting.led_control.LedControl') as mock:
        instance = Mock()
        instance.color_wipe = Mock()
        instance.theater_chase = Mock()
        instance.rainbow = Mock()
        mock.return_value = instance
        yield instance

@pytest.fixture(autouse=True)
def mock_hardware_modules():
    """Automatically mock hardware modules for all tests."""
    with patch.dict('sys.modules', {
        'rpi_ws281x': sys.modules.get('rpi_ws281x'),
        'PCA9685': sys.modules.get('PCA9685'),
        'RPi': sys.modules.get('RPi'),
        'RPi.GPIO': sys.modules.get('RPi.GPIO'),
        'picamera2': sys.modules.get('picamera2'),
    }):
        yield

# Test utilities
class WebSocketTestClient:
    """Utility class for testing WebSocket connections."""
    
    def __init__(self, uri):
        self.uri = uri
        self.websocket = None
        self.messages = []
    
    async def connect(self):
        """Connect to the WebSocket server."""
        try:
            import websockets
            self.websocket = await websockets.connect(self.uri)
            return self.websocket
        except ImportError:
            pytest.skip("websockets library not available")
        except Exception:
            pytest.skip(f"WebSocket server not available at {self.uri}")
    
    async def disconnect(self):
        """Disconnect from the WebSocket server."""
        if self.websocket:
            await self.websocket.close()
    
    async def send_command(self, command, data=None):
        """Send a command to the WebSocket server."""
        import json
        message = {"command": command}
        if data:
            message.update(data)
        
        await self.websocket.send(json.dumps(message))
        return await self.websocket.recv()
    
    async def send_json(self, data):
        """Send JSON data to the WebSocket server."""
        import json
        await self.websocket.send(json.dumps(data))
        return await self.websocket.recv()
    
    async def receive_message(self, timeout=5):
        """Receive a message from the WebSocket server."""
        try:
            message = await asyncio.wait_for(self.websocket.recv(), timeout=timeout)
            import json
            return json.loads(message)
        except asyncio.TimeoutError:
            return None

# Test data generators
def generate_motor_telemetry():
    """Generate mock motor telemetry data."""
    return {
        "frontLeft": {
            "speed": 100.0,
            "power": 20.0,
            "pwm": 1500,
            "current": 2.5,
            "temperature": 25.0
        },
        "frontRight": {
            "speed": 100.0,
            "power": 20.0,
            "pwm": 1500,
            "current": 2.5,
            "temperature": 25.0
        },
        "rearLeft": {
            "speed": 100.0,
            "power": 20.0,
            "pwm": 1500,
            "current": 2.5,
            "temperature": 25.0
        },
        "rearRight": {
            "speed": 100.0,
            "power": 20.0,
            "pwm": 1500,
            "current": 2.5,
            "temperature": 25.0
        }
    }

def generate_servo_telemetry():
    """Generate mock servo telemetry data."""
    return {
        "horizontal": 90,
        "vertical": 45,
        "min": 0,
        "max": 180
    }

def generate_sensor_data():
    """Generate mock sensor data."""
    return {
        "lineTracking": {
            "left": 0,
            "center": 1,
            "right": 0
        },
        "ultrasonic": {
            "distance": 50.0,
            "unit": "cm"
        }
    }

# Performance testing utilities
class PerformanceMonitor:
    """Monitor performance metrics during tests."""
    
    def __init__(self):
        self.metrics = {}
        self.start_times = {}
    
    def start_timer(self, operation):
        """Start timing an operation."""
        import time
        self.start_times[operation] = time.time()
    
    def end_timer(self, operation):
        """End timing an operation."""
        import time
        if operation in self.start_times:
            duration = time.time() - self.start_times[operation]
            if operation not in self.metrics:
                self.metrics[operation] = []
            self.metrics[operation].append(duration)
            return duration
        return 0
    
    def get_average_time(self, operation):
        """Get average time for an operation."""
        if operation in self.metrics and self.metrics[operation]:
            return sum(self.metrics[operation]) / len(self.metrics[operation])
        return 0
    
    def get_max_time(self, operation):
        """Get maximum time for an operation."""
        if operation in self.metrics and self.metrics[operation]:
            return max(self.metrics[operation])
        return 0

# Error testing utilities
def simulate_network_error():
    """Simulate network errors for testing."""
    raise ConnectionError("Simulated network error")

def simulate_hardware_error():
    """Simulate hardware errors for testing."""
    raise RuntimeError("Simulated hardware error")

def simulate_timeout_error():
    """Simulate timeout errors for testing."""
    raise asyncio.TimeoutError("Simulated timeout error")

# Test markers
pytest.mark.unit = pytest.mark.unit
pytest.mark.integration = pytest.mark.integration
pytest.mark.performance = pytest.mark.performance
pytest.mark.hardware = pytest.mark.hardware
pytest.mark.websocket = pytest.mark.websocket
pytest.mark.motor = pytest.mark.motor
pytest.mark.servo = pytest.mark.servo
pytest.mark.sensor = pytest.mark.sensor
