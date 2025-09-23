# Backend Test Configuration
# This file contains test configurations and utilities for the robot controller backend

import pytest
import asyncio
import websockets
import json
import time
from unittest.mock import Mock, patch
import logging

# Configure logging for tests
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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
async def mock_pca9685():
    """Mock PCA9685 for testing without hardware."""
    with patch('movement.motor_telemetry.PCA9685') as mock:
        mock_instance = Mock()
        mock_instance.setPWMFreq.return_value = None
        mock_instance.setPWM.return_value = None
        mock_instance.setMotorPwm.return_value = None
        mock_instance.setServoPulse.return_value = None
        mock.return_value = mock_instance
        yield mock_instance

@pytest.fixture
async def movement_websocket():
    """Create a WebSocket connection to the movement server."""
    uri = f"ws://localhost:{TEST_CONFIG['MOVEMENT_WS_PORT']}/"
    try:
        async with websockets.connect(uri) as websocket:
            yield websocket
    except ConnectionRefusedError:
        pytest.skip("Movement WebSocket server not running")

@pytest.fixture
async def sensor_websocket():
    """Create a WebSocket connection to the sensor server."""
    uri = f"ws://localhost:{TEST_CONFIG['SENSOR_WS_PORT']}/"
    try:
        async with websockets.connect(uri) as websocket:
            yield websocket
    except ConnectionRefusedError:
        pytest.skip("Sensor WebSocket server not running")

# Test utilities
class WebSocketTestClient:
    """Utility class for testing WebSocket connections."""
    
    def __init__(self, uri):
        self.uri = uri
        self.websocket = None
        self.messages = []
    
    async def connect(self):
        """Connect to the WebSocket server."""
        self.websocket = await websockets.connect(self.uri)
        return self.websocket
    
    async def disconnect(self):
        """Disconnect from the WebSocket server."""
        if self.websocket:
            await self.websocket.close()
    
    async def send_command(self, command, data=None):
        """Send a command to the WebSocket server."""
        message = {"command": command}
        if data:
            message.update(data)
        
        await self.websocket.send(json.dumps(message))
        return await self.websocket.recv()
    
    async def send_json(self, data):
        """Send JSON data to the WebSocket server."""
        await self.websocket.send(json.dumps(data))
        return await self.websocket.recv()
    
    async def receive_message(self, timeout=5):
        """Receive a message from the WebSocket server."""
        try:
            message = await asyncio.wait_for(self.websocket.recv(), timeout=timeout)
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
        self.start_times[operation] = time.time()
    
    def end_timer(self, operation):
        """End timing an operation."""
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