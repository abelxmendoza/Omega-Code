import asyncio
import json
import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum
import websockets
from websockets.server import WebSocketServerProtocol

# Import hardware modules
from hardware.hardware_manager import hardware_manager, HardwareManager
from hardware.motor_control import motor_system, MotorType, DCMotorConfig, ServoMotorConfig
from hardware.sensor_drivers import sensor_manager, SensorType, UltrasonicConfig, TemperatureConfig, LightConfig
from hardware.camera_drivers import camera_manager, CameraType, CameraConfig
from hardware.led_control import led_manager, LEDStripType, LEDStripConfig, LEDPattern, LEDColor

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotCommand(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    STOP = "stop"
    SPEED_UP = "speed_up"
    SPEED_DOWN = "speed_down"
    CAMERA_UP = "camera_up"
    CAMERA_DOWN = "camera_down"
    CAMERA_LEFT = "camera_left"
    CAMERA_RIGHT = "camera_right"

@dataclass
class RobotState:
    speed: int = 50
    direction: str = "stop"
    camera_tilt: int = 0
    camera_pan: int = 0
    led_color: str = "#ff0000"
    led_brightness: int = 75
    led_pattern: str = "static"
    connected: bool = False
    battery_level: int = 85
    sensors: Dict[str, Any] = None

    def __post_init__(self):
        if self.sensors is None:
            self.sensors = {
                "ultrasonic": {"distance": 25, "unit": "cm"},
                "temperature": {"value": 22.5, "unit": "°C"},
                "humidity": {"value": 45, "unit": "%"},
                "light": {"value": 850, "unit": "lux"}
            }

class RobotController:
    def __init__(self):
        self.state = RobotState()
        self.clients: set[WebSocketServerProtocol] = set()
        self.running = False
        
    async def start(self):
        """Start the robot controller"""
        self.running = True
        logger.info("Robot controller started")
        
        # Initialize hardware components
        await self._initialize_hardware()
        
        # Start background tasks
        asyncio.create_task(self._update_sensors())
        asyncio.create_task(self._simulate_robot_movement())
        
    async def stop(self):
        """Stop the robot controller"""
        self.running = False
        logger.info("Robot controller stopped")
        
        # Cleanup hardware components
        await self._cleanup_hardware()
    
    async def _initialize_hardware(self):
        """Initialize all hardware components"""
        try:
            logger.info("Initializing hardware components...")
            
            # Initialize hardware manager
            await hardware_manager.initialize("mock")  # Use mock for testing
            
            # Initialize motor system
            await motor_system.initialize()
            
            # Add motors
            left_motor_config = DCMotorConfig(
                enable_pin=18, in1_pin=23, in2_pin=24
            )
            right_motor_config = DCMotorConfig(
                enable_pin=19, in1_pin=25, in2_pin=8
            )
            
            await motor_system.add_motor("left_motor", MotorType.DC_MOTOR, left_motor_config)
            await motor_system.add_motor("right_motor", MotorType.DC_MOTOR, right_motor_config)
            
            # Add camera servo
            camera_servo_config = ServoMotorConfig(control_pin=21)
            await motor_system.add_motor("camera_servo", MotorType.SERVO_MOTOR, camera_servo_config)
            
            # Initialize sensor manager
            await sensor_manager.initialize()
            
            # Add sensors
            ultrasonic_config = UltrasonicConfig(trigger_pin=20, echo_pin=21)
            temperature_config = TemperatureConfig(data_pin=22)
            light_config = LightConfig(analog_pin=23)
            
            await sensor_manager.add_sensor("ultrasonic", SensorType.ULTRASONIC, ultrasonic_config)
            await sensor_manager.add_sensor("temperature", SensorType.TEMPERATURE, temperature_config)
            await sensor_manager.add_sensor("light", SensorType.LIGHT, light_config)
            
            # Initialize camera manager
            await camera_manager.initialize()
            
            # Add camera
            camera_config = CameraConfig(
                camera_type=CameraType.USB_CAMERA,
                device_id=0,
                resolution=(640, 480),
                fps=30
            )
            await camera_manager.add_camera("main_camera", camera_config)
            
            # Initialize LED manager
            await led_manager.initialize()
            
            # Add LED strip
            led_config = LEDStripConfig(
                strip_type=LEDStripType.WS2812B,
                pin=18,
                num_leds=50
            )
            await led_manager.add_strip("main_leds", led_config)
            
            logger.info("Hardware initialization completed")
            
        except Exception as e:
            logger.error(f"Hardware initialization failed: {e}")
    
    async def _cleanup_hardware(self):
        """Cleanup all hardware components"""
        try:
            logger.info("Cleaning up hardware components...")
            
            await hardware_manager.cleanup()
            await motor_system.cleanup()
            await sensor_manager.cleanup()
            await camera_manager.cleanup()
            await led_manager.cleanup()
            
            logger.info("Hardware cleanup completed")
            
        except Exception as e:
            logger.error(f"Hardware cleanup failed: {e}")
        
    async def add_client(self, websocket: WebSocketServerProtocol):
        """Add a new WebSocket client"""
        self.clients.add(websocket)
        logger.info(f"Client connected. Total clients: {len(self.clients)}")
        
        # Send current state to new client
        await self._send_state_update(websocket)
        
    async def remove_client(self, websocket: WebSocketServerProtocol):
        """Remove a WebSocket client"""
        self.clients.discard(websocket)
        logger.info(f"Client disconnected. Total clients: {len(self.clients)}")
        
    async def handle_command(self, command: str, data: Dict[str, Any] = None):
        """Handle robot commands with real hardware integration"""
        try:
            cmd = RobotCommand(command)
            logger.info(f"Executing command: {cmd.value}")
            
            # Execute commands using real hardware
            if cmd == RobotCommand.FORWARD:
                self.state.direction = "forward"
                await motor_system.move_forward(self.state.speed)
            elif cmd == RobotCommand.BACKWARD:
                self.state.direction = "backward"
                await motor_system.move_backward(self.state.speed)
            elif cmd == RobotCommand.LEFT:
                self.state.direction = "left"
                await motor_system.turn_left(self.state.speed)
            elif cmd == RobotCommand.RIGHT:
                self.state.direction = "right"
                await motor_system.turn_right(self.state.speed)
            elif cmd == RobotCommand.STOP:
                self.state.direction = "stop"
                await motor_system.stop_all()
            elif cmd == RobotCommand.SPEED_UP:
                self.state.speed = min(100, self.state.speed + 10)
                # Update motor speeds
                for motor_id in ["left_motor", "right_motor"]:
                    await motor_system.motors[motor_id].set_speed(self.state.speed)
            elif cmd == RobotCommand.SPEED_DOWN:
                self.state.speed = max(0, self.state.speed - 10)
                # Update motor speeds
                for motor_id in ["left_motor", "right_motor"]:
                    await motor_system.motors[motor_id].set_speed(self.state.speed)
            elif cmd == RobotCommand.CAMERA_UP:
                self.state.camera_tilt = min(45, self.state.camera_tilt + 5)
                await motor_system.motors["camera_servo"].set_angle(self.state.camera_tilt)
            elif cmd == RobotCommand.CAMERA_DOWN:
                self.state.camera_tilt = max(-45, self.state.camera_tilt - 5)
                await motor_system.motors["camera_servo"].set_angle(self.state.camera_tilt)
            elif cmd == RobotCommand.CAMERA_LEFT:
                self.state.camera_pan = min(90, self.state.camera_pan + 5)
                # Would need second servo for pan
            elif cmd == RobotCommand.CAMERA_RIGHT:
                self.state.camera_pan = max(-90, self.state.camera_pan - 5)
                # Would need second servo for pan
                
            # Handle LED commands with real hardware
            if data:
                if "led_color" in data:
                    self.state.led_color = data["led_color"]
                    # Convert hex color to RGB
                    hex_color = data["led_color"].lstrip('#')
                    r = int(hex_color[0:2], 16)
                    g = int(hex_color[2:4], 16)
                    b = int(hex_color[4:6], 16)
                    led_color = LEDColor(r, g, b)
                    await led_manager.set_strip_color("main_leds", led_color)
                    
                if "led_brightness" in data:
                    self.state.led_brightness = data["led_brightness"]
                    await led_manager.set_strip_brightness("main_leds", self.state.led_brightness)
                    
                if "led_pattern" in data:
                    self.state.led_pattern = data["led_pattern"]
                    pattern_map = {
                        "static": LEDPattern.STATIC,
                        "pulse": LEDPattern.PULSE,
                        "blink": LEDPattern.BLINK,
                        "rainbow": LEDPattern.RAINBOW,
                        "chase": LEDPattern.CHASE,
                        "breathing": LEDPattern.BREATHING
                    }
                    if data["led_pattern"] in pattern_map:
                        await led_manager.set_strip_pattern("main_leds", pattern_map[data["led_pattern"]])
                    
            # Broadcast state update to all clients
            await self._broadcast_state_update()
            
        except ValueError:
            logger.error(f"Invalid command: {command}")
        except Exception as e:
            logger.error(f"Error executing command {command}: {e}")
            
    async def _execute_movement(self, direction: str):
        """Execute robot movement (simulated)"""
        logger.info(f"Robot moving {direction} at {self.state.speed}% speed")
        # In a real implementation, this would send commands to the robot hardware
        # For now, we'll just simulate the movement
        
    async def _update_sensors(self):
        """Update sensor readings periodically using real hardware"""
        while self.running:
            try:
                # Get real sensor readings
                sensor_readings = await sensor_manager.get_all_readings()
                
                # Update state with real sensor data
                for sensor_id, reading in sensor_readings.items():
                    if sensor_id == "ultrasonic":
                        self.state.sensors["ultrasonic"]["distance"] = reading.value
                    elif sensor_id == "temperature":
                        self.state.sensors["temperature"]["value"] = reading.value
                    elif sensor_id == "light":
                        self.state.sensors["light"]["value"] = reading.value
                
                # Broadcast sensor updates
                await self._broadcast_sensor_update()
                
            except Exception as e:
                logger.error(f"Error updating sensors: {e}")
                
            await asyncio.sleep(1)  # Update every second
            
    async def _simulate_robot_movement(self):
        """Simulate robot movement effects on sensors"""
        while self.running:
            try:
                if self.state.direction != "stop":
                    # Simulate movement affecting ultrasonic sensor
                    if self.state.direction == "forward":
                        self.state.sensors["ultrasonic"]["distance"] = max(5,
                            self.state.sensors["ultrasonic"]["distance"] - 0.5)
                    elif self.state.direction == "backward":
                        self.state.sensors["ultrasonic"]["distance"] = min(200,
                            self.state.sensors["ultrasonic"]["distance"] + 0.5)
                            
            except Exception as e:
                logger.error(f"Error simulating movement: {e}")
                
            await asyncio.sleep(0.1)  # Update every 100ms
            
    async def _send_state_update(self, websocket: WebSocketServerProtocol):
        """Send current state to a specific client"""
        try:
            message = {
                "type": "state_update",
                "data": {
                    "speed": self.state.speed,
                    "direction": self.state.direction,
                    "camera_tilt": self.state.camera_tilt,
                    "camera_pan": self.state.camera_pan,
                    "led_color": self.state.led_color,
                    "led_brightness": self.state.led_brightness,
                    "led_pattern": self.state.led_pattern,
                    "connected": self.state.connected,
                    "battery_level": self.state.battery_level
                }
            }
            await websocket.send(json.dumps(message))
        except Exception as e:
            logger.error(f"Error sending state update: {e}")
            
    async def _broadcast_state_update(self):
        """Broadcast state update to all clients"""
        if not self.clients:
            return
            
        message = {
            "type": "state_update",
            "data": {
                "speed": self.state.speed,
                "direction": self.state.direction,
                "camera_tilt": self.state.camera_tilt,
                "camera_pan": self.state.camera_pan,
                "led_color": self.state.led_color,
                "led_brightness": self.state.led_brightness,
                "led_pattern": self.state.led_pattern,
                "connected": self.state.connected,
                "battery_level": self.state.battery_level
            }
        }
        
        # Send to all connected clients
        disconnected = set()
        for client in self.clients:
            try:
                await client.send(json.dumps(message))
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(client)
            except Exception as e:
                logger.error(f"Error broadcasting to client: {e}")
                disconnected.add(client)
                
        # Remove disconnected clients
        self.clients -= disconnected
        
    async def _broadcast_sensor_update(self):
        """Broadcast sensor update to all clients"""
        if not self.clients:
            return
            
        message = {
            "type": "sensor_update",
            "data": {
                "sensors": self.state.sensors,
                "timestamp": asyncio.get_event_loop().time()
            }
        }
        
        # Send to all connected clients
        disconnected = set()
        for client in self.clients:
            try:
                await client.send(json.dumps(message))
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(client)
            except Exception as e:
                logger.error(f"Error broadcasting sensor update: {e}")
                disconnected.add(client)
                
        # Remove disconnected clients
        self.clients -= disconnected

# Global robot controller instance
robot_controller = RobotController()

async def handle_client(websocket: WebSocketServerProtocol, path: str):
    """Handle WebSocket client connections"""
    await robot_controller.add_client(websocket)
    
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                command = data.get("command")
                command_data = data.get("data", {})
                
                if command:
                    await robot_controller.handle_command(command, command_data)
                else:
                    logger.warning("Received message without command")
                    
            except json.JSONDecodeError:
                logger.error("Invalid JSON received")
            except Exception as e:
                logger.error(f"Error handling message: {e}")
                
    except websockets.exceptions.ConnectionClosed:
        logger.info("Client connection closed")
    except Exception as e:
        logger.error(f"Error in client handler: {e}")
    finally:
        await robot_controller.remove_client(websocket)

async def main():
    """Main server function"""
    logger.info("Starting robot controller WebSocket server...")
    
    # Start the robot controller
    await robot_controller.start()
    
    # Start WebSocket server
    server = await websockets.serve(
        handle_client,
        "localhost",
        8080,
        ping_interval=20,
        ping_timeout=10
    )
    
    logger.info("WebSocket server started on ws://localhost:8080")
    
    try:
        await server.wait_closed()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        await robot_controller.stop()
        server.close()
        await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())
