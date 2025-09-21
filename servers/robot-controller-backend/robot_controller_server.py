import asyncio
import json
import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum
import websockets
from websockets.server import WebSocketServerProtocol

# Import AI modules
from ai.navigation_system import ai_navigation, NavigationMode
from ai.computer_vision import cv_ml
from ai.predictive_analytics import predictive_analytics
from ai.autonomous_engine import autonomous_engine, DecisionContext

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
        """Initialize all hardware and AI components"""
        try:
            logger.info("Initializing hardware and AI components...")
            
            # Initialize AI systems first
            ai_nav_ok = await ai_navigation.initialize()
            cv_ml_ok = await cv_ml.initialize()
            analytics_ok = await predictive_analytics.initialize()
            engine_ok = await autonomous_engine.initialize()
            
            if not all([ai_nav_ok, cv_ml_ok, analytics_ok, engine_ok]):
                logger.warning("Some AI components failed to initialize")
            
            # Start AI systems
            await ai_navigation.start()
            await cv_ml.start_processing()
            await predictive_analytics.start_analysis()
            await autonomous_engine.start_decision_making()
            
            logger.info("AI systems initialized and started")
            
        except Exception as e:
            logger.error(f"AI initialization failed: {e}")
    
    async def _cleanup_hardware(self):
        """Cleanup all hardware and AI components"""
        try:
            logger.info("Cleaning up hardware and AI components...")
            
            # Stop AI systems
            await ai_navigation.stop()
            await cv_ml.stop_processing()
            await predictive_analytics.stop_analysis()
            await autonomous_engine.stop_decision_making()
            
            logger.info("AI systems stopped and cleaned up")
            
        except Exception as e:
            logger.error(f"AI cleanup failed: {e}")
        
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
        """Handle robot commands with AI-powered decision making"""
        try:
            cmd = RobotCommand(command)
            logger.info(f"Executing command: {cmd.value}")
            
            # Create decision context for AI
            context = DecisionContext(
                timestamp=time.time(),
                sensor_data=self.state.sensors,
                navigation_state={
                    'position_x': 0, 'position_y': 0, 'heading': 0,
                    'velocity': self.state.speed, 'obstacle_count': 0
                },
                system_state={
                    'battery_level': self.state.battery_level,
                    'cpu_usage': 50, 'memory_usage': 60, 'uptime': 0
                },
                predictions={},  # Would be populated by predictive analytics
                user_commands=[command],
                environment_context={}
            )
            
            # Get AI decisions
            ai_decisions = await autonomous_engine.make_decision(context)
            
            # Execute user command with AI oversight
            if cmd == RobotCommand.FORWARD:
                self.state.direction = "forward"
                # Check AI decisions for safety
                if not any(d.action.value == "emergency_stop" for d in ai_decisions):
                    await ai_navigation.set_mode(NavigationMode.AUTONOMOUS)
            elif cmd == RobotCommand.BACKWARD:
                self.state.direction = "backward"
            elif cmd == RobotCommand.LEFT:
                self.state.direction = "left"
            elif cmd == RobotCommand.RIGHT:
                self.state.direction = "right"
            elif cmd == RobotCommand.STOP:
                self.state.direction = "stop"
                await ai_navigation.set_mode(NavigationMode.MANUAL)
            elif cmd == RobotCommand.SPEED_UP:
                self.state.speed = min(100, self.state.speed + 10)
            elif cmd == RobotCommand.SPEED_DOWN:
                self.state.speed = max(0, self.state.speed - 10)
            elif cmd == RobotCommand.CAMERA_UP:
                self.state.camera_tilt = min(45, self.state.camera_tilt + 5)
            elif cmd == RobotCommand.CAMERA_DOWN:
                self.state.camera_tilt = max(-45, self.state.camera_tilt - 5)
            elif cmd == RobotCommand.CAMERA_LEFT:
                self.state.camera_pan = min(90, self.state.camera_pan + 5)
            elif cmd == RobotCommand.CAMERA_RIGHT:
                self.state.camera_pan = max(-90, self.state.camera_pan - 5)
                
            # Handle LED commands
            if data:
                if "led_color" in data:
                    self.state.led_color = data["led_color"]
                if "led_brightness" in data:
                    self.state.led_brightness = data["led_brightness"]
                if "led_pattern" in data:
                    self.state.led_pattern = data["led_pattern"]
                    
            # Execute AI decisions
            for decision in ai_decisions:
                await self._execute_ai_decision(decision)
                    
            # Broadcast state update to all clients
            await self._broadcast_state_update()
            
        except ValueError:
            logger.error(f"Invalid command: {command}")
        except Exception as e:
            logger.error(f"Error executing command {command}: {e}")
    
    async def _execute_ai_decision(self, decision):
        """Execute AI decision"""
        try:
            logger.info(f"Executing AI decision: {decision.action.value} - {decision.reasoning}")
            
            # Execute decision based on action type
            if decision.action.value == "emergency_stop":
                self.state.direction = "stop"
                await ai_navigation.set_mode(NavigationMode.EMERGENCY)
            elif decision.action.value == "change_speed":
                new_speed = decision.parameters.get('speed', self.state.speed)
                self.state.speed = max(0, min(100, new_speed))
            elif decision.action.value == "turn_left":
                self.state.direction = "left"
            elif decision.action.value == "turn_right":
                self.state.direction = "right"
            elif decision.action.value == "send_alert":
                logger.warning(f"AI Alert: {decision.reasoning}")
            
        except Exception as e:
            logger.error(f"Error executing AI decision: {e}")
            
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
