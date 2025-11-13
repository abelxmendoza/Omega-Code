# api/ros_web_bridge.py
"""
ROS2-Web Bridge Service

Bridges ROS2 topics to WebSocket for real-time web app integration.
Subscribes to ROS2 topics and streams to web clients via WebSocket.
Also accepts commands from web app and publishes to ROS2.
"""

import asyncio
import json
import logging
from typing import Dict, List, Set, Optional, Callable
from fastapi import WebSocket, WebSocketDisconnect
from datetime import datetime

log = logging.getLogger(__name__)

# Get or create event loop for scheduling callbacks
try:
    _event_loop = asyncio.get_event_loop()
except RuntimeError:
    _event_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(_event_loop)

# Try to import ROS2 bridge
try:
    from .ros_native_bridge import get_bridge, init_ros2_bridge, _rclpy_available
    ROS2_AVAILABLE = _rclpy_available
except ImportError:
    ROS2_AVAILABLE = False
    get_bridge = None
    init_ros2_bridge = None


class ROS2WebBridge:
    """
    Bridges ROS2 topics to WebSocket connections.
    Manages multiple WebSocket clients and topic subscriptions.
    """
    
    def __init__(self):
        self.active_connections: Set[WebSocket] = set()
        self.topic_subscriptions: Dict[str, List[WebSocket]] = {}
        self.bridge = None
        
        if ROS2_AVAILABLE:
            if init_ros2_bridge():
                self.bridge = get_bridge()
                log.info("ROS2-Web bridge initialized")
            else:
                log.warning("ROS2 bridge initialization failed")
        else:
            log.warning("ROS2 not available - bridge disabled")
    
    async def connect(self, websocket: WebSocket):
        """Accept new WebSocket connection."""
        await websocket.accept()
        self.active_connections.add(websocket)
        log.info(f"WebSocket connected. Total connections: {len(self.active_connections)}")
    
    def disconnect(self, websocket: WebSocket):
        """Remove WebSocket connection."""
        self.active_connections.discard(websocket)
        
        # Remove from all topic subscriptions
        for topic, subscribers in self.topic_subscriptions.items():
            if websocket in subscribers:
                subscribers.remove(websocket)
        
        log.info(f"WebSocket disconnected. Total connections: {len(self.active_connections)}")
    
    async def subscribe_topic(self, websocket: WebSocket, topic: str, msg_type: str = "String"):
        """Subscribe WebSocket client to a ROS2 topic."""
        if not self.bridge:
            await websocket.send_json({
                "error": "ROS2 not available",
                "topic": topic
            })
            return
        
        # Add to subscription list
        if topic not in self.topic_subscriptions:
            self.topic_subscriptions[topic] = []
            # Subscribe bridge to topic
            try:
                # Import message type dynamically
                if msg_type == "String":
                    from std_msgs.msg import String
                    self.bridge.subscribe_topic(topic, String, self._create_topic_callback(topic))
                elif msg_type == "Float32":
                    from std_msgs.msg import Float32
                    self.bridge.subscribe_topic(topic, Float32, self._create_topic_callback(topic))
                elif msg_type == "Twist":
                    from geometry_msgs.msg import Twist
                    self.bridge.subscribe_topic(topic, Twist, self._create_topic_callback(topic))
                elif msg_type == "BatteryState":
                    from sensor_msgs.msg import BatteryState
                    self.bridge.subscribe_topic(topic, BatteryState, self._create_topic_callback(topic))
                elif msg_type == "Int32MultiArray":
                    from std_msgs.msg import Int32MultiArray
                    self.bridge.subscribe_topic(topic, Int32MultiArray, self._create_topic_callback(topic))
                elif msg_type == "Image":
                    from sensor_msgs.msg import Image
                    self.bridge.subscribe_topic(topic, Image, self._create_topic_callback(topic))
                elif msg_type == "CompressedImage":
                    from sensor_msgs.msg import CompressedImage
                    self.bridge.subscribe_topic(topic, CompressedImage, self._create_topic_callback(topic))
                
                log.info(f"Subscribed to ROS2 topic: {topic}")
            except Exception as e:
                log.error(f"Error subscribing to topic {topic}: {e}")
                await websocket.send_json({
                    "error": f"Failed to subscribe: {str(e)}",
                    "topic": topic
                })
                return
        
        if websocket not in self.topic_subscriptions[topic]:
            self.topic_subscriptions[topic].append(websocket)
        
        await websocket.send_json({
            "type": "subscription_confirmed",
            "topic": topic,
            "status": "subscribed"
        })
    
    def _create_topic_callback(self, topic: str):
        """Create a callback function for a topic."""
        def topic_callback(msg):
            # Convert ROS2 message to dict
            msg_dict = self._ros_msg_to_dict(msg)
            
            # Schedule broadcast in event loop
            # Note: ROS2 callbacks run in ROS2 thread, need to schedule in asyncio
            try:
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    asyncio.create_task(self._broadcast_to_topic(topic, {
                        "type": "ros2_message",
                        "topic": topic,
                        "data": msg_dict,
                        "timestamp": datetime.now().isoformat()
                    }))
                else:
                    loop.run_until_complete(self._broadcast_to_topic(topic, {
                        "type": "ros2_message",
                        "topic": topic,
                        "data": msg_dict,
                        "timestamp": datetime.now().isoformat()
                    }))
            except Exception as e:
                log.error(f"Error in topic callback for {topic}: {e}")
        
        return topic_callback
    
    def _ros_msg_to_dict(self, msg) -> dict:
        """Convert ROS2 message to dictionary."""
        try:
            # Handle different message types
            if hasattr(msg, 'data'):
                # Check if it's CompressedImage (has format attribute)
                if hasattr(msg, 'format') and hasattr(msg, 'header'):
                    # CompressedImage - return base64 encoded JPEG
                    import base64
                    return {
                        "format": msg.format,
                        "data": base64.b64encode(msg.data).decode('utf-8'),
                        "header": {
                            "stamp": {
                                "sec": msg.header.stamp.sec,
                                "nanosec": msg.header.stamp.nanosec
                            },
                            "frame_id": msg.header.frame_id
                        }
                    }
                # Regular data field
                return {"data": msg.data}
            elif hasattr(msg, 'linear') and hasattr(msg, 'angular'):
                return {
                    "linear": {
                        "x": msg.linear.x,
                        "y": msg.linear.y,
                        "z": msg.linear.z
                    },
                    "angular": {
                        "x": msg.angular.x,
                        "y": msg.angular.y,
                        "z": msg.angular.z
                    }
                }
            elif hasattr(msg, 'voltage'):
                return {
                    "voltage": msg.voltage,
                    "percentage": msg.percentage,
                    "status": msg.power_supply_status
                }
            elif hasattr(msg, 'data') and isinstance(msg.data, list):
                return {"data": list(msg.data)}
            elif hasattr(msg, 'encoding') and hasattr(msg, 'data'):
                # Image message - convert to base64
                import base64
                return {
                    "encoding": msg.encoding,
                    "width": msg.width,
                    "height": msg.height,
                    "data": base64.b64encode(bytes(msg.data)).decode('utf-8'),
                    "header": {
                        "stamp": {
                            "sec": msg.header.stamp.sec,
                            "nanosec": msg.header.stamp.nanosec
                        },
                        "frame_id": msg.header.frame_id
                    }
                }
            else:
                return {"raw": str(msg)}
        except Exception as e:
            log.error(f"Error converting ROS2 message: {e}")
            return {"error": str(e)}
    
    async def _broadcast_to_topic(self, topic: str, message: dict):
        """Broadcast message to all WebSocket clients subscribed to topic."""
        if topic not in self.topic_subscriptions:
            return
        
        disconnected = []
        for websocket in self.topic_subscriptions[topic]:
            try:
                await websocket.send_json(message)
            except Exception as e:
                log.warning(f"Error sending to WebSocket: {e}")
                disconnected.append(websocket)
        
        # Remove disconnected clients
        for ws in disconnected:
            self.disconnect(ws)
    
    async def publish_command(self, topic: str, command: dict, msg_type: str = "String"):
        """Publish command from web app to ROS2 topic."""
        if not self.bridge:
            log.warning("ROS2 not available - cannot publish command")
            return False
        
        try:
            if msg_type == "String":
                data = command.get("data", str(command))
                self.bridge.publish_string(topic, data)
            elif msg_type == "Twist":
                linear_x = command.get("linear", {}).get("x", 0.0)
                angular_z = command.get("angular", {}).get("z", 0.0)
                self.bridge.publish_twist(topic, linear_x, angular_z)
            else:
                log.warning(f"Unsupported message type: {msg_type}")
                return False
            
            log.info(f"Published command to {topic}: {command}")
            return True
        except Exception as e:
            log.error(f"Error publishing command: {e}")
            return False
    
    async def handle_websocket_message(self, websocket: WebSocket, message: dict):
        """Handle incoming WebSocket message."""
        msg_type = message.get("type")
        
        if msg_type == "subscribe":
            topic = message.get("topic")
            msg_type_str = message.get("msg_type", "String")
            await self.subscribe_topic(websocket, topic, msg_type_str)
        
        elif msg_type == "publish":
            topic = message.get("topic")
            command = message.get("command", {})
            msg_type_str = message.get("msg_type", "String")
            success = await self.publish_command(topic, command, msg_type_str)
            
            await websocket.send_json({
                "type": "publish_result",
                "topic": topic,
                "success": success
            })
        
        elif msg_type == "unsubscribe":
            topic = message.get("topic")
            if topic in self.topic_subscriptions:
                if websocket in self.topic_subscriptions[topic]:
                    self.topic_subscriptions[topic].remove(websocket)
            
            await websocket.send_json({
                "type": "unsubscription_confirmed",
                "topic": topic
            })
        
        elif msg_type == "list_topics":
            # Return list of available topics
            topics = []
            if self.bridge:
                # Get topics from bridge (implement if needed)
                topics = list(self.topic_subscriptions.keys())
            
            await websocket.send_json({
                "type": "topics_list",
                "topics": topics
            })


# Global bridge instance
_ros2_web_bridge: Optional[ROS2WebBridge] = None


def get_ros2_web_bridge() -> Optional[ROS2WebBridge]:
    """Get global ROS2-Web bridge instance."""
    global _ros2_web_bridge
    if _ros2_web_bridge is None:
        _ros2_web_bridge = ROS2WebBridge()
    return _ros2_web_bridge

