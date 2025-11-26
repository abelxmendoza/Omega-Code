# api/ros_native_bridge.py
"""
Native ROS2 Bridge Service

Provides native ROS2 integration (not Docker-based) for the robot controller.
Works from laptop or Pi, connects to ROS2 topics directly.

Usage:
    Set ROS_NATIVE_MODE=true in environment
    Ensure ROS2 is sourced: source /opt/ros/rolling/setup.bash
"""

import os
import sys
import threading
import logging
from typing import Optional, Dict, List, Callable
from queue import Queue, Empty

log = logging.getLogger(__name__)

# Try to import ROS2 Python bindings
_rclpy_available = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String, Float32, Int32, Int32MultiArray
    from sensor_msgs.msg import Image, CompressedImage, BatteryState
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry, Path
    _rclpy_available = True
except ImportError as e:
    log.warning(f"rclpy not available: {e}. Native ROS2 features disabled.")
    log.warning("Install with: sudo apt install ros-rolling-rclpy")
    rclpy = None
    Node = None
    String = None
    Float32 = None
    Int32 = None
    Int32MultiArray = None
    Image = None
    CompressedImage = None
    BatteryState = None
    Twist = None
    PoseStamped = None
    Odometry = None
    Path = None
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None


# Check if ROS is enabled via environment variable
_ros_enabled = os.getenv("ROS_ENABLED", "true").lower() in ("true", "1", "yes")
_ros_native_mode = os.getenv("ROS_NATIVE_MODE", "false").lower() == "true"


if _rclpy_available and _ros_enabled:
    # ROS is available and enabled - use real implementation
    class ROS2NativeBridge(Node):
    """
    Native ROS2 bridge node that connects to ROS2 topics
    and provides a bridge to the FastAPI backend.
    """
    
    def __init__(self, node_name: str = "omega_robot_bridge"):
        if not _rclpy_available:
            raise RuntimeError("rclpy not available. Cannot create ROS2 bridge.")
        
        super().__init__(node_name)
        
        # QoS profile for reliable communication
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers
        self._subscribers: Dict[str, any] = {}
        
        # Publishers
        self._publishers: Dict[str, any] = {}
        
        # Message queues for each topic
        self._message_queues: Dict[str, Queue] = {}
        
        # Callbacks
        self._callbacks: Dict[str, List[Callable]] = {}
        
        log.info(f"ROS2 Native Bridge node '{node_name}' initialized")
    
    def subscribe_topic(self, topic_name: str, msg_type, callback: Optional[Callable] = None):
        """Subscribe to a ROS2 topic"""
        if topic_name in self._subscribers:
            log.warning(f"Already subscribed to {topic_name}")
            return
        
        if not _rclpy_available:
            raise RuntimeError("rclpy not available")
        
        # Create message queue
        self._message_queues[topic_name] = Queue()
        
        # Create callback
        def topic_callback(msg):
            # Store in queue
            self._message_queues[topic_name].put(msg)
            
            # Call user callback if provided
            if callback:
                try:
                    callback(msg)
                except Exception as e:
                    log.error(f"Error in callback for {topic_name}: {e}")
            
            # Call registered callbacks
            for cb in self._callbacks.get(topic_name, []):
                try:
                    cb(msg)
                except Exception as e:
                    log.error(f"Error in registered callback for {topic_name}: {e}")
        
        # Create subscriber
        self._subscribers[topic_name] = self.create_subscription(
            msg_type,
            topic_name,
            topic_callback,
            self.qos_profile
        )
        
        log.info(f"Subscribed to ROS2 topic: {topic_name}")
    
    def publish_topic(self, topic_name: str, msg_type):
        """Create a publisher for a ROS2 topic"""
        if topic_name in self._publishers:
            log.warning(f"Already publishing to {topic_name}")
            return self._publishers[topic_name]
        
        if not _rclpy_available:
            raise RuntimeError("rclpy not available")
        
        publisher = self.create_publisher(
            msg_type,
            topic_name,
            self.qos_profile
        )
        
        self._publishers[topic_name] = publisher
        log.info(f"Publishing to ROS2 topic: {topic_name}")
        
        return publisher
    
    def get_latest_message(self, topic_name: str, timeout: float = 0.1):
        """Get the latest message from a topic (non-blocking)"""
        if topic_name not in self._message_queues:
            return None
        
        queue = self._message_queues[topic_name]
        
        # Get latest message (discard old ones)
        latest = None
        try:
            while True:
                latest = queue.get_nowait()
        except Empty:
            pass
        
        return latest
    
    def register_callback(self, topic_name: str, callback: Callable):
        """Register a callback for a topic"""
        if topic_name not in self._callbacks:
            self._callbacks[topic_name] = []
        self._callbacks[topic_name].append(callback)
    
    def publish_string(self, topic_name: str, data: str):
        """Publish a string message"""
        if topic_name not in self._publishers:
            self.publish_topic(topic_name, String)
        
        msg = String()
        msg.data = data
        self._publishers[topic_name].publish(msg)
    
    def publish_twist(self, topic_name: str, linear_x: float = 0.0, angular_z: float = 0.0):
        """Publish a Twist message (for robot movement)"""
        if topic_name not in self._publishers:
            self.publish_topic(topic_name, Twist)
        
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._publishers[topic_name].publish(msg)

else:
    # ROS is not available or disabled - use mock implementation
    class ROS2NativeBridge:
        """
        Mock ROS2 bridge for when ROS is not available or disabled.
        All methods are no-ops that log warnings.
        """
        
        def __init__(self, node_name: str = "omega_robot_bridge"):
            log.info(f"ROS2NativeBridge initialized in mock mode (ROS not available or disabled)")
            self._subscribers: Dict[str, any] = {}
            self._publishers: Dict[str, any] = {}
            self._message_queues: Dict[str, Queue] = {}
            self._callbacks: Dict[str, List[Callable]] = {}
        
        def subscribe_topic(self, topic_name: str, msg_type=None, callback: Optional[Callable] = None):
            """Mock subscribe - no-op"""
            log.debug(f"Mock: subscribe_topic({topic_name}) - ROS not available")
        
        def publish_topic(self, topic_name: str, msg_type=None):
            """Mock publish_topic - no-op"""
            log.debug(f"Mock: publish_topic({topic_name}) - ROS not available")
            return None
        
        def get_latest_message(self, topic_name: str, timeout: float = 0.1):
            """Mock get_latest_message - returns None"""
            return None
        
        def register_callback(self, topic_name: str, callback: Callable):
            """Mock register_callback - no-op"""
            pass
        
        def publish_string(self, topic_name: str, data: str):
            """Mock publish_string - no-op"""
            log.debug(f"Mock: publish_string({topic_name}) - ROS not available")
        
        def publish_twist(self, topic_name: str, linear_x: float = 0.0, angular_z: float = 0.0):
            """Mock publish_twist - no-op"""
            log.debug(f"Mock: publish_twist({topic_name}) - ROS not available")
        
        def destroy_node(self):
            """Mock destroy_node - no-op"""
            pass


# Global bridge instance
_bridge_instance: Optional[ROS2NativeBridge] = None
_bridge_thread: Optional[threading.Thread] = None
_bridge_running = False


def init_ros2_bridge() -> bool:
    """Initialize the ROS2 native bridge"""
    global _bridge_instance, _bridge_thread, _bridge_running
    
    if not _ros_enabled:
        log.info("ROS is disabled via ROS_ENABLED environment variable")
        return False
    
    if not _rclpy_available:
        log.warning("Cannot initialize ROS2 bridge: rclpy not available")
        return False
    
    if _bridge_running:
        log.warning("ROS2 bridge already running")
        return True
    
    try:
        # Initialize rclpy if not already done
        if not rclpy.ok():
            rclpy.init()
        
        # Create bridge node
        _bridge_instance = ROS2NativeBridge()
        
        # Start ROS2 executor in background thread
        def run_executor():
            global _bridge_running
            _bridge_running = True
            try:
                while rclpy.ok() and _bridge_running:
                    rclpy.spin_once(_bridge_instance, timeout_sec=0.1)
            except Exception as e:
                log.error(f"Error in ROS2 executor: {e}")
            finally:
                _bridge_running = False
        
        _bridge_thread = threading.Thread(target=run_executor, daemon=True)
        _bridge_thread.start()
        
        log.info("ROS2 Native Bridge initialized and running")
        return True
        
    except Exception as e:
        log.error(f"Failed to initialize ROS2 bridge: {e}")
        return False


def get_bridge() -> Optional[ROS2NativeBridge]:
    """Get the global ROS2 bridge instance"""
    return _bridge_instance


def shutdown_ros2_bridge():
    """Shutdown the ROS2 bridge"""
    global _bridge_instance, _bridge_running
    
    _bridge_running = False
    
    if _bridge_instance:
        try:
            _bridge_instance.destroy_node()
        except:
            pass
    
    if rclpy and rclpy.ok():
        try:
            rclpy.shutdown()
        except:
            pass
    
    log.info("ROS2 Native Bridge shut down")


# Auto-initialize if ROS_NATIVE_MODE is enabled and ROS is enabled
if _ros_native_mode and _ros_enabled:
    if _rclpy_available:
        init_ros2_bridge()
    else:
        log.warning("ROS_NATIVE_MODE=true but rclpy not available")
elif not _ros_enabled:
    log.info("ROS features are disabled. Set ROS_ENABLED=true to enable.")

