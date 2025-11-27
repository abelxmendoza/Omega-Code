"""
Pi Sensor Hub ROS2 Node

Publishes compressed frames, events, and telemetry from Raspberry Pi to Jetson Orin Nano.
This node runs on the Pi and acts as the sensor hub in the hybrid vision system.
"""

import os
import logging
import time
import threading
import json
import uuid
from typing import Optional, Dict, Any
import numpy as np

log = logging.getLogger(__name__)

# ROS2 imports (optional)
ros2_available = False
rclpy = None
Node = None
CompressedImage = None
String = None
Bool = None
Float32 = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String, Bool, Float32
    ros2_available = True
except ImportError:
    ros2_available = False
    log.debug("ROS2 not available (rclpy missing)")
    # Create a dummy Node class for when ROS2 is not available
    class Node:
        """Dummy Node class when ROS2 is not available."""
        pass


if ros2_available:
    class PiSensorHubNode(Node):
        """ROS2 node for Pi sensor hub - publishes to Orin."""
        
        def __init__(self):
            """Initialize Pi sensor hub node."""
            if not ros2_available:
                raise RuntimeError("ROS2 not available")
            
            super().__init__('pi_sensor_hub')
            
            # Publishers (Pi → Orin)
            self.compressed_frame_pub = self.create_publisher(
                CompressedImage,
                '/omega/camera/compressed',
                5  # QoS depth for video stream
            )
            
            self.aruco_pub = self.create_publisher(
                String,
                '/omega/events/aruco',
                10
            )
            
            self.tracking_pub = self.create_publisher(
                String,
                '/omega/events/tracking',
                10
            )
            
            self.motion_pub = self.create_publisher(
                Bool,
                '/omega/events/motion',
                10
            )
            
            self.telemetry_pub = self.create_publisher(
                String,
                '/omega/telemetry',
                10
            )
            
            # Subscribers (Orin → Pi)
            self.detection_sub = self.create_subscription(
                String,
                '/omega/brain/detections',
                self._on_detection_received,
                10
            )
            
            self.navigation_sub = self.create_subscription(
                String,
                '/omega/brain/navigation',
                self._on_navigation_received,
                10
            )
            
            self.command_sub = self.create_subscription(
                String,
                '/omega/brain/commands',
                self._on_command_received,
                10
            )
            
            # Callback storage for external handlers
            self.detection_callback = None
            self.navigation_callback = None
            self.command_callback = None
            
            # Latency measurement: UUID tracking for round-trip measurement
            self._frame_uuid_map: Dict[str, Dict[str, Any]] = {}  # UUID -> {send_time, frame_info}
            self._uuid_lock = threading.Lock()
            self._max_uuid_history = 1000
            
            log.info("✅ Pi Sensor Hub node initialized")
            log.info("   Publishing: /omega/camera/compressed, /omega/events/*, /omega/telemetry")
            log.info("   Subscribing: /omega/brain/detections, /omega/brain/navigation, /omega/brain/commands")
        
        def publish_compressed_frame(self, frame: np.ndarray, quality: int = 75):
            """
            Publish compressed frame to Orin.
            
            Args:
                frame: BGR frame (numpy array)
                quality: JPEG quality (0-100)
            """
            if frame is None or not ros2_available:
                return
            
            try:
                import cv2
                
                # Encode to JPEG
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
                _, jpeg_data = cv2.imencode('.jpg', frame, encode_param)
                
                # Create compressed image message
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                # TODO: Generate UUID and embed in frame_id for latency measurement
                frame_uuid = str(uuid.uuid4())
                send_time_ns = time.time_ns() if hasattr(time, 'time_ns') else int(time.time() * 1e9)
                msg.header.frame_id = f"camera_frame_{frame_uuid}"  # Embed UUID in frame_id
                msg.format = "jpeg"
                msg.data = jpeg_data.tobytes()
                
                # TODO: Store UUID and send time for latency calculation
                # Store UUID and send time for latency calculation
                with self._uuid_lock:
                    self._frame_uuid_map[frame_uuid] = {
                        "send_time_ns": send_time_ns,
                        "frame_size": len(jpeg_data),
                    }
                    # Cleanup old entries
                    if len(self._frame_uuid_map) > self._max_uuid_history:
                        oldest_uuid = min(self._frame_uuid_map.keys(), 
                                        key=lambda k: self._frame_uuid_map[k]["send_time_ns"])
                        del self._frame_uuid_map[oldest_uuid]
                
                self.compressed_frame_pub.publish(msg)
                
            except Exception as e:
                log.error(f"Failed to publish compressed frame: {e}", exc_info=True)
        
        def publish_aruco_markers(self, markers: list):
            """
            Publish ArUco marker detections.
            
            Args:
                markers: List of ArUcoMarker objects
            """
            if not markers or not ros2_available:
                return
            
            try:
                from .hybrid_messages import serialize_message, ArUcoMarker
                
                # Convert markers to JSON
                markers_data = [serialize_message(m) for m in markers]
                json_str = json.dumps(markers_data)
                
                msg = String()
                msg.data = json_str
                self.aruco_pub.publish(msg)
                
            except Exception as e:
                log.error(f"Failed to publish ArUco markers: {e}", exc_info=True)
        
        def publish_tracking_bbox(self, bbox: Any):
            """
            Publish tracking bounding box.
            
            Args:
                bbox: TrackingBBox object
            """
            if bbox is None or not ros2_available:
                return
            
            try:
                from .hybrid_messages import serialize_message
                
                json_str = serialize_message(bbox)
                
                msg = String()
                msg.data = json_str
                self.tracking_pub.publish(msg)
                
            except Exception as e:
                log.error(f"Failed to publish tracking bbox: {e}", exc_info=True)
        
        def publish_motion_event(self, detected: bool, regions: list = None):
            """
            Publish motion detection event.
            
            Args:
                detected: Whether motion was detected
                regions: List of motion regions (TrackingBBox objects)
            """
            if not ros2_available:
                return
            
            try:
                msg = Bool()
                msg.data = detected
                self.motion_pub.publish(msg)
                
                # Also publish detailed regions if available
                if regions:
                    from .hybrid_messages import serialize_message, MotionEvent
                    event = MotionEvent(detected=detected, regions=regions)
                    json_str = serialize_message(event)
                    
                    region_msg = String()
                    region_msg.data = json_str
                    # Could publish to separate topic if needed
                    
            except Exception as e:
                log.error(f"Failed to publish motion event: {e}", exc_info=True)
        
        def publish_telemetry(self, telemetry: Any):
            """
            Publish telemetry data.
            
            Args:
                telemetry: TelemetryData object
            """
            if telemetry is None or not ros2_available:
                return
            
            try:
                from .hybrid_messages import serialize_message
                
                json_str = serialize_message(telemetry)
                
                msg = String()
                msg.data = json_str
                self.telemetry_pub.publish(msg)
                
            except Exception as e:
                log.error(f"Failed to publish telemetry: {e}", exc_info=True)
        
        def _on_detection_received(self, msg: String):
            """Handle detection results from Orin."""
            try:
                from .hybrid_messages import deserialize_message, DetectionResult
                import json
                
                data = json.loads(msg.data)
                
                # Extract UUID and inference duration if present
                frame_uuid = data.get("frame_uuid")
                inference_duration_ms = data.get("inference_duration_ms")
                inference_start_ns = data.get("inference_start_ns")
                inference_end_ns = data.get("inference_end_ns")
                
                # Calculate round-trip latency if UUID matches
                if frame_uuid:
                    with self._uuid_lock:
                        if frame_uuid in self._frame_uuid_map:
                            send_time_ns = self._frame_uuid_map[frame_uuid]["send_time_ns"]
                            receive_time_ns = time.time_ns() if hasattr(time, 'time_ns') else int(time.time() * 1e9)
                            
                            round_trip_ms = (receive_time_ns - send_time_ns) / 1e6
                            
                            log.debug(f"Round-trip latency for UUID {frame_uuid[:8]}: {round_trip_ms:.2f}ms")
                            if inference_duration_ms:
                                log.debug(f"Inference duration: {inference_duration_ms:.2f}ms")
                            
                            # Store latency metrics
                            self._frame_uuid_map[frame_uuid]["round_trip_ms"] = round_trip_ms
                            self._frame_uuid_map[frame_uuid]["inference_duration_ms"] = inference_duration_ms
                            self._frame_uuid_map[frame_uuid]["receive_time_ns"] = receive_time_ns
                
                detections = data.get("detections", [])
                if isinstance(detections, list) and detections:
                    results = [deserialize_message(json.dumps(d), DetectionResult) for d in detections]
                else:
                    results = []
                
                if self.detection_callback:
                    self.detection_callback(results)
                else:
                    log.debug(f"Received {len(results)} detections from Orin")
                    
            except Exception as e:
                log.error(f"Failed to process detection message: {e}", exc_info=True)
        
        def get_latency_stats(self) -> Dict[str, Any]:
            """
            Get latency statistics for recent frames.
            
            Returns:
                Dict with latency statistics
            """
            with self._uuid_lock:
                if not self._frame_uuid_map:
                    return {
                        "ok": False,
                        "message": "No latency data available"
                    }
                
                # Calculate statistics from recent entries with round-trip data
                latencies = [
                    entry["round_trip_ms"] 
                    for entry in self._frame_uuid_map.values() 
                    if "round_trip_ms" in entry
                ]
                
                inference_times = [
                    entry["inference_duration_ms"]
                    for entry in self._frame_uuid_map.values()
                    if "inference_duration_ms" in entry
                ]
                
                if not latencies:
                    return {
                        "ok": False,
                        "message": "No round-trip latency data available yet"
                    }
                
                return {
                    "ok": True,
                    "round_trip_ms": {
                        "min": round(min(latencies), 2),
                        "max": round(max(latencies), 2),
                        "avg": round(sum(latencies) / len(latencies), 2),
                        "count": len(latencies),
                    },
                    "inference_ms": {
                        "min": round(min(inference_times), 2) if inference_times else None,
                        "max": round(max(inference_times), 2) if inference_times else None,
                        "avg": round(sum(inference_times) / len(inference_times), 2) if inference_times else None,
                        "count": len(inference_times),
                    } if inference_times else None,
                }
        
        def _on_navigation_received(self, msg: String):
            """Handle navigation commands from Orin."""
            try:
                from .hybrid_messages import deserialize_message, NavigationCommand
                
                nav_cmd = deserialize_message(msg.data, NavigationCommand)
                
                if self.navigation_callback:
                    self.navigation_callback(nav_cmd)
                else:
                    log.debug(f"Received navigation command: {nav_cmd.command_type}")
                    
            except Exception as e:
                log.error(f"Failed to process navigation message: {e}", exc_info=True)
        
        def _on_command_received(self, msg: String):
            """Handle control commands from Orin."""
            try:
                import json
                cmd = json.loads(msg.data)
                
                if self.command_callback:
                    self.command_callback(cmd)
                else:
                    log.debug(f"Received command: {cmd}")
                    
            except Exception as e:
                log.error(f"Failed to process command message: {e}", exc_info=True)
        
        def set_detection_callback(self, callback):
            """Set callback for detection results."""
            self.detection_callback = callback
        
        def set_navigation_callback(self, callback):
            """Set callback for navigation commands."""
            self.navigation_callback = callback
        
        def set_command_callback(self, callback):
            """Set callback for control commands."""
            self.command_callback = callback


# Global node instance
_pi_sensor_hub_node: Optional[Any] = None  # Use Any to avoid NameError when ros2_available is False
_executor_thread: Optional[threading.Thread] = None


def init_pi_sensor_hub() -> Optional[Any]:
    """Initialize Pi sensor hub node."""
    global _pi_sensor_hub_node, _executor_thread
    
    if not ros2_available:
        log.warning("ROS2 not available, Pi sensor hub disabled")
        return None
    
    if _pi_sensor_hub_node is not None:
        return _pi_sensor_hub_node
    
    try:
        if not rclpy.ok():
            rclpy.init()
        
        # PiSensorHubNode is only defined when ros2_available is True
        if ros2_available:
            _pi_sensor_hub_node = PiSensorHubNode()
            
            # Start executor in background thread
            def spin_ros2():
                executor = rclpy.executors.SingleThreadedExecutor()
                executor.add_node(_pi_sensor_hub_node)
                try:
                    executor.spin()
                except Exception as e:
                    log.error(f"ROS2 executor error: {e}", exc_info=True)
            
            _executor_thread = threading.Thread(target=spin_ros2, daemon=True)
            _executor_thread.start()
            
            log.info("✅ Pi Sensor Hub node started")
            return _pi_sensor_hub_node
    
    except Exception as e:
        log.error(f"Failed to initialize Pi sensor hub: {e}", exc_info=True)
        return None
    
    return None


def get_pi_sensor_hub() -> Optional[Any]:
    """Get Pi sensor hub node instance."""
    return _pi_sensor_hub_node


def shutdown_pi_sensor_hub():
    """Shutdown Pi sensor hub node."""
    global _pi_sensor_hub_node, _executor_thread
    
    if _pi_sensor_hub_node is not None:
        try:
            _pi_sensor_hub_node.destroy_node()
        except Exception:
            pass
        _pi_sensor_hub_node = None
    
    if rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            pass

