#!/usr/bin/env python3
"""
Orin AI Brain ROS2 Node

Subscribes to compressed frames and events from Pi, processes with deep learning,
and publishes detections, tracking, and navigation commands back to Pi.

This node runs on the Jetson Orin Nano and acts as the AI brain in the hybrid vision system.
"""

import os
import logging
import json
import time
from typing import List, Optional, Dict, Any
import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge

log = logging.getLogger(__name__)


class OrinAIBrainNode(Node):
    """ROS2 node for Orin AI brain - processes Pi data and sends commands back."""
    
    def __init__(self):
        """Initialize Orin AI brain node."""
        super().__init__('orin_ai_brain')
        
        self.bridge = CvBridge()
        
        # Subscribers (Pi → Orin)
        self.compressed_frame_sub = self.create_subscription(
            CompressedImage,
            '/omega/camera/compressed',
            self._on_frame_received,
            5  # QoS depth for video stream
        )
        
        self.aruco_sub = self.create_subscription(
            String,
            '/omega/events/aruco',
            self._on_aruco_received,
            10
        )
        
        self.tracking_sub = self.create_subscription(
            String,
            '/omega/events/tracking',
            self._on_tracking_received,
            10
        )
        
        self.motion_sub = self.create_subscription(
            Bool,
            '/omega/events/motion',
            self._on_motion_received,
            10
        )
        
        self.telemetry_sub = self.create_subscription(
            String,
            '/omega/telemetry',
            self._on_telemetry_received,
            10
        )
        
        # Publishers (Orin → Pi)
        self.detection_pub = self.create_publisher(
            String,
            '/omega/brain/detections',
            10
        )
        
        self.tracking_pub = self.create_publisher(
            String,
            '/omega/brain/tracking',
            10
        )
        
        self.navigation_pub = self.create_publisher(
            String,
            '/omega/brain/navigation',
            10
        )
        
        self.command_pub = self.create_publisher(
            String,
            '/omega/brain/commands',
            10
        )
        
        # AI/ML models (placeholder - to be implemented)
        self.yolo_model = None
        self.face_recognition_model = None
        self.byte_track_tracker = None
        
        # State
        self.last_frame = None
        self.frame_count = 0
        self.detection_enabled = True
        self.tracking_enabled = True
        
        # Initialize models
        self._initialize_models()
        
        log.info("✅ Orin AI Brain node initialized")
        log.info("   Subscribing: /omega/camera/compressed, /omega/events/*, /omega/telemetry")
        log.info("   Publishing: /omega/brain/detections, /omega/brain/tracking, /omega/brain/navigation, /omega/brain/commands")
    
    def _initialize_models(self):
        """Initialize AI/ML models (YOLOv8, DeepFace, ByteTrack)."""
        try:
            # Check for TensorRT availability
            tensorrt_available = self._check_tensorrt()
            
            if tensorrt_available:
                log.info("TensorRT detected - will use optimized models")
            else:
                log.warning("TensorRT not available - using CPU/GPU models")
            
            # TODO: Initialize YOLOv8 model
            # self.yolo_model = self._load_yolo_model()
            
            # TODO: Initialize DeepFace model
            # self.face_recognition_model = self._load_face_model()
            
            # TODO: Initialize ByteTrack tracker
            # self.byte_track_tracker = self._init_byte_track()
            
            log.info("AI models initialized (placeholder)")
            
        except Exception as e:
            log.error(f"Failed to initialize AI models: {e}", exc_info=True)
            self.detection_enabled = False
            self.tracking_enabled = False
    
    def _check_tensorrt(self) -> bool:
        """Check if TensorRT is available."""
        try:
            import tensorrt
            return True
        except ImportError:
            return False
    
    def _on_frame_received(self, msg: CompressedImage):
        """Process compressed frame from Pi."""
        try:
            if cv2 is None:
                log.warning("OpenCV not available, skipping frame processing")
                return
            
            # TODO: Extract UUID from frame_id and record inference timestamps for latency measurement
            # Extract UUID from frame_id for latency measurement
            frame_uuid = None
            if "_" in msg.header.frame_id:
                parts = msg.header.frame_id.split("_")
                if len(parts) >= 3 and parts[-1]:  # camera_frame_{uuid}
                    frame_uuid = parts[-1]
            
            # Record inference start time
            import time
            inference_start_ns = time.time_ns() if hasattr(time, 'time_ns') else int(time.time() * 1e9)
            
            # Decode JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                return
            
            self.last_frame = frame
            self.frame_count += 1
            
            # Process with AI models
            detections = []
            if self.detection_enabled:
                detections = self._process_detections(frame)
                if detections:
                    self._publish_detections(detections, frame_uuid, inference_start_ns)
            
            if self.tracking_enabled:
                tracking_results = self._process_tracking(frame)
                if tracking_results:
                    self._publish_tracking(tracking_results)
            
        except Exception as e:
            log.error(f"Failed to process frame: {e}", exc_info=True)
    
    def _process_detections(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        Process frame with YOLOv8 and return detections.
        
        Args:
            frame: BGR frame
            
        Returns:
            List of detection dictionaries
        """
        # Placeholder - implement YOLOv8 inference
        # if self.yolo_model:
        #     results = self.yolo_model(frame)
        #     return self._format_detections(results)
        
        return []
    
    def _process_tracking(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """
        Process frame with ByteTrack multi-object tracking.
        
        Args:
            frame: BGR frame
            
        Returns:
            List of tracking results
        """
        # Placeholder - implement ByteTrack
        # if self.byte_track_tracker:
        #     tracks = self.byte_track_tracker.update(frame, detections)
        #     return self._format_tracking(tracks)
        
        return []
    
    def _publish_detections(self, detections: List[Dict[str, Any]], frame_uuid: Optional[str] = None, inference_start_ns: Optional[int] = None):
        """Publish detection results to Pi with latency information."""
        try:
            import time
            
            # Record inference end time
            inference_end_ns = time.time_ns() if hasattr(time, 'time_ns') else int(time.time() * 1e9)
            inference_duration_ms = None
            
            if inference_start_ns and inference_end_ns:
                inference_duration_ms = (inference_end_ns - inference_start_ns) / 1e6  # Convert to ms
            
            # TODO: Include UUID and inference duration in response for round-trip latency calculation
            # Include UUID and inference duration in response
            response_data = {
                "detections": detections,
            }
            
            if frame_uuid:
                response_data["frame_uuid"] = frame_uuid
            
            if inference_duration_ms is not None:
                response_data["inference_duration_ms"] = round(inference_duration_ms, 2)
                response_data["inference_start_ns"] = inference_start_ns
                response_data["inference_end_ns"] = inference_end_ns
            
            msg = String()
            msg.data = json.dumps(response_data)
            self.detection_pub.publish(msg)
        except Exception as e:
            log.error(f"Failed to publish detections: {e}", exc_info=True)
    
    def _publish_tracking(self, tracking_results: List[Dict[str, Any]]):
        """Publish tracking results to Pi."""
        try:
            msg = String()
            msg.data = json.dumps(tracking_results)
            self.tracking_pub.publish(msg)
        except Exception as e:
            log.error(f"Failed to publish tracking: {e}", exc_info=True)
    
    def _on_aruco_received(self, msg: String):
        """Handle ArUco markers from Pi."""
        try:
            markers = json.loads(msg.data)
            log.debug(f"Received {len(markers)} ArUco markers from Pi")
            # Could use markers for navigation/pose estimation
        except Exception as e:
            log.error(f"Failed to process ArUco markers: {e}", exc_info=True)
    
    def _on_tracking_received(self, msg: String):
        """Handle tracking bbox from Pi."""
        try:
            bbox = json.loads(msg.data)
            log.debug(f"Received tracking bbox from Pi: {bbox}")
            # Could enhance with deep learning tracking
        except Exception as e:
            log.error(f"Failed to process tracking bbox: {e}", exc_info=True)
    
    def _on_motion_received(self, msg: Bool):
        """Handle motion event from Pi."""
        try:
            motion_detected = msg.data
            if motion_detected:
                log.debug("Motion detected on Pi - triggering enhanced detection")
                # Could trigger high-resolution detection on Orin
        except Exception as e:
            log.error(f"Failed to process motion event: {e}", exc_info=True)
    
    def _on_telemetry_received(self, msg: String):
        """Handle telemetry from Pi."""
        try:
            telemetry = json.loads(msg.data)
            log.debug(f"Received telemetry from Pi: {telemetry}")
            # Could use telemetry for adaptive processing
        except Exception as e:
            log.error(f"Failed to process telemetry: {e}", exc_info=True)
    
    def publish_navigation_command(self, command_type: str, **kwargs):
        """
        Publish navigation command to Pi.
        
        Args:
            command_type: "stop", "go", "target_bbox", "path_planning"
            **kwargs: Additional command parameters
        """
        try:
            cmd = {
                "command_type": command_type,
                **kwargs
            }
            msg = String()
            msg.data = json.dumps(cmd)
            self.navigation_pub.publish(msg)
        except Exception as e:
            log.error(f"Failed to publish navigation command: {e}", exc_info=True)
    
    def publish_control_command(self, command: Dict[str, Any]):
        """Publish control command to Pi."""
        try:
            msg = String()
            msg.data = json.dumps(command)
            self.command_pub.publish(msg)
        except Exception as e:
            log.error(f"Failed to publish control command: {e}", exc_info=True)


def main(args=None):
    """Main entry point for Orin AI Brain node."""
    rclpy.init(args=args)
    
    node = OrinAIBrainNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

