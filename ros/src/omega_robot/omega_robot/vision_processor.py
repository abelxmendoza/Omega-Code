#!/usr/bin/env python3
"""
Vision Processor Node for Jetson Orin Nano

GPU-accelerated vision processing node for object detection, image analysis,
and computer vision tasks. Designed to run on Jetson Orin Nano with CUDA support.

Topics:
- Subscribes: /camera/image_raw (sensor_msgs/Image)
- Publishes: /omega/vision/detections (vision_msgs/Detection2DArray)
- Publishes: /omega/vision/processed_image (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
import numpy as np

# Try to import OpenCV
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: OpenCV not available. Install: pip3 install opencv-python")

# Try to import cv_bridge
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False
    CvBridge = None
    print("Warning: cv_bridge not available. Install: sudo apt install ros-humble-cv-bridge")

# Try to import CUDA/GPU libraries
try:
    import torch
    TORCH_AVAILABLE = torch.cuda.is_available()
    if TORCH_AVAILABLE:
        print(f"✅ CUDA available: {torch.cuda.get_device_name(0)}")
except ImportError:
    TORCH_AVAILABLE = False
    print("Warning: PyTorch not available. Install: pip3 install torch torchvision")


class VisionProcessor(Node):
    """GPU-accelerated vision processing node."""

    def __init__(self):
        super().__init__('vision_processor')
        
        # Check dependencies
        if not CV2_AVAILABLE:
            self.get_logger().error('OpenCV not available - vision processing disabled')
            return
        
        if not CV_BRIDGE_AVAILABLE:
            self.get_logger().error('cv_bridge not available - vision processing disabled')
            return
        
        # Initialize cv_bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/omega/vision/detections',
            10
        )
        
        self.processed_image_pub = self.create_publisher(
            Image,
            '/omega/vision/processed_image',
            10
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Parameters
        self.enable_gpu = self.declare_parameter('enable_gpu', TORCH_AVAILABLE).value
        self.detection_threshold = self.declare_parameter('detection_threshold', 0.5).value
        self.process_rate = self.declare_parameter('process_rate', 30.0).value  # Hz
        
        # State
        self.frame_count = 0
        self.last_process_time = self.get_clock().now()
        
        # Initialize ML model if available
        self.model = None
        if TORCH_AVAILABLE and self.enable_gpu:
            self._load_model()
        
        self.get_logger().info(
            f'Vision processor started (GPU: {self.enable_gpu}, '
            f'Rate: {self.process_rate} Hz)'
        )

    def _load_model(self):
        """Load ML model for object detection."""
        try:
            # Example: Load YOLO or custom model
            # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            # self.model.cuda()
            self.get_logger().info('ML model loaded (placeholder)')
        except Exception as e:
            self.get_logger().warn(f'Failed to load ML model: {e}')

    def image_callback(self, msg: Image):
        """Process incoming camera image."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process image
            processed_image, detections = self.process_image(cv_image, msg.header)
            
            # Publish processed image
            if processed_image is not None:
                processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
                processed_msg.header = msg.header
                self.processed_image_pub.publish(processed_msg)
            
            # Publish detections
            if detections:
                detections_msg = Detection2DArray()
                detections_msg.header = msg.header
                detections_msg.detections = detections
                self.detections_pub.publish(detections_msg)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_image(self, cv_image, header: Header):
        """
        Process image with GPU acceleration if available.
        
        Returns:
            processed_image: OpenCV image with annotations
            detections: List of Detection2D messages
        """
        detections = []
        processed_image = cv_image.copy()
        
        # Basic image processing (edge detection, color filtering, etc.)
        if self.enable_gpu and TORCH_AVAILABLE:
            # GPU-accelerated processing
            processed_image, detections = self._process_gpu(cv_image, header)
        else:
            # CPU-based processing
            processed_image, detections = self._process_cpu(cv_image, header)
        
        return processed_image, detections

    def _process_gpu(self, cv_image, header: Header):
        """GPU-accelerated image processing."""
        detections = []
        processed_image = cv_image.copy()
        
        # Example: Run ML model inference
        if self.model is not None:
            try:
                # Convert to tensor and move to GPU
                # img_tensor = torch.from_numpy(cv_image).cuda()
                # results = self.model(img_tensor)
                # Process results and create detections
                pass
            except Exception as e:
                self.get_logger().warn(f'GPU processing error: {e}')
        
        # Add visual annotations
        cv2.putText(
            processed_image,
            f'GPU Processed Frame: {self.frame_count}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )
        
        return processed_image, detections

    def _process_cpu(self, cv_image, header: Header):
        """CPU-based image processing."""
        detections = []
        processed_image = cv_image.copy()
        
        # Example: Basic edge detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Overlay edges
        processed_image = cv2.addWeighted(processed_image, 0.8, cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR), 0.2, 0)
        
        # Add visual annotations
        cv2.putText(
            processed_image,
            f'CPU Processed Frame: {self.frame_count}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 0, 0),
            2
        )
        
        return processed_image, detections


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

