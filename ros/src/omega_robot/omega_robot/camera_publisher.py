#!/usr/bin/env python3
"""
Camera Image Publisher Node for Omega Robot

Publishes camera frames to ROS2 image topics:
- /camera/image_raw (sensor_msgs/Image) - Raw uncompressed
- /camera/image_raw/compressed (sensor_msgs/CompressedImage) - Compressed JPEG
- /camera/image_raw/theora (theora_image_transport) - Theora video

Integrates with existing camera.py backend for hardware abstraction.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import threading
import time

# Try to import cv_bridge
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False
    CvBridge = None
    print("Warning: cv_bridge not available. Install: sudo apt install ros-rolling-cv-bridge")

# Try to import camera backend
CAMERA_AVAILABLE = False
Camera = None

try:
    # Try to import from installed package or relative path
    try:
        from video.camera import Camera
        CAMERA_AVAILABLE = True
    except ImportError:
        # Try absolute path (for development)
        import sys
        import os
        backend_path = os.path.join(
            os.path.dirname(__file__), 
            '../../../../servers/robot_controller_backend'
        )
        if os.path.exists(backend_path):
            sys.path.insert(0, backend_path)
            from video.camera import Camera
            CAMERA_AVAILABLE = True
        else:
            raise ImportError("Camera backend path not found")
except Exception as e:
    CAMERA_AVAILABLE = False
    Camera = None
    print(f"Warning: Camera backend not available: {e}")


class CameraPublisher(Node):
    """Publishes camera frames to ROS2 topics."""

    def __init__(self):
        super().__init__('camera_publisher')
        
        # Initialize CV bridge
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn('cv_bridge not available - image conversion limited')
        
        # Publishers
        self.image_pub = self.create_publisher(
            Image, '/camera/image_raw', 10
        )
        self.compressed_pub = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', 10
        )
        
        # Load capability profile for defaults
        try:
            from .capability_utils import get_max_resolution, get_max_fps
            default_width, default_height = get_max_resolution()
            default_fps = get_max_fps()
        except ImportError:
            default_width, default_height = 640, 480
            default_fps = 30
            self.get_logger().debug("capability_utils not available, using defaults")
        
        # Camera configuration (respects capability profile defaults)
        self.width = int(self.declare_parameter('width', default_width).value)
        self.height = int(self.declare_parameter('height', default_height).value)
        self.fps = int(self.declare_parameter('fps', default_fps).value)
        self.publish_compressed = self.declare_parameter('publish_compressed', True).value
        
        # Log capability-aware settings
        self.get_logger().info(
            f'Camera configured: {self.width}x{self.height} @ {self.fps} FPS '
            f'(capability-aware defaults)'
        )
        
        # Initialize camera
        self.camera = None
        self.camera_available = False
        
        if CAMERA_AVAILABLE:
            try:
                self.camera = Camera(
                    width=self.width,
                    height=self.height,
                    fps=self.fps
                )
                self.camera_available = True
                self.get_logger().info('Camera initialized successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize camera: {e}')
                self.camera_available = False
        else:
            self.get_logger().warn('Camera backend not available - using placeholder')
        
        # Publishing timer
        period = 1.0 / self.fps if self.fps > 0 else 0.033  # Default 30 FPS
        self.timer = self.create_timer(period, self.publish_frame)
        
        # Frame counter
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps_actual = 0.0
        
        self.get_logger().info(f'Camera publisher started: {self.width}x{self.height} @ {self.fps} FPS')

    def publish_frame(self):
        """Capture and publish camera frame."""
        if not self.camera_available or self.camera is None:
            # Publish placeholder frame
            self._publish_placeholder()
            return
        
        try:
            # Get frame from camera
            frame = self.camera.get_frame()
            
            if frame is None:
                self.get_logger().warn('No frame available from camera')
                return
            
            # Convert BGR to RGB (OpenCV uses BGR, ROS uses RGB)
            frame_rgb = frame[:, :, ::-1]  # BGR to RGB
            
            # Publish uncompressed image
            if self.bridge:
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame_rgb, 'rgb8')
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    ros_image.header.frame_id = 'camera_frame'
                    self.image_pub.publish(ros_image)
                except Exception as e:
                    self.get_logger().error(f'Error publishing image: {e}')
            
            # Publish compressed image (more efficient for network)
            if self.publish_compressed:
                try:
                    import cv2
                    # Encode to JPEG
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
                    _, jpeg_data = cv2.imencode('.jpg', frame, encode_param)
                    
                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = self.get_clock().now().to_msg()
                    compressed_msg.header.frame_id = 'camera_frame'
                    compressed_msg.format = 'jpeg'
                    compressed_msg.data = jpeg_data.tobytes()
                    self.compressed_pub.publish(compressed_msg)
                except Exception as e:
                    self.get_logger().error(f'Error publishing compressed image: {e}')
            
            # Update FPS counter
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.last_fps_time >= 1.0:
                self.fps_actual = self.frame_count / (current_time - self.last_fps_time)
                self.frame_count = 0
                self.last_fps_time = current_time
                self.get_logger().debug(f'Publishing at {self.fps_actual:.1f} FPS')
        
        except Exception as e:
            self.get_logger().error(f'Error in publish_frame: {e}')

    def _publish_placeholder(self):
        """Publish placeholder frame when camera is not available."""
        try:
            import cv2
            # Create a simple placeholder image
            placeholder = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            cv2.putText(placeholder, 'No Camera', (self.width//4, self.height//2),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Convert to RGB
            placeholder_rgb = placeholder[:, :, ::-1]
            
            # Publish
            ros_image = self.bridge.cv2_to_imgmsg(placeholder_rgb, 'rgb8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing placeholder: {e}')

    def destroy_node(self):
        """Cleanup on shutdown."""
        if self.camera is not None:
            try:
                # Camera cleanup handled by context manager if available
                pass
            except:
                pass
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

