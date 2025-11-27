"""
ROS2 Integration for Video Server
Publishes camera frames to ROS2 topics.
"""

import os
import logging
from typing import Optional
import numpy as np

log = logging.getLogger(__name__)

# ROS2 imports (optional)
ros2_available = False
ros2_node = None
ros2_publisher = None
Node = None
Image = None
CompressedImage = None
Header = None
CvBridge = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CompressedImage
    from std_msgs.msg import Header
    from cv_bridge import CvBridge
    ros2_available = True
except ImportError:
    ros2_available = False
    log.debug("ROS2 not available (rclpy or cv_bridge missing)")


class ROS2VideoPublisher:
    """ROS2 node for publishing video frames."""
    
    def __init__(self):
        """Initialize ROS2 video publisher."""
        if not ros2_available or Node is None:
            raise RuntimeError("ROS2 not available")
        
        # Create node instance
        self.node = Node('omega_video_publisher')
        
        # Check if ROS2 is initialized
        if not rclpy.ok():
            rclpy.init()
        
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = self.node.create_publisher(
            Image,
            '/omega/camera/image_raw',
            10
        )
        
        self.compressed_pub = self.node.create_publisher(
            CompressedImage,
            '/omega/camera/image_raw/compressed',
            10
        )
        
        log.info("✅ ROS2 video publisher initialized")
        log.info("   Topics: /omega/camera/image_raw, /omega/camera/image_raw/compressed")
    
    def publish_frame(self, frame: np.ndarray, compressed: bool = True):
        """
        Publish frame to ROS2 topics.
        
        Args:
            frame: BGR frame
            compressed: If True, publish compressed image (recommended)
        """
        if frame is None:
            return
        
        try:
            # Convert BGR to RGB for ROS2
            rgb_frame = frame[:, :, ::-1]  # BGR to RGB
            
            if compressed:
                # Publish compressed image (more efficient)
                try:
                    compressed_msg = CompressedImage()
                    compressed_msg.header = Header()
                    compressed_msg.header.stamp = self.node.get_clock().now().to_msg()
                    compressed_msg.header.frame_id = "camera_frame"
                    compressed_msg.format = "jpeg"
                    
                    # Encode to JPEG
                    import cv2
                    _, jpeg_data = cv2.imencode('.jpg', frame)
                    compressed_msg.data = jpeg_data.tobytes()
                    
                    self.compressed_pub.publish(compressed_msg)
                except Exception as e:
                    log.warning(f"Failed to publish compressed image: {e}")
            else:
                # Publish raw image
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(rgb_frame, "rgb8")
                    ros_image.header = Header()
                    ros_image.header.stamp = self.node.get_clock().now().to_msg()
                    ros_image.header.frame_id = "camera_frame"
                    
                    self.image_pub.publish(ros_image)
                except Exception as e:
                    log.warning(f"Failed to publish raw image: {e}")
        
        except Exception as e:
            log.error(f"ROS2 publish error: {e}", exc_info=True)
    
    def destroy_node(self):
        """Cleanup ROS2 node."""
        try:
            if self.node:
                self.node.destroy_node()
        except Exception:
            pass


def init_ros2_publisher() -> Optional[ROS2VideoPublisher]:
    """Initialize ROS2 publisher if available."""
    if not ros2_available:
        return None
    
    try:
        if not rclpy.ok():
            rclpy.init()
        
        publisher = ROS2VideoPublisher()
        
        # Start executor in background thread
        import threading
        def spin_ros2():
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(publisher)
            try:
                executor.spin()
            except Exception:
                pass
        
        thread = threading.Thread(target=spin_ros2, daemon=True)
        thread.start()
        
        return publisher
    
    except Exception as e:
        log.warning(f"Failed to initialize ROS2 publisher: {e}")
        return None


def shutdown_ros2():
    """Shutdown ROS2."""
    if ros2_available and rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            pass

