# File: /Omega-Code/ros/scripts/sensor_fusion.py

"""
Sensor Fusion Node

This script fuses data from multiple sensors (camera, ultrasonic, line tracking) and publishes the fused data.
It subscribes to sensor topics, processes the data, and publishes the fused information.

Classes:
- SensorFusion: Handles subscription to sensor topics and data fusion.

Functions:
- image_callback: Processes incoming camera images.
- ultrasonic_callback: Processes incoming ultrasonic data.
- line_tracking_callback: Processes incoming line tracking data.
- fuse_sensors: Fuses data from all sensors and publishes it.

Dependencies:
- ROS: rospy, sensor_msgs, std_msgs, cv_bridge
- OpenCV: cv2

Usage:
- Run this script to start the sensor fusion node.
"""

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import warnings
try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    warnings.warn("OpenCV not installed. Sensor fusion image processing disabled.", ImportWarning)

class SensorFusion:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.ultrasonic_sub = rospy.Subscriber('/ultrasonic', Range, self.ultrasonic_callback)
        self.line_tracking_sub = rospy.Subscriber('/line_tracking', Float32, self.line_tracking_callback)
        self.fused_data_pub = rospy.Publisher('/fused_sensor_data', Image, queue_size=10)

        self.latest_image = None
        self.latest_ultrasonic = None
        self.latest_line_tracking = None

    def image_callback(self, data):
        if cv2 is None:
            rospy.logwarn("OpenCV not available, skipping image processing")
            return
        self.latest_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.fuse_sensors()

    def ultrasonic_callback(self, data):
        self.latest_ultrasonic = data.range
        self.fuse_sensors()

    def line_tracking_callback(self, data):
        self.latest_line_tracking = data.data
        self.fuse_sensors()

    def fuse_sensors(self):
        if self.latest_image is not None and self.latest_ultrasonic is not None and self.latest_line_tracking is not None:
            # Example of simple fusion: overlay ultrasonic and line tracking data on the image
            fused_image = self.latest_image.copy()
            if cv2 is not None:
                cv2.putText(fused_image, f'Ultrasonic: {self.latest_ultrasonic}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.putText(fused_image, f'Line Tracking: {self.latest_line_tracking}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            # Publish the fused data as an image
            fused_msg = self.bridge.cv2_to_imgmsg(fused_image, "bgr8")
            self.fused_data_pub.publish(fused_msg)

if __name__ == '__main__':
    rospy.init_node('sensor_fusion')
    SensorFusion()
    rospy.spin()
