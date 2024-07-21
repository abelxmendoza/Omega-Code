# File: /Omega-Code/ros/scripts/simulated_sensors.py

"""
Simulated Sensors Node

This script simulates sensor data (camera, ultrasonic, line tracking) and publishes it as ROS messages.
It continuously publishes simulated data to the specified ROS topics.

Functions:
- publish_camera: Publishes a blank image as simulated camera data.
- publish_ultrasonic: Publishes a constant distance as simulated ultrasonic data.
- publish_line_tracking: Publishes a constant value as simulated line tracking data.
- main: Initializes the ROS node and starts publishing simulated data.

Dependencies:
- ROS: rospy, sensor_msgs, std_msgs, cv_bridge
- OpenCV: cv2

Usage:
- Run this script to start the simulated sensors node.
"""

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np

def publish_camera():
    image = np.zeros((480, 640, 3), np.uint8)
    msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
    camera_pub.publish(msg)

def publish_ultrasonic():
    range_msg = Range()
    range_msg.range = 1.0
    ultrasonic_pub.publish(range_msg)

def publish_line_tracking():
    line_msg = Float32()
    line_msg.data = 0.5
    line_tracking_pub.publish(line_msg)

def main():
    rospy.init_node('simulated_sensors')
    global bridge, camera_pub, ultrasonic_pub, line_tracking_pub
    bridge = CvBridge()
    camera_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
    ultrasonic_pub = rospy.Publisher('/ultrasonic', Range, queue_size=10)
    line_tracking_pub = rospy.Publisher('/line_tracking', Float32, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        publish_camera()
        publish_ultrasonic()
        publish_line_tracking()
        rate.sleep()

if __name__ == '__main__':
    main()
