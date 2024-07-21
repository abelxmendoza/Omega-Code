# File: /Omega-Code/ros/scripts/camera_publisher.py

"""
Camera Publisher Node

This script captures video from the camera and publishes the frames as ROS Image messages.
It continuously reads frames from the camera and publishes them to a specified ROS topic.

Functions:
- publish_camera: Captures frames from the camera and publishes them as ROS Image messages.

Dependencies:
- ROS: rospy, sensor_msgs, cv_bridge
- OpenCV: cv2

Usage:
- Run this script to start the camera publisher node.
"""

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_camera():
    pub = rospy.Publisher('camera/image', Image, queue_size=10)
    rospy.init_node('camera_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(image_message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera()
    except rospy.ROSInterruptException:
        pass
