#!/usr/bin/env python

"""
Camera Publisher Node

This script captures video from the Raspberry Pi camera and publishes the frames as ROS Image messages.
It continuously reads frames from the camera and publishes them to a specified ROS topic.

Functions:
- publish_camera: Captures frames from the camera and publishes them as ROS Image messages.

Dependencies:
- ROS: rospy, sensor_msgs, cv_bridge
- picamera: picamera

Usage:
- Run this script to start the camera publisher node.
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

def publish_camera():
    pub = rospy.Publisher('camera/image', Image, queue_size=10)
    rospy.init_node('camera_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 10
    raw_capture = PiRGBArray(camera, size=(640, 480))

    bridge = CvBridge()

    time.sleep(0.1)  # Allow the camera to warm up

    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        image = frame.array
        image_message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        pub.publish(image_message)
        raw_capture.truncate(0)  # Clear the stream for the next frame
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera()
    except rospy.ROSInterruptException:
        pass
