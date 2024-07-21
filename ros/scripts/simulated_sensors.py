#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32
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
