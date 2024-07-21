# File: /Omega-Code/ros/scripts/line_tracking_publisher.py

"""
Line Tracking Publisher Node

This script reads line tracking sensor data and publishes it as ROS Int32MultiArray messages.
It continuously reads sensor data from GPIO pins and publishes the values to a specified ROS topic.

Functions:
- read_line_sensors: Reads values from line tracking sensors.
- publish_line_tracking: Reads sensor data and publishes it as ROS messages.

Dependencies:
- ROS: rospy, std_msgs
- RPi.GPIO: GPIO

Usage:
- Run this script to start the line tracking publisher node.
"""

#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
IR01 = 14
IR02 = 15
IR03 = 18
GPIO.setup(IR01, GPIO.IN)
GPIO.setup(IR02, GPIO.IN)
GPIO.setup(IR03, GPIO.IN)

def read_line_sensors():
    return [GPIO.input(IR01), GPIO.input(IR02), GPIO.input(IR03)]

def publish_line_tracking():
    pub = rospy.Publisher('line_tracking/data', Int32MultiArray, queue_size=10)
    rospy.init_node('line_tracking_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        sensor_data = read_line_sensors()
        msg = Int32MultiArray(data=sensor_data)
        rospy.loginfo(sensor_data)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_line_tracking()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
