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
- lgpio

Usage:
- Run this script to start the line tracking publisher node.
"""

#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
import lgpio

IR01 = 14
IR02 = 15
IR03 = 18
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(h, IR01)
lgpio.gpio_claim_input(h, IR02)
lgpio.gpio_claim_input(h, IR03)

def read_line_sensors():
    return [lgpio.gpio_read(h, IR01), lgpio.gpio_read(h, IR02), lgpio.gpio_read(h, IR03)]

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
        pass
    finally:
        lgpio.gpiochip_close(h)
