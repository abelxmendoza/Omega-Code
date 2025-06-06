# File: /Omega-Code/ros/scripts/ultrasonic_publisher.py

"""
Ultrasonic Publisher Node

This script reads distance data from an ultrasonic sensor and publishes it as ROS Float32 messages.
It continuously reads distance values from the sensor and publishes them to a specified ROS topic.

Functions:
- distance: Measures the distance using the ultrasonic sensor.
- publish_ultrasonic: Reads distance data and publishes it as ROS messages.

Dependencies:
- ROS: rospy, std_msgs
- lgpio

Usage:
- Run this script to start the ultrasonic publisher node.
"""

#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import lgpio
import time

TRIG = 23
ECHO = 24
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, TRIG, 0)
lgpio.gpio_claim_input(h, ECHO)

def distance():
    lgpio.gpio_write(h, TRIG, 0)
    time.sleep(0.000002)
    lgpio.gpio_write(h, TRIG, 1)
    time.sleep(0.00001)
    lgpio.gpio_write(h, TRIG, 0)

    timeout_ns = 1_000_000_000
    wait_start = time.monotonic_ns()
    while lgpio.gpio_read(h, ECHO) == 0:
        if time.monotonic_ns() - wait_start > timeout_ns:
            return -1

    start = time.monotonic_ns()
    while lgpio.gpio_read(h, ECHO) == 1:
        if time.monotonic_ns() - start > timeout_ns:
            return -1
    end = time.monotonic_ns()

    pulse_us = (end - start) / 1000.0
    distance_cm = round(pulse_us / 58.0, 2)
    return distance_cm

def publish_ultrasonic():
    pub = rospy.Publisher('ultrasonic/distance', Float32, queue_size=10)
    rospy.init_node('ultrasonic_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        dist = distance()
        rospy.loginfo(dist)
        pub.publish(dist)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_ultrasonic()
    except rospy.ROSInterruptException:
        pass
    finally:
        lgpio.gpiochip_close(h)
