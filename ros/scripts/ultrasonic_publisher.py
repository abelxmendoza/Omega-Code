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
- RPi.GPIO: GPIO

Usage:
- Run this script to start the ultrasonic publisher node.
"""

#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
TRIG = 23
ECHO = 24
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def distance():
    GPIO.output(TRIG, False)
    time.sleep(2)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    return distance

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
        GPIO.cleanup()
