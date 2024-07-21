# File: /Omega-Code/ros/scripts/log_sensor_data.py

"""
Log Sensor Data

This script logs data from various sensors to a file for later analysis.

Functions:
- ultrasonic_callback: Logs data from the ultrasonic sensor.
- line_tracking_callback: Logs data from the line tracking sensor.
- main: Initializes the ROS node and starts the subscribers.

Dependencies:
- ROS: rospy, sensor_msgs, std_msgs
- Python: datetime

Usage:
- Run this script to start logging sensor data.
"""

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int32MultiArray
from datetime import datetime

def ultrasonic_callback(data):
    with open('/path/to/ultrasonic_data.log', 'a') as f:
        f.write(f"{datetime.now()}: {data.range}\n")

def line_tracking_callback(data):
    with open('/path/to/line_tracking_data.log', 'a') as f:
        f.write(f"{datetime.now()}: {data.data}\n")

def main():
    rospy.init_node('log_sensor_data')
    rospy.Subscriber('/ultrasonic', Range, ultrasonic_callback)
    rospy.Subscriber('/line_tracking/data', Int32MultiArray, line_tracking_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
