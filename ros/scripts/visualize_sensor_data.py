# File: /Omega-Code/ros/scripts/visualize_sensor_data.py

"""
Visualize Sensor Data

This script visualizes data from various sensors in real-time using Matplotlib.

Functions:
- ultrasonic_callback: Updates the plot with data from the ultrasonic sensor.
- line_tracking_callback: Updates the plot with data from the line tracking sensor.
- main: Initializes the ROS node and starts the subscribers.

Dependencies:
- ROS: rospy, sensor_msgs, std_msgs
- Python: matplotlib, datetime

Usage:
- Run this script to start visualizing sensor data in real-time.
"""

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

ultrasonic_data = []
line_tracking_data = []

def ultrasonic_callback(data):
    ultrasonic_data.append(data.range)

def line_tracking_callback(data):
    line_tracking_data.append(data.data)

def update_plot(frame):
    plt.clf()
    plt.subplot(2, 1, 1)
    plt.plot(ultrasonic_data, label='Ultrasonic')
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot(line_tracking_data, label='Line Tracking')
    plt.legend()

def main():
    rospy.init_node('visualize_sensor_data')
    rospy.Subscriber('/ultrasonic', Range, ultrasonic_callback)
    rospy.Subscriber('/line_tracking/data', Int32MultiArray, line_tracking_callback)
    
    ani = FuncAnimation(plt.gcf(), update_plot, interval=1000)
    plt.show()
    
    rospy.spin()

if __name__ == '__main__':
    main()
