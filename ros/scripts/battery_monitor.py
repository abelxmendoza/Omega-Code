# File: /Omega-Code/ros/scripts/battery_monitor.py

"""
Battery Monitor

This script monitors the battery status of the robot and sends alerts if the battery level is low.

Functions:
- battery_callback: Logs and checks the battery status.
- main: Initializes the ROS node and starts the subscriber.

Dependencies:
- ROS: rospy, std_msgs
- Python: smtplib (for sending email alerts)

Usage:
- Run this script to start monitoring the battery status.
"""

#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def battery_callback(data):
    battery_level = data.data
    rospy.loginfo(f"Battery level: {battery_level}%")
    if battery_level < 20:
        send_alert(battery_level)

def send_alert(battery_level):
    import smtplib
    from email.mime.text import MIMEText

    msg = MIMEText(f"Warning: Battery level is low at {battery_level}%")
    msg['Subject'] = 'Robot Battery Alert'
    msg['From'] = 'your-email@example.com'
    msg['To'] = 'alert-recipient@example.com'

    with smtplib.SMTP('smtp.example.com') as server:
        server.login('your-email@example.com', 'your-email-password')
        server.sendmail('your-email@example.com', ['alert-recipient@example.com'], msg.as_string())

def main():
    rospy.init_node('battery_monitor')
    rospy.Subscriber('/battery_level', Float32, battery_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

