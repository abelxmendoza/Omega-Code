# File: /Omega-Code/ros/scripts/start_sensors.py

"""
Start Sensors Script

This script manages the startup of sensor nodes based on the provided configuration.
It handles three scenarios:
1. Only working with the MacBook (or other Ubuntu Linux laptop).
2. Working with the MacBook (or other Ubuntu Linux laptop) and Omega1 (Raspberry Pi).
3. Working with the MacBook (or other Ubuntu Linux laptop), Omega1 (Raspberry Pi), and the Jetson Nano.

Functions:
- start_simulated_sensors: Starts simulated sensors on the MacBook.
- start_raspberry_pi_sensors: Starts sensor nodes on the Raspberry Pi.
- start_jetson_nano_sensors: Placeholder function for starting sensors on the Jetson Nano.
- main: Main function to determine and start the appropriate sensors based on the scenario.

Dependencies:
- ROS: rospy, subprocess

Usage:
- Run this script to start the appropriate sensor nodes based on the configuration.
"""

import os
import subprocess
import rospy

def start_simulated_sensors():
    print("Starting simulated sensors...")
    subprocess.Popen(["rosrun", "ros_scripts", "simulated_sensors.py"])

def start_raspberry_pi_sensors():
    print("Starting Raspberry Pi sensors...")
    subprocess.Popen(["ssh", "omega1@$TAILSCALE_IP_PI", "roslaunch", "omega_robot", "robot_sensors.launch"])

def start_jetson_nano_sensors():
    print("Starting Jetson Nano sensors... (Not implemented)")
    # Placeholder for starting Jetson Nano sensors

def main():
    rospy.init_node('start_sensors')
    scenario = int(input("Enter the scenario (1: MacBook only, 2: MacBook + Raspberry Pi, 3: MacBook + Raspberry Pi + Jetson Nano): "))
    
    if scenario == 1:
        start_simulated_sensors()
    elif scenario == 2:
        start_simulated_sensors()
        start_raspberry_pi_sensors()
    elif scenario == 3:
        start_simulated_sensors()
        start_raspberry_pi_sensors()
        start_jetson_nano_sensors()
    else:
        print("Invalid scenario selected")

if __name__ == "__main__":
    main()
    rospy.spin()
