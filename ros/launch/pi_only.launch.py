#!/usr/bin/env python3
"""
Pi-Only Launch File

Launches the hardware IO layer on the Raspberry Pi 4B
without the Jetson Orin AI nodes.

Run on Pi:
  ros2 launch omega_robot pi_only.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pi_ip = os.getenv('PI_IP', '192.168.1.107')
    ros_domain_id = os.getenv('ROS_DOMAIN_ID', '0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value=ros_domain_id,
            description='ROS2 domain ID'
        ),
        DeclareLaunchArgument(
            'pi_ip',
            default_value=pi_ip,
            description='Raspberry Pi IP address'
        ),

        LogInfo(msg=[
            'Pi-Only Mode | IP: ', LaunchConfiguration('pi_ip'),
            ' | Domain: ', LaunchConfiguration('ros_domain_id'),
        ]),

        # Motor controller — cmd_vel → PCA9685 + /odom
        Node(
            package='omega_robot',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # Sensor node — HC-SR04, line tracking, battery
        Node(
            package='omega_robot',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])
