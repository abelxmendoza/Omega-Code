#!/usr/bin/env python3
"""
Full Robot Launch File

Launches all ROS2 nodes for Omega robot:
- Sensor data publisher
- Robot controller
- Enhanced telemetry
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Sensor Data Publisher
        Node(
            package='omega_robot',
            executable='sensor_data_publisher',
            name='sensor_data_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Robot Controller
        Node(
            package='omega_robot',
            executable='robot_controller',
            name='robot_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Enhanced Telemetry
        Node(
            package='omega_robot',
            executable='enhanced_telemetry',
            name='enhanced_telemetry',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])

