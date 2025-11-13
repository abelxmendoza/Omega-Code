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
        
        # Camera Publisher
        Node(
            package='omega_robot',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'width': 640,
                'height': 480,
                'fps': 30,
                'publish_compressed': True
            }]
        ),
        
        # Action Servers
        Node(
            package='omega_robot',
            executable='navigate_to_goal_action_server',
            name='navigate_to_goal_action_server',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        Node(
            package='omega_robot',
            executable='follow_line_action_server',
            name='follow_line_action_server',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        Node(
            package='omega_robot',
            executable='obstacle_avoidance_action_server',
            name='obstacle_avoidance_action_server',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])

