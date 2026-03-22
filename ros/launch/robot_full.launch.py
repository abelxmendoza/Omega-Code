#!/usr/bin/env python3
"""
Full Robot Launch File

Launches all ROS2 nodes for Omega robot (Pi hardware layer):
- Motor controller (cmd_vel → PCA9685, publishes /odom)
- Sensor node (HC-SR04, line tracking, battery)
- Camera publisher
- Action servers (navigate_to_goal, follow_line, obstacle_avoidance)
- Path planner

Run on Pi:
  ros2 launch omega_robot robot_full.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Motor controller — cmd_vel → PCA9685 PWM + odometry
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

        # Camera publisher
        Node(
            package='omega_robot',
            executable='camera_publisher_node',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'width': 640,
                'height': 480,
                'fps': 30,
                'publish_compressed': True,
            }]
        ),

        # Action servers
        Node(
            package='omega_robot',
            executable='navigate_to_goal_action_server',
            name='navigate_to_goal_action_server',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='omega_robot',
            executable='follow_line_action_server',
            name='follow_line_action_server',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='omega_robot',
            executable='obstacle_avoidance_action_server',
            name='obstacle_avoidance_action_server',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # Path planner (runs on laptop or Orin when available)
        Node(
            package='omega_robot',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'algorithm': 'astar',
            }]
        ),
    ])
