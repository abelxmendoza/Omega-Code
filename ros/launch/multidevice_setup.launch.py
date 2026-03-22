#!/usr/bin/env python3
"""
Multi-device ROS2 reference launch for Omega-1

Shows the full node distribution across Laptop, Pi 4B, and Jetson Orin.
Each device should run its own launch file — this is a reference config.

Laptop:   ros2 launch omega_robot multidevice_setup.launch.py
Pi:       ros2 launch omega_robot pi_only.launch.py
Orin:     ros2 run omega_robot vision_processor / orin_ai_brain

Requires: .env.ros2.multidevice with LAPTOP_IP, PI_IP, ORIN_IP set.
See docs/PLATFORMS.md for full setup.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    laptop_ip = os.getenv('LAPTOP_IP', '192.168.1.100')
    pi_ip = os.getenv('PI_IP', '192.168.1.107')
    orin_ip = os.getenv('ORIN_IP', '100.107.112.110')
    ros_domain_id = os.getenv('ROS_DOMAIN_ID', '0')

    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value=ros_domain_id),
        DeclareLaunchArgument('laptop_ip', default_value=laptop_ip),
        DeclareLaunchArgument('pi_ip', default_value=pi_ip),
        DeclareLaunchArgument('orin_ip', default_value=orin_ip),

        LogInfo(msg=[
            'Multi-device ROS2 | Laptop: ', LaunchConfiguration('laptop_ip'),
            ' | Pi: ', LaunchConfiguration('pi_ip'),
            ' | Orin: ', LaunchConfiguration('orin_ip'),
            ' | Domain: ', LaunchConfiguration('ros_domain_id'),
        ]),

        # ── Laptop: path planner + action servers ─────────────────────
        Node(
            package='omega_robot',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[{'use_sim_time': False, 'algorithm': 'astar'}]
        ),
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

        # ── Pi 4B: hardware IO (run pi_only.launch.py on Pi directly) ─
        # Node(package='omega_robot', executable='motor_controller', ...),
        # Node(package='omega_robot', executable='sensor_node', ...),

        # ── Jetson Orin: AI compute (run on Orin directly) ────────────
        # Node(package='omega_robot', executable='vision_processor', ...),
        # Node(package='omega_robot', executable='orin_ai_brain', ...),
    ])
