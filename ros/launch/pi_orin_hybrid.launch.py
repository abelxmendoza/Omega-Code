#!/usr/bin/env python3
"""
Pi + Orin Hybrid Launch File

Pi 4B handles hardware IO; Jetson Orin Nano handles AI compute.
Each device should launch its own nodes — this file is the
reference configuration showing the full node graph.

Pi (run directly on Pi):
  ros2 launch omega_robot pi_only.launch.py

Orin (run directly on Orin):
  ros2 run omega_robot vision_processor
  ros2 run omega_robot orin_ai_brain

Or launch everything from this file (requires DDS cross-device config):
  ros2 launch omega_robot pi_orin_hybrid.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pi_ip = os.getenv('PI_IP', '192.168.1.107')
    orin_ip = os.getenv('ORIN_IP', '100.107.112.110')
    ros_domain_id = os.getenv('ROS_DOMAIN_ID', '0')

    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value=ros_domain_id),
        DeclareLaunchArgument('pi_ip', default_value=pi_ip),
        DeclareLaunchArgument('orin_ip', default_value=orin_ip),

        LogInfo(msg=[
            'Pi + Orin Hybrid | Pi: ', LaunchConfiguration('pi_ip'),
            ' | Orin: ', LaunchConfiguration('orin_ip'),
            ' | Domain: ', LaunchConfiguration('ros_domain_id'),
        ]),

        # ── Pi 4B nodes (hardware IO) ─────────────────────────────────
        Node(
            package='omega_robot',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='omega_robot',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='omega_robot',
            executable='camera_publisher_node',
            name='camera_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'width': 640, 'height': 480, 'fps': 30}]
        ),

        # ── Jetson Orin Nano nodes (AI compute) ───────────────────────
        Node(
            package='omega_robot',
            executable='vision_processor',
            name='orin_vision_processor',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='omega_robot',
            executable='orin_ai_brain',
            name='orin_ai_brain',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'tensorrt_enabled': True,
                'yolo_enabled': True,
                'tracking_enabled': True,
            }]
        ),
    ])
