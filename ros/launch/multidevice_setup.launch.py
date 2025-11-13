#!/usr/bin/env python3
"""
Multi-device ROS2 launch file for Omega-1
Distributes nodes across Laptop, Pi 4B, and Jetson Orin Nano
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get device IPs from environment
    laptop_ip = os.getenv('LAPTOP_IP', '192.168.1.100')
    pi_ip = os.getenv('PI_IP', '192.168.1.107')
    orin_ip = os.getenv('ORIN_IP', '192.168.1.200')
    ros_domain_id = os.getenv('ROS_DOMAIN_ID', '0')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value=ros_domain_id,
            description='ROS2 domain ID for multi-device communication'
        ),
        DeclareLaunchArgument(
            'laptop_ip',
            default_value=laptop_ip,
            description='Laptop IP address'
        ),
        DeclareLaunchArgument(
            'pi_ip',
            default_value=pi_ip,
            description='Raspberry Pi 4B IP address'
        ),
        DeclareLaunchArgument(
            'orin_ip',
            default_value=orin_ip,
            description='Jetson Orin Nano IP address'
        ),

        # Log device configuration
        LogInfo(msg=[
            'Multi-device ROS2 setup:',
            '  Laptop: ', LaunchConfiguration('laptop_ip'),
            '  Pi 4B: ', LaunchConfiguration('pi_ip'),
            '  Orin Nano: ', LaunchConfiguration('orin_ip'),
            '  Domain ID: ', LaunchConfiguration('ros_domain_id')
        ]),

        # ============================================
        # LAPTOP NODES (Development & Visualization)
        # ============================================
        
        # Telemetry listener (monitors all devices)
        Node(
            package='omega_robot',
            executable='telemetry_listener',
            name='laptop_telemetry_listener',
            output='screen',
            parameters=[{
                'device_type': 'laptop',
                'device_name': 'dev_cockpit'
            }]
        ),

        # ============================================
        # PI 4B NODES (Hardware IO - run via SSH)
        # ============================================
        # Note: These nodes should be launched on Pi via SSH
        # This is a reference configuration
        
        # Telemetry publisher (runs on Pi)
        Node(
            package='omega_robot',
            executable='telemetry_publisher',
            name='pi_telemetry_publisher',
            output='screen',
            parameters=[{
                'device_type': 'pi4b',
                'device_name': 'hardware_controller',
                'publish_rate': 10.0
            }]
        ),

        # ============================================
        # JETSON ORIN NANO NODES (AI/Compute - run via SSH)
        # ============================================
        # Note: These nodes should be launched on Orin via SSH
        # This is a reference configuration

        # Vision processor (runs on Orin)
        # Node(
        #     package='omega_robot',
        #     executable='vision_processor',
        #     name='orin_vision_processor',
        #     output='screen',
        #     parameters=[{
        #         'device_type': 'orin',
        #         'device_name': 'ai_compute',
        #         'use_gpu': True,
        #         'camera_topic': '/camera/image_raw'
        #     }]
        # ),

        # ============================================
        # SHARED NODES (can run on any device)
        # ============================================

        # Sensor fusion (runs on laptop for now, can move to Orin)
        # Node(
        #     package='omega_robot',
        #     executable='sensor_fusion',
        #     name='sensor_fusion',
        #     output='screen',
        #     parameters=[{
        #         'fused_data_topic': '/sensor_fusion/data',
        #         'ultrasonic_topic': '/ultrasonic/distance',
        #         'line_tracking_topic': '/line_tracking/sensors',
        #         'camera_topic': '/camera/image_raw'
        #     }]
        # ),

        # Path planning (runs on laptop, can move to Orin)
        # Node(
        #     package='omega_robot',
        #     executable='a_star_ros',
        #     name='a_star_planner',
        #     output='screen',
        #     parameters=[{
        #         'path_topic': '/a_star/path',
        #         'map_topic': '/map',
        #         'goal_topic': '/goal_pose'
        #     }]
        # ),
    ])

