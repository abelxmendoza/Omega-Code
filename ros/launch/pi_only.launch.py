#!/usr/bin/env python3
"""
Pi-Only Launch File for Omega Vision System

Launches Pi sensor hub nodes for Pi-only operation mode.
This is the default mode when Orin is not available.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch description for Pi-only mode."""
    
    # Get Pi IP from environment
    pi_ip = os.getenv('PI_IP', '192.168.1.107')
    ros_domain_id = os.getenv('ROS_DOMAIN_ID', '0')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value=ros_domain_id,
            description='ROS2 domain ID for communication'
        ),
        DeclareLaunchArgument(
            'pi_ip',
            default_value=pi_ip,
            description='Raspberry Pi IP address'
        ),
        
        # Log configuration
        LogInfo(msg=[
            'Pi-Only Mode:',
            '  Pi IP: ', LaunchConfiguration('pi_ip'),
            '  Domain ID: ', LaunchConfiguration('ros_domain_id')
        ]),
        
        # ============================================
        # PI SENSOR HUB NODES
        # ============================================
        
        # Pi Sensor Hub (runs on Pi)
        # Note: This is typically started by video_server.py, but can be launched standalone
        Node(
            package='omega_robot',
            executable='telemetry_publisher',
            name='pi_telemetry_publisher',
            output='screen',
            parameters=[{
                'device_type': 'raspberry_pi',
                'device_name': 'pi_sensor_hub'
            }]
        ),
        
        # Sensor data publisher (runs on Pi)
        Node(
            package='omega_robot',
            executable='sensor_data_publisher',
            name='pi_sensor_data_publisher',
            output='screen',
            parameters=[{
                'device_type': 'raspberry_pi'
            }]
        ),
        
        # Enhanced telemetry (runs on Pi)
        Node(
            package='omega_robot',
            executable='enhanced_telemetry',
            name='pi_enhanced_telemetry',
            output='screen',
            parameters=[{
                'device_type': 'raspberry_pi'
            }]
        ),
    ])

