#!/usr/bin/env python3
"""
Pi + Orin Hybrid Launch File for Omega Vision System

Launches both Pi sensor hub nodes and Orin AI brain nodes for hybrid operation.
This mode is activated when Orin is available (NVMe installed).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch description for Pi+Orin hybrid mode."""
    
    # Get device IPs from environment
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
            'pi_ip',
            default_value=pi_ip,
            description='Raspberry Pi IP address'
        ),
        DeclareLaunchArgument(
            'orin_ip',
            default_value=orin_ip,
            description='Jetson Orin Nano IP address'
        ),
        
        # Log configuration
        LogInfo(msg=[
            'Pi + Orin Hybrid Mode:',
            '  Pi IP: ', LaunchConfiguration('pi_ip'),
            '  Orin IP: ', LaunchConfiguration('orin_ip'),
            '  Domain ID: ', LaunchConfiguration('ros_domain_id')
        ]),
        
        # ============================================
        # PI SENSOR HUB NODES (run on Pi via SSH)
        # ============================================
        # Note: These nodes should be launched on Pi via SSH
        # This is a reference configuration
        
        # Pi Sensor Hub (runs on Pi)
        # Note: This is typically started by video_server.py
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
        
        # ============================================
        # ORIN AI BRAIN NODES (run on Orin via SSH)
        # ============================================
        # Note: These nodes should be launched on Orin via SSH
        # This is a reference configuration
        
        # Orin AI Brain (runs on Orin)
        Node(
            package='omega_robot',
            executable='orin_ai_brain',
            name='orin_ai_brain',
            output='screen',
            parameters=[{
                'device_type': 'jetson_orin_nano',
                'device_name': 'orin_ai_brain',
                'tensorrt_enabled': True,
                'yolo_enabled': True,
                'tracking_enabled': True
            }]
        ),
        
        # Vision processor (runs on Orin)
        Node(
            package='omega_robot',
            executable='vision_processor',
            name='orin_vision_processor',
            output='screen',
            parameters=[{
                'device_type': 'jetson_orin_nano'
            }]
        ),
    ])

