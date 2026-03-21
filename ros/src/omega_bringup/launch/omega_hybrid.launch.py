#!/usr/bin/env python3
"""
omega_hybrid.launch.py -- Full hybrid stack on Raspberry Pi
=============================================================
Launches the complete ROS2 layer.  Non-ROS services (FastAPI, Go lighting
server, MJPEG stream) are managed separately by OmegaOS / systemd.

Run on Pi:
    ros2 launch omega_bringup omega_hybrid.launch.py

Common overrides:
    sim_mode:=true               -- no hardware I/O
    launch_camera:=false         -- skip camera node (MJPEG server handles it)
    wheel_base:=0.18             -- robot-specific geometry
"""

from __future__ import annotations
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument('sim_mode',           default_value='false'),
        DeclareLaunchArgument('wheel_base',         default_value='0.20'),
        DeclareLaunchArgument('wheel_radius',       default_value='0.05'),
        DeclareLaunchArgument('max_pwm',            default_value='4095'),
        DeclareLaunchArgument('max_rpm',            default_value='300.0'),
        DeclareLaunchArgument('watchdog_timeout',   default_value='0.50'),
        DeclareLaunchArgument('ramp_type',          default_value='linear'),
        DeclareLaunchArgument('accel_rate',         default_value='4000.0'),
        DeclareLaunchArgument('decel_rate',         default_value='6000.0'),
        DeclareLaunchArgument('thermal_enabled',    default_value='true'),
        DeclareLaunchArgument('launch_camera',      default_value='true'),
        DeclareLaunchArgument('camera_width',       default_value='640'),
        DeclareLaunchArgument('camera_height',      default_value='480'),
        DeclareLaunchArgument('camera_fps',         default_value='30'),
        DeclareLaunchArgument('camera_jpeg_quality',default_value='80'),
        DeclareLaunchArgument('publish_raw_images', default_value='false'),
        DeclareLaunchArgument('ultrasonic_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('line_rate_hz',       default_value='20.0'),
        DeclareLaunchArgument('battery_rate_hz',    default_value='1.0'),
        # Physical sensor GPIO pins
        DeclareLaunchArgument('pin_left',           default_value='14'),
        DeclareLaunchArgument('pin_center',         default_value='15'),
        DeclareLaunchArgument('pin_right',          default_value='23'),
        # TF geometry (metres) -- adjust to your robot
        # camera: x=forward, z=up from base_link origin
        DeclareLaunchArgument('camera_x',           default_value='0.08'),
        DeclareLaunchArgument('camera_y',           default_value='0.0'),
        DeclareLaunchArgument('camera_z',           default_value='0.10'),
        # ultrasonic: front-mounted at base height
        DeclareLaunchArgument('us_x',               default_value='0.12'),
        DeclareLaunchArgument('us_y',               default_value='0.0'),
        DeclareLaunchArgument('us_z',               default_value='0.02'),
    ]

    sim         = LaunchConfiguration('sim_mode')
    launch_cam  = LaunchConfiguration('launch_camera')

    # ------------------------------------------------------------------
    # Motor controller
    # ------------------------------------------------------------------
    motor_node = Node(
        package='omega_robot',
        executable='motor_controller',
        name='omega_motor_controller',
        output='screen',
        parameters=[{
            'sim_mode':         sim,
            'wheel_base':       LaunchConfiguration('wheel_base'),
            'wheel_radius':     LaunchConfiguration('wheel_radius'),
            'max_pwm':          LaunchConfiguration('max_pwm'),
            'max_rpm':          LaunchConfiguration('max_rpm'),
            'watchdog_timeout': LaunchConfiguration('watchdog_timeout'),
            'ramp_type':        LaunchConfiguration('ramp_type'),
            'accel_rate':       LaunchConfiguration('accel_rate'),
            'decel_rate':       LaunchConfiguration('decel_rate'),
            'thermal_enabled':  LaunchConfiguration('thermal_enabled'),
            'ramp_enabled':     True,
        }],
    )

    # ------------------------------------------------------------------
    # Sensor node
    # ------------------------------------------------------------------
    sensor_node = Node(
        package='omega_robot',
        executable='sensor_node',
        name='omega_sensor_node',
        output='screen',
        parameters=[{
            'sim_mode':           sim,
            'ultrasonic_rate_hz': LaunchConfiguration('ultrasonic_rate_hz'),
            'line_rate_hz':       LaunchConfiguration('line_rate_hz'),
            'battery_rate_hz':    LaunchConfiguration('battery_rate_hz'),
            'pin_left':           LaunchConfiguration('pin_left'),
            'pin_center':         LaunchConfiguration('pin_center'),
            'pin_right':          LaunchConfiguration('pin_right'),
        }],
    )

    # ------------------------------------------------------------------
    # Camera node (optional)
    # ------------------------------------------------------------------
    camera_node = Node(
        package='omega_robot',
        executable='camera_publisher_node',
        name='omega_camera_publisher',
        output='screen',
        condition=IfCondition(launch_cam),
        parameters=[{
            'standalone':      True,
            'width':           LaunchConfiguration('camera_width'),
            'height':          LaunchConfiguration('camera_height'),
            'fps':             LaunchConfiguration('camera_fps'),
            'jpeg_quality':    LaunchConfiguration('camera_jpeg_quality'),
            'publish_raw':     LaunchConfiguration('publish_raw_images'),
            'camera_frame_id': 'camera_link',
        }],
    )

    # ------------------------------------------------------------------
    # Capability detector (publishes /omega/capabilities)
    # ------------------------------------------------------------------
    capability_node = Node(
        package='omega_robot',
        executable='system_capabilities',
        name='omega_capability_detector',
        output='screen',
        parameters=[{'publish_interval': 5.0}],
    )

    # ------------------------------------------------------------------
    # Static TF publishers
    # base_link --> camera_link
    # ------------------------------------------------------------------
    tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_camera',
        output='screen',
        arguments=[
            LaunchConfiguration('camera_x'),
            LaunchConfiguration('camera_y'),
            LaunchConfiguration('camera_z'),
            '0.0', '0.0', '0.0',
            'base_link', 'camera_link',
        ],
    )

    # base_link --> ultrasonic_front
    tf_ultrasonic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_ultrasonic',
        output='screen',
        arguments=[
            LaunchConfiguration('us_x'),
            LaunchConfiguration('us_y'),
            LaunchConfiguration('us_z'),
            '0.0', '0.0', '0.0',
            'base_link', 'ultrasonic_front',
        ],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg=['[omega_hybrid] Starting full hybrid stack (sim=', sim, ')']),
        motor_node,
        sensor_node,
        camera_node,
        capability_node,
        tf_camera,
        tf_ultrasonic,
    ])
