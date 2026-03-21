#!/usr/bin/env python3
"""
omega_sim.launch.py -- Simulation mode for laptop development
==============================================================
Launches the full ROS2 stack with sim_mode=true.
All hardware I/O is replaced with no-ops and synthetic data.
Use this on a laptop or CI machine -- no Raspberry Pi required.

Run:
    ros2 launch omega_bringup omega_sim.launch.py

Topics you can test against:
    ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'
    ros2 topic echo /odom
    ros2 topic echo /omega/ultrasonic
    ros2 topic echo /omega/motor_state
    rviz2  (open config from omega_bringup/config/omega_sim.rviz if present)
"""

from __future__ import annotations
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    bringup_share = FindPackageShare('omega_bringup')

    # Include the hybrid launch with sim_mode forced to true
    hybrid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'omega_hybrid.launch.py'])
        ),
        launch_arguments={
            'sim_mode':           'true',
            'launch_camera':      'true',
            'camera_width':       '640',
            'camera_height':      '480',
            'camera_fps':         '15',        # lower fps in sim to save CPU
            'ultrasonic_rate_hz': '5.0',       # lower rates in sim
            'line_rate_hz':       '10.0',
            'battery_rate_hz':    '0.5',
            'watchdog_timeout':   '1.0',       # longer watchdog in sim
        }.items(),
    )

    return LaunchDescription([
        LogInfo(msg='[omega_sim] Simulation mode -- no hardware required'),
        hybrid,
    ])
