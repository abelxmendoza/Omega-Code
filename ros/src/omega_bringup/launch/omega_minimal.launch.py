#!/usr/bin/env python3
"""
omega_minimal.launch.py -- Minimal stack for resource-constrained scenarios
=============================================================================
Launches only the core nodes needed for tele-operation:
  - motor_controller_node   (/cmd_vel -> motors)
  - sensor_node             (ultrasonic, line tracking, battery)
  - static TF publishers

Does NOT launch:
  - camera (saves ~25% Pi CPU and eliminates camera bus contention)
  - capability_detector
  - any simulation nodes

Use this when:
  - Running alongside video_server.py (which manages the camera itself)
  - CPU/RAM is tight on the Pi
  - Doing hardware-only motor testing

Run:
    ros2 launch omega_bringup omega_minimal.launch.py
    ros2 launch omega_bringup omega_minimal.launch.py sim_mode:=true
"""

from __future__ import annotations
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument('sim_mode', default_value='false'),
    ]

    bringup_share = FindPackageShare('omega_bringup')

    hybrid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'omega_hybrid.launch.py'])
        ),
        launch_arguments={
            'sim_mode':       LaunchConfiguration('sim_mode'),
            'launch_camera':  'false',         # camera disabled
            'battery_rate_hz':'0.2',           # 5-second battery updates
        }.items(),
    )

    return LaunchDescription([
        *args,
        LogInfo(msg=[
            '[omega_minimal] Minimal stack (motor + sensors only, sim=',
            LaunchConfiguration('sim_mode'), ')',
        ]),
        hybrid,
    ])
