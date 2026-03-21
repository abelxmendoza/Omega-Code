#!/usr/bin/env python3
"""
omega_teleop.launch.py -- Minimal stack + Xbox controller teleoperation
=======================================================================
Extends omega_minimal (motor + sensors, no camera) with the Xbox teleop node
so you can drive the robot with a controller plugged into the Pi over USB or
Bluetooth.

Run on Pi:
    ros2 launch omega_bringup omega_teleop.launch.py

Common overrides:
    sim_mode:=true                 -- no hardware writes (useful for testing)
    device_path:=/dev/input/event3 -- pin to a specific evdev device
    max_linear:=0.6                -- cap speed to 60 % (safer while testing)

Setup (one-time on Pi):
    pip install evdev
    sudo usermod -aG input omega1
    # Reconnect controller, then: ls /dev/input/by-id/
"""

from __future__ import annotations
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ---- launch arguments ------------------------------------------------
    args = [
        DeclareLaunchArgument('sim_mode',    default_value='false'),
        DeclareLaunchArgument('device_path', default_value='',
                              description='evdev path (empty = auto-detect)'),
        DeclareLaunchArgument('max_linear',  default_value='1.0',
                              description='Normalised max linear velocity [0-1]'),
        DeclareLaunchArgument('max_angular', default_value='1.0',
                              description='Normalised max angular velocity [0-1]'),
        DeclareLaunchArgument('dead_zone',   default_value='0.10'),
        DeclareLaunchArgument('cmd_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('watchdog_s',  default_value='0.50'),
    ]

    # ---- bring up motor + sensors (omega_minimal delegates to omega_hybrid) -
    bringup_share = FindPackageShare('omega_bringup')

    minimal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'omega_minimal.launch.py'])
        ),
        launch_arguments={
            'sim_mode': LaunchConfiguration('sim_mode'),
        }.items(),
    )

    # ---- Xbox teleop node -------------------------------------------------
    xbox_teleop = Node(
        package='omega_robot',
        executable='xbox_teleop',
        name='omega_xbox_teleop',
        output='screen',
        parameters=[{
            'device_path':  LaunchConfiguration('device_path'),
            'max_linear':   LaunchConfiguration('max_linear'),
            'max_angular':  LaunchConfiguration('max_angular'),
            'dead_zone':    LaunchConfiguration('dead_zone'),
            'cmd_rate_hz':  LaunchConfiguration('cmd_rate_hz'),
            'watchdog_s':   LaunchConfiguration('watchdog_s'),
            'sim_mode':     LaunchConfiguration('sim_mode'),
        }],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg=[
            '[omega_teleop] Xbox teleop stack '
            '(motor + sensors + controller, sim=',
            LaunchConfiguration('sim_mode'), ')',
        ]),
        minimal,
        xbox_teleop,
    ])
