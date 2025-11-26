#!/usr/bin/env python3
"""
Omega Full System Launch - Complete Robot Stack

Launches everything: camera, vision, navigation, and autonomy.
Automatically adapts to system capabilities.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    """Generate full system launch description."""
    
    # Get launch file directory
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    
    ld = LaunchDescription()
    
    ld.add_action(LogInfo(msg="🚀 Omega Full System Launch"))
    ld.add_action(LogInfo(msg="   Starting capability-aware system..."))
    
    # Include camera launch (with capability detection)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "omega_camera.launch.py")
        )
    ))
    
    # Include brain launch (autonomy stack)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "omega_brain.launch.py")
        )
    ))
    
    return ld

