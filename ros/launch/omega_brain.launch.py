#!/usr/bin/env python3
"""
Omega Brain Launch File - Full Autonomy Stack

Launches the complete autonomy stack based on system capabilities.
Only runs on Jetson (omega mode) or Lenovo (dev mode).
"""

import os
import json
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def load_capability_profile():
    """Load capability profile from temp file."""
    temp_file = os.path.join(tempfile.gettempdir(), "omega_capabilities.json")
    
    if os.path.exists(temp_file):
        try:
            with open(temp_file, "r") as f:
                return json.load(f)
        except Exception:
            pass
    
    return {
        "profile_mode": "mac",
        "ml_capable": False,
        "slam_capable": False,
    }


def generate_launch_description():
    """Generate launch description for autonomy stack."""
    
    profile = load_capability_profile()
    profile_mode = profile.get("profile_mode", "mac")
    
    ld = LaunchDescription()
    
    # Only launch if not in light mode
    if profile_mode == "mac":
        ld.add_action(LogInfo(
            msg="⚠️  Omega Brain requires Jetson or Lenovo Linux. "
                "Skipping autonomy stack launch."
        ))
        return ld
    
    ld.add_action(LogInfo(
        msg=f"🧠 Omega Brain Launch - Profile: {profile_mode.upper()}"
    ))
    
    # Navigation stack (Lenovo and Jetson)
    if profile.get("slam_capable", False):
        ld.add_action(LogInfo(msg="   Navigation stack enabled"))
        
        # Path planner
        ld.add_action(Node(
            package="omega_robot",
            executable="path_planner",
            name="path_planner",
            output="screen"
        ))
        
        # Odometry publisher
        ld.add_action(Node(
            package="omega_robot",
            executable="odometry_publisher",
            name="odometry_publisher",
            output="screen"
        ))
    
    # Action servers (all modes except light)
    ld.add_action(Node(
        package="omega_robot",
        executable="navigate_to_goal_action_server",
        name="navigate_to_goal_action_server",
        output="screen"
    ))
    
    ld.add_action(Node(
        package="omega_robot",
        executable="follow_line_action_server",
        name="follow_line_action_server",
        output="screen"
    ))
    
    ld.add_action(Node(
        package="omega_robot",
        executable="obstacle_avoidance_action_server",
        name="obstacle_avoidance_action_server",
        output="screen"
    ))
    
    # Jetson-specific nodes
    if profile_mode == "jetson":
        ld.add_action(LogInfo(msg="   GPU-accelerated nodes enabled"))
        
        # Vision processor (if not already running)
        if profile.get("ml_capable", False):
            ld.add_action(LogInfo(msg="   ML vision processing enabled"))
    
    return ld

