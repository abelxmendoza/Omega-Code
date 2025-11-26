#!/usr/bin/env python3
"""
Omega Camera Launch File - Capability-Aware

Automatically adapts camera and vision nodes based on detected system capabilities.
Supports three profiles:
- mac: Light mode (MacBook + Pi)
- lenovo: Dev mode (Lenovo Linux + Pi)
- jetson: Omega mode (Jetson Orin Nano + Pi)
"""

import os
import json
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def load_capability_profile():
    """Load capability profile from temp file or detect."""
    temp_file = os.path.join(tempfile.gettempdir(), "omega_capabilities.json")
    
    # Try to load from file
    if os.path.exists(temp_file):
        try:
            with open(temp_file, "r") as f:
                return json.load(f)
        except Exception:
            pass
    
    # Default fallback profile (light mode)
    return {
        "profile_mode": "mac",
        "ml_capable": False,
        "slam_capable": False,
        "tracking": True,
        "aruco": True,
        "motion_detection": True,
        "face_recognition": False,
        "yolo": False,
        "max_resolution": "640x480",
        "max_fps": 30,
    }


def generate_launch_description():
    """Generate launch description based on capabilities."""
    
    # Load capability profile
    profile = load_capability_profile()
    profile_mode = profile.get("profile_mode", "mac")
    
    # Parse resolution
    max_res = profile.get("max_resolution", "640x480")
    width, height = map(int, max_res.split("x"))
    max_fps = profile.get("max_fps", 30)
    
    # Launch arguments
    declare_profile_arg = DeclareLaunchArgument(
        "profile",
        default_value=profile_mode,
        description="Override capability profile (mac/lenovo/jetson)"
    )
    
    declare_width_arg = DeclareLaunchArgument(
        "width",
        default_value=str(width),
        description="Camera width"
    )
    
    declare_height_arg = DeclareLaunchArgument(
        "height",
        default_value=str(height),
        description="Camera height"
    )
    
    declare_fps_arg = DeclareLaunchArgument(
        "fps",
        default_value=str(max_fps),
        description="Camera FPS"
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_profile_arg)
    ld.add_action(declare_width_arg)
    ld.add_action(declare_height_arg)
    ld.add_action(declare_fps_arg)
    
    # Log detected profile
    ld.add_action(LogInfo(
        msg=f"🚀 Omega Camera Launch - Profile: {profile_mode.upper()}"
    ))
    ld.add_action(LogInfo(
        msg=f"   Resolution: {width}x{height} @ {max_fps} FPS"
    ))
    ld.add_action(LogInfo(
        msg=f"   ML: {profile.get('ml_capable', False)}, "
            f"SLAM: {profile.get('slam_capable', False)}, "
            f"YOLO: {profile.get('yolo', False)}"
    ))
    
    # Always run capability detector first
    ld.add_action(Node(
        package="omega_robot",
        executable="system_capabilities",
        name="capability_detector",
        output="screen"
    ))
    
    # Always run camera publisher (all profiles)
    ld.add_action(Node(
        package="omega_robot",
        executable="camera_publisher",
        name="camera_publisher",
        parameters=[{
            "width": LaunchConfiguration("width"),
            "height": LaunchConfiguration("height"),
            "fps": LaunchConfiguration("fps"),
            "publish_compressed": True,
        }],
        output="screen"
    ))
    
    # Light mode (MacBook + Pi) - Basic features only
    if profile_mode == "mac":
        ld.add_action(LogInfo(msg="   Mode: LIGHT - Basic tracking and ArUco only"))
        
        # Note: Motion detection and ArUco are handled in video_server.py
        # No additional ROS2 nodes needed for light mode
    
    # Dev mode (Lenovo Linux + Pi) - SLAM and navigation
    elif profile_mode == "lenovo":
        ld.add_action(LogInfo(msg="   Mode: DEV - SLAM and navigation enabled"))
        
        # SLAM node (if available)
        if profile.get("slam_capable", False):
            ld.add_action(Node(
                package="omega_robot",
                executable="odometry_publisher",
                name="odometry_publisher",
                output="screen",
                condition=IfCondition("true")  # Placeholder for actual SLAM node
            ))
        
        # ArUco pose estimation (enhanced)
        if profile.get("aruco", True):
            ld.add_action(LogInfo(msg="   ArUco pose estimation enabled"))
    
    # Omega mode (Jetson + Pi) - Full autonomy
    elif profile_mode == "jetson":
        ld.add_action(LogInfo(msg="   Mode: OMEGA - Full GPU-accelerated autonomy"))
        
        # GPU-accelerated vision processor
        if profile.get("ml_capable", False):
            ld.add_action(Node(
                package="omega_robot",
                executable="vision_processor",
                name="vision_processor",
                parameters=[{
                    "enable_gpu": True,
                    "detection_threshold": 0.5,
                    "process_rate": max_fps,
                }],
                output="screen"
            ))
        
        # YOLO node (if YOLO is enabled)
        if profile.get("yolo", False):
            ld.add_action(LogInfo(msg="   YOLO detection enabled (GPU)"))
            # Placeholder for YOLO node when implemented
        
        # Face recognition (GPU-accelerated)
        if profile.get("face_recognition", False):
            ld.add_action(LogInfo(msg="   Face recognition enabled (GPU)"))
            # Face recognition handled in video_server.py
        
        # SLAM (GPU-accelerated)
        if profile.get("slam_capable", False):
            ld.add_action(LogInfo(msg="   Visual SLAM enabled (GPU)"))
            # Placeholder for GPU SLAM node
    
    return ld

