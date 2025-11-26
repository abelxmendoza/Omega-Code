#!/usr/bin/env python3
"""
Capability Utilities for Omega Robot Nodes

Provides helper functions for nodes to check system capabilities
and adapt their behavior accordingly.
"""

import os
import json
import tempfile
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def load_capability_profile():
    """Load capability profile from temp file."""
    temp_file = os.path.join(tempfile.gettempdir(), "omega_capabilities.json")
    
    if os.path.exists(temp_file):
        try:
            with open(temp_file, "r") as f:
                return json.load(f)
        except Exception as e:
            print(f"Warning: Failed to load capability profile: {e}")
    
    # Default fallback (light mode)
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


def get_capability_profile():
    """Get current capability profile."""
    return load_capability_profile()


def is_ml_capable():
    """Check if ML/GPU processing is available."""
    profile = load_capability_profile()
    return profile.get("ml_capable", False)


def is_slam_capable():
    """Check if SLAM is available."""
    profile = load_capability_profile()
    return profile.get("slam_capable", False)


def is_jetson_mode():
    """Check if running in Jetson mode."""
    profile = load_capability_profile()
    return profile.get("profile_mode") == "jetson"


def get_max_resolution():
    """Get maximum supported resolution."""
    profile = load_capability_profile()
    res = profile.get("max_resolution", "640x480")
    width, height = map(int, res.split("x"))
    return width, height


def get_max_fps():
    """Get maximum supported FPS."""
    profile = load_capability_profile()
    return profile.get("max_fps", 30)


def get_profile_mode():
    """Get current profile mode."""
    profile = load_capability_profile()
    return profile.get("profile_mode", "mac")


class CapabilitySubscriber:
    """Helper class for nodes to subscribe to capability updates."""
    
    def __init__(self, node: Node):
        self.node = node
        self.profile = load_capability_profile()
        
        # Subscribe to capability updates
        self.subscription = node.create_subscription(
            String,
            "omega/capabilities",
            self.capability_callback,
            10
        )
    
    def capability_callback(self, msg: String):
        """Update profile when capabilities change."""
        try:
            self.profile = json.loads(msg.data)
            self.node.get_logger().debug(
                f"Capability profile updated: {self.profile['profile_mode']}"
            )
        except Exception as e:
            self.node.get_logger().warn(f"Failed to parse capability update: {e}")
    
    def is_ml_capable(self):
        """Check if ML is available."""
        return self.profile.get("ml_capable", False)
    
    def is_slam_capable(self):
        """Check if SLAM is available."""
        return self.profile.get("slam_capable", False)
    
    def get_max_resolution(self):
        """Get max resolution."""
        res = self.profile.get("max_resolution", "640x480")
        width, height = map(int, res.split("x"))
        return width, height
    
    def get_max_fps(self):
        """Get max FPS."""
        return self.profile.get("max_fps", 30)
    
    def get_profile_mode(self):
        """Get profile mode."""
        return self.profile.get("profile_mode", "mac")

