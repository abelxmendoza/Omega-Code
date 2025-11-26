#!/usr/bin/env python3
"""
System Capability Detector Node for Omega Robot

Auto-detects system capabilities and publishes capability profile:
- Detects device type (MacBook, Lenovo Linux, Jetson Orin Nano)
- Checks for CUDA/GPU availability
- Checks for ROS2 development tools
- Determines maximum capabilities
- Publishes capability profile to /omega/capabilities topic
"""

import os
import platform
import subprocess
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tempfile


class CapabilityDetector(Node):
    """Detects system capabilities and publishes profile."""

    def __init__(self):
        super().__init__("capability_detector")
        
        # Publisher for capability profile
        self.pub = self.create_publisher(String, "omega/capabilities", 10)
        
        # Service to get current capabilities on demand
        self.declare_parameter('publish_interval', 5.0)
        self.publish_interval = self.get_parameter('publish_interval').value
        
        # Timer to periodically publish capabilities
        self.timer = self.create_timer(self.publish_interval, self.publish_capabilities)
        
        # Cache profile to avoid recomputing
        self.cached_profile = None
        
        # Detect immediately on startup
        self.detect_capabilities()
        
        self.get_logger().info("Capability detector started")
        self.get_logger().info(f"Detected profile: {self.cached_profile['profile_mode']}")

    def detect_capabilities(self):
        """Detect system capabilities and create profile."""
        profile = {
            "device": platform.node(),
            "hostname": platform.node(),
            "arch": platform.machine(),
            "os": platform.system(),
            "os_version": platform.version(),
            "is_jetson": self.is_jetson(),
            "cuda": self.check_cuda(),
            "ros2_dev": self.check_ros_tools(),
            "ml_capable": False,
            "slam_capable": False,
            "tracking": True,
            "aruco": True,
            "motion_detection": True,
            "face_recognition": False,
            "yolo": False,
            "max_resolution": "640x480",
            "max_fps": 30,
            "profile_mode": "unknown",
            "gpu_available": False,
            "gpu_name": None,
            "cpu_count": os.cpu_count() or 1,
        }

        # Jetson mode (Profile C - Omega Mode)
        if profile["is_jetson"] and profile["cuda"]:
            profile.update({
                "ml_capable": True,
                "slam_capable": True,
                "face_recognition": True,
                "yolo": True,
                "max_resolution": "1920x1080",
                "max_fps": 60,
                "profile_mode": "jetson",
                "gpu_available": True,
                "gpu_name": self.get_gpu_name(),
            })
            self.get_logger().info("✅ Jetson Orin Nano detected - Full autonomy mode enabled")

        # Lenovo Linux mode (Profile B - Dev Mode)
        elif profile["ros2_dev"] and platform.system() == "Linux" and not profile["is_jetson"]:
            profile.update({
                "slam_capable": True,
                "ml_capable": False,  # Can run CPU YOLO but slow
                "face_recognition": True,  # CPU-based, slow
                "yolo": False,  # Disabled by default (too slow)
                "max_resolution": "1280x720",
                "max_fps": 25,
                "profile_mode": "lenovo",
            })
            self.get_logger().info("✅ Lenovo Linux detected - Dev mode enabled")

        # MacBook mode (Profile A - Light Mode)
        else:
            profile.update({
                "slam_capable": False,
                "ml_capable": False,
                "face_recognition": False,
                "yolo": False,
                "max_resolution": "640x480",
                "max_fps": 20,
                "profile_mode": "mac",
            })
            self.get_logger().info("✅ MacBook/Light system detected - Light mode enabled")

        # Save profile to temp file for launch files
        self.save_profile(profile)
        
        self.cached_profile = profile
        return profile

    def is_jetson(self):
        """Check if running on Jetson hardware."""
        try:
            # Check device tree
            if os.path.exists("/proc/device-tree/model"):
                with open("/proc/device-tree/model", "r") as f:
                    model = f.read()
                    if "NVIDIA" in model or "Jetson" in model:
                        return True
            
            # Check for Tegra-specific files
            if os.path.exists("/sys/devices/soc0/family"):
                with open("/sys/devices/soc0/family", "r") as f:
                    if "Tegra" in f.read():
                        return True
            
            # Check environment variables
            if os.getenv("JETSON") or os.getenv("JETSON_MODEL"):
                return True
                
        except Exception as e:
            self.get_logger().debug(f"Error checking Jetson: {e}")
        
        return False

    def check_cuda(self):
        """Check if CUDA is available."""
        # Check nvcc compiler
        try:
            result = subprocess.run(
                ["which", "nvcc"],
                capture_output=True,
                timeout=1
            )
            if result.returncode == 0:
                return True
        except Exception:
            pass
        
        # Check PyTorch CUDA
        try:
            import torch
            if torch.cuda.is_available():
                return True
        except ImportError:
            pass
        
        # Check CUDA libraries
        try:
            result = subprocess.run(
                ["ldconfig", "-p"],
                capture_output=True,
                text=True,
                timeout=2
            )
            if "libcuda.so" in result.stdout or "libcudart.so" in result.stdout:
                return True
        except Exception:
            pass
        
        return False

    def check_ros_tools(self):
        """Check if ROS2 development tools are available."""
        try:
            result = subprocess.run(
                ["which", "ros2"],
                capture_output=True,
                timeout=1
            )
            return result.returncode == 0
        except Exception:
            return False

    def get_gpu_name(self):
        """Get GPU name if available."""
        try:
            # Try nvidia-smi
            result = subprocess.run(
                ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0 and result.stdout.strip():
                return result.stdout.strip()
        except Exception:
            pass
        
        # Try PyTorch
        try:
            import torch
            if torch.cuda.is_available():
                return torch.cuda.get_device_name(0)
        except Exception:
            pass
        
        return None

    def save_profile(self, profile):
        """Save profile to temp file for launch files to read."""
        try:
            temp_file = os.path.join(tempfile.gettempdir(), "omega_capabilities.json")
            with open(temp_file, "w") as f:
                json.dump(profile, f, indent=2)
            self.get_logger().debug(f"Saved capability profile to {temp_file}")
        except Exception as e:
            self.get_logger().warn(f"Failed to save profile: {e}")

    def publish_capabilities(self):
        """Publish current capability profile."""
        if self.cached_profile is None:
            self.detect_capabilities()
        
        msg = String()
        msg.data = json.dumps(self.cached_profile)
        self.pub.publish(msg)
        
        self.get_logger().debug(
            f"Published capabilities: {self.cached_profile['profile_mode']} "
            f"(ML: {self.cached_profile['ml_capable']}, "
            f"SLAM: {self.cached_profile['slam_capable']})"
        )

    def get_capabilities(self):
        """Get current capability profile (for other nodes to query)."""
        if self.cached_profile is None:
            self.detect_capabilities()
        return self.cached_profile.copy()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = CapabilityDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

