#!/usr/bin/env python3
"""
System Capability Detector Node for Omega Robot

Auto-detects system capabilities and publishes capability profile:
- Detects device type (Raspberry Pi 4, Alienware Aurora, Jetson Orin Nano)
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
            "is_raspberry_pi": self.is_raspberry_pi(),
            "is_alienware": self.is_alienware(),
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
            "sim": True,
            "i2c": False,
            "gpio": False,
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
                "sim": False,
                "i2c": False,
                "gpio": False,
            })
            self.get_logger().info("✅ Jetson Orin Nano detected - Full autonomy mode enabled")

        # Raspberry Pi mode (Profile R - Hardware Mode)
        elif self.is_raspberry_pi():
            profile.update({
                "ml_capable": False,
                "slam_capable": False,
                "face_recognition": False,
                "yolo": False,
                "max_resolution": "640x480",
                "max_fps": 30,
                "profile_mode": "raspberry_pi",
                "sim": False,
                "i2c": True,
                "gpio": True,
            })
            self.get_logger().info("✅ Raspberry Pi 4 detected - Hardware mode enabled")

        # Alienware Aurora mode (Profile A - Dev Mode)
        elif self.is_alienware():
            profile.update({
                "ml_capable": True,
                "slam_capable": False,
                "face_recognition": False,
                "yolo": False,
                "max_resolution": "1280x720",
                "max_fps": 30,
                "profile_mode": "alienware",
                "gpu_available": True,
                "gpu_name": self.get_gpu_name(),
                "sim": True,
                "i2c": False,
                "gpio": False,
            })
            self.get_logger().info("✅ Alienware Aurora detected - Dev mode enabled")

        # Unknown Linux fallback
        else:
            profile.update({
                "slam_capable": False,
                "ml_capable": False,
                "face_recognition": False,
                "yolo": False,
                "max_resolution": "640x480",
                "max_fps": 20,
                "profile_mode": "unknown_linux",
                "sim": True,
                "i2c": False,
                "gpio": False,
            })
            self.get_logger().warning(
                f"Unknown Linux system (node={platform.node()}, arch={platform.machine()}) "
                "- sim mode enabled. Add detection in is_raspberry_pi() or is_alienware()."
            )

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

    def is_raspberry_pi(self):
        """Check if running on Raspberry Pi hardware."""
        try:
            # Primary: /proc/device-tree/model (most reliable on Pi)
            if os.path.exists("/proc/device-tree/model"):
                with open("/proc/device-tree/model", "r") as f:
                    if "Raspberry Pi" in f.read():
                        return True

            # /proc/cpuinfo Model line
            if os.path.exists("/proc/cpuinfo"):
                with open("/proc/cpuinfo", "r") as f:
                    for line in f:
                        if line.startswith("Model") and "Raspberry Pi" in line:
                            return True

            # Hostname set on omegaOne
            if platform.node() == "omegaOne":
                return True

            # aarch64 with raspi kernel
            if platform.machine() == "aarch64" and "raspi" in platform.version():
                return True

        except Exception as e:
            self.get_logger().debug(f"Error checking Raspberry Pi: {e}")

        return False

    def is_alienware(self):
        """Check if running on Alienware Aurora hardware (Scythe dev machine)."""
        try:
            # DMI chassis vendor
            vendor_path = "/sys/class/dmi/id/chassis_vendor"
            if os.path.exists(vendor_path):
                with open(vendor_path, "r") as f:
                    if "Alienware" in f.read():
                        return True

            # DMI product name
            product_path = "/sys/class/dmi/id/product_name"
            if os.path.exists(product_path):
                with open(product_path, "r") as f:
                    if "Alienware" in f.read():
                        return True

            # Hostname set on Scythe
            if platform.node() == "scythe":
                return True

        except Exception as e:
            self.get_logger().debug(f"Error checking Alienware: {e}")

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

