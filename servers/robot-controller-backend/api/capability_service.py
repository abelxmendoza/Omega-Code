"""
Capability Detection Service for Backend

Detects system capabilities and provides them to the API and frontend.
Integrates with ROS2 capability detector if available, otherwise detects locally.
"""

import os
import platform
import subprocess
import json
import tempfile
import logging
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)


class CapabilityService:
    """Service for detecting and managing system capabilities."""
    
    def __init__(self):
        self._cached_profile: Optional[Dict[str, Any]] = None
        self._profile_file = os.path.join(tempfile.gettempdir(), "omega_capabilities.json")
    
    def detect_capabilities(self) -> Dict[str, Any]:
        """Detect system capabilities."""
        # Try to load from ROS2 profile file first
        if os.path.exists(self._profile_file):
            try:
                with open(self._profile_file, "r") as f:
                    profile = json.load(f)
                    self._cached_profile = profile
                    logger.info(f"Loaded capability profile from ROS2: {profile.get('profile_mode')}")
                    return profile
            except Exception as e:
                logger.warning(f"Failed to load ROS2 profile: {e}")
        
        # Fallback: detect locally
        return self._detect_local()
    
    def _detect_local(self) -> Dict[str, Any]:
        """Detect capabilities locally (without ROS2)."""
        profile = {
            "device": platform.node(),
            "hostname": platform.node(),
            "arch": platform.machine(),
            "os": platform.system(),
            "os_version": platform.version(),
            "is_jetson": self._is_jetson(),
            "cuda": self._check_cuda(),
            "ros2_dev": self._check_ros_tools(),
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
        
        # Jetson mode
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
                "gpu_name": self._get_gpu_name(),
            })
            logger.info("✅ Jetson Orin Nano detected - Full autonomy mode")
        
        # Lenovo Linux mode
        elif profile["ros2_dev"] and platform.system() == "Linux" and not profile["is_jetson"]:
            profile.update({
                "slam_capable": True,
                "ml_capable": False,
                "face_recognition": True,  # CPU-based, slow
                "yolo": False,
                "max_resolution": "1280x720",
                "max_fps": 25,
                "profile_mode": "lenovo",
            })
            logger.info("✅ Lenovo Linux detected - Dev mode")
        
        # MacBook mode
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
            logger.info("✅ MacBook/Light system detected - Light mode")
        
        self._cached_profile = profile
        return profile
    
    def _is_jetson(self) -> bool:
        """Check if running on Jetson hardware."""
        try:
            if os.path.exists("/proc/device-tree/model"):
                with open("/proc/device-tree/model", "r") as f:
                    model = f.read()
                    if "NVIDIA" in model or "Jetson" in model:
                        return True
            
            if os.path.exists("/sys/devices/soc0/family"):
                with open("/sys/devices/soc0/family", "r") as f:
                    if "Tegra" in f.read():
                        return True
            
            if os.getenv("JETSON") or os.getenv("JETSON_MODEL"):
                return True
        except Exception:
            pass
        return False
    
    def _check_cuda(self) -> bool:
        """Check if CUDA is available."""
        # Check nvcc
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
    
    def _check_ros_tools(self) -> bool:
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
    
    def _get_gpu_name(self) -> Optional[str]:
        """Get GPU name if available."""
        try:
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
        
        try:
            import torch
            if torch.cuda.is_available():
                return torch.cuda.get_device_name(0)
        except Exception:
            pass
        
        return None
    
    def get_capabilities(self, refresh: bool = False) -> Dict[str, Any]:
        """Get current capability profile."""
        if refresh or self._cached_profile is None:
            self._cached_profile = self.detect_capabilities()
        return self._cached_profile.copy()
    
    def is_ml_capable(self) -> bool:
        """Check if ML/GPU processing is available."""
        profile = self.get_capabilities()
        return profile.get("ml_capable", False)
    
    def is_slam_capable(self) -> bool:
        """Check if SLAM is available."""
        profile = self.get_capabilities()
        return profile.get("slam_capable", False)
    
    def get_max_resolution(self) -> tuple[int, int]:
        """Get maximum supported resolution."""
        profile = self.get_capabilities()
        res = profile.get("max_resolution", "640x480")
        width, height = map(int, res.split("x"))
        return width, height
    
    def get_max_fps(self) -> int:
        """Get maximum supported FPS."""
        profile = self.get_capabilities()
        return profile.get("max_fps", 30)
    
    def get_profile_mode(self) -> str:
        """Get current profile mode."""
        profile = self.get_capabilities()
        return profile.get("profile_mode", "mac")


# Singleton instance
_capability_service: Optional[CapabilityService] = None


def get_capability_service() -> CapabilityService:
    """Get singleton capability service instance."""
    global _capability_service
    if _capability_service is None:
        _capability_service = CapabilityService()
    return _capability_service

