"""
CameraManager: Intelligent camera backend detection and selection.

Implements CAMERA_PATCH_BLUEPRINT for deterministic camera backend selection.
"""

import os
import platform
import subprocess
import logging
from typing import Optional, Dict, List, Tuple
from enum import Enum

# Color codes for terminal output
class Colors:
    """ANSI color codes for logging."""
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'

log = logging.getLogger(__name__)


class BackendType(Enum):
    """Camera backend types."""
    PICAMERA2 = "picamera2"
    V4L2 = "v4l2"
    MOCK = "mock"


class HardwareType(Enum):
    """Hardware platform types."""
    RASPBERRY_PI = "raspberry_pi"
    JETSON = "jetson"
    LINUX = "linux"
    MACOS = "macos"
    UNKNOWN = "unknown"


class CameraManager:
    """
    Intelligent camera backend detection and selection manager.
    
    Implements deterministic detection strategy:
    1. OS/Hardware detection
    2. Device scanning (/dev/video*)
    3. Driver detection (unicam vs uvcvideo)
    4. Backend selection with fallbacks
    """
    
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        """
        Initialize CameraManager.
        
        Args:
            width: Camera width
            height: Camera height
            fps: Target FPS
        """
        self.width = width
        self.height = height
        self.fps = fps
        
        self.hardware_type: Optional[HardwareType] = None
        self.backend_type: Optional[BackendType] = None
        self.camera_instance = None
        self.device_info: Dict = {}
        
        self.debug = os.getenv("OMEGA_DEBUG_CAMERA", "0").lower() in ("1", "true", "yes")
        
        # Detect hardware
        self.hardware_type = self._detect_hardware()
        
        # Scan devices
        self.device_info = self._scan_devices()
        
        # Select backend
        self.backend_type = self._select_backend()
        
        # Log detection results
        self._log_detection_results()
    
    def _detect_hardware(self) -> HardwareType:
        """
        Step 1: Detect machine type using platform + CPU info.
        
        Returns:
            HardwareType enum
        """
        system = platform.system().lower()
        
        # Check for Raspberry Pi
        try:
            if os.path.exists("/proc/device-tree/model"):
                with open("/proc/device-tree/model", "r") as f:
                    model = f.read().strip()
                    if "Raspberry Pi" in model:
                        return HardwareType.RASPBERRY_PI
                    elif "NVIDIA" in model or "Jetson" in model:
                        return HardwareType.JETSON
        except Exception:
            pass
        
        # Check CPU info for Pi
        try:
            if os.path.exists("/proc/cpuinfo"):
                with open("/proc/cpuinfo", "r") as f:
                    cpuinfo = f.read()
                    if "BCM2711" in cpuinfo or "BCM2835" in cpuinfo:
                        return HardwareType.RASPBERRY_PI
        except Exception:
            pass
        
        # Check OS
        if system == "darwin":
            return HardwareType.MACOS
        elif system == "linux":
            return HardwareType.LINUX
        
        return HardwareType.UNKNOWN
    
    def _scan_devices(self) -> Dict:
        """
        Step 2: Scan /dev/video* devices and detect drivers.
        
        Returns:
            Dictionary with device information
        """
        info = {
            "devices": [],
            "uvc_devices": [],
            "unicam_devices": [],
            "device_count": 0,
        }
        
        # List /dev/video* devices
        video_devices = []
        for i in range(10):  # Check up to /dev/video9
            device_path = f"/dev/video{i}"
            if os.path.exists(device_path):
                video_devices.append(device_path)
        
        info["devices"] = video_devices
        info["device_count"] = len(video_devices)
        
        # Check each device for driver type
        for device_path in video_devices:
            driver_info = self._get_device_driver(device_path)
            if driver_info:
                if driver_info.get("driver") == "uvcvideo":
                    info["uvc_devices"].append({
                        "path": device_path,
                        "driver": "uvcvideo",
                        "name": driver_info.get("name", "Unknown USB Camera"),
                    })
                elif driver_info.get("driver") == "unicam":
                    info["unicam_devices"].append({
                        "path": device_path,
                        "driver": "unicam",
                        "name": driver_info.get("name", "Raspberry Pi Camera"),
                    })
        
        return info
    
    def _get_device_driver(self, device_path: str) -> Optional[Dict]:
        """
        Get driver information for a device using v4l2-ctl.
        
        Args:
            device_path: Path to device (e.g., /dev/video0)
            
        Returns:
            Dictionary with driver info or None
        """
        try:
            # Try v4l2-ctl to get device info
            result = subprocess.run(
                ["v4l2-ctl", "--device", device_path, "--all"],
                capture_output=True,
                text=True,
                timeout=2,
            )
            
            if result.returncode == 0:
                output = result.stdout.lower()
                driver_info = {}
                
                # Check for UVC driver
                if "uvcvideo" in output or "driver name.*uvcvideo" in output:
                    driver_info["driver"] = "uvcvideo"
                # Check for unicam (libcamera)
                elif "unicam" in output or "driver name.*unicam" in output:
                    driver_info["driver"] = "unicam"
                else:
                    # Try to extract driver name
                    for line in result.stdout.split("\n"):
                        if "driver name" in line.lower():
                            driver_name = line.split(":")[-1].strip()
                            driver_info["driver"] = driver_name
                            break
                
                # Extract device name
                for line in result.stdout.split("\n"):
                    if "card" in line.lower() and ":" in line:
                        name = line.split(":")[-1].strip()
                        driver_info["name"] = name
                        break
                
                return driver_info if driver_info.get("driver") else None
        except (subprocess.TimeoutExpired, FileNotFoundError, Exception):
            # v4l2-ctl not available or device not accessible
            pass
        
        # Fallback: check sysfs
        try:
            device_num = device_path.replace("/dev/video", "")
            sysfs_path = f"/sys/class/video4linux/video{device_num}/name"
            if os.path.exists(sysfs_path):
                with open(sysfs_path, "r") as f:
                    name = f.read().strip()
                    # Heuristic: unicam devices often have "camera" in name
                    if "camera" in name.lower() and self.hardware_type == HardwareType.RASPBERRY_PI:
                        return {"driver": "unicam", "name": name}
                    # UVC devices often have manufacturer/model names
                    return {"driver": "uvcvideo", "name": name}
        except Exception:
            pass
        
        return None
    
    def _select_backend(self) -> BackendType:
        """
        Step 3-5: Select backend based on detection results.
        
        Returns:
            BackendType enum
        """
        # Check for forced backend
        force_picamera2 = os.getenv("OMEGA_FORCE_PICAMERA2", "0").lower() in ("1", "true", "yes")
        force_v4l2 = os.getenv("OMEGA_FORCE_V4L2", "0").lower() in ("1", "true", "yes")
        
        if force_picamera2:
            if self._can_use_picamera2():
                return BackendType.PICAMERA2
            else:
                log.warning(f"{Colors.YELLOW}⚠️  OMEGA_FORCE_PICAMERA2 set but Picamera2 not available{Colors.RESET}")
        
        if force_v4l2:
            if self._can_use_v4l2():
                return BackendType.V4L2
            else:
                log.warning(f"{Colors.YELLOW}⚠️  OMEGA_FORCE_V4L2 set but V4L2 not available{Colors.RESET}")
        
        # Detection-based selection
        
        # Priority 1: Picamera2 for ribbon cameras (unicam)
        if self.device_info.get("unicam_devices"):
            if self._can_use_picamera2():
                return BackendType.PICAMERA2
        
        # Priority 2: Picamera2 on Raspberry Pi (even if no device detected yet)
        if self.hardware_type == HardwareType.RASPBERRY_PI:
            if self._can_use_picamera2():
                return BackendType.PICAMERA2
        
        # Priority 3: V4L2 for USB cameras (uvcvideo)
        if self.device_info.get("uvc_devices"):
            if self._can_use_v4l2():
                return BackendType.V4L2
        
        # Priority 4: V4L2 fallback if devices exist
        if self.device_info.get("device_count", 0) > 0:
            if self._can_use_v4l2():
                return BackendType.V4L2
        
        # Priority 5: Mock camera
        return BackendType.MOCK
    
    def _can_use_picamera2(self) -> bool:
        """Check if Picamera2 is available."""
        try:
            from picamera2 import Picamera2
            return True
        except ImportError:
            return False
    
    def _can_use_v4l2(self) -> bool:
        """Check if V4L2/OpenCV is available."""
        try:
            import cv2
            return cv2 is not None
        except ImportError:
            return False
    
    def _log_detection_results(self):
        """Log detection results with colored output."""
        hw_name = self.hardware_type.value if self.hardware_type else "unknown"
        backend_name = self.backend_type.value if self.backend_type else "unknown"
        
        log.info(f"{Colors.CYAN}{'='*60}{Colors.RESET}")
        log.info(f"{Colors.BOLD}📷 Camera Detection Results{Colors.RESET}")
        log.info(f"{Colors.CYAN}{'='*60}{Colors.RESET}")
        log.info(f"{Colors.BLUE}Hardware:{Colors.RESET} {Colors.GREEN}{hw_name}{Colors.RESET}")
        log.info(f"{Colors.BLUE}Backend:{Colors.RESET} {Colors.GREEN}{backend_name}{Colors.RESET}")
        log.info(f"{Colors.BLUE}Devices found:{Colors.RESET} {Colors.YELLOW}{self.device_info.get('device_count', 0)}{Colors.RESET}")
        
        if self.device_info.get("unicam_devices"):
            log.info(f"{Colors.BLUE}Ribbon cameras:{Colors.RESET} {Colors.GREEN}{len(self.device_info['unicam_devices'])}{Colors.RESET}")
            for dev in self.device_info["unicam_devices"]:
                log.info(f"  • {Colors.GREEN}{dev['path']}{Colors.RESET} ({dev.get('name', 'Unknown')})")
        
        if self.device_info.get("uvc_devices"):
            log.info(f"{Colors.BLUE}USB cameras:{Colors.RESET} {Colors.GREEN}{len(self.device_info['uvc_devices'])}{Colors.RESET}")
            for dev in self.device_info["uvc_devices"]:
                log.info(f"  • {Colors.GREEN}{dev['path']}{Colors.RESET} ({dev.get('name', 'Unknown')})")
        
        if self.debug:
            log.info(f"{Colors.MAGENTA}Debug: All devices:{Colors.RESET}")
            for dev in self.device_info.get("devices", []):
                driver_info = self._get_device_driver(dev)
                log.info(f"  • {dev}: {driver_info}")
        
        log.info(f"{Colors.CYAN}{'='*60}{Colors.RESET}")
    
    def initialize_backend(self):
        """
        Initialize the selected camera backend.
        
        Returns:
            Camera instance
        """
        if self.backend_type == BackendType.PICAMERA2:
            # Import here to avoid circular imports
            try:
                from .camera import _PiCam2Backend
            except ImportError:
                # Fallback for different import paths
                import sys
                import os
                sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
                from video.camera import _PiCam2Backend
            
            self.camera_instance = _PiCam2Backend(
                width=self.width,
                height=self.height,
                target_fps=self.fps,
            )
        elif self.backend_type == BackendType.V4L2:
            # Import here to avoid circular imports
            try:
                from .camera import _V4L2Backend
            except ImportError:
                # Fallback for different import paths
                import sys
                import os
                sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
                from video.camera import _V4L2Backend
            
            # Use first UVC device if available, else first device
            device = "/dev/video0"
            if self.device_info.get("uvc_devices"):
                device = self.device_info["uvc_devices"][0]["path"]
            elif self.device_info.get("devices"):
                device = self.device_info["devices"][0]
            
            self.camera_instance = _V4L2Backend(
                device=device,
                width=self.width,
                height=self.height,
                target_fps=self.fps,
            )
        elif self.backend_type == BackendType.MOCK:
            try:
                from .mock_camera_server import MockCamera
            except ImportError:
                # Fallback for different import paths
                import sys
                import os
                sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
                from video.mock_camera_server import MockCamera
            
            self.camera_instance = MockCamera(
                width=self.width,
                height=self.height,
                fps=self.fps,
            )
            self.camera_instance.start()
        
        return self.camera_instance

