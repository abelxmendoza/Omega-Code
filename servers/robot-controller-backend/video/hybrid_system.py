"""
Hybrid Vision System Integration

Manages the distributed compute architecture: Pi (sensor hub) + Orin (AI brain).
Detects system mode (Pi-only vs Pi+Orin) and coordinates communication.
"""

import os
import logging
import time
import threading
from typing import Optional, Dict, Any, List
from enum import Enum

log = logging.getLogger(__name__)


class SystemMode(Enum):
    """System operation modes."""
    PI_ONLY = "pi_only"  # Pi-only mode (current hardware)
    PI_ORIN_HYBRID = "pi_orin_hybrid"  # Pi + Orin hybrid mode
    ORIN_ONLY = "orin_only"  # Orin-only mode (future)


class HybridSystemManager:
    """Manages hybrid vision system configuration and communication."""
    
    def __init__(self):
        """Initialize hybrid system manager."""
        self.system_mode = self._detect_system_mode()
        self.orin_available = False
        self.pi_sensor_hub = None
        self.thermal_monitor = ThermalMonitor()
        self.cpu_monitor = CPULoadMonitor()
        
        # Manual mode override (0-7 system modes)
        self._manual_mode: Optional[int] = None
        self._lock = threading.Lock()
        
        # Initialize Pi sensor hub if ROS2 available
        if self.system_mode == SystemMode.PI_ONLY or self.system_mode == SystemMode.PI_ORIN_HYBRID:
            self._init_pi_sensor_hub()
        
        log.info(f"✅ Hybrid System Manager initialized: {self.system_mode.value}")
    
    def _detect_system_mode(self) -> SystemMode:
        """
        Detect system mode based on hardware and environment.
        
        Returns:
            SystemMode enum
        """
        # Check for Orin availability (NVMe installed is activation condition)
        orin_available = self._check_orin_availability()
        
        # Check if running on Pi
        is_pi = self._is_raspberry_pi()
        
        if orin_available and is_pi:
            return SystemMode.PI_ORIN_HYBRID
        elif is_pi:
            return SystemMode.PI_ONLY
        else:
            # Could be Orin-only in future
            return SystemMode.PI_ONLY  # Default for now
    
    def _is_raspberry_pi(self) -> bool:
        """Check if running on Raspberry Pi."""
        try:
            if os.path.exists("/proc/device-tree/model"):
                with open("/proc/device-tree/model", "r") as f:
                    model = f.read().strip()
                    return "Raspberry Pi" in model
            elif os.path.exists("/proc/cpuinfo"):
                with open("/proc/cpuinfo", "r") as f:
                    cpuinfo = f.read()
                    return "BCM2711" in cpuinfo or "Raspberry Pi" in cpuinfo
        except Exception:
            pass
        return False
    
    def _check_orin_availability(self) -> bool:
        """
        Check if Jetson Orin Nano is available.
        Activation condition: NVMe installed (as per blueprint).
        """
        # Check for NVMe device (activation condition)
        try:
            if os.path.exists("/dev/nvme0n1"):
                self.orin_available = True
                return True
        except Exception:
            pass
        
        # Also check for Jetson hardware
        try:
            if os.path.exists("/proc/device-tree/model"):
                with open("/proc/device-tree/model", "r") as f:
                    model = f.read().strip()
                    if "Jetson" in model or "Orin" in model:
                        self.orin_available = True
                        return True
        except Exception:
            pass
        
        # Check environment variable override
        if os.getenv("OMEGA_ORIN_AVAILABLE", "0").lower() in ("1", "true", "yes"):
            self.orin_available = True
            return True
        
        return False
    
    def _init_pi_sensor_hub(self):
        """Initialize Pi sensor hub ROS2 node."""
        try:
            from .pi_sensor_hub import init_pi_sensor_hub, get_pi_sensor_hub
            
            self.pi_sensor_hub = init_pi_sensor_hub()
            if self.pi_sensor_hub:
                log.info("✅ Pi Sensor Hub initialized")
            else:
                log.warning("⚠️ Pi Sensor Hub not available (ROS2 not available)")
        except Exception as e:
            log.warning(f"⚠️ Failed to initialize Pi Sensor Hub: {e}")
            self.pi_sensor_hub = None
    
    def get_system_mode(self) -> SystemMode:
        """Get current system mode."""
        return self.system_mode
    
    def is_hybrid_mode(self) -> bool:
        """Check if running in hybrid mode (Pi + Orin)."""
        return self.system_mode == SystemMode.PI_ORIN_HYBRID
    
    def set_manual_mode(self, mode: int):
        """
        Set manual mode override (0-7 system modes).
        
        Args:
            mode: System mode number (0-7)
        """
        if mode < 0 or mode > 7:
            log.error(f"Invalid manual mode: {mode}. Must be 0-7")
            return
        
        with self._lock:
            self._manual_mode = mode
            log.info(f"Manual mode override set to: {mode}")
    
    def clear_manual_mode(self):
        """Clear manual mode override."""
        with self._lock:
            self._manual_mode = None
            log.info("Manual mode override cleared")
    
    def get_effective_mode(self) -> Optional[int]:
        """
        Get effective mode (manual override or None for auto-detected).
        
        Returns:
            Manual mode (0-7) if set, None otherwise
        """
        with self._lock:
            return self._manual_mode
    
    def get_manual_mode(self) -> Optional[int]:
        """Get current manual mode override."""
        return self.get_effective_mode()
    
    def get_pi_capabilities(self) -> Dict[str, Any]:
        """Get Pi capabilities based on system mode."""
        capabilities = {
            "mjpeg_streaming": True,
            "motion_detection": True,
            "kcf_tracking": True,
            "aruco_detection": True,
            "face_detection_light": True,  # Haar only
            "frame_overlays": True,
            "frame_buffering": True,
            "video_recording_xvid": True,
            "ros2_micro_nodes": True,
            "deep_learning_models": False,  # Disabled by default
            "highres_streaming": False,  # 720p/1080p disabled
        }
        
        # In hybrid mode, Pi focuses on sensor hub role
        if self.is_hybrid_mode():
            capabilities.update({
                "deep_learning_models": False,  # Orin handles this
                "highres_streaming": False,  # Keep at 640x480
            })
        
        return capabilities
    
    def should_throttle_modules(self) -> bool:
        """Check if modules should be throttled due to thermal/CPU load."""
        return self.thermal_monitor.should_throttle() or self.cpu_monitor.should_throttle()
    
    def get_throttle_priority(self) -> List[str]:
        """Get module throttle priority order."""
        return ["motion", "tracking", "aruco", "face_detection"]
    
    def publish_frame_to_orin(self, frame, quality: int = 75):
        """Publish compressed frame to Orin."""
        if self.pi_sensor_hub and self.is_hybrid_mode():
            try:
                self.pi_sensor_hub.publish_compressed_frame(frame, quality)
            except Exception as e:
                log.error(f"Failed to publish frame to Orin: {e}", exc_info=True)
    
    def publish_aruco_markers(self, markers: List[Any]):
        """Publish ArUco markers to Orin."""
        if self.pi_sensor_hub and self.is_hybrid_mode():
            try:
                self.pi_sensor_hub.publish_aruco_markers(markers)
            except Exception as e:
                log.error(f"Failed to publish ArUco markers: {e}", exc_info=True)
    
    def publish_tracking_bbox(self, bbox: Any):
        """Publish tracking bbox to Orin."""
        if self.pi_sensor_hub and self.is_hybrid_mode():
            try:
                self.pi_sensor_hub.publish_tracking_bbox(bbox)
            except Exception as e:
                log.error(f"Failed to publish tracking bbox: {e}", exc_info=True)
    
    def publish_motion_event(self, detected: bool, regions: List[Any] = None):
        """Publish motion event to Orin."""
        if self.pi_sensor_hub and self.is_hybrid_mode():
            try:
                self.pi_sensor_hub.publish_motion_event(detected, regions)
            except Exception as e:
                log.error(f"Failed to publish motion event: {e}", exc_info=True)
    
    def publish_telemetry(self, telemetry: Any):
        """Publish telemetry to Orin."""
        if self.pi_sensor_hub and self.is_hybrid_mode():
            try:
                self.pi_sensor_hub.publish_telemetry(telemetry)
            except Exception as e:
                log.error(f"Failed to publish telemetry: {e}", exc_info=True)


class ThermalMonitor:
    """Monitors thermal conditions and triggers throttling."""
    
    def __init__(self, max_temp: float = 70.0):
        """
        Initialize thermal monitor.
        
        Args:
            max_temp: Maximum temperature in Celsius before throttling
        """
        self.max_temp = max_temp
        self.current_temp = 0.0
        self._monitoring = False
        self._monitor_thread = None
    
    def start_monitoring(self):
        """Start thermal monitoring thread."""
        if self._monitoring:
            return
        
        self._monitoring = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
    
    def _monitor_loop(self):
        """Monitor thermal conditions."""
        while self._monitoring:
            try:
                self.current_temp = self._read_temperature()
                time.sleep(2.0)  # Check every 2 seconds
            except Exception:
                time.sleep(5.0)
    
    def _read_temperature(self) -> float:
        """Read CPU temperature."""
        try:
            # Raspberry Pi thermal path
            if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                    temp_millidegrees = int(f.read().strip())
                    return temp_millidegrees / 1000.0
            
            # Jetson thermal path
            if os.path.exists("/sys/devices/virtual/thermal/thermal_zone0/temp"):
                with open("/sys/devices/virtual/thermal/thermal_zone0/temp", "r") as f:
                    temp_millidegrees = int(f.read().strip())
                    return temp_millidegrees / 1000.0
        except Exception:
            pass
        
        return 0.0
    
    def should_throttle(self) -> bool:
        """Check if throttling is needed."""
        return self.current_temp > self.max_temp
    
    def get_temperature(self) -> float:
        """Get current temperature."""
        return self.current_temp


class CPULoadMonitor:
    """Monitors CPU load and triggers throttling."""
    
    def __init__(self, throttle_threshold: float = 75.0):
        """
        Initialize CPU load monitor.
        
        Args:
            throttle_threshold: CPU percentage threshold for throttling
        """
        self.throttle_threshold = throttle_threshold
        self.current_load = 0.0
        
        try:
            import psutil
            self.psutil = psutil
        except ImportError:
            self.psutil = None
            log.warning("psutil not available, CPU monitoring disabled")
    
    def update_load(self):
        """Update current CPU load."""
        if self.psutil:
            try:
                self.current_load = self.psutil.cpu_percent(interval=0.1)
            except Exception:
                pass
    
    def should_throttle(self) -> bool:
        """Check if throttling is needed."""
        return self.current_load > self.throttle_threshold
    
    def get_load(self) -> float:
        """Get current CPU load."""
        return self.current_load


# Global instance
_hybrid_system_manager: Optional[HybridSystemManager] = None


def get_hybrid_system_manager() -> HybridSystemManager:
    """Get global hybrid system manager instance."""
    global _hybrid_system_manager
    
    if _hybrid_system_manager is None:
        _hybrid_system_manager = HybridSystemManager()
        # Start thermal monitoring
        _hybrid_system_manager.thermal_monitor.start_monitoring()
    
    return _hybrid_system_manager

