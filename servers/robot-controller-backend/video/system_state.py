"""
System State Management

Manages current system mode state and provides thread-safe access.
"""

import threading
import logging
from typing import Optional, Dict, Any
from enum import IntEnum

log = logging.getLogger(__name__)


class SystemMode(IntEnum):
    """System operation modes (0-7)."""
    CAMERA_ONLY = 0
    MOTION_DETECTION = 1
    TRACKING = 2
    FACE_DETECTION = 3
    ARUCO_DETECTION = 4
    RECORDING_ONLY = 5
    ORIN_ENHANCED_DETECTION = 6
    ORIN_NAVIGATION_MODE = 7


# System mode descriptions
MODE_DESCRIPTIONS = {
    0: "Camera Only - Raw MJPEG stream without processing",
    1: "Motion Detection - Detect motion in video frames",
    2: "Tracking - Object tracking using KCF/CSRT",
    3: "Face Detection - Light face detection using Haar cascades",
    4: "ArUco Detection - Detect ArUco fiducial markers",
    5: "Recording Only - Record video without processing",
    6: "Orin-Enhanced Detection - Hybrid mode with YOLOv8 (requires Orin)",
    7: "Orin Navigation Mode - Full navigation with Orin AI brain (requires Orin)",
}


class SystemState:
    """Thread-safe system state manager."""
    
    def __init__(self):
        """Initialize system state."""
        self._lock = threading.Lock()
        self._current_mode: Optional[int] = None
        self._manual_override: bool = False
        self._mode_history: list = []
        self._max_history = 100
        
        # Initialize to default mode (Camera Only)
        self._current_mode = SystemMode.CAMERA_ONLY
    
    def get_current_mode(self) -> int:
        """Get current system mode."""
        with self._lock:
            return self._current_mode if self._current_mode is not None else SystemMode.CAMERA_ONLY
    
    def set_mode(self, mode: int, manual: bool = True) -> bool:
        """
        Set system mode.
        
        Args:
            mode: Mode number (0-7)
            manual: Whether this is a manual override
            
        Returns:
            True if mode was set successfully
        """
        if mode < 0 or mode > 7:
            log.error(f"Invalid mode: {mode}. Must be 0-7")
            return False
        
        with self._lock:
            old_mode = self._current_mode
            self._current_mode = mode
            self._manual_override = manual
            
            # Add to history
            self._mode_history.append({
                "mode": mode,
                "timestamp": self._get_timestamp(),
                "manual": manual,
                "previous_mode": old_mode
            })
            
            # Keep history size manageable
            if len(self._mode_history) > self._max_history:
                self._mode_history.pop(0)
            
            log.info(f"System mode changed: {old_mode} -> {mode} (manual={manual})")
            return True
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get current system status.
        
        Returns:
            Dict with mode, description, manual override status
        """
        with self._lock:
            mode = self._current_mode if self._current_mode is not None else SystemMode.CAMERA_ONLY
            return {
                "mode": mode,
                "description": MODE_DESCRIPTIONS.get(mode, "Unknown mode"),
                "manual_override": self._manual_override,
                "mode_name": SystemMode(mode).name if mode in SystemMode.__members__.values() else "UNKNOWN",
            }
    
    def list_modes(self) -> Dict[int, Dict[str, Any]]:
        """
        List all available modes.
        
        Returns:
            Dict mapping mode numbers to mode info
        """
        modes = {}
        for mode_num in range(8):
            modes[mode_num] = {
                "mode": mode_num,
                "name": SystemMode(mode_num).name if mode_num in SystemMode.__members__.values() else "UNKNOWN",
                "description": MODE_DESCRIPTIONS.get(mode_num, "Unknown mode"),
                "available": True,  # TODO: Check if mode is available based on hardware
            }
        return modes
    
    def get_mode_history(self, limit: int = 10) -> list:
        """Get recent mode change history."""
        with self._lock:
            return self._mode_history[-limit:]
    
    def is_manual_override(self) -> bool:
        """Check if current mode is manually overridden."""
        with self._lock:
            return self._manual_override
    
    def clear_manual_override(self):
        """Clear manual override (revert to auto-detected mode)."""
        with self._lock:
            self._manual_override = False
            log.info("Manual override cleared")
    
    def _get_timestamp(self) -> float:
        """Get current timestamp."""
        import time
        return time.time()


# Global system state instance
_system_state: Optional[SystemState] = None


def get_system_state() -> SystemState:
    """Get global system state instance."""
    global _system_state
    if _system_state is None:
        _system_state = SystemState()
    return _system_state

