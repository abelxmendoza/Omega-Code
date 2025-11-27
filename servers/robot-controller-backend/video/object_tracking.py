"""
📌 Object Tracking for Raspberry Pi Camera

✅ Tracks an object in real-time using OpenCV's tracking algorithms
✅ Highlights the object with a bounding box
✅ Hardware-aware tracker selection for optimal performance
✅ Allows tracking a manually selected region
"""

import os
import logging
import warnings
from typing import Optional, Tuple

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    warnings.warn("OpenCV not installed. Object tracking disabled.", ImportWarning)

import numpy as np

log = logging.getLogger(__name__)


def _detect_hardware() -> dict:
    """Detect hardware for tracker optimization."""
    is_pi4b = False
    is_jetson = False
    
    try:
        if os.path.exists("/proc/device-tree/model"):
            with open("/proc/device-tree/model", "r") as f:
                model = f.read().strip()
                if "Raspberry Pi 4" in model:
                    is_pi4b = True
                elif "NVIDIA" in model or "Jetson" in model:
                    is_jetson = True
    except Exception:
        pass
    
    return {"is_pi4b": is_pi4b, "is_jetson": is_jetson}


_hardware = _detect_hardware()


class ObjectTracker:
    """
    Object tracker with hardware-aware algorithm selection.
    
    Automatically selects the best tracker based on available hardware.
    """
    
    def __init__(self, tracker_type: Optional[str] = None):
        """
        Initialize object tracker.
        
        Args:
            tracker_type: Tracker type (CSRT, KCF, MIL, MOSSE).
                         Auto-selected based on hardware if None.
        """
        if cv2 is None:
            warnings.warn("OpenCV not available. ObjectTracker disabled.", RuntimeWarning)
            self.tracker = None
            self.tracker_type = None
        else:
            # Hardware-aware default selection
            if tracker_type is None:
                if _hardware["is_jetson"]:
                    tracker_type = "CSRT"  # Best quality on Jetson
                elif _hardware["is_pi4b"]:
                    tracker_type = "KCF"  # Faster on Pi 4B
                else:
                    tracker_type = "KCF"  # Balanced default
            
            # Check which trackers are actually available
            OPENCV_OBJECT_TRACKERS = {}
            
            # Check CSRT
            if hasattr(cv2, 'TrackerCSRT_create'):
                try:
                    OPENCV_OBJECT_TRACKERS["CSRT"] = cv2.TrackerCSRT_create
                except AttributeError:
                    pass
            
            # Check KCF
            if hasattr(cv2, 'TrackerKCF_create'):
                try:
                    OPENCV_OBJECT_TRACKERS["KCF"] = cv2.TrackerKCF_create
                except AttributeError:
                    pass
            
            # Check MIL
            if hasattr(cv2, 'TrackerMIL_create'):
                try:
                    OPENCV_OBJECT_TRACKERS["MIL"] = cv2.TrackerMIL_create
                except AttributeError:
                    pass
            
            # Check MOSSE (legacy)
            if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerMOSSE_create'):
                try:
                    OPENCV_OBJECT_TRACKERS["MOSSE"] = cv2.legacy.TrackerMOSSE_create
                except AttributeError:
                    pass
            
            # If requested tracker not available, try fallbacks
            if tracker_type not in OPENCV_OBJECT_TRACKERS:
                log.warning(f"Tracker '{tracker_type}' not available, trying fallbacks...")
                # Try KCF first (most common)
                if "KCF" in OPENCV_OBJECT_TRACKERS:
                    tracker_type = "KCF"
                elif "MOSSE" in OPENCV_OBJECT_TRACKERS:
                    tracker_type = "MOSSE"
                elif "MIL" in OPENCV_OBJECT_TRACKERS:
                    tracker_type = "MIL"
                elif "CSRT" in OPENCV_OBJECT_TRACKERS:
                    tracker_type = "CSRT"
                else:
                    log.warning("No OpenCV trackers available. Object tracking disabled.")
                    self.tracker = None
                    self.tracker_type = None
                    return
            
            tracker_factory = OPENCV_OBJECT_TRACKERS.get(tracker_type)
            if tracker_factory is None:
                log.warning(f"Tracker '{tracker_type}' factory not available. Object tracking disabled.")
                self.tracker = None
                self.tracker_type = None
                return
            
            try:
                self.tracker = tracker_factory()
                self.tracker_type = tracker_type
                log.info(f"Object tracker initialized: {tracker_type} "
                        f"(hardware={'Pi4B' if _hardware['is_pi4b'] else 'Jetson' if _hardware['is_jetson'] else 'Other'})")
            except Exception as e:
                log.error(f"Failed to create tracker '{tracker_type}': {e}")
                # Try fallback trackers
                for fallback_type in ["KCF", "MOSSE", "MIL", "CSRT"]:
                    if fallback_type in OPENCV_OBJECT_TRACKERS and fallback_type != tracker_type:
                        try:
                            self.tracker = OPENCV_OBJECT_TRACKERS[fallback_type]()
                            self.tracker_type = fallback_type
                            log.info(f"Fallback tracker '{fallback_type}' initialized successfully")
                            return
                        except Exception:
                            continue
                # If all trackers fail, disable tracking
                log.warning("All tracker initialization attempts failed. Object tracking disabled.")
                self.tracker = None
                self.tracker_type = None
        
        self.bounding_box: Optional[Tuple[int, int, int, int]] = None
        self.tracking = False
        self._success_count = 0
        self._failure_count = 0

    def start_tracking(self, frame: np.ndarray, bounding_box: Tuple[int, int, int, int]) -> bool:
        """
        Initialize tracking with a selected bounding box.
        
        Args:
            frame: Initial frame
            bounding_box: (x, y, width, height) of object to track
            
        Returns:
            True if tracking started successfully
        """
        if self.tracker is None or frame is None:
            return False
        
        try:
            self.bounding_box = bounding_box
            self.tracking = self.tracker.init(frame, bounding_box)
            if self.tracking:
                log.info(f"Tracking started with {self.tracker_type} tracker")
            return self.tracking
        except Exception as e:
            log.error(f"Failed to start tracking: {e}")
            self.tracking = False
            return False

    def update_tracking(self, frame: Optional[np.ndarray]) -> Tuple[np.ndarray, bool]:
        """
        Update object tracking and draw bounding box.
        
        Args:
            frame: Current frame
            
        Returns:
            Tuple of (annotated_frame, tracking_success)
        """
        if self.tracker is None or not self.tracking or frame is None:
            return frame if frame is not None else np.zeros((480, 640, 3), dtype=np.uint8), False

        try:
            success, box = self.tracker.update(frame)
            
            if success:
                (x, y, w, h) = [int(v) for v in box]
                
                # Validate bounding box
                if w > 0 and h > 0 and x >= 0 and y >= 0:
                    # Draw blue bounding box
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    
                    # Add tracker type label (only on Jetson for performance)
                    if _hardware["is_jetson"]:
                        cv2.putText(
                            frame, f"{self.tracker_type}",
                            (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 0, 0),
                            1
                        )
                    
                    self._success_count += 1
                    return frame, True
                else:
                    # Invalid bounding box
                    self.tracking = False
                    self._failure_count += 1
                    log.warning("Invalid bounding box, stopping tracking")
                    return frame, False
            else:
                self.tracking = False
                self._failure_count += 1
                log.debug("Tracking lost")
                return frame, False
                
        except Exception as e:
            log.error(f"Tracking update error: {e}", exc_info=True)
            self.tracking = False
            self._failure_count += 1
            return frame, False
    
    def get_stats(self) -> dict:
        """Get tracking statistics."""
        total = self._success_count + self._failure_count
        return {
            "tracker_type": self.tracker_type,
            "is_tracking": self.tracking,
            "success_count": self._success_count,
            "failure_count": self._failure_count,
            "success_rate": self._success_count / max(1, total),
        }

