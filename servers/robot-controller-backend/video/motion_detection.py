"""
📌 File: video/motion_detection.py

💪 Summary:
This module implements motion detection for a video stream using OpenCV.
It compares consecutive frames to identify motion and highlights areas where movement occurs.

🛠️ Features:
- Detects motion by analyzing frame differences.
- Highlights detected motion with bounding boxes.
- Hardware-aware sensitivity adjustments.
- Performance optimized for different platforms.
"""

import os
import logging
import warnings
from typing import Optional, Tuple

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    warnings.warn("OpenCV not installed. Motion detection disabled.", ImportWarning)

import numpy as np

log = logging.getLogger(__name__)


def _detect_hardware() -> dict:
    """Detect hardware for optimization."""
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


class MotionDetector:
    """
    Motion detector with hardware-aware optimizations.
    
    Automatically adjusts sensitivity and processing based on detected hardware.
    """
    
    def __init__(self, sensitivity: Optional[int] = None, min_area: Optional[int] = None):
        """
        Initialize motion detector.
        
        Args:
            sensitivity: Threshold for motion detection (lower = more sensitive).
                        Auto-adjusted based on hardware if None.
            min_area: Minimum contour area to consider as motion.
                     Auto-adjusted based on hardware if None.
        """
        self.previous_frame: Optional[np.ndarray] = None
        
        # Hardware-aware defaults
        if sensitivity is None:
            if _hardware["is_pi4b"]:
                sensitivity = 30  # Slightly less sensitive for Pi 4B
            elif _hardware["is_jetson"]:
                sensitivity = 20  # More sensitive on Jetson
            else:
                sensitivity = 25  # Default
        
        if min_area is None:
            if _hardware["is_pi4b"]:
                min_area = 800  # Larger area threshold for Pi
            elif _hardware["is_jetson"]:
                min_area = 400  # Smaller area for Jetson
            else:
                min_area = 500  # Default
        
        self.sensitivity = int(sensitivity)
        self.min_area = int(min_area)
        self._frame_count = 0
        self._motion_count = 0
        
        # Hardware-aware blur kernel size
        if _hardware["is_pi4b"]:
            self._blur_kernel = (15, 15)  # Smaller kernel for Pi
        else:
            self._blur_kernel = (21, 21)  # Default
        
        log.info(f"Motion detector initialized: sensitivity={self.sensitivity}, min_area={self.min_area}, "
                f"hardware={'Pi4B' if _hardware['is_pi4b'] else 'Jetson' if _hardware['is_jetson'] else 'Other'}")

    def detect_motion(self, frame: Optional[np.ndarray]) -> Tuple[np.ndarray, bool]:
        """
        Detect motion by comparing current frame with the previous one.
        
        Args:
            frame: BGR frame from camera
            
        Returns:
            Tuple of (annotated_frame, motion_detected)
        """
        if cv2 is None:
            return frame if frame is not None else np.zeros((480, 640, 3), dtype=np.uint8), False
        
        if frame is None:
            return np.zeros((480, 640, 3), dtype=np.uint8), False
        
        try:
            # Ensure the frame has the correct dimensions
            if len(frame.shape) < 3 or frame.shape[2] != 3:
                log.warning("Invalid frame format, skipping motion detection.")
                return frame, False

            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur to reduce noise (hardware-aware kernel)
            gray = cv2.GaussianBlur(gray, self._blur_kernel, 0)

            if self.previous_frame is None:
                self.previous_frame = gray
                self._frame_count += 1
                return frame, False

            # Compute the absolute difference between frames
            delta_frame = cv2.absdiff(self.previous_frame, gray)
            self.previous_frame = gray  # Update previous frame

            # Apply threshold to get regions with motion
            _, threshold_frame = cv2.threshold(
                delta_frame, self.sensitivity, 255, cv2.THRESH_BINARY
            )
            
            # Dilate to fill holes
            threshold_frame = cv2.dilate(threshold_frame, None, iterations=2)

            # Find contours of moving objects
            contours, _ = cv2.findContours(
                threshold_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            motion_detected = False
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_area:
                    motion_detected = True
                    (x, y, w, h) = cv2.boundingRect(contour)
                    # Draw green box around movement
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Add area label for debugging (optional)
                    if _hardware["is_jetson"]:  # Only on Jetson for performance
                        cv2.putText(
                            frame, f"{int(area)}",
                            (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1
                        )

            self._frame_count += 1
            if motion_detected:
                self._motion_count += 1

            return frame, motion_detected
            
        except Exception as e:
            log.error(f"Motion detection error: {e}", exc_info=True)
            return frame, False
    
    def get_stats(self) -> dict:
        """Get motion detection statistics."""
        return {
            "frames_processed": self._frame_count,
            "motion_detected": self._motion_count,
            "motion_rate": self._motion_count / max(1, self._frame_count),
            "sensitivity": self.sensitivity,
            "min_area": self.min_area,
        }
