"""
📌 Video Processing Package

This package contains:
✅ Camera module
✅ Motion detection
✅ Object tracking
"""

from .aruco_detection import ArucoDetection, ArucoDetector
from .camera import Camera
from .face_recognition import FaceRecognizer
from .motion_detection import MotionDetector
from .object_tracking import ObjectTracker

__all__ = [
    "ArucoDetection",
    "ArucoDetector",
    "Camera",
    "FaceRecognizer",
    "MotionDetector",
    "ObjectTracker",
]
