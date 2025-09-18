"""
📌 Video Processing Package

This package contains:
✅ Camera module
✅ Motion detection
✅ Object tracking
"""

from .camera import Camera
from .face_recognition import FaceRecognizer
from .motion_detection import MotionDetector
from .object_tracking import ObjectTracker

__all__ = [
    "Camera",
    "FaceRecognizer",
    "MotionDetector",
    "ObjectTracker",
]
