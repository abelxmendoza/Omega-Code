"""
Custom message definitions for Omega Hybrid Vision System (Pi + Orin).

These messages are used for ROS2 communication between Pi (sensor hub) and Orin (AI brain).
"""

from dataclasses import dataclass, asdict
from typing import List, Optional, Dict, Any
import json
import time


@dataclass
class ArUcoMarker:
    """ArUco marker detection result."""
    marker_id: int
    corners: List[List[float]]  # 4 corners, each [x, y]
    centre: List[float]  # [x, y]
    pose: Optional[Dict[str, Any]] = None  # 6DOF pose if available


@dataclass
class TrackingBBox:
    """Object tracking bounding box."""
    x: int
    y: int
    width: int
    height: int
    tracker_id: Optional[int] = None
    confidence: float = 1.0
    label: Optional[str] = None


@dataclass
class MotionEvent:
    """Motion detection event."""
    detected: bool
    regions: List[TrackingBBox]  # Motion regions
    timestamp: float = 0.0
    
    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()


@dataclass
class DetectionResult:
    """High-level detection result from Orin AI brain."""
    label: str
    confidence: float
    bbox: TrackingBBox
    class_id: Optional[int] = None
    tracker_id: Optional[int] = None


@dataclass
class NavigationCommand:
    """Navigation command from Orin to Pi."""
    command_type: str  # "stop", "go", "target_bbox", "path_planning"
    target_bbox: Optional[TrackingBBox] = None
    path_points: Optional[List[List[float]]] = None  # [[x, y], ...]
    velocity: Optional[float] = None
    angular_velocity: Optional[float] = None


@dataclass
class TelemetryData:
    """Telemetry data from Pi."""
    cpu_temp: float
    battery_voltage: float
    battery_percentage: float
    cpu_usage: float
    memory_usage: float
    timestamp: float = 0.0
    
    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()


# Serialization helpers
def serialize_message(msg: Any) -> str:
    """Serialize a message to JSON string."""
    return json.dumps(asdict(msg))


def deserialize_message(json_str: str, msg_class: type) -> Any:
    """Deserialize JSON string to message object."""
    data = json.loads(json_str)
    return msg_class(**data)

