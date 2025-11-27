"""
Mock Orin Message Definitions
Message formats for Pi ↔ Orin communication
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import json
import uuid
import time


@dataclass
class ArUcoMarker:
    """ArUco marker detection."""
    id: int
    corners: List[List[float]]
    pose: Dict[str, float]


@dataclass
class TrackingBBox:
    """Tracking bounding box."""
    x: float
    y: float
    width: float
    height: float
    track_id: int
    confidence: float


@dataclass
class DetectionResult:
    """Detection result from Orin."""
    frame_uuid: str
    detections: List[Dict[str, Any]]
    inference_duration_ms: float
    timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "ok": True,
            "type": "detection",
            "frame_uuid": self.frame_uuid,
            "detections": self.detections,
            "inference_duration_ms": self.inference_duration_ms,
            "timestamp": self.timestamp,
        }
    
    def to_json(self) -> str:
        """Convert to JSON."""
        return json.dumps(self.to_dict())


@dataclass
class FrameMessage:
    """Frame message from Pi to Orin."""
    frame_uuid: str
    frame_data: bytes
    timestamp: float
    width: Optional[int] = None
    height: Optional[int] = None
    compressed: bool = True
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "frame_uuid": self.frame_uuid,
            "frame_data": self.frame_data.hex() if isinstance(self.frame_data, bytes) else self.frame_data,
            "timestamp": self.timestamp,
            "width": self.width,
            "height": self.height,
            "compressed": self.compressed,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "FrameMessage":
        """Create from dictionary."""
        frame_data = data.get("frame_data")
        if isinstance(frame_data, str):
            frame_data = bytes.fromhex(frame_data)
        
        return cls(
            frame_uuid=data.get("frame_uuid", str(uuid.uuid4())),
            frame_data=frame_data or b"",
            timestamp=data.get("timestamp", time.time()),
            width=data.get("width"),
            height=data.get("height"),
            compressed=data.get("compressed", True),
        )


@dataclass
class UUIDEchoMessage:
    """UUID echo message for latency testing."""
    frame_uuid: str
    echo_timestamp: float
    original_timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "ok": True,
            "type": "uuid_echo",
            "frame_uuid": self.frame_uuid,
            "echo_timestamp": self.echo_timestamp,
            "original_timestamp": self.original_timestamp,
            "round_trip_ms": (self.echo_timestamp - self.original_timestamp) * 1000,
        }


@dataclass
class InferenceStatusMessage:
    """Inference status message."""
    frame_uuid: str
    inference_duration_ms: float
    status: str  # "success", "failure", "timeout"
    timestamp: float
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "ok": self.status == "success",
            "type": "inference_status",
            "frame_uuid": self.frame_uuid,
            "inference_duration_ms": self.inference_duration_ms,
            "status": self.status,
            "timestamp": self.timestamp,
        }

