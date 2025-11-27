"""
Mock Orin AI Brain
Mock implementation for testing without Jetson Orin hardware
"""

from .orin_mock_server import OrinMockServer, FakeInference
from .messages import (
    DetectionResult,
    FrameMessage,
    UUIDEchoMessage,
    InferenceStatusMessage,
)
from .fake_inference import FakeInferenceGenerator

__all__ = [
    "OrinMockServer",
    "FakeInference",
    "DetectionResult",
    "FrameMessage",
    "UUIDEchoMessage",
    "InferenceStatusMessage",
    "FakeInferenceGenerator",
]

