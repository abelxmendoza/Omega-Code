"""
Fake Inference Generator
Generates fake inference results with configurable delays
"""

import random
import time
from typing import List, Dict, Any
from .messages import DetectionResult


class FakeInferenceGenerator:
    """Generates fake inference results."""
    
    def __init__(self, delay_range_ms: tuple = (15, 80)):
        """
        Initialize fake inference generator.
        
        Args:
            delay_range_ms: Tuple of (min_delay_ms, max_delay_ms)
        """
        self.delay_range_ms = delay_range_ms
        self.detection_classes = [
            "person", "car", "bicycle", "dog", "cat",
            "truck", "bus", "motorcycle", "bird",
        ]
    
    def generate_inference(self, frame_uuid: str) -> DetectionResult:
        """
        Generate fake inference result.
        
        Args:
            frame_uuid: UUID of the frame
        
        Returns:
            Detection result
        """
        # Generate random delay
        delay_ms = random.uniform(*self.delay_range_ms)
        
        # Generate fake detections
        num_detections = random.randint(0, 3)
        detections = []
        
        for _ in range(num_detections):
            detections.append({
                "class": random.choice(self.detection_classes),
                "confidence": random.uniform(0.5, 0.95),
                "bbox": [
                    random.randint(0, 500),
                    random.randint(0, 400),
                    random.randint(100, 600),
                    random.randint(100, 500),
                ],
            })
        
        return DetectionResult(
            frame_uuid=frame_uuid,
            detections=detections,
            inference_duration_ms=delay_ms,
            timestamp=time.time()
        )
    
    def generate_fast_inference(self, frame_uuid: str) -> DetectionResult:
        """Generate fast inference (15-80ms)."""
        return self.generate_inference(frame_uuid)
    
    def generate_medium_inference(self, frame_uuid: str) -> DetectionResult:
        """Generate medium inference (90-200ms)."""
        original_range = self.delay_range_ms
        self.delay_range_ms = (90, 200)
        result = self.generate_inference(frame_uuid)
        self.delay_range_ms = original_range
        return result
    
    def generate_slow_inference(self, frame_uuid: str) -> DetectionResult:
        """Generate slow inference (200-500ms)."""
        original_range = self.delay_range_ms
        self.delay_range_ms = (200, 500)
        result = self.generate_inference(frame_uuid)
        self.delay_range_ms = original_range
        return result
    
    def generate_timeout(self, frame_uuid: str) -> DetectionResult:
        """Generate timeout result."""
        return DetectionResult(
            frame_uuid=frame_uuid,
            detections=[],
            inference_duration_ms=5000.0,  # 5 second timeout
            timestamp=time.time()
        )


# Preset generators
fast_inference = FakeInferenceGenerator((15, 80))
medium_inference = FakeInferenceGenerator((90, 200))
slow_inference = FakeInferenceGenerator((200, 500))

