"""
Mock Orin AI Brain Server
Simulates Jetson Orin Nano behavior for testing without hardware
"""

import asyncio
import json
import uuid
import time
import random
from typing import Dict, Any, Optional
from dataclasses import dataclass
import logging

log = logging.getLogger(__name__)


@dataclass
class MockInferenceResult:
    """Mock inference result."""
    detections: list
    frame_uuid: str
    inference_duration_ms: float
    timestamp: float


class FakeInference:
    """Fake inference generator with configurable delays."""
    
    def __init__(self, delay_mode: str = "fast"):
        """
        Initialize fake inference.
        
        Args:
            delay_mode: "fast" (15-80ms), "medium" (90-200ms), "slow" (200-500ms)
        """
        self.delay_mode = delay_mode
        self.delay_ranges = {
            "fast": (15, 80),
            "medium": (90, 200),
            "slow": (200, 500),
        }
    
    def generate_delay(self) -> float:
        """Generate random delay based on mode."""
        min_delay, max_delay = self.delay_ranges.get(self.delay_mode, (15, 80))
        return random.uniform(min_delay, max_delay) / 1000.0  # Convert to seconds
    
    async def process_frame(self, frame_uuid: str, frame_data: bytes) -> MockInferenceResult:
        """
        Process frame with fake inference.
        
        Args:
            frame_uuid: UUID of the frame
            frame_data: Frame data (not actually processed)
        
        Returns:
            Mock inference result
        """
        # Simulate inference delay
        delay = self.generate_delay()
        await asyncio.sleep(delay)
        
        # Generate fake detections
        detections = self._generate_fake_detections()
        
        return MockInferenceResult(
            detections=detections,
            frame_uuid=frame_uuid,
            inference_duration_ms=delay * 1000,
            timestamp=time.time()
        )
    
    def _generate_fake_detections(self) -> list:
        """Generate fake detection results."""
        # Randomly generate 0-3 detections
        num_detections = random.randint(0, 3)
        detections = []
        
        for _ in range(num_detections):
            detections.append({
                "class": random.choice(["person", "car", "bicycle", "dog"]),
                "confidence": random.uniform(0.5, 0.95),
                "bbox": [
                    random.randint(0, 500),
                    random.randint(0, 400),
                    random.randint(100, 600),
                    random.randint(100, 500),
                ],
            })
        
        return detections


class OrinMockServer:
    """Mock Orin AI Brain server."""
    
    def __init__(self, delay_mode: str = "fast", simulate_failures: bool = False):
        """
        Initialize mock Orin server.
        
        Args:
            delay_mode: Inference delay mode
            simulate_failures: Whether to simulate failures
        """
        self.delay_mode = delay_mode
        self.simulate_failures = simulate_failures
        self.inference = FakeInference(delay_mode)
        self.processed_frames = 0
        self.failed_frames = 0
        self.last_frame_uuid: Optional[str] = None
        
    async def handle_frame(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handle incoming frame message.
        
        Args:
            message: Frame message from Pi
        
        Returns:
            Detection result message
        """
        try:
            # Extract UUID
            frame_uuid = message.get("frame_uuid") or message.get("uuid")
            if not frame_uuid:
                frame_uuid = str(uuid.uuid4())
            
            self.last_frame_uuid = frame_uuid
            
            # Simulate failures if enabled
            if self.simulate_failures and random.random() < 0.1:  # 10% failure rate
                self.failed_frames += 1
                raise Exception("Simulated Orin failure")
            
            # Process frame
            frame_data = message.get("frame_data", b"")
            result = await self.inference.process_frame(frame_uuid, frame_data)
            
            self.processed_frames += 1
            
            # Return detection result
            return {
                "ok": True,
                "type": "detection",
                "frame_uuid": result.frame_uuid,
                "detections": result.detections,
                "inference_duration_ms": result.inference_duration_ms,
                "timestamp": result.timestamp,
            }
            
        except Exception as e:
            log.error(f"Mock Orin error: {e}")
            self.failed_frames += 1
            return {
                "ok": False,
                "error": str(e),
                "frame_uuid": frame_uuid if 'frame_uuid' in locals() else None,
            }
    
    async def echo_uuid(self, frame_uuid: str) -> Dict[str, Any]:
        """
        Echo UUID back (for latency testing).
        
        Args:
            frame_uuid: UUID to echo
        
        Returns:
            Echo response
        """
        # Simulate processing delay
        await asyncio.sleep(0.01)  # 10ms
        
        return {
            "ok": True,
            "type": "uuid_echo",
            "frame_uuid": frame_uuid,
            "timestamp": time.time(),
        }
    
    def get_stats(self) -> Dict[str, Any]:
        """Get mock server statistics."""
        total = self.processed_frames + self.failed_frames
        success_rate = self.processed_frames / total if total > 0 else 0.0
        
        return {
            "processed_frames": self.processed_frames,
            "failed_frames": self.failed_frames,
            "success_rate": success_rate,
            "delay_mode": self.delay_mode,
            "last_frame_uuid": self.last_frame_uuid,
        }


# Example usage
async def main():
    """Example usage of mock Orin server."""
    server = OrinMockServer(delay_mode="fast")
    
    # Simulate frame processing
    test_message = {
        "frame_uuid": str(uuid.uuid4()),
        "frame_data": b"fake_frame_data",
    }
    
    result = await server.handle_frame(test_message)
    print(f"Result: {result}")
    print(f"Stats: {server.get_stats()}")


if __name__ == "__main__":
    asyncio.run(main())

