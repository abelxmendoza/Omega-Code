"""
Frame Overlay Module
Adds overlays to video frames (timestamp, FPS, telemetry, etc.)
"""

import time
import logging
from typing import Optional, Dict, Any
import numpy as np

# High-precision timestamp for latency measurement
try:
    from time import time_ns
except ImportError:
    # Fallback for Python < 3.7
    def time_ns():
        return int(time.time() * 1e9)

try:
    import cv2  # type: ignore
except ImportError:
    cv2 = None

log = logging.getLogger(__name__)


class FrameOverlay:
    """
    Adds overlays to video frames.
    Hardware-aware rendering for optimal performance.
    """
    
    def __init__(
        self,
        show_timestamp: bool = True,
        show_fps: bool = True,
        show_telemetry: bool = False,
        telemetry_data: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize frame overlay.
        
        Args:
            show_timestamp: Show timestamp overlay
            show_fps: Show FPS counter
            show_telemetry: Show telemetry data (battery, temp, etc.)
            telemetry_data: Dict with telemetry values
        """
        self.show_timestamp = show_timestamp
        self.show_fps = show_fps
        self.show_telemetry = show_telemetry
        self.telemetry_data = telemetry_data or {}
        
        self._fps_history = []
        self._last_fps_time = time.time()
        self._frame_count = 0
        self._current_fps = 0.0
        
        # Latency measurement timestamps
        self._capture_timestamp_ns: Optional[int] = None
        self._encode_start_ns: Optional[int] = None
        self._encode_end_ns: Optional[int] = None
    
    def update_fps(self):
        """Update FPS calculation."""
        self._frame_count += 1
        now = time.time()
        elapsed = now - self._last_fps_time
        
        if elapsed >= 1.0:
            self._current_fps = self._frame_count / elapsed
            self._fps_history.append(self._current_fps)
            if len(self._fps_history) > 10:
                self._fps_history.pop(0)
            self._frame_count = 0
            self._last_fps_time = now
    
    def add_overlays(self, frame: np.ndarray, capture_timestamp_ns: Optional[int] = None) -> np.ndarray:
        """
        Add overlays to frame.
        
        Args:
            frame: BGR frame
            capture_timestamp_ns: High-precision capture timestamp (nanoseconds) for latency measurement
            
        Returns:
            Frame with overlays
        """
        if cv2 is None or frame is None:
            return frame
        
        # Store capture timestamp for latency measurement
        if capture_timestamp_ns is not None:
            self._capture_timestamp_ns = capture_timestamp_ns
        else:
            self._capture_timestamp_ns = time_ns()
        
        try:
            h, w = frame.shape[:2]
            y_offset = 10
            
            # Timestamp overlay (with high-precision timestamp embedded)
            if self.show_timestamp:
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                # Embed high-precision timestamp in frame (bottom right corner, small)
                timestamp_ns_str = str(self._capture_timestamp_ns)
                cv2.putText(
                    frame, timestamp,
                    (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                )
                # Embed timestamp in bottom-right corner (very small, for extraction)
                cv2.putText(
                    frame, f"TS:{timestamp_ns_str[-12:]}",  # Last 12 digits for compactness
                    (w - 150, h - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.3,
                    (64, 64, 64),  # Very subtle
                    1,
                    cv2.LINE_AA
                )
                y_offset += 25
            
            # FPS overlay
            if self.show_fps:
                self.update_fps()
                fps_text = f"FPS: {self._current_fps:.1f}"
                # Color based on FPS
                color = (0, 255, 0) if self._current_fps >= 25 else (0, 165, 255) if self._current_fps >= 15 else (0, 0, 255)
                cv2.putText(
                    frame, fps_text,
                    (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                    cv2.LINE_AA
                )
                y_offset += 25
            
            # Telemetry overlay
            if self.show_telemetry and self.telemetry_data:
                # Battery
                if "battery" in self.telemetry_data:
                    battery = self.telemetry_data["battery"]
                    battery_text = f"Battery: {battery:.1f}%"
                    battery_color = (0, 255, 0) if battery > 50 else (0, 165, 255) if battery > 20 else (0, 0, 255)
                    cv2.putText(
                        frame, battery_text,
                        (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        battery_color,
                        1,
                        cv2.LINE_AA
                    )
                    y_offset += 25
                
                # Temperature
                if "temperature" in self.telemetry_data:
                    temp = self.telemetry_data["temperature"]
                    temp_text = f"Temp: {temp:.1f}°C"
                    temp_color = (0, 255, 0) if temp < 60 else (0, 165, 255) if temp < 80 else (0, 0, 255)
                    cv2.putText(
                        frame, temp_text,
                        (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        temp_color,
                        1,
                        cv2.LINE_AA
                    )
                    y_offset += 25
                
                # CPU usage
                if "cpu_usage" in self.telemetry_data:
                    cpu = self.telemetry_data["cpu_usage"]
                    cpu_text = f"CPU: {cpu:.1f}%"
                    cv2.putText(
                        frame, cpu_text,
                        (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA
                    )
                    y_offset += 25
            
            # Corner watermark (bottom right)
            watermark = "Omega Robot"
            (text_width, text_height), _ = cv2.getTextSize(
                watermark, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1
            )
            cv2.putText(
                frame, watermark,
                (w - text_width - 10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (128, 128, 128),
                1,
                cv2.LINE_AA
            )
            
        except Exception as e:
            log.warning(f"Overlay error: {e}")
        
        return frame
    
    def update_telemetry(self, telemetry: Dict[str, Any]):
        """Update telemetry data."""
        self.telemetry_data.update(telemetry)
    
    def get_fps(self) -> float:
        """Get current FPS."""
        return self._current_fps
    
    def set_encode_timestamps(self, start_ns: int, end_ns: int):
        """
        Set encode timestamps for latency measurement.
        
        Args:
            start_ns: Encode start time (nanoseconds)
            end_ns: Encode end time (nanoseconds)
        """
        self._encode_start_ns = start_ns
        self._encode_end_ns = end_ns
    
    def get_latency_metrics(self) -> Dict[str, Optional[int]]:
        """
        Get latency metrics.
        
        Returns:
            Dict with capture, encode start, encode end timestamps (nanoseconds)
        """
        return {
            "capture_timestamp_ns": self._capture_timestamp_ns,
            "encode_start_ns": self._encode_start_ns,
            "encode_end_ns": self._encode_end_ns,
        }

