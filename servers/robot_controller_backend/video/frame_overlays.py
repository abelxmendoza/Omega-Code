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
        self._distance_cm: Optional[float] = None
        
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

    def update_distance(self, distance_cm: Optional[float]) -> None:
        """Store latest distance reading for the obstacle overlay."""
        self._distance_cm = distance_cm
    
    def set_encode_timestamps(self, start_ns: int, end_ns: int):
        """
        Set encode timestamps for latency measurement.
        
        Args:
            start_ns: Encode start time (nanoseconds)
            end_ns: Encode end time (nanoseconds)
        """
        self._encode_start_ns = start_ns
        self._encode_end_ns = end_ns
    
    def add_obstacle_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Render distance reading onto frame using stored _distance_cm."""
        return add_obstacle_overlay(frame, self._distance_cm)

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


def draw_obstacle_visualization(frame: np.ndarray, distance_cm: Optional[float]) -> np.ndarray:
    """
    Mode 8 full obstacle visualization:
    - Central box sized inversely to distance (closer → bigger)
    - Color-coded: green (clear) / yellow (near) / orange (caution) / red (danger)
    - Red border flash around entire frame when obstacle < 20 cm
    - Distance label inside box, status label above
    """
    if cv2 is None or frame is None:
        return frame

    h, w = frame.shape[:2]

    if distance_cm is None:
        cv2.putText(frame, "SENSOR OFFLINE", (w // 2 - 130, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (128, 128, 128), 2, cv2.LINE_AA)
        cv2.putText(frame, "OBSTACLE DETECT", (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1, cv2.LINE_AA)
        return frame

    # Threshold-based color and status
    if distance_cm < 20:
        color = (0, 0, 255)      # red
        status = "DANGER"
        border_width = 8
    elif distance_cm < 40:
        color = (0, 100, 255)    # orange
        status = "CAUTION"
        border_width = 0
    elif distance_cm < 60:
        color = (0, 255, 255)    # yellow
        status = "NEAR"
        border_width = 0
    else:
        color = (0, 255, 0)      # green
        status = "CLEAR"
        border_width = 0

    # Full-frame border flash on danger
    if border_width:
        cv2.rectangle(frame, (2, 2), (w - 2, h - 2), color, border_width)

    # Central warning box — grows as obstacle approaches
    max_dist = 200.0
    frac = max(0.0, 1.0 - (min(distance_cm, max_dist) / max_dist))
    min_frac, max_frac = 0.06, 0.72
    box_frac = min_frac + (max_frac - min_frac) * frac
    bw = int(w * box_frac)
    bh = int(h * box_frac)
    cx, cy = w // 2, h // 2
    x1, y1 = cx - bw // 2, cy - bh // 2
    x2, y2 = cx + bw // 2, cy + bh // 2
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

    # Distance text centred inside box
    dist_text = f"{distance_cm:.0f} cm"
    fs = 1.6
    (tw, th), _ = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, fs, 3)
    cv2.putText(frame, dist_text, (cx - tw // 2, cy + th // 2),
                cv2.FONT_HERSHEY_SIMPLEX, fs, color, 3, cv2.LINE_AA)

    # Status label above box
    cv2.putText(frame, status, (max(x1, 4), max(y1 - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)

    # Corner HUD tag
    cv2.putText(frame, "OBSTACLE DETECT", (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    return frame


def add_obstacle_overlay(frame: np.ndarray, distance_cm: Optional[float]) -> np.ndarray:
    """
    Stamp the current ultrasonic distance onto the frame.

    Colors:
      green  → distance >= 60 cm (clear)
      yellow → 30–59 cm (caution)
      red    → < 30 cm (obstacle)
    """
    if cv2 is None or frame is None or distance_cm is None:
        return frame

    text = f"{distance_cm:.1f} cm"
    if distance_cm < 30:
        color = (0, 0, 255)    # red
    elif distance_cm < 60:
        color = (0, 255, 255)  # yellow
    else:
        color = (0, 255, 0)    # green

    cv2.putText(
        frame, text, (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0, color, 2, cv2.LINE_AA,
    )
    return frame
