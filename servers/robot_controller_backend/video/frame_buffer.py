"""
Frame Buffer Module
Thread-safe frame buffering for smooth video streaming.
"""

import time
import threading
from collections import deque
from typing import Optional
import numpy as np

import logging

log = logging.getLogger(__name__)


class FrameBuffer:
    """
    Thread-safe frame buffer for video streaming.
    Maintains a queue of recent frames for smooth playback.
    """
    
    def __init__(self, max_size: int = 5):
        """
        Initialize frame buffer.
        
        Args:
            max_size: Maximum number of frames to buffer
        """
        self.max_size = max_size
        self._buffer: deque = deque(maxlen=max_size)
        self._lock = threading.Lock()
        self._last_frame_time = 0.0
        self._drops = 0
        self._total_frames = 0
    
    def add_frame(self, frame: Optional[np.ndarray]) -> bool:
        """
        Add frame to buffer.
        
        Args:
            frame: Frame to add (None frames are ignored)
            
        Returns:
            True if added, False if buffer full
        """
        if frame is None:
            return False
        
        with self._lock:
            if len(self._buffer) >= self.max_size:
                # Remove oldest frame
                self._buffer.popleft()
                self._drops += 1
            
            self._buffer.append((frame.copy(), time.time()))
            self._last_frame_time = time.time()
            self._total_frames += 1
            return True
    
    def get_latest(self) -> Optional[np.ndarray]:
        """
        Get latest frame from buffer.
        
        Returns:
            Latest frame or None if buffer empty
        """
        with self._lock:
            if not self._buffer:
                return None
            return self._buffer[-1][0].copy()
    
    def get_frame_at(self, index: int = -1) -> Optional[np.ndarray]:
        """
        Get frame at specific index.
        
        Args:
            index: Frame index (-1 = latest, -2 = previous, etc.)
            
        Returns:
            Frame or None if index out of range
        """
        with self._lock:
            if not self._buffer or abs(index) > len(self._buffer):
                return None
            return self._buffer[index][0].copy()
    
    def clear(self):
        """Clear buffer."""
        with self._lock:
            self._buffer.clear()
    
    def size(self) -> int:
        """Get current buffer size."""
        with self._lock:
            return len(self._buffer)
    
    def get_stats(self) -> dict:
        """Get buffer statistics."""
        with self._lock:
            return {
                "buffer_size": len(self._buffer),
                "max_size": self.max_size,
                "total_frames": self._total_frames,
                "drops": self._drops,
                "drop_rate": self._drops / max(1, self._total_frames),
                "last_frame_age": time.time() - self._last_frame_time if self._last_frame_time > 0 else 0,
            }

