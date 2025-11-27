"""
Video Recording Module
Records video streams to MP4 files with hardware-aware encoding.
"""

import os
import time
import logging
import threading
from pathlib import Path
from typing import Optional, Dict, Any
from queue import Queue, Empty
import numpy as np

try:
    import cv2  # type: ignore
except ImportError:
    cv2 = None

log = logging.getLogger(__name__)


def _detect_hardware() -> dict:
    """Detect hardware for encoding optimization."""
    is_pi4b = False
    is_jetson = False
    
    try:
        if os.path.exists("/proc/device-tree/model"):
            with open("/proc/device-tree/model", "r") as f:
                model = f.read().strip()
                if "Raspberry Pi 4" in model:
                    is_pi4b = True
                elif "NVIDIA" in model or "Jetson" in model:
                    is_jetson = True
    except Exception:
        pass
    
    return {"is_pi4b": is_pi4b, "is_jetson": is_jetson}


_hardware = _detect_hardware()


class VideoRecorder:
    """
    Records video frames to MP4 files.
    Hardware-aware encoding for optimal performance.
    """
    
    def __init__(
        self,
        output_dir: Optional[str] = None,
        max_file_size_mb: int = 100,
        max_files: int = 10,
    ):
        """
        Initialize video recorder.
        
        Args:
            output_dir: Directory to save recordings (default: /tmp/omega_recordings)
            max_file_size_mb: Maximum file size before rotation (MB)
            max_files: Maximum number of files to keep
        """
        if cv2 is None:
            raise RuntimeError("OpenCV required for video recording")
        
        self.output_dir = Path(output_dir or os.getenv("RECORDING_DIR", "/tmp/omega_recordings"))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.max_file_size_bytes = max_file_size_mb * 1024 * 1024
        self.max_files = max_files
        
        self.is_recording = False
        self._writer: Optional[cv2.VideoWriter] = None
        self._frame_queue: Queue = Queue(maxsize=30)
        self._thread: Optional[threading.Thread] = None
        self._current_file: Optional[Path] = None
        self._frames_written = 0
        self._start_time = 0.0
        
        # Hardware-aware codec and settings
        if _hardware["is_jetson"]:
            self._fourcc = cv2.VideoWriter_fourcc(*'H264')  # Hardware accelerated
            self._fps = 30
        elif _hardware["is_pi4b"]:
            self._fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Software, Pi-friendly
            self._fps = 25
        else:
            self._fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self._fps = 30
        
        log.info(f"Video recorder initialized: codec={self._fourcc}, fps={self._fps}, "
                f"hardware={'Pi4B' if _hardware['is_pi4b'] else 'Jetson' if _hardware['is_jetson'] else 'Other'}")
    
    def start_recording(self, width: int, height: int, filename: Optional[str] = None) -> str:
        """
        Start recording.
        
        Args:
            width: Frame width
            height: Frame height
            filename: Optional custom filename (auto-generated if None)
            
        Returns:
            Path to recording file
        """
        if self.is_recording:
            log.warning("Recording already in progress")
            return str(self._current_file) if self._current_file else ""
        
        if filename is None:
            timestamp = int(time.time())
            filename = f"recording_{timestamp}.mp4"
        
        filepath = self.output_dir / filename
        
        try:
            self._writer = cv2.VideoWriter(
                str(filepath),
                self._fourcc,
                self._fps,
                (width, height)
            )
            
            if not self._writer.isOpened():
                raise RuntimeError(f"Failed to open video writer: {filepath}")
            
            self._current_file = filepath
            self.is_recording = True
            self._frames_written = 0
            self._start_time = time.time()
            
            # Start recording thread
            self._thread = threading.Thread(target=self._recording_loop, daemon=True)
            self._thread.start()
            
            log.info(f"📹 Recording started: {filepath}")
            return str(filepath)
            
        except Exception as e:
            log.error(f"Failed to start recording: {e}")
            self.is_recording = False
            raise
    
    def stop_recording(self) -> Optional[str]:
        """Stop recording and return file path."""
        if not self.is_recording:
            return None
        
        self.is_recording = False
        
        # Wait for thread to finish
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        
        # Close writer
        if self._writer:
            self._writer.release()
            self._writer = None
        
        filepath = str(self._current_file) if self._current_file else None
        duration = time.time() - self._start_time if self._start_time > 0 else 0
        
        log.info(f"📹 Recording stopped: {filepath} ({self._frames_written} frames, {duration:.1f}s)")
        
        # Cleanup old files
        self._cleanup_old_files()
        
        return filepath
    
    def add_frame(self, frame: np.ndarray) -> bool:
        """
        Add frame to recording queue.
        
        Args:
            frame: BGR frame to record
            
        Returns:
            True if frame was queued successfully
        """
        if not self.is_recording or self._writer is None:
            return False
        
        try:
            # Non-blocking queue put
            self._frame_queue.put_nowait(frame)
            return True
        except Exception:
            # Queue full, drop frame
            return False
    
    def _recording_loop(self):
        """Background thread that writes frames to file."""
        while self.is_recording:
            try:
                # Get frame with timeout
                frame = self._frame_queue.get(timeout=0.1)
                
                if self._writer and frame is not None:
                    self._writer.write(frame)
                    self._frames_written += 1
                    
                    # Check file size and rotate if needed
                    if self._current_file and self._current_file.exists():
                        size = self._current_file.stat().st_size
                        if size > self.max_file_size_bytes:
                            log.info(f"File size limit reached, rotating: {self._current_file}")
                            self._rotate_file()
                
            except Empty:
                continue
            except Exception as e:
                log.error(f"Recording error: {e}", exc_info=True)
                time.sleep(0.1)
    
    def _rotate_file(self):
        """Rotate to new file when size limit reached."""
        if not self._current_file:
            return
        
        # Close current writer
        if self._writer:
            self._writer.release()
            self._writer = None
        
        # Create new file
        timestamp = int(time.time())
        new_filename = f"recording_{timestamp}.mp4"
        new_filepath = self.output_dir / new_filename
        
        # Get dimensions from current file (would need to track)
        # For now, reuse same dimensions
        width, height = 640, 480  # Default, should be tracked
        
        self._writer = cv2.VideoWriter(
            str(new_filepath),
            self._fourcc,
            self._fps,
            (width, height)
        )
        
        self._current_file = new_filepath
        log.info(f"Rotated to new file: {new_filepath}")
    
    def _cleanup_old_files(self):
        """Remove old files if max_files exceeded."""
        try:
            recordings = sorted(
                self.output_dir.glob("recording_*.mp4"),
                key=lambda p: p.stat().st_mtime,
                reverse=True
            )
            
            if len(recordings) > self.max_files:
                for old_file in recordings[self.max_files:]:
                    old_file.unlink()
                    log.info(f"Removed old recording: {old_file}")
        except Exception as e:
            log.warning(f"Failed to cleanup old files: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """Get recording status."""
        duration = time.time() - self._start_time if self._start_time > 0 else 0
        file_size_mb = 0
        if self._current_file and self._current_file.exists():
            file_size_mb = self._current_file.stat().st_size / (1024 * 1024)
        
        return {
            "is_recording": self.is_recording,
            "current_file": str(self._current_file) if self._current_file else None,
            "frames_written": self._frames_written,
            "duration_seconds": round(duration, 2),
            "file_size_mb": round(file_size_mb, 2),
            "queue_size": self._frame_queue.qsize(),
        }

