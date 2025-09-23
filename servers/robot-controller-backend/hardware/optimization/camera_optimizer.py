"""
Camera Optimization for Robot Controller
- High-performance camera operations
- Frame rate optimization
- Memory management
- Image processing optimization
- Camera performance monitoring
"""

import time
import threading
import logging
import cv2
import numpy as np
from typing import Dict, List, Optional, Tuple, Any, Callable
from dataclasses import dataclass
from enum import Enum
import asyncio
from queue import Queue, Empty

logger = logging.getLogger(__name__)

class CameraBackend(Enum):
    """Camera backend types"""
    PICAMERA2 = "picamera2"
    OPENCV = "opencv"
    V4L2 = "v4l2"

@dataclass
class CameraConfig:
    """Camera configuration"""
    width: int = 640
    height: int = 480
    fps: int = 30
    backend: CameraBackend = CameraBackend.PICAMERA2
    format: str = "RGB888"
    quality: int = 85
    buffer_size: int = 3
    auto_exposure: bool = True
    auto_white_balance: bool = True
    brightness: float = 0.5
    contrast: float = 1.0
    saturation: float = 1.0

@dataclass
class FrameInfo:
    """Frame information"""
    timestamp: float
    frame_number: int
    size: Tuple[int, int]
    format: str
    processing_time: float
    quality_score: float

class CameraOptimizer:
    """High-performance camera system"""
    
    def __init__(self):
        self.config = CameraConfig()
        self.camera = None
        self.frame_queue = Queue(maxsize=self.config.buffer_size)
        self.running = False
        self.capture_thread = None
        self.processing_thread = None
        self.lock = threading.Lock()
        
        # Performance monitoring
        self.frame_count = 0
        self.dropped_frames = 0
        self.processing_times = []
        self.quality_scores = []
        
        # Frame callbacks
        self.frame_callbacks: List[Callable] = []
        
        # Image processing pipeline
        self.processing_pipeline = []
        
        # Memory management
        self.max_memory_usage = 100 * 1024 * 1024  # 100MB
        self.current_memory_usage = 0
    
    def configure(self, config: CameraConfig) -> bool:
        """Configure camera with optimization settings"""
        try:
            with self.lock:
                self.config = config
                
                # Initialize camera based on backend
                if config.backend == CameraBackend.PICAMERA2:
                    self._init_picamera2()
                elif config.backend == CameraBackend.OPENCV:
                    self._init_opencv()
                elif config.backend == CameraBackend.V4L2:
                    self._init_v4l2()
                else:
                    logger.error(f"Unsupported camera backend: {config.backend}")
                    return False
                
                logger.info(f"Camera configured: {config.width}x{config.height} @ {config.fps}fps")
                return True
                
        except Exception as e:
            logger.error(f"Failed to configure camera: {e}")
            return False
    
    def _init_picamera2(self):
        """Initialize PiCamera2 backend"""
        try:
            from picamera2 import Picamera2
            from libcamera import controls
            
            self.camera = Picamera2()
            
            # Configure camera
            camera_config = self.camera.create_video_configuration(
                main={"size": (self.config.width, self.config.height), 
                      "format": self.config.format},
                buffer_count=self.config.buffer_size
            )
            
            self.camera.configure(camera_config)
            
            # Set camera controls for performance
            if self.config.auto_exposure:
                self.camera.set_controls({"ExposureTime": 0})  # Auto exposure
            if self.config.auto_white_balance:
                self.camera.set_controls({"AwbMode": 0})  # Auto white balance
            
            # Set image quality
            self.camera.set_controls({
                "Brightness": self.config.brightness,
                "Contrast": self.config.contrast,
                "Saturation": self.config.saturation
            })
            
            logger.info("PiCamera2 initialized successfully")
            
        except ImportError:
            logger.warning("PiCamera2 not available, falling back to OpenCV")
            self._init_opencv()
        except Exception as e:
            logger.error(f"Failed to initialize PiCamera2: {e}")
            raise
    
    def _init_opencv(self):
        """Initialize OpenCV backend"""
        try:
            self.camera = cv2.VideoCapture(0)
            
            if not self.camera.isOpened():
                raise RuntimeError("Failed to open camera")
            
            # Set camera properties
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
            self.camera.set(cv2.CAP_PROP_FPS, self.config.fps)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, self.config.buffer_size)
            
            # Optimize for performance
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            
            logger.info("OpenCV camera initialized successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize OpenCV camera: {e}")
            raise
    
    def _init_v4l2(self):
        """Initialize V4L2 backend"""
        try:
            import v4l2capture
            
            self.camera = v4l2capture.Video_device("/dev/video0")
            self.camera.set_format(self.config.width, self.config.height, fourcc="MJPG")
            self.camera.set_fps(self.config.fps)
            
            logger.info("V4L2 camera initialized successfully")
            
        except ImportError:
            logger.warning("V4L2 not available, falling back to OpenCV")
            self._init_opencv()
        except Exception as e:
            logger.error(f"Failed to initialize V4L2 camera: {e}")
            raise
    
    def start(self) -> bool:
        """Start camera capture"""
        try:
            with self.lock:
                if self.running:
                    return True
                
                if not self.camera:
                    logger.error("Camera not initialized")
                    return False
                
                # Start camera
                if hasattr(self.camera, 'start'):
                    self.camera.start()
                
                self.running = True
                
                # Start capture thread
                self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
                self.capture_thread.start()
                
                # Start processing thread
                self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
                self.processing_thread.start()
                
                logger.info("Camera started successfully")
                return True
                
        except Exception as e:
            logger.error(f"Failed to start camera: {e}")
            return False
    
    def stop(self):
        """Stop camera capture"""
        with self.lock:
            if not self.running:
                return
            
            self.running = False
            
            # Stop camera
            if self.camera and hasattr(self.camera, 'stop'):
                self.camera.stop()
            
            # Wait for threads to finish
            if self.capture_thread:
                self.capture_thread.join(timeout=2.0)
            if self.processing_thread:
                self.processing_thread.join(timeout=2.0)
            
            logger.info("Camera stopped")
    
    def _capture_loop(self):
        """Camera capture loop"""
        while self.running:
            try:
                start_time = time.time()
                
                # Capture frame
                if self.config.backend == CameraBackend.PICAMERA2:
                    frame = self._capture_picamera2()
                elif self.config.backend == CameraBackend.OPENCV:
                    frame = self._capture_opencv()
                elif self.config.backend == CameraBackend.V4L2:
                    frame = self._capture_v4l2()
                else:
                    frame = None
                
                if frame is not None:
                    capture_time = time.time() - start_time
                    
                    # Create frame info
                    frame_info = FrameInfo(
                        timestamp=time.time(),
                        frame_number=self.frame_count,
                        size=frame.shape[:2],
                        format=self.config.format,
                        processing_time=capture_time,
                        quality_score=self._calculate_quality_score(frame)
                    )
                    
                    # Add to queue (non-blocking)
                    try:
                        self.frame_queue.put_nowait((frame, frame_info))
                        self.frame_count += 1
                    except:
                        self.dropped_frames += 1
                        logger.warning("Frame queue full, dropping frame")
                
                # Control frame rate
                target_interval = 1.0 / self.config.fps
                elapsed = time.time() - start_time
                if elapsed < target_interval:
                    time.sleep(target_interval - elapsed)
                
            except Exception as e:
                logger.error(f"Camera capture error: {e}")
                time.sleep(0.1)
    
    def _capture_picamera2(self) -> Optional[np.ndarray]:
        """Capture frame using PiCamera2"""
        try:
            if hasattr(self.camera, 'capture_array'):
                return self.camera.capture_array()
            return None
        except Exception as e:
            logger.error(f"PiCamera2 capture error: {e}")
            return None
    
    def _capture_opencv(self) -> Optional[np.ndarray]:
        """Capture frame using OpenCV"""
        try:
            ret, frame = self.camera.read()
            if ret:
                return frame
            return None
        except Exception as e:
            logger.error(f"OpenCV capture error: {e}")
            return None
    
    def _capture_v4l2(self) -> Optional[np.ndarray]:
        """Capture frame using V4L2"""
        try:
            if hasattr(self.camera, 'read'):
                return self.camera.read()
            return None
        except Exception as e:
            logger.error(f"V4L2 capture error: {e}")
            return None
    
    def _processing_loop(self):
        """Frame processing loop"""
        while self.running:
            try:
                # Get frame from queue
                try:
                    frame, frame_info = self.frame_queue.get(timeout=0.1)
                except Empty:
                    continue
                
                start_time = time.time()
                
                # Apply processing pipeline
                processed_frame = self._apply_processing_pipeline(frame)
                
                processing_time = time.time() - start_time
                frame_info.processing_time += processing_time
                
                # Call frame callbacks
                for callback in self.frame_callbacks:
                    try:
                        callback(processed_frame, frame_info)
                    except Exception as e:
                        logger.error(f"Frame callback error: {e}")
                
                # Update performance metrics
                self.processing_times.append(processing_time)
                self.quality_scores.append(frame_info.quality_score)
                
                # Keep only recent metrics
                if len(self.processing_times) > 100:
                    self.processing_times = self.processing_times[-50:]
                if len(self.quality_scores) > 100:
                    self.quality_scores = self.quality_scores[-50:]
                
                self.frame_queue.task_done()
                
            except Exception as e:
                logger.error(f"Frame processing error: {e}")
    
    def _apply_processing_pipeline(self, frame: np.ndarray) -> np.ndarray:
        """Apply image processing pipeline"""
        processed_frame = frame.copy()
        
        for processor in self.processing_pipeline:
            try:
                processed_frame = processor(processed_frame)
            except Exception as e:
                logger.error(f"Processing pipeline error: {e}")
        
        return processed_frame
    
    def _calculate_quality_score(self, frame: np.ndarray) -> float:
        """Calculate frame quality score"""
        try:
            # Convert to grayscale for analysis
            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            else:
                gray = frame
            
            # Calculate Laplacian variance (sharpness)
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            
            # Calculate brightness
            brightness = np.mean(gray)
            
            # Calculate contrast
            contrast = np.std(gray)
            
            # Combine metrics (normalized)
            sharpness_score = min(laplacian_var / 1000.0, 1.0)
            brightness_score = 1.0 - abs(brightness - 128) / 128.0
            contrast_score = min(contrast / 64.0, 1.0)
            
            quality_score = (sharpness_score + brightness_score + contrast_score) / 3.0
            return quality_score
            
        except Exception as e:
            logger.error(f"Quality score calculation error: {e}")
            return 0.5
    
    def add_frame_callback(self, callback: Callable):
        """Add frame callback"""
        self.frame_callbacks.append(callback)
    
    def remove_frame_callback(self, callback: Callable):
        """Remove frame callback"""
        if callback in self.frame_callbacks:
            self.frame_callbacks.remove(callback)
    
    def add_processor(self, processor: Callable):
        """Add image processor to pipeline"""
        self.processing_pipeline.append(processor)
    
    def remove_processor(self, processor: Callable):
        """Remove image processor from pipeline"""
        if processor in self.processing_pipeline:
            self.processing_pipeline.remove(processor)
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """Get camera performance statistics"""
        with self.lock:
            stats = {
                "frame_count": self.frame_count,
                "dropped_frames": self.dropped_frames,
                "frame_rate": self.frame_count / max(time.time() - getattr(self, 'start_time', time.time()), 1),
                "drop_rate": self.dropped_frames / max(self.frame_count + self.dropped_frames, 1),
                "queue_size": self.frame_queue.qsize(),
                "running": self.running
            }
            
            if self.processing_times:
                stats["avg_processing_time"] = sum(self.processing_times) / len(self.processing_times)
                stats["max_processing_time"] = max(self.processing_times)
                stats["min_processing_time"] = min(self.processing_times)
            
            if self.quality_scores:
                stats["avg_quality_score"] = sum(self.quality_scores) / len(self.quality_scores)
                stats["min_quality_score"] = min(self.quality_scores)
                stats["max_quality_score"] = max(self.quality_scores)
            
            return stats
    
    def get_latest_frame(self) -> Optional[Tuple[np.ndarray, FrameInfo]]:
        """Get latest frame (non-blocking)"""
        try:
            return self.frame_queue.get_nowait()
        except Empty:
            return None
    
    def cleanup(self):
        """Cleanup camera resources"""
        self.stop()
        
        if self.camera:
            if hasattr(self.camera, 'release'):
                self.camera.release()
            elif hasattr(self.camera, 'close'):
                self.camera.close()
        
        logger.info("Camera cleanup completed")

# Global camera optimizer instance
camera_optimizer = CameraOptimizer()
