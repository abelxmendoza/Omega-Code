"""
Camera Hardware Integration
Supports USB cameras, Raspberry Pi Camera, and IP cameras
"""

import asyncio
import logging
import time
import base64
from typing import Dict, List, Optional, Tuple, Any, Callable
from dataclasses import dataclass
from enum import Enum
import json

logger = logging.getLogger(__name__)

class CameraType(Enum):
    USB_CAMERA = "usb_camera"
    RASPBERRY_PI_CAMERA = "raspberry_pi_camera"
    IP_CAMERA = "ip_camera"

class CameraMode(Enum):
    STREAMING = "streaming"
    CAPTURE = "capture"
    RECORDING = "recording"

@dataclass
class CameraConfig:
    """Camera configuration"""
    camera_type: CameraType
    device_id: int = 0
    resolution: Tuple[int, int] = (640, 480)
    fps: int = 30
    quality: int = 80
    ip_address: Optional[str] = None
    port: int = 8080
    username: Optional[str] = None
    password: Optional[str] = None

@dataclass
class FrameData:
    """Camera frame data"""
    timestamp: float
    width: int
    height: int
    format: str
    data: bytes
    metadata: Dict[str, Any] = None

class CameraDriver:
    """Base class for camera drivers"""
    
    def __init__(self, camera_id: str, config: CameraConfig):
        self.camera_id = camera_id
        self.config = config
        self.initialized = False
        self.current_mode = CameraMode.STREAMING
        self.frame_callbacks: List[Callable[[FrameData], None]] = []
        self.is_streaming = False
        self.streaming_task: Optional[asyncio.Task] = None
    
    async def initialize(self) -> bool:
        """Initialize camera driver"""
        raise NotImplementedError
    
    async def cleanup(self) -> None:
        """Cleanup camera resources"""
        raise NotImplementedError
    
    async def start_streaming(self) -> bool:
        """Start camera streaming"""
        raise NotImplementedError
    
    async def stop_streaming(self) -> bool:
        """Stop camera streaming"""
        raise NotImplementedError
    
    async def capture_frame(self) -> Optional[FrameData]:
        """Capture a single frame"""
        raise NotImplementedError
    
    def add_frame_callback(self, callback: Callable[[FrameData], None]) -> None:
        """Add frame callback for streaming"""
        self.frame_callbacks.append(callback)
    
    def remove_frame_callback(self, callback: Callable[[FrameData], None]) -> None:
        """Remove frame callback"""
        if callback in self.frame_callbacks:
            self.frame_callbacks.remove(callback)

class USBCameraDriver(CameraDriver):
    """USB Camera driver using OpenCV"""
    
    def __init__(self, camera_id: str, config: CameraConfig):
        super().__init__(camera_id, config)
        self.camera = None
        
        # Import OpenCV
        try:
            import cv2
            self.cv2 = cv2
        except ImportError:
            logger.error("OpenCV not available for USB camera")
            raise
    
    async def initialize(self) -> bool:
        """Initialize USB camera"""
        try:
            self.camera = self.cv2.VideoCapture(self.config.device_id)
            
            if not self.camera.isOpened():
                logger.error(f"Failed to open USB camera {self.camera_id}")
                return False
            
            # Set camera properties
            self.camera.set(self.cv2.CAP_PROP_FRAME_WIDTH, self.config.resolution[0])
            self.camera.set(self.cv2.CAP_PROP_FRAME_HEIGHT, self.config.resolution[1])
            self.camera.set(self.cv2.CAP_PROP_FPS, self.config.fps)
            
            # Verify settings
            actual_width = int(self.camera.get(self.cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(self.cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = int(self.camera.get(self.cv2.CAP_PROP_FPS))
            
            logger.info(f"USB Camera {self.camera_id} initialized: {actual_width}x{actual_height} @ {actual_fps}fps")
            
            self.initialized = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize USB camera {self.camera_id}: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup USB camera resources"""
        try:
            if self.camera:
                self.camera.release()
            
            self.initialized = False
            logger.info(f"USB Camera {self.camera_id} cleaned up")
            
        except Exception as e:
            logger.error(f"Error cleaning up USB camera {self.camera_id}: {e}")
    
    async def start_streaming(self) -> bool:
        """Start USB camera streaming"""
        if not self.initialized or self.is_streaming:
            return False
        
        try:
            self.is_streaming = True
            self.streaming_task = asyncio.create_task(self._streaming_loop())
            
            logger.info(f"USB Camera {self.camera_id} streaming started")
            return True
            
        except Exception as e:
            logger.error(f"Error starting USB camera streaming: {e}")
            self.is_streaming = False
            return False
    
    async def stop_streaming(self) -> bool:
        """Stop USB camera streaming"""
        try:
            self.is_streaming = False
            
            if self.streaming_task:
                self.streaming_task.cancel()
                try:
                    await self.streaming_task
                except asyncio.CancelledError:
                    pass
            
            logger.info(f"USB Camera {self.camera_id} streaming stopped")
            return True
            
        except Exception as e:
            logger.error(f"Error stopping USB camera streaming: {e}")
            return False
    
    async def _streaming_loop(self) -> None:
        """USB camera streaming loop"""
        while self.is_streaming and self.initialized:
            try:
                ret, frame = self.camera.read()
                
                if ret:
                    # Encode frame as JPEG
                    encode_param = [int(self.cv2.IMWRITE_JPEG_QUALITY), self.config.quality]
                    _, encoded_frame = self.cv2.imencode('.jpg', frame, encode_param)
                    
                    frame_data = FrameData(
                        timestamp=time.time(),
                        width=frame.shape[1],
                        height=frame.shape[0],
                        format="jpeg",
                        data=encoded_frame.tobytes(),
                        metadata={
                            "camera_id": self.camera_id,
                            "fps": self.config.fps,
                            "quality": self.config.quality
                        }
                    )
                    
                    # Notify callbacks
                    for callback in self.frame_callbacks:
                        try:
                            callback(frame_data)
                        except Exception as e:
                            logger.error(f"Error in frame callback: {e}")
                
                # Control frame rate
                await asyncio.sleep(1.0 / self.config.fps)
                
            except Exception as e:
                logger.error(f"Error in USB camera streaming loop: {e}")
                await asyncio.sleep(0.1)
    
    async def capture_frame(self) -> Optional[FrameData]:
        """Capture a single frame from USB camera"""
        if not self.initialized:
            return None
        
        try:
            ret, frame = self.camera.read()
            
            if ret:
                # Encode frame as JPEG
                encode_param = [int(self.cv2.IMWRITE_JPEG_QUALITY), self.config.quality]
                _, encoded_frame = self.cv2.imencode('.jpg', frame, encode_param)
                
                return FrameData(
                    timestamp=time.time(),
                    width=frame.shape[1],
                    height=frame.shape[0],
                    format="jpeg",
                    data=encoded_frame.tobytes(),
                    metadata={
                        "camera_id": self.camera_id,
                        "capture_mode": True
                    }
                )
            
            return None
            
        except Exception as e:
            logger.error(f"Error capturing frame from USB camera: {e}")
            return None

class RaspberryPiCameraDriver(CameraDriver):
    """Raspberry Pi Camera driver using picamera2"""
    
    def __init__(self, camera_id: str, config: CameraConfig):
        super().__init__(camera_id, config)
        self.camera = None
        
        # Import picamera2
        try:
            from picamera2 import Picamera2
            self.Picamera2 = Picamera2
        except ImportError:
            logger.error("picamera2 not available for Raspberry Pi camera")
            raise
    
    async def initialize(self) -> bool:
        """Initialize Raspberry Pi camera"""
        try:
            self.camera = self.Picamera2()
            
            # Configure camera
            config = self.camera.create_still_configuration(
                main={"size": self.config.resolution, "format": "RGB888"}
            )
            
            self.camera.configure(config)
            self.camera.start()
            
            logger.info(f"Raspberry Pi Camera {self.camera_id} initialized: {self.config.resolution[0]}x{self.config.resolution[1]}")
            
            self.initialized = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize Raspberry Pi camera {self.camera_id}: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup Raspberry Pi camera resources"""
        try:
            if self.camera:
                self.camera.stop()
                self.camera.close()
            
            self.initialized = False
            logger.info(f"Raspberry Pi Camera {self.camera_id} cleaned up")
            
        except Exception as e:
            logger.error(f"Error cleaning up Raspberry Pi camera {self.camera_id}: {e}")
    
    async def start_streaming(self) -> bool:
        """Start Raspberry Pi camera streaming"""
        if not self.initialized or self.is_streaming:
            return False
        
        try:
            self.is_streaming = True
            self.streaming_task = asyncio.create_task(self._streaming_loop())
            
            logger.info(f"Raspberry Pi Camera {self.camera_id} streaming started")
            return True
            
        except Exception as e:
            logger.error(f"Error starting Raspberry Pi camera streaming: {e}")
            self.is_streaming = False
            return False
    
    async def stop_streaming(self) -> bool:
        """Stop Raspberry Pi camera streaming"""
        try:
            self.is_streaming = False
            
            if self.streaming_task:
                self.streaming_task.cancel()
                try:
                    await self.streaming_task
                except asyncio.CancelledError:
                    pass
            
            logger.info(f"Raspberry Pi Camera {self.camera_id} streaming stopped")
            return True
            
        except Exception as e:
            logger.error(f"Error stopping Raspberry Pi camera streaming: {e}")
            return False
    
    async def _streaming_loop(self) -> None:
        """Raspberry Pi camera streaming loop"""
        while self.is_streaming and self.initialized:
            try:
                # Capture frame
                frame = self.camera.capture_array()
                
                # Convert to JPEG
                import cv2
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.config.quality]
                _, encoded_frame = cv2.imencode('.jpg', frame, encode_param)
                
                frame_data = FrameData(
                    timestamp=time.time(),
                    width=frame.shape[1],
                    height=frame.shape[0],
                    format="jpeg",
                    data=encoded_frame.tobytes(),
                    metadata={
                        "camera_id": self.camera_id,
                        "fps": self.config.fps,
                        "quality": self.config.quality
                    }
                )
                
                # Notify callbacks
                for callback in self.frame_callbacks:
                    try:
                        callback(frame_data)
                    except Exception as e:
                        logger.error(f"Error in frame callback: {e}")
                
                # Control frame rate
                await asyncio.sleep(1.0 / self.config.fps)
                
            except Exception as e:
                logger.error(f"Error in Raspberry Pi camera streaming loop: {e}")
                await asyncio.sleep(0.1)
    
    async def capture_frame(self) -> Optional[FrameData]:
        """Capture a single frame from Raspberry Pi camera"""
        if not self.initialized:
            return None
        
        try:
            frame = self.camera.capture_array()
            
            # Convert to JPEG
            import cv2
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.config.quality]
            _, encoded_frame = cv2.imencode('.jpg', frame, encode_param)
            
            return FrameData(
                timestamp=time.time(),
                width=frame.shape[1],
                height=frame.shape[0],
                format="jpeg",
                data=encoded_frame.tobytes(),
                metadata={
                    "camera_id": self.camera_id,
                    "capture_mode": True
                }
            )
            
        except Exception as e:
            logger.error(f"Error capturing frame from Raspberry Pi camera: {e}")
            return None

class IPCameraDriver(CameraDriver):
    """IP Camera driver using HTTP streaming"""
    
    def __init__(self, camera_id: str, config: CameraConfig):
        super().__init__(camera_id, config)
        self.stream_url = None
        
        # Import requests
        try:
            import requests
            self.requests = requests
        except ImportError:
            logger.error("requests library not available for IP camera")
            raise
    
    async def initialize(self) -> bool:
        """Initialize IP camera"""
        try:
            # Construct stream URL
            if self.config.username and self.config.password:
                auth_url = f"http://{self.config.username}:{self.config.password}@{self.config.ip_address}:{self.config.port}"
            else:
                auth_url = f"http://{self.config.ip_address}:{self.config.port}"
            
            self.stream_url = f"{auth_url}/video"
            
            # Test connection
            response = self.requests.get(self.stream_url, timeout=5)
            if response.status_code == 200:
                logger.info(f"IP Camera {self.camera_id} initialized: {self.config.ip_address}:{self.config.port}")
                self.initialized = True
                return True
            else:
                logger.error(f"Failed to connect to IP camera {self.camera_id}: {response.status_code}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to initialize IP camera {self.camera_id}: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup IP camera resources"""
        try:
            self.initialized = False
            logger.info(f"IP Camera {self.camera_id} cleaned up")
        except Exception as e:
            logger.error(f"Error cleaning up IP camera {self.camera_id}: {e}")
    
    async def start_streaming(self) -> bool:
        """Start IP camera streaming"""
        if not self.initialized or self.is_streaming:
            return False
        
        try:
            self.is_streaming = True
            self.streaming_task = asyncio.create_task(self._streaming_loop())
            
            logger.info(f"IP Camera {self.camera_id} streaming started")
            return True
            
        except Exception as e:
            logger.error(f"Error starting IP camera streaming: {e}")
            self.is_streaming = False
            return False
    
    async def stop_streaming(self) -> bool:
        """Stop IP camera streaming"""
        try:
            self.is_streaming = False
            
            if self.streaming_task:
                self.streaming_task.cancel()
                try:
                    await self.streaming_task
                except asyncio.CancelledError:
                    pass
            
            logger.info(f"IP Camera {self.camera_id} streaming stopped")
            return True
            
        except Exception as e:
            logger.error(f"Error stopping IP camera streaming: {e}")
            return False
    
    async def _streaming_loop(self) -> None:
        """IP camera streaming loop"""
        while self.is_streaming and self.initialized:
            try:
                # Get frame from IP camera
                response = self.requests.get(self.stream_url, timeout=1)
                
                if response.status_code == 200:
                    frame_data = FrameData(
                        timestamp=time.time(),
                        width=self.config.resolution[0],
                        height=self.config.resolution[1],
                        format="jpeg",
                        data=response.content,
                        metadata={
                            "camera_id": self.camera_id,
                            "ip_address": self.config.ip_address,
                            "port": self.config.port
                        }
                    )
                    
                    # Notify callbacks
                    for callback in self.frame_callbacks:
                        try:
                            callback(frame_data)
                        except Exception as e:
                            logger.error(f"Error in frame callback: {e}")
                
                # Control frame rate
                await asyncio.sleep(1.0 / self.config.fps)
                
            except Exception as e:
                logger.error(f"Error in IP camera streaming loop: {e}")
                await asyncio.sleep(0.1)
    
    async def capture_frame(self) -> Optional[FrameData]:
        """Capture a single frame from IP camera"""
        if not self.initialized:
            return None
        
        try:
            response = self.requests.get(self.stream_url, timeout=5)
            
            if response.status_code == 200:
                return FrameData(
                    timestamp=time.time(),
                    width=self.config.resolution[0],
                    height=self.config.resolution[1],
                    format="jpeg",
                    data=response.content,
                    metadata={
                        "camera_id": self.camera_id,
                        "capture_mode": True
                    }
                )
            
            return None
            
        except Exception as e:
            logger.error(f"Error capturing frame from IP camera: {e}")
            return None

class CameraManager:
    """Manages all cameras and provides unified interface"""
    
    def __init__(self):
        self.cameras: Dict[str, CameraDriver] = {}
        self.initialized = False
        self.frame_callbacks: List[Callable[[str, FrameData], None]] = []
    
    async def add_camera(self, camera_id: str, config: CameraConfig) -> bool:
        """Add a camera to the system"""
        try:
            if config.camera_type == CameraType.USB_CAMERA:
                camera = USBCameraDriver(camera_id, config)
            elif config.camera_type == CameraType.RASPBERRY_PI_CAMERA:
                camera = RaspberryPiCameraDriver(camera_id, config)
            elif config.camera_type == CameraType.IP_CAMERA:
                camera = IPCameraDriver(camera_id, config)
            else:
                logger.error(f"Unsupported camera type: {config.camera_type}")
                return False
            
            success = await camera.initialize()
            if success:
                self.cameras[camera_id] = camera
                
                # Add frame callback to notify manager
                camera.add_frame_callback(lambda frame: self._on_frame_received(camera_id, frame))
                
                logger.info(f"Camera {camera_id} added to system")
            
            return success
            
        except Exception as e:
            logger.error(f"Failed to add camera {camera_id}: {e}")
            return False
    
    async def initialize(self) -> bool:
        """Initialize camera manager"""
        try:
            self.initialized = True
            logger.info("Camera manager initialized")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize camera manager: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup all cameras"""
        try:
            for camera in self.cameras.values():
                await camera.stop_streaming()
                await camera.cleanup()
            
            self.cameras.clear()
            self.initialized = False
            logger.info("Camera manager cleaned up")
            
        except Exception as e:
            logger.error(f"Error cleaning up camera manager: {e}")
    
    def _on_frame_received(self, camera_id: str, frame: FrameData) -> None:
        """Handle frame received from camera"""
        for callback in self.frame_callbacks:
            try:
                callback(camera_id, frame)
            except Exception as e:
                logger.error(f"Error in camera frame callback: {e}")
    
    def add_frame_callback(self, callback: Callable[[str, FrameData], None]) -> None:
        """Add frame callback for all cameras"""
        self.frame_callbacks.append(callback)
    
    def remove_frame_callback(self, callback: Callable[[str, FrameData], None]) -> None:
        """Remove frame callback"""
        if callback in self.frame_callbacks:
            self.frame_callbacks.remove(callback)
    
    async def start_camera_streaming(self, camera_id: str) -> bool:
        """Start streaming for specific camera"""
        if camera_id in self.cameras:
            return await self.cameras[camera_id].start_streaming()
        return False
    
    async def stop_camera_streaming(self, camera_id: str) -> bool:
        """Stop streaming for specific camera"""
        if camera_id in self.cameras:
            return await self.cameras[camera_id].stop_streaming()
        return False
    
    async def capture_camera_frame(self, camera_id: str) -> Optional[FrameData]:
        """Capture frame from specific camera"""
        if camera_id in self.cameras:
            return await self.cameras[camera_id].capture_frame()
        return None
    
    async def start_all_streaming(self) -> Dict[str, bool]:
        """Start streaming for all cameras"""
        results = {}
        for camera_id in self.cameras:
            results[camera_id] = await self.start_camera_streaming(camera_id)
        return results
    
    async def stop_all_streaming(self) -> Dict[str, bool]:
        """Stop streaming for all cameras"""
        results = {}
        for camera_id in self.cameras:
            results[camera_id] = await self.stop_camera_streaming(camera_id)
        return results

# Global camera manager instance
camera_manager = CameraManager()
