# File: /Omega-Code/servers/robot_controller_backend/video/video_server.py
"""
Hardened MJPEG video server for the robot.

Features:
- Background camera (re)init retries if startup fails
- Optional stall watchdog to recreate the camera if frames go stale
- Keeps /video_feed and /health usable while reconnecting (UI shows 'connecting')
- Snapshot endpoint for single frame capture
- Performance metrics endpoint
- Configurable JPEG quality and FPS limiting
- Runtime configuration updates

Endpoints:
  GET  /video_feed              - MJPEG stream (main)
  GET  /video_feed_low          - Low quality stream (320x240, quality 60)
  GET  /video_feed_medium       - Medium quality stream (640x480, quality 75)
  GET  /video_feed_high         - High quality stream (1280x720, quality 90)
  GET  /health                  - Health check with camera status
  GET  /snapshot                - Capture single frame (query: ?save=1, ?quality=85)
  GET  /metrics                 - Performance metrics
  GET  /camera/list             - List available cameras
  POST /config                  - Update runtime config (JSON body)
  POST /start_tracking          - Start object tracking
  POST /face_recognition/reload - Reload known faces
  POST /recording/start         - Start video recording (JSON: {"filename": "optional"})
  POST /recording/stop          - Stop video recording
  GET  /recording/status        - Get recording status
  POST /camera/switch           - Switch camera device (JSON: {"device": "/dev/video1"})
  POST /overlays/config         - Configure frame overlays (JSON body)
  WebSocket /socket.io          - Real-time metrics (if flask-socketio available)

Env:
  BIND_HOST                (default "0.0.0.0")
  VIDEO_PORT               (default "5000")
  ORIGIN_ALLOW             (comma-separated origins, else "*" dev fallback)
  PUBLIC_BASE_URL(_ALT)    (pretty logs only)
  CERT_PATH, KEY_PATH      (TLS if both exist)
  CAMERA_*                 (see camera.py)
  PLACEHOLDER_WHEN_NO_CAMERA = 0|1
  JPEG_QUALITY             (hardware-optimized: Pi=75, Jetson=90, Mac=85, Linux=80)
  MAX_FPS_LIMIT            (hardware-optimized: Pi=30, Jetson=60, Mac=20, Linux=25)
  ENABLE_METRICS           (default 1)
  CAPTURE_DIR              (default "/tmp/omega_captures" for snapshots)
  FACE_RECOGNITION         (auto=hardware-aware, 1=enabled, 0=disabled)
  ARUCO_DETECTION          (auto=hardware-aware, 1=enabled, 0=disabled)
  ENABLE_RECORDING         (default 1) - Enable video recording
  ENABLE_OVERLAYS          (default 1) - Enable frame overlays
  ENABLE_FRAME_BUFFER      (default 1) - Enable frame buffering
  ENABLE_MULTI_RESOLUTION  (default 0) - Enable multi-resolution streams
  ENABLE_ROS2              (default 0) - Enable ROS2 frame publishing
  RECORDING_DIR            (default "/tmp/omega_recordings")
  
Hardware Optimizations:
  - Raspberry Pi 4B: Lower JPEG quality (75), adaptive quality under load, frame skipping
  - Jetson: High quality (90), full features enabled, 60fps capable
  - MacBook: Medium quality (85), limited features, 20fps
  - Linux Dev: Medium quality (80), CPU-based features, 25fps

  # Watchdog settings:
  STARTUP_RETRY_SEC        (default 5)    # retry camera init when it fails
  RESTART_ON_STALL         (default 1)    # 1=auto recreate on stall
  STALE_MS                 (default 2500) # what 'stale' means
  WATCHDOG_PERIOD_MS       (default 1500)
"""

import os
import time
import logging
import platform
from typing import Optional, List, Dict, Any

try:
    import psutil
except ImportError:
    psutil = None  # type: ignore

try:
    import cv2  # noqa: F401
except Exception:
    cv2 = None  # type: ignore

from flask import Flask, Response, jsonify, request
from flask_cors import CORS
from dotenv import load_dotenv, find_dotenv

# WebSocket support (optional)
try:
    from .websocket_server import VideoMetricsWebSocket
    WEBSOCKET_AVAILABLE = True
except ImportError:
    try:
        from websocket_server import VideoMetricsWebSocket
        WEBSOCKET_AVAILABLE = True
    except ImportError:
        VideoMetricsWebSocket = None
        WEBSOCKET_AVAILABLE = False

# ROS2 integration (optional)
try:
    from .ros2_integration import init_ros2_publisher, shutdown_ros2, ROS2VideoPublisher
    ROS2_AVAILABLE = True
except ImportError:
    try:
        from ros2_integration import init_ros2_publisher, shutdown_ros2, ROS2VideoPublisher
        ROS2_AVAILABLE = True
    except ImportError:
        init_ros2_publisher = None
        shutdown_ros2 = None
        ROS2VideoPublisher = None
        ROS2_AVAILABLE = False

# Hybrid system integration (optional)
try:
    from .hybrid_system import get_hybrid_system_manager, SystemMode
    from .hybrid_messages import TelemetryData, TrackingBBox, ArUcoMarker
    HYBRID_SYSTEM_AVAILABLE = True
except ImportError:
    try:
        from hybrid_system import get_hybrid_system_manager, SystemMode
        from hybrid_messages import TelemetryData, TrackingBBox, ArUcoMarker
        HYBRID_SYSTEM_AVAILABLE = True
    except ImportError:
        get_hybrid_system_manager = None
        SystemMode = None
        TelemetryData = None
        TrackingBBox = None
        ArUcoMarker = None
        HYBRID_SYSTEM_AVAILABLE = False
except Exception:
    # Fallback if any other error occurs
    get_hybrid_system_manager = None
    SystemMode = None
    TelemetryData = None
    TrackingBBox = None
    ArUcoMarker = None
    HYBRID_SYSTEM_AVAILABLE = False

# Local imports - try relative first, then absolute
try:
    from .aruco_detection import ArucoDetector
    from .camera import Camera
    from .face_recognition import FaceRecognizer
    from .motion_detection import MotionDetector
    from .object_tracking import ObjectTracker
    from .video_recorder import VideoRecorder
    from .frame_overlays import FrameOverlay
    from .frame_buffer import FrameBuffer
except ImportError:
    # Fallback to absolute imports when running as script
    from aruco_detection import ArucoDetector
    from camera import Camera
    from face_recognition import FaceRecognizer
    from motion_detection import MotionDetector
    from object_tracking import ObjectTracker
    try:
        from video_recorder import VideoRecorder
        from frame_overlays import FrameOverlay
        from frame_buffer import FrameBuffer
    except ImportError:
        VideoRecorder = None
        FrameOverlay = None
        FrameBuffer = None

# Mock camera fallback
try:
    from .mock_camera_server import MockCamera
except ImportError:
    try:
        from mock_camera_server import MockCamera
    except ImportError:
        MockCamera = None

load_dotenv(find_dotenv())

BIND_HOST = os.getenv("BIND_HOST", "0.0.0.0")
PORT = int(os.getenv("VIDEO_PORT", "5000"))

PUBLIC_BASE_URL = os.getenv("PUBLIC_BASE_URL")
PUBLIC_BASE_URL_ALT = os.getenv("PUBLIC_BASE_URL_ALT")

CERT_PATH = os.getenv("CERT_PATH") or None
KEY_PATH  = os.getenv("KEY_PATH") or None
SSL_ENABLED = bool(CERT_PATH and KEY_PATH and os.path.exists(CERT_PATH) and os.path.exists(KEY_PATH))

_raw_origins = [o.strip() for o in (os.getenv("ORIGIN_ALLOW", "")).split(",") if o.strip()]
CORS_ORIGINS: Optional[List[str]] = _raw_origins or None
ALLOWED_ORIGINS = CORS_ORIGINS if CORS_ORIGINS else "*"

# Try to load capabilities for default resolution
try:
    from api.capability_service import get_capability_service
    _capability_service = get_capability_service()
    _default_width, _default_height = _capability_service.get_max_resolution()
    _default_fps = _capability_service.get_max_fps()
except Exception:
    _default_width, _default_height = 640, 480
    _default_fps = 30

CAMERA_WIDTH   = int(os.getenv("CAMERA_WIDTH", str(_default_width)))
CAMERA_HEIGHT  = int(os.getenv("CAMERA_HEIGHT", str(_default_height)))
CAMERA_FPS     = int(os.getenv("CAMERA_FPS", str(_default_fps)))
PLACEHOLDER_WHEN_NO_CAMERA = os.getenv("PLACEHOLDER_WHEN_NO_CAMERA", "0").lower() in ("1", "true", "yes")

# Hardware detection and optimization
def _detect_hardware_profile() -> Dict[str, Any]:
    """Detect hardware and return optimized profile."""
    profile = {
        "is_pi4b": False,
        "is_jetson": False,
        "is_mac": platform.system() == "Darwin",
        "cpu_cores": os.cpu_count() or 1,
        "memory_gb": psutil.virtual_memory().total / (1024**3) if psutil else 4.0,
    }
    
    # Detect Raspberry Pi 4B
    try:
        if os.path.exists("/proc/device-tree/model"):
            with open("/proc/device-tree/model", "r") as f:
                model = f.read().strip()
                if "Raspberry Pi 4" in model:
                    profile["is_pi4b"] = True
                    profile["hardware_name"] = "Raspberry Pi 4B"
        elif os.path.exists("/proc/cpuinfo"):
            with open("/proc/cpuinfo", "r") as f:
                cpuinfo = f.read()
                if "BCM2711" in cpuinfo:
                    profile["is_pi4b"] = True
                    profile["hardware_name"] = "Raspberry Pi 4B"
    except Exception:
        pass
    
    # Detect Jetson
    try:
        if os.path.exists("/proc/device-tree/model"):
            with open("/proc/device-tree/model", "r") as f:
                model = f.read()
                if "NVIDIA" in model or "Jetson" in model:
                    profile["is_jetson"] = True
                    profile["hardware_name"] = "Jetson"
    except Exception:
        pass
    
    # Set hardware-specific defaults
    if profile["is_jetson"]:
        profile.update({
            "jpeg_quality": 90,
            "max_fps": 60,
            "enable_face_recognition": True,
            "enable_aruco": True,
            "enable_motion": True,
            "threading_enabled": True,
        })
    elif profile["is_pi4b"]:
        profile.update({
            "jpeg_quality": 75,  # Lower for Pi to reduce CPU load
            "max_fps": 30,
            "enable_face_recognition": False,  # Too CPU intensive
            "enable_aruco": True,
            "enable_motion": True,
            "threading_enabled": True,
        })
    elif profile["is_mac"]:
        profile.update({
            "jpeg_quality": 85,
            "max_fps": 20,
            "enable_face_recognition": False,
            "enable_aruco": True,
            "enable_motion": True,
            "threading_enabled": False,
        })
    else:  # Linux dev machine (Lenovo, etc.)
        profile.update({
            "jpeg_quality": 80,
            "max_fps": 25,
            "enable_face_recognition": True,  # CPU-based, slower
            "enable_aruco": True,
            "enable_motion": True,
            "threading_enabled": True,
        })
    
    return profile

_hardware_profile = _detect_hardware_profile()

# JPEG quality and performance settings (hardware-optimized defaults)
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", str(_hardware_profile.get("jpeg_quality", 85))))
MAX_FPS_LIMIT = int(os.getenv("MAX_FPS_LIMIT", str(_hardware_profile.get("max_fps", 0))))
ENABLE_METRICS = os.getenv("ENABLE_METRICS", "1").lower() in ("1", "true", "yes")

# Hardware-optimized feature flags (can be overridden by env vars)
ENABLE_FACE_RECOGNITION_HW = _hardware_profile.get("enable_face_recognition", True)
ENABLE_ARUCO_HW = _hardware_profile.get("enable_aruco", True)
ENABLE_MOTION_HW = _hardware_profile.get("enable_motion", True)
THREADING_ENABLED = _hardware_profile.get("threading_enabled", False)

STARTUP_RETRY_SEC = int(os.getenv("STARTUP_RETRY_SEC", "5"))
RESTART_ON_STALL  = os.getenv("RESTART_ON_STALL", "1").lower() in ("1", "true", "yes")
STALE_MS          = int(os.getenv("STALE_MS", os.getenv("FRAME_STALE_MS", "2500")))
WATCHDOG_PERIOD_MS= int(os.getenv("WATCHDOG_PERIOD_MS", "1500"))

# Feature flags (respect hardware capabilities unless explicitly overridden)
FACE_RECOGNITION_ENABLED = (
    os.getenv("FACE_RECOGNITION", "auto").lower() == "1" or
    (os.getenv("FACE_RECOGNITION", "auto").lower() == "auto" and ENABLE_FACE_RECOGNITION_HW)
) and os.getenv("FACE_RECOGNITION", "auto").lower() not in ("0", "false", "no", "off")
FACE_RECOGNITION_THRESHOLD = float(os.getenv("FACE_RECOGNITION_THRESHOLD", "0.6"))
KNOWN_FACES_DIR = os.getenv("KNOWN_FACES_DIR") or None

ARUCO_DETECTION_ENABLED = (
    os.getenv("ARUCO_DETECTION", "auto").lower() == "1" or
    (os.getenv("ARUCO_DETECTION", "auto").lower() == "auto" and ENABLE_ARUCO_HW)
) and os.getenv("ARUCO_DETECTION", "auto").lower() not in ("0", "false", "no", "off")

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logging.getLogger("werkzeug").setLevel(logging.WARNING)

app = Flask(__name__)
CORS(app, resources={
    r"/video_feed":     {"origins": ALLOWED_ORIGINS},
    r"/start_tracking": {"origins": ALLOWED_ORIGINS},
    r"/health":         {"origins": ALLOWED_ORIGINS},
    r"/face_recognition/reload": {"origins": ALLOWED_ORIGINS},
})

# WebSocket server (optional)
websocket_server = None
if WEBSOCKET_AVAILABLE and VideoMetricsWebSocket is not None:
    try:
        websocket_server = VideoMetricsWebSocket(app)
        websocket_server.start()
        logging.info("✅ WebSocket server enabled")
    except Exception as e:
        logging.warning(f"WebSocket server initialization failed: {e}")
        websocket_server = None

# Globals
camera: Optional[Camera] = None
motion_detector: Optional[MotionDetector] = None
tracker: Optional[ObjectTracker] = None
face_recognizer: Optional[FaceRecognizer] = None
aruco_detector: Optional[ArucoDetector] = None
tracking_enabled = False
_last_init_attempt = 0.0

# New modules
video_recorder: Optional[VideoRecorder] = None if VideoRecorder is None else None
frame_overlay: Optional[FrameOverlay] = None if FrameOverlay is None else None
frame_buffer: Optional[FrameBuffer] = None if FrameBuffer is None else None
ros2_publisher: Optional[ROS2VideoPublisher] = None

# Feature flags
ENABLE_ROS2 = os.getenv("ENABLE_ROS2", "0").lower() in ("1", "true", "yes") and ROS2_AVAILABLE
ENABLE_HYBRID_SYSTEM = os.getenv("ENABLE_HYBRID_SYSTEM", "1").lower() in ("1", "true", "yes") and HYBRID_SYSTEM_AVAILABLE

# Hybrid system manager
hybrid_system_manager = None
if ENABLE_HYBRID_SYSTEM and get_hybrid_system_manager is not None:
    try:
        hybrid_system_manager = get_hybrid_system_manager()
        logging.info(f"✅ Hybrid System enabled: {hybrid_system_manager.get_system_mode().value}")
    except Exception as e:
        logging.warning(f"⚠️ Failed to initialize Hybrid System: {e}")
        hybrid_system_manager = None

# Multi-resolution streams
ENABLE_MULTI_RESOLUTION = os.getenv("ENABLE_MULTI_RESOLUTION", "0").lower() in ("1", "true", "yes")
ENABLE_OVERLAYS = os.getenv("ENABLE_OVERLAYS", "1").lower() in ("1", "true", "yes")
ENABLE_RECORDING = os.getenv("ENABLE_RECORDING", "1").lower() in ("1", "true", "yes")
ENABLE_FRAME_BUFFER = os.getenv("ENABLE_FRAME_BUFFER", "1").lower() in ("1", "true", "yes")

# Metrics tracking
_metrics = {
    "requests_total": 0,
    "frames_served": 0,
    "errors_total": 0,
    "clients_connected": 0,
    "start_time": time.time(),
    "last_frame_time": 0.0,
    "frames_skipped": 0,  # For load management
}
_last_frame_time = 0.0
_frame_skip_counter = 0  # For Pi 4B load management

if FACE_RECOGNITION_ENABLED:
    try:
        face_recognizer = FaceRecognizer(
            known_faces_dir=KNOWN_FACES_DIR,
            recognition_threshold=FACE_RECOGNITION_THRESHOLD,
        )
        if face_recognizer and not face_recognizer.active:
            logging.info("Face recognition configured but inactive (cascade or OpenCV missing).")
    except Exception as exc:  # pragma: no cover - defensive guard for runtime issues
        face_recognizer = None
        logging.warning("Face recognition initialisation failed: %s", exc)

if ARUCO_DETECTION_ENABLED:
    try:
        aruco_detector = ArucoDetector()
        if aruco_detector and aruco_detector.active:
            logging.info(
                "ArUco detection enabled using %s.",
                aruco_detector.dictionary_name,
            )
        elif aruco_detector and not aruco_detector.active:
            logging.info("ArUco detection configured but inactive (cv2 contrib module missing or dictionary invalid).")
    except Exception as exc:  # pragma: no cover - defensive guard for runtime issues
        aruco_detector = None
        logging.warning("ArUco detection initialisation failed: %s", exc)

# Initialize new modules
if ENABLE_OVERLAYS and FrameOverlay is not None:
    try:
        frame_overlay = FrameOverlay(
            show_timestamp=True,
            show_fps=True,
            show_telemetry=False,  # Can be enabled via API
        )
        logging.info("✅ Frame overlays enabled")
    except Exception as e:
        logging.warning(f"Frame overlay initialization failed: {e}")
        frame_overlay = None

if ENABLE_FRAME_BUFFER and FrameBuffer is not None:
    try:
        frame_buffer = FrameBuffer(max_size=5)
        logging.info("✅ Frame buffer enabled")
    except Exception as e:
        logging.warning(f"Frame buffer initialization failed: {e}")
        frame_buffer = None

if ENABLE_RECORDING and VideoRecorder is not None:
    try:
        video_recorder = VideoRecorder()
        logging.info("✅ Video recording enabled")
    except Exception as e:
        logging.warning(f"Video recorder initialization failed: {e}")
        video_recorder = None

# Initialize ROS2 publisher
if ENABLE_ROS2 and init_ros2_publisher is not None:
    try:
        ros2_publisher = init_ros2_publisher()
        if ros2_publisher:
            logging.info("✅ ROS2 video publisher enabled")
    except Exception as e:
        logging.warning(f"ROS2 publisher initialization failed: {e}")
        ros2_publisher = None



def _log_startup() -> None:
    logging.info("🚀 Video server starting…")
    
    # Hardware detection info
    hw_name = _hardware_profile.get("hardware_name", "Unknown")
    cpu_cores = _hardware_profile.get("cpu_cores", 1)
    memory_gb = _hardware_profile.get("memory_gb", 0)
    logging.info(f"   🖥️  Hardware: {hw_name} ({cpu_cores} cores, {memory_gb:.1f}GB RAM)")
    
    if PUBLIC_BASE_URL:
        base = PUBLIC_BASE_URL if "://" in PUBLIC_BASE_URL else (
            f"https://{PUBLIC_BASE_URL}" if SSL_ENABLED else f"http://{PUBLIC_BASE_URL}:{PORT}"
        )
        logging.info(f"   🌐 Public: {base}/video_feed")
    if PUBLIC_BASE_URL_ALT:
        alt = PUBLIC_BASE_URL_ALT if "://" in PUBLIC_BASE_URL_ALT else f"http://{PUBLIC_BASE_URL_ALT}:{PORT}"
        logging.info(f"   🌐 Alt:    {alt}/video_feed")
    if not PUBLIC_BASE_URL and not PUBLIC_BASE_URL_ALT:
        logging.info("   ℹ️ Set PUBLIC_BASE_URL(_ALT) for friendlier logs (no raw IPs).")
    if ALLOWED_ORIGINS == "*":
        logging.info("   🔐 CORS: * (dev fallback)")
    else:
        logging.info("   🔐 CORS allowlist: " + ", ".join(CORS_ORIGINS or []))
    
    # Hardware-optimized settings
    logging.info(f"   📸 JPEG Quality: {JPEG_QUALITY} (hardware-optimized)")
    if MAX_FPS_LIMIT > 0:
        logging.info(f"   ⚡ Max FPS Limit: {MAX_FPS_LIMIT}")
    logging.info(f"   📊 Metrics: {'enabled' if ENABLE_METRICS else 'disabled'}")
    logging.info(f"   🎭 Features: Face Recognition={'✓' if FACE_RECOGNITION_ENABLED else '✗'}, "
                f"ArUco={'✓' if ARUCO_DETECTION_ENABLED else '✗'}, "
                f"Motion={'✓' if ENABLE_MOTION_HW else '✗'}")
    logging.info(f"   📹 Recording: {'enabled' if video_recorder else 'disabled'}")
    logging.info(f"   🖼️  Overlays: {'enabled' if frame_overlay else 'disabled'}")
    logging.info(f"   📦 Buffer: {'enabled' if frame_buffer else 'disabled'}")
    logging.info(f"   🔌 WebSocket: {'enabled' if websocket_server else 'disabled'}")
    logging.info(f"   🤖 ROS2: {'enabled' if ros2_publisher else 'disabled'}")
    if ENABLE_MULTI_RESOLUTION:
        logging.info(f"   📐 Multi-resolution streams: enabled")


def _create_camera(device: Optional[str] = None) -> bool:
    """
    Create camera instance.
    
    Args:
        device: Optional camera device path (for multi-camera support)
    """
    global camera, motion_detector, tracker, _last_init_attempt
    _last_init_attempt = time.time()
    
    # Run hardware verification if CameraManager is available
    try:
        from .camera_manager import CameraManager
        manager = CameraManager(width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS)
        hw_status = manager.verify_hardware()
        
        logging.info(f"Camera hardware check: detected={hw_status['camera_detected']}, "
                     f"supported={hw_status['camera_supported']}, "
                     f"devices={len(hw_status['devices_found'])}")
        
        if hw_status['devices_found']:
            logging.info(f"Found video devices: {', '.join(hw_status['devices_found'])}")
        
        if hw_status['recommendations']:
            for rec in hw_status['recommendations']:
                logging.warning(f"⚠ {rec}")
        
        # If camera not detected but device exists, provide helpful error
        if not hw_status['camera_detected'] and hw_status['devices_found']:
            logging.warning("⚠ Camera device exists but hardware not detected")
            logging.warning("  → Device exists but no frames — likely ribbon loose or interface disabled")
            logging.warning("  → Run diagnostic: python3 video/hw_check.py")
            logging.warning("  → Try reseating CSI ribbon connector (silver contacts face HDMI side)")
    except ImportError:
        logging.debug("CameraManager not available, skipping hardware verification")
    except Exception as e:
        logging.debug(f"Hardware verification failed: {e}")
    
    try:
        # Use specified device or default
        camera_device = device or os.getenv("CAMERA_DEVICE", "/dev/video0")
        camera = Camera(device=camera_device, width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
        motion_detector = MotionDetector()
        tracker = ObjectTracker()
        logging.info(f"Camera initialization successful (device: {camera_device}).")
        return True
    except Exception as e:
        logging.warning(f"Camera initialization failed ({e}). Trying mock camera fallback...")
        
        # Try mock camera as fallback
        if MockCamera is not None:
            try:
                camera = MockCamera(width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
                camera.start()
                # Initialize motion detector and tracker, but don't fail if they don't work
                try:
                    motion_detector = MotionDetector()
                except Exception as md_e:
                    logging.warning(f"Motion detector initialization failed: {md_e}")
                    motion_detector = None
                
                try:
                    tracker = ObjectTracker()
                except Exception as tr_e:
                    logging.warning(f"Object tracker initialization failed: {tr_e}")
                    tracker = None
                
                logging.info("Mock camera fallback successful.")
                return True
            except Exception as mock_e:
                logging.warning(f"Mock camera fallback also failed ({mock_e}). Running without camera.")
        
        camera = None
        return False


def camera_present() -> bool:
    return camera is not None


def camera_connected() -> bool:
    return camera_present() and camera.is_alive(stale_ms=STALE_MS)  # type: ignore[union-attr]


def _placeholder_generator():
    if cv2 is None:
        while True:
            time.sleep(1)
            yield b""
    try:
        import numpy as np
    except Exception:
        while True:
            time.sleep(0.25)
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n\r\n")
    else:
        import cv2 as _cv2  # type: ignore
        W, H = 640, 360
        last_log = 0.0
        while True:
            frame = np.zeros((H, W, 3), dtype=np.uint8)
            _cv2.putText(frame, "CONNECTING…", (30, 80), _cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 255), 3, _cv2.LINE_AA)
            _cv2.putText(frame, "Initializing camera", (30, 140), _cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2, _cv2.LINE_AA)
            ok, buf = _cv2.imencode(".jpg", frame)
            if ok:
                jpg = buf.tobytes()
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            time.sleep(0.12)
            now = time.time()
            if now - last_log > 5:
                logging.info("📷 Placeholder streaming (connecting).")
                last_log = now


def generate_frames():
    """
    MJPEG generator: reads frames, overlays motion (and tracker if enabled).
    If no camera yet, yields placeholder frames (when enabled).
    """
    global tracking_enabled, camera

    while True:
        try:
            if not camera_present():
                if PLACEHOLDER_WHEN_NO_CAMERA:
                    yield from _placeholder_generator()
                    return
                else:
                    # no camera and no placeholder → end stream (UI will retry)
                    yield b""
                    return

            if camera is None or motion_detector is None:
                time.sleep(0.1)
                continue

            frame = camera.get_frame()
            if frame is None:
                time.sleep(0.03)
                continue

            # Frame skipping for Pi 4B under heavy CPU load
            if _hardware_profile.get("is_pi4b") and psutil:
                try:
                    cpu_percent = psutil.cpu_percent(interval=0.05)
                    if cpu_percent > 90:
                        # Skip every other frame when CPU > 90%
                        _frame_skip_counter += 1
                        if _frame_skip_counter % 2 == 0:
                            _metrics["frames_skipped"] += 1
                            time.sleep(0.01)
                            continue
                    elif cpu_percent > 75:
                        # Skip every 3rd frame when CPU > 75%
                        _frame_skip_counter += 1
                        if _frame_skip_counter % 3 == 0:
                            _metrics["frames_skipped"] += 1
                            time.sleep(0.01)
                            continue
                except Exception:
                    pass  # Continue if psutil fails

            # Hybrid system: Check for throttling
            # TODO: Integrate thermal/CPU watchdog with mode manager
            should_throttle = False
            throttle_priority = []
            if hybrid_system_manager:
                hybrid_system_manager.cpu_monitor.update_load()
                should_throttle = hybrid_system_manager.should_throttle_modules()
                throttle_priority = hybrid_system_manager.get_throttle_priority()
                # Check and auto-switch mode if needed
                hybrid_system_manager.check_and_auto_switch_mode()
            
            # Motion overlay (with error handling, hardware-aware)
            motion_detected = False
            motion_regions = []
            if ENABLE_MOTION_HW and motion_detector is not None:
                if not should_throttle or "motion" not in throttle_priority[:1]:  # Only throttle if priority allows
                    try:
                        frame, motion_detected = motion_detector.detect_motion(frame)
                        # Extract motion regions for hybrid system
                        if motion_detected and hybrid_system_manager:
                            # Get motion regions from detector (if available)
                            pass  # TODO: Extract regions from motion detector
                    except Exception as e:
                        logging.warning(f"⚠️ Motion detection error: {e}")
                        _metrics["errors_total"] += 1
            
            # Publish motion event to Orin (hybrid system)
            if hybrid_system_manager and motion_detected:
                hybrid_system_manager.publish_motion_event(motion_detected, motion_regions)

            # Tracker overlay (optional)
            tracking_bbox = None
            if tracking_enabled and tracker is not None:
                if not should_throttle or "tracking" not in throttle_priority[:2]:  # Only throttle if priority allows
                    try:
                        frame, tracking_info = tracker.update_tracking(frame)
                        # Extract tracking bbox for hybrid system
                        if tracking_info and hybrid_system_manager:
                            # TODO: Extract bbox from tracker
                            pass
                    except Exception as e:
                        logging.warning(f"⚠️ Tracking error: {e}")
                        _metrics["errors_total"] += 1
            
            # Publish tracking bbox to Orin (hybrid system)
            if hybrid_system_manager and tracking_bbox:
                hybrid_system_manager.publish_tracking_bbox(tracking_bbox)

            if FACE_RECOGNITION_ENABLED and face_recognizer and face_recognizer.active:
                if not should_throttle or "face_detection" not in throttle_priority[:4]:  # Lowest priority
                    try:
                        frame, _ = face_recognizer.annotate(frame)
                    except Exception as e:
                        logging.warning(f"⚠️ Face recognition error: {e}")
                        _metrics["errors_total"] += 1

            # ArUco detection
            aruco_markers = []
            if ARUCO_DETECTION_ENABLED and aruco_detector and aruco_detector.active:
                if not should_throttle or "aruco" not in throttle_priority[:3]:  # Medium priority
                    try:
                        frame, detections = aruco_detector.annotate(frame)
                        if detections and hybrid_system_manager:
                            # Convert detections to hybrid messages format
                            for det in detections:
                                if hasattr(det, 'marker_id'):
                                    marker = ArUcoMarker(
                                        marker_id=det.marker_id,
                                        corners=[[float(c[0]), float(c[1])] for c in det.corners],
                                        centre=[float(det.centre[0]), float(det.centre[1])],
                                        pose=getattr(det, 'pose', None)
                                    )
                                    aruco_markers.append(marker)
                    except Exception as e:
                        logging.warning(f"⚠️ ArUco detection error: {e}")
                        _metrics["errors_total"] += 1
            
            # Publish ArUco markers to Orin (hybrid system)
            if hybrid_system_manager and aruco_markers:
                hybrid_system_manager.publish_aruco_markers(aruco_markers)

            # TODO: Capture timestamp for latency measurement
            # Capture timestamp for latency measurement
            import time
            try:
                from time import time_ns
            except ImportError:
                def time_ns():
                    return int(time.time() * 1e9)
            capture_timestamp_ns = time_ns()
            
            # Frame overlays (timestamp, FPS, telemetry)
            if ENABLE_OVERLAYS and frame_overlay is not None:
                try:
                    frame = frame_overlay.add_overlays(frame, capture_timestamp_ns=capture_timestamp_ns)
                except Exception as e:
                    logging.warning(f"⚠️ Overlay error: {e}")
                    _metrics["errors_total"] += 1

            # Add to frame buffer
            if ENABLE_FRAME_BUFFER and frame_buffer is not None:
                frame_buffer.add_frame(frame)

            # Add to video recorder if recording
            if ENABLE_RECORDING and video_recorder is not None and video_recorder.is_recording:
                video_recorder.add_frame(frame)

            # Publish to ROS2 if enabled
            if ENABLE_ROS2 and ros2_publisher is not None:
                try:
                    ros2_publisher.publish_frame(frame, compressed=True)
                except Exception as e:
                    logging.warning(f"⚠️ ROS2 publish error: {e}")
            
            # Publish compressed frame to Orin (hybrid system)
            if hybrid_system_manager and hybrid_system_manager.is_hybrid_mode():
                try:
                    hybrid_system_manager.publish_frame_to_orin(frame, quality=quality)
                except Exception as e:
                    logging.warning(f"⚠️ Hybrid system publish error: {e}")
            
            # Publish telemetry to Orin (hybrid system)
            if hybrid_system_manager and hybrid_system_manager.is_hybrid_mode():
                try:
                    if psutil:
                        cpu_usage = psutil.cpu_percent(interval=0.1)
                        memory_usage = psutil.virtual_memory().percent
                    else:
                        cpu_usage = 0.0
                        memory_usage = 0.0
                    
                    telemetry = TelemetryData(
                        cpu_temp=hybrid_system_manager.thermal_monitor.get_temperature(),
                        battery_voltage=0.0,  # TODO: Get from battery sensor
                        battery_percentage=0.0,  # TODO: Get from battery sensor
                        cpu_usage=cpu_usage,
                        memory_usage=memory_usage
                    )
                    hybrid_system_manager.publish_telemetry(telemetry)
                except Exception as e:
                    logging.warning(f"⚠️ Telemetry publish error: {e}")

            # Encode JPEG with quality setting (hardware-optimized)
            import cv2 as _cv2  # type: ignore
            
            # TODO: Record encode timestamps for latency measurement
            # Record encode start time for latency measurement
            encode_start_ns = time_ns()
            
            # Adaptive quality based on hardware load (for Pi 4B)
            quality = JPEG_QUALITY
            if _hardware_profile.get("is_pi4b") and psutil:
                # Reduce quality slightly if CPU is under heavy load
                try:
                    cpu_percent = psutil.cpu_percent(interval=0.1)
                    if cpu_percent > 80:
                        quality = max(60, JPEG_QUALITY - 10)  # Reduce quality under load
                    elif cpu_percent > 60:
                        quality = max(65, JPEG_QUALITY - 5)
                except Exception:
                    pass  # Use default if psutil fails
            
            encode_params = [_cv2.IMWRITE_JPEG_QUALITY, quality]
            ok, buf = _cv2.imencode(".jpg", frame, encode_params)
            
            # Record encode end time for latency measurement
            encode_end_ns = time_ns()
            
            # TODO: Update frame overlay with encode timestamps for latency reporting
            # Update frame overlay with encode timestamps
            if frame_overlay is not None:
                try:
                    frame_overlay.set_encode_timestamps(encode_start_ns, encode_end_ns)
                except Exception:
                    pass
            if not ok:
                logging.error("❌ JPEG encode failed.")
                _metrics["errors_total"] += 1
                time.sleep(0.1)
                continue

            # Frame rate limiting
            if MAX_FPS_LIMIT > 0:
                current_time = time.time()
                min_interval = 1.0 / MAX_FPS_LIMIT
                if current_time - _last_frame_time < min_interval:
                    time.sleep(min_interval - (current_time - _last_frame_time))
                _last_frame_time = time.time()

            jpg = buf.tobytes()
            _metrics["frames_served"] += 1
            _metrics["last_frame_time"] = time.time()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        
        except Exception as e:
            logging.error(f"🔥 Frame generation error: {e}", exc_info=True)
            _metrics["errors_total"] += 1
            time.sleep(0.1)  # Brief pause before retrying
            # Continue loop to try again


@app.get("/video_feed")
def video_feed():
    """
    MJPEG stream endpoint.
    """
    _metrics["requests_total"] += 1
    _metrics["clients_connected"] += 1
    
    # If there's no camera yet and placeholder is disabled, tell the client to try later
    if not camera_present() and not PLACEHOLDER_WHEN_NO_CAMERA:
        _metrics["clients_connected"] = max(0, _metrics["clients_connected"] - 1)
        return jsonify({"ok": False, "error": "camera_unavailable"}), 503

    def on_close():
        _metrics["clients_connected"] = max(0, _metrics["clients_connected"] - 1)

    response = Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
            "X-Accel-Buffering": "no",
            "X-Content-Type-Options": "nosniff",
        },
    )
    # Note: Flask doesn't have built-in connection close callbacks, 
    # but we track connections via request count
    return response


@app.get("/health")
def health():
    """
    Liveness/readiness for UI.
    200 when actually streaming frames, 503 otherwise (down/connecting/absent).
    """
    present = camera_present()
    connected = camera_connected()
    connecting = (not present) or (present and not connected)

    status = "connected" if connected else ("connecting" if connecting else "down")

    fps          = getattr(camera, "fps", 0.0) if present else 0.0
    frames_total = getattr(camera, "frames_total", 0) if present else 0
    drops_total  = getattr(camera, "drops_total", 0) if present else 0

    payload = {
        "service": "video",
        "status": status,             # "connected" | "connecting" | "down"
        "ok": connected,
        "camera_present": present,
        "placeholder": PLACEHOLDER_WHEN_NO_CAMERA and not connected,
        "device": os.getenv("CAMERA_DEVICE", "/dev/video0"),
        "size": [CAMERA_WIDTH, CAMERA_HEIGHT],
        "fps": round(float(fps), 2),
        "frames_total": int(frames_total),
        "drops_total": int(drops_total),
        "hardware": {
            "name": _hardware_profile.get("hardware_name", "Unknown"),
            "is_pi4b": _hardware_profile.get("is_pi4b", False),
            "is_jetson": _hardware_profile.get("is_jetson", False),
            "cpu_cores": _hardware_profile.get("cpu_cores", 1),
            "memory_gb": round(_hardware_profile.get("memory_gb", 0), 1),
        },
        "ts": int(time.time() * 1000),
    }
    
    # Add optional features if available
    if video_recorder:
        payload["recording"] = video_recorder.get_status()
    if frame_buffer:
        payload["buffer"] = frame_buffer.get_stats()
    if frame_overlay:
        payload["overlays"] = {
            "enabled": True,
            "show_timestamp": frame_overlay.show_timestamp,
            "show_fps": frame_overlay.show_fps,
        }
    if ros2_publisher:
        payload["ros2"] = {"enabled": True, "topics": ["/omega/camera/image_raw/compressed"]}
    
    # Broadcast metrics via WebSocket if available
    if websocket_server:
        try:
            websocket_server.broadcast_metrics(payload)
        except Exception:
            pass
    
    return jsonify(payload), (200 if connected else 503)
    payload["face_recognition"] = {
        "configured": FACE_RECOGNITION_ENABLED,
        "active": bool(face_recognizer and face_recognizer.active),
        "known_faces": int(face_recognizer.known_faces_count if face_recognizer else 0),
    }
    payload["aruco"] = {
        "configured": ARUCO_DETECTION_ENABLED,
        "active": bool(aruco_detector and aruco_detector.active),
        "dictionary": aruco_detector.dictionary_name if aruco_detector else None,
        "pose_estimation": bool(aruco_detector and aruco_detector.pose_estimation_enabled),
    }
    return jsonify(payload), (200 if connected else 503)


@app.post("/start_tracking")
def start_tracking():
    global tracking_enabled

    if not camera_present():
        return "❌ Camera not available", 400

    f = camera.get_frame()  # type: ignore[union-attr]
    if f is None:
        return "❌ No frame available", 400

    try:
        import cv2 as _cv2  # type: ignore
        if os.environ.get("DISPLAY"):
            bbox = _cv2.selectROI("Select Object to Track", f, fromCenter=False)
            if bbox and all(i > 0 for i in bbox):
                tracker.start_tracking(f, bbox)  # type: ignore[union-attr]
                tracking_enabled = True
                return "Tracking started", 200
            return "❌ No object selected", 400
        return "❌ No GUI available for object selection", 500
    except Exception as e:
        logging.error(f"⚠️ Tracking error: {e}")
        return f"❌ Tracking failed: {e}", 500


@app.post("/face_recognition/reload")
def reload_face_recognition():
    if not FACE_RECOGNITION_ENABLED:
        return jsonify({"ok": False, "error": "disabled"}), 400

    if face_recognizer is None or not face_recognizer.active:
        return jsonify({"ok": False, "error": "not_available"}), 503

    count = face_recognizer.reload_known_faces()
    return jsonify({"ok": True, "faces": count}), 200


@app.post("/camera/switch")
def switch_camera():
    """Switch to a different camera device (multi-camera support)."""
    global camera
    
    try:
        data = request.get_json() or {}
        device = data.get("device")
        
        if not device:
            return jsonify({"ok": False, "error": "device_required"}), 400
        
        # Stop current camera
        if camera:
            try:
                camera.stop()
            except Exception:
                pass
        
        # Create new camera
        success = _create_camera(device=device)
        
        if success:
            return jsonify({
                "ok": True,
                "device": device,
                "message": f"Switched to camera: {device}"
            }), 200
        else:
            return jsonify({
                "ok": False,
                "error": "failed_to_switch",
                "message": f"Failed to switch to camera: {device}"
            }), 500
    
    except Exception as e:
        logging.error(f"⚠️ Camera switch error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500


@app.get("/camera/list")
def list_cameras():
    """List available camera devices."""
    import glob
    
    cameras = []
    
    # Check /dev/video* devices
    for device in glob.glob("/dev/video*"):
        try:
            # Try to open device to verify it's accessible
            import cv2
            cap = cv2.VideoCapture(device)
            if cap.isOpened():
                cameras.append({
                    "device": device,
                    "available": True,
                })
                cap.release()
        except Exception:
            pass
    
    # Add current camera
    if camera:
        cameras.append({
            "device": getattr(camera, "backend_name", "unknown"),
            "current": True,
            "available": True,
        })
    
    return jsonify({
        "ok": True,
        "cameras": cameras,
        "current": os.getenv("CAMERA_DEVICE", "/dev/video0"),
    }), 200


@app.get("/snapshot")
def snapshot():
    """
    Capture a single frame as JPEG.
    Query params:
      - save: if "1", "true", or "yes", save to disk
      - quality: JPEG quality 1-100 (overrides JPEG_QUALITY env var)
    """
    if not camera_present():
        return jsonify({"ok": False, "error": "camera_unavailable"}), 503

    frame = camera.get_frame()  # type: ignore[union-attr]
    if frame is None:
        return jsonify({"ok": False, "error": "no_frame_available"}), 503

    # Apply overlays (same as video feed)
    if motion_detector:
        frame, _ = motion_detector.detect_motion(frame)
    
    if tracking_enabled and tracker:
        frame, _ = tracker.update_tracking(frame)
    
    if FACE_RECOGNITION_ENABLED and face_recognizer and face_recognizer.active:
        frame, _ = face_recognizer.annotate(frame)
    
    if ARUCO_DETECTION_ENABLED and aruco_detector and aruco_detector.active:
        frame, _ = aruco_detector.annotate(frame)

    # Get quality from query param or use default
    quality = int(request.args.get("quality", JPEG_QUALITY))
    quality = max(1, min(100, quality))  # Clamp to valid range

    import cv2 as _cv2  # type: ignore
    ok, buf = _cv2.imencode(".jpg", frame, [_cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        return jsonify({"ok": False, "error": "jpeg_encode_failed"}), 500

    jpg = buf.tobytes()

    # Optionally save to disk
    if request.args.get("save", "0").lower() in ("1", "true", "yes"):
        import pathlib
        capture_dir = pathlib.Path(os.getenv("CAPTURE_DIR", "/tmp/omega_captures"))
        capture_dir.mkdir(parents=True, exist_ok=True)
        ts = int(time.time())
        (capture_dir / f"snapshot_{ts}.jpg").write_bytes(jpg)
        logging.info(f"📸 Snapshot saved: snapshot_{ts}.jpg")

    return Response(
        jpg,
        mimetype="image/jpeg",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "X-Content-Type-Options": "nosniff",
        },
    )


@app.get("/metrics")
def metrics():
    """
    Performance metrics endpoint.
    Returns statistics about the video server.
    """
    if not ENABLE_METRICS:
        return jsonify({"ok": False, "error": "metrics_disabled"}), 403

    uptime = time.time() - _metrics["start_time"]
    fps = getattr(camera, "fps", 0.0) if camera_present() else 0.0
    
    payload = {
        "ok": True,
        "uptime_seconds": round(uptime, 2),
        "requests_total": _metrics["requests_total"],
        "frames_served": _metrics["frames_served"],
        "frames_skipped": _metrics.get("frames_skipped", 0),
        "errors_total": _metrics["errors_total"],
        "clients_connected": _metrics["clients_connected"],
        "fps": round(float(fps), 2),
        "jpeg_quality": JPEG_QUALITY,
        "max_fps_limit": MAX_FPS_LIMIT if MAX_FPS_LIMIT > 0 else None,
        "camera_present": camera_present(),
        "camera_connected": camera_connected(),
        "hardware": {
            "name": _hardware_profile.get("hardware_name", "Unknown"),
            "is_pi4b": _hardware_profile.get("is_pi4b", False),
            "is_jetson": _hardware_profile.get("is_jetson", False),
            "cpu_cores": _hardware_profile.get("cpu_cores", 1),
        },
        "ts": int(time.time() * 1000),
    }
    
    if camera_present():
        payload["camera_stats"] = {
            "frames_total": getattr(camera, "frames_total", 0),
            "drops_total": getattr(camera, "drops_total", 0),
            "fps": round(float(fps), 2),
        }
    
    return jsonify(payload), 200


@app.get("/latency")
def latency():
    """
    Latency metrics endpoint (Pi-only).
    Returns frame processing latency metrics.
    """
    if not frame_overlay:
        return jsonify({"ok": False, "error": "frame_overlay_not_available"}), 503
    
    try:
        latency_metrics = frame_overlay.get_latency_metrics()
        
        # Calculate latencies if timestamps are available
        latencies = {}
        if latency_metrics.get("capture_timestamp_ns") and latency_metrics.get("encode_start_ns"):
            capture_to_encode = (latency_metrics["encode_start_ns"] - latency_metrics["capture_timestamp_ns"]) / 1e6  # ms
            latencies["capture_to_encode_ms"] = round(capture_to_encode, 2)
        
        if latency_metrics.get("encode_start_ns") and latency_metrics.get("encode_end_ns"):
            encode_duration = (latency_metrics["encode_end_ns"] - latency_metrics["encode_start_ns"]) / 1e6  # ms
            latencies["encode_duration_ms"] = round(encode_duration, 2)
        
        if latency_metrics.get("capture_timestamp_ns") and latency_metrics.get("encode_end_ns"):
            total_processing = (latency_metrics["encode_end_ns"] - latency_metrics["capture_timestamp_ns"]) / 1e6  # ms
            latencies["total_processing_ms"] = round(total_processing, 2)
        
        return jsonify({
            "ok": True,
            "type": "pi_only",
            "timestamps_ns": latency_metrics,
            "latencies_ms": latencies,
            "ts": int(time.time() * 1000),
        }), 200
    
    except Exception as e:
        logging.error(f"Failed to get latency metrics: {e}", exc_info=True)
        return jsonify({"ok": False, "error": str(e)}), 500


@app.get("/latency/hybrid")
def latency_hybrid():
    """
    Hybrid latency metrics endpoint (Pi ↔ Orin round-trip).
    Returns round-trip latency and inference duration metrics.
    """
    try:
        # Get latency stats from Pi sensor hub if available
        if hybrid_system_manager and hybrid_system_manager.pi_sensor_hub:
            latency_stats = hybrid_system_manager.pi_sensor_hub.get_latency_stats()
            return jsonify({
                "ok": True,
                "type": "hybrid",
                **latency_stats,
                "ts": int(time.time() * 1000),
            }), 200
        else:
            return jsonify({
                "ok": False,
                "error": "hybrid_system_not_available",
                "message": "Hybrid system or Pi sensor hub not initialized"
            }), 503
    
    except Exception as e:
        logging.error(f"Failed to get hybrid latency metrics: {e}", exc_info=True)
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/config")
def update_config():
    """
    Update server configuration at runtime.
    POST body: JSON with config keys (jpeg_quality, max_fps_limit)
    """
    global JPEG_QUALITY, MAX_FPS_LIMIT
    
    try:
        data = request.get_json() or {}
        
        if "jpeg_quality" in data:
            new_quality = int(data["jpeg_quality"])
            JPEG_QUALITY = max(1, min(100, new_quality))
            logging.info(f"📝 JPEG quality updated to {JPEG_QUALITY}")
        
        if "max_fps_limit" in data:
            new_limit = int(data["max_fps_limit"])
            MAX_FPS_LIMIT = max(0, new_limit)
            logging.info(f"📝 Max FPS limit updated to {MAX_FPS_LIMIT if MAX_FPS_LIMIT > 0 else 'unlimited'}")
        
        return jsonify({
            "ok": True,
            "jpeg_quality": JPEG_QUALITY,
            "max_fps_limit": MAX_FPS_LIMIT if MAX_FPS_LIMIT > 0 else None,
        }), 200
    
    except Exception as e:
        logging.error(f"⚠️ Config update error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 400


@app.post("/recording/start")
def start_recording():
    """Start video recording."""
    global video_recorder, camera
    
    if video_recorder is None:
        return jsonify({"ok": False, "error": "recording_not_available"}), 503
    
    if not camera_present():
        return jsonify({"ok": False, "error": "camera_unavailable"}), 503
    
    if video_recorder.is_recording:
        return jsonify({"ok": False, "error": "already_recording"}), 400
    
    try:
        data = request.get_json() or {}
        filename = data.get("filename")
        
        filepath = video_recorder.start_recording(
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
            filename=filename
        )
        
        return jsonify({
            "ok": True,
            "filepath": filepath,
            "message": "Recording started"
        }), 200
    
    except Exception as e:
        logging.error(f"⚠️ Recording start error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500


@app.post("/recording/stop")
def stop_recording():
    """Stop video recording."""
    global video_recorder
    
    if video_recorder is None:
        return jsonify({"ok": False, "error": "recording_not_available"}), 503
    
    if not video_recorder.is_recording:
        return jsonify({"ok": False, "error": "not_recording"}), 400
    
    try:
        filepath = video_recorder.stop_recording()
        status = video_recorder.get_status()
        
        return jsonify({
            "ok": True,
            "filepath": filepath,
            "status": status,
            "message": "Recording stopped"
        }), 200
    
    except Exception as e:
        logging.error(f"⚠️ Recording stop error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 500


@app.get("/recording/status")
def recording_status():
    """Get recording status."""
    if video_recorder is None:
        return jsonify({"ok": False, "error": "recording_not_available"}), 503
    
    status = video_recorder.get_status()
    return jsonify({"ok": True, **status}), 200


@app.get("/video_feed_low")
def video_feed_low():
    """Low quality MJPEG stream (320x240, quality 60)."""
    return _generate_resolution_stream(320, 240, 60)


@app.get("/video_feed_medium")
def video_feed_medium():
    """Medium quality MJPEG stream (640x480, quality 75)."""
    return _generate_resolution_stream(640, 480, 75)


@app.get("/video_feed_high")
def video_feed_high():
    """High quality MJPEG stream (1280x720, quality 90)."""
    return _generate_resolution_stream(1280, 720, 90)


def _generate_resolution_stream(target_width: int, target_height: int, quality: int):
    """Generate MJPEG stream at specific resolution and quality."""
    if not camera_present() and not PLACEHOLDER_WHEN_NO_CAMERA:
        return jsonify({"ok": False, "error": "camera_unavailable"}), 503
    
    def generate():
        import cv2 as _cv2  # type: ignore
        while True:
            if not camera_present():
                if PLACEHOLDER_WHEN_NO_CAMERA:
                    yield from _placeholder_generator()
                    return
                else:
                    yield b""
                    return
            
            frame = camera.get_frame()  # type: ignore[union-attr]
            if frame is None:
                time.sleep(0.03)
                continue
            
            # Resize frame to target resolution
            h, w = frame.shape[:2]
            if w != target_width or h != target_height:
                frame = _cv2.resize(frame, (target_width, target_height))
            
            # Encode with specified quality
            encode_params = [_cv2.IMWRITE_JPEG_QUALITY, quality]
            ok, buf = _cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                time.sleep(0.1)
                continue
            
            jpg = buf.tobytes()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
    
    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
            "X-Accel-Buffering": "no",
        },
    )


@app.post("/overlays/config")
def configure_overlays():
    """Configure frame overlays."""
    global frame_overlay
    
    if frame_overlay is None:
        return jsonify({"ok": False, "error": "overlays_not_available"}), 503
    
    try:
        data = request.get_json() or {}
        
        if "show_timestamp" in data:
            frame_overlay.show_timestamp = bool(data["show_timestamp"])
        if "show_fps" in data:
            frame_overlay.show_fps = bool(data["show_fps"])
        if "show_telemetry" in data:
            frame_overlay.show_telemetry = bool(data["show_telemetry"])
        if "telemetry" in data:
            frame_overlay.update_telemetry(data["telemetry"])
        
        return jsonify({
            "ok": True,
            "show_timestamp": frame_overlay.show_timestamp,
            "show_fps": frame_overlay.show_fps,
            "show_telemetry": frame_overlay.show_telemetry,
        }), 200
    
    except Exception as e:
        logging.error(f"⚠️ Overlay config error: {e}")
        return jsonify({"ok": False, "error": str(e)}), 400


# ---------------- Watchdog / init loop ----------------

def _watchdog_tick():
    """Periodic camera init/restart logic."""
    global _last_init_attempt, camera

    # Retry init if none
    if not camera_present():
        if (time.time() - _last_init_attempt) >= STARTUP_RETRY_SEC:
            _create_camera()
        return

    # Restart on stall (optional)
    if RESTART_ON_STALL and not camera_connected():
        logging.warning("Camera stalled > %dms – recreating…", STALE_MS)
        try:
            # Handle both regular Camera and MockCamera cleanup
            if hasattr(camera, 'stop'):
                camera.stop()  # type: ignore[union-attr]
            elif hasattr(camera, 'cleanup'):
                camera.cleanup()  # type: ignore[union-attr]
        except Exception:
            pass
        camera = None
        _create_camera()


@app.after_request
def _schedule_watchdog(resp):
    # Very light-touch watchdog: run opportunistically with requests
    try:
        _watchdog_tick()
    except Exception:
        pass
    return resp


if __name__ == "__main__":
    _log_startup()
    _create_camera()  # first attempt (non-fatal if it fails)
    try:
        if SSL_ENABLED:
            logging.info("🔒 SSL enabled")
            if websocket_server and websocket_server.socketio:
                websocket_server.socketio.run(app, host=BIND_HOST, port=PORT, ssl_context=(CERT_PATH, KEY_PATH))
            else:
                app.run(host=BIND_HOST, port=PORT, ssl_context=(CERT_PATH, KEY_PATH), use_reloader=False)
        else:
            logging.info("⚡ Running without SSL")
            if websocket_server and websocket_server.socketio:
                websocket_server.socketio.run(app, host=BIND_HOST, port=PORT)
            else:
                app.run(host=BIND_HOST, port=PORT, use_reloader=False)
    except KeyboardInterrupt:
        logging.info("🛑 Shutting down video server...")
    except Exception as e:
        logging.error(f"🔥 Error starting video server: {e}")
    finally:
        # Cleanup
        if camera:
            try:
                camera.stop()
            except Exception:
                pass
        if video_recorder and video_recorder.is_recording:
            try:
                video_recorder.stop_recording()
            except Exception:
                pass
        if ENABLE_ROS2 and shutdown_ros2:
            try:
                shutdown_ros2()
            except Exception:
                pass
        logging.info("✅ Video server shutdown complete")
