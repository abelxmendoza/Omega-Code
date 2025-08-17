# File: /Omega-Code/servers/robot-controller-backend/video/video_server.py
"""
Summary
-------
Secure, hostname/proxy-friendly MJPEG video server for the robot.

Key features:
- Never logs raw IP bind addresses (privacy-friendly logs)
- Strict, env-driven CORS allowlist (or explicit dev fallback "*")
- Optional TLS via CERT_PATH/KEY_PATH
- Motion detection + optional object tracking (OpenCV)
- Graceful behavior when no camera is attached (placeholder MJPEG stream)
- Rich /health endpoint for your UI (connected/absent/down + fps/counters)
- Works as a module: `python -m video.video_server`

Environment (.env)
- BIND_HOST                      (default "0.0.0.0")
- VIDEO_PORT                     (default "5000")
- ORIGIN_ALLOW                   (comma-separated origins)
- PUBLIC_BASE_URL, PUBLIC_BASE_URL_ALT (for friendly logs ONLY)
- CERT_PATH, KEY_PATH            (if both exist => SSL)
- CAMERA_DEVICE                  (default "/dev/video0")
- CAMERA_WIDTH, CAMERA_HEIGHT    (defaults "640", "480")
- PLACEHOLDER_WHEN_NO_CAMERA     (default "0"; set "1"/"true" to stream a placeholder)
"""

# ----------------------- Imports & setup -----------------------

import os
import time
import warnings
import logging
from typing import Optional, List

try:
    import cv2  # type: ignore
except Exception as e:  # pragma: no cover
    cv2 = None  # type: ignore
    warnings.warn(f"OpenCV not available ({e}). Video streaming disabled.", ImportWarning)

from flask import Flask, Response, jsonify
from flask_cors import CORS
from dotenv import load_dotenv, find_dotenv

# Local modules (import as package so `python -m video.video_server` works)
from video.camera import Camera
from video.motion_detection import MotionDetector
from video.object_tracking import ObjectTracker

# ----------------------- Configuration ------------------------

# Load nearest .env (robust to different working directories)
load_dotenv(find_dotenv())

BIND_HOST = os.getenv("BIND_HOST", "0.0.0.0")
PORT = int(os.getenv("VIDEO_PORT", "5000"))

PUBLIC_BASE_URL = os.getenv("PUBLIC_BASE_URL")
PUBLIC_BASE_URL_ALT = os.getenv("PUBLIC_BASE_URL_ALT")

# TLS support (enabled only if both files exist)
CERT_PATH = os.getenv("CERT_PATH") or None
KEY_PATH  = os.getenv("KEY_PATH") or None
SSL_ENABLED = bool(CERT_PATH and KEY_PATH and os.path.exists(CERT_PATH) and os.path.exists(KEY_PATH))

# CORS allowlist. If empty → dev fallback "*".
_raw_origins = [o.strip() for o in (os.getenv("ORIGIN_ALLOW", "")).split(",") if o.strip()]
CORS_ORIGINS: Optional[List[str]] = _raw_origins or None
ALLOWED_ORIGINS = CORS_ORIGINS if CORS_ORIGINS else "*"

# Camera settings
CAMERA_DEVICE  = os.getenv("CAMERA_DEVICE", "/dev/video0")
CAMERA_WIDTH   = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT  = int(os.getenv("CAMERA_HEIGHT", "480"))
PLACEHOLDER_WHEN_NO_CAMERA = os.getenv("PLACEHOLDER_WHEN_NO_CAMERA", "0").lower() in ("1", "true", "yes")

# Logging
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logging.getLogger("werkzeug").setLevel(logging.WARNING)

# ----------------------- Flask app ----------------------------

app = Flask(__name__)
# Route-scoped CORS: only set headers on the endpoints we expose to the UI.
CORS(app, resources={
    r"/video_feed":     {"origins": ALLOWED_ORIGINS},
    r"/start_tracking": {"origins": ALLOWED_ORIGINS},
    r"/health":         {"origins": ALLOWED_ORIGINS},
})

# Globals for the video pipeline (initialized in _init_video_stack)
camera: Optional[Camera] = None
motion_detector: Optional[MotionDetector] = None
tracker: Optional[ObjectTracker] = None
tracking_enabled = False

# ----------------------- Helpers ------------------------------

def _log_startup() -> None:
    """Print human-friendly startup info (no raw IPs)."""
    logging.info("🚀 Video server starting…")
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

def _init_video_stack() -> bool:
    """Initialize camera + processing modules. Returns True if usable."""
    global camera, motion_detector, tracker
    if cv2 is None:
        logging.warning("OpenCV not installed — video pipeline disabled.")
        camera = motion_detector = tracker = None
        return False
    try:
        camera = Camera(device=CAMERA_DEVICE, width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
        motion_detector = MotionDetector()
        tracker = ObjectTracker()
        _ = camera.get_frame()  # probe once; OK if None (handled below)
        return True
    except Exception as e:
        logging.warning(f"Camera initialization failed ({e}). Running without camera.")
        camera = motion_detector = tracker = None
        return False

def camera_present() -> bool:
    """True when OpenCV is available and we have an initialized camera object."""
    return cv2 is not None and camera is not None

def camera_connected() -> bool:
    """
    True when we consider streaming healthy: camera is present and has produced
    a frame recently (≤ ~2.5s). Uses Camera.is_alive() if available.
    """
    if not camera_present():
        return False
    try:
        return camera.is_alive(stale_ms=2500)  # type: ignore[attr-defined]
    except Exception:
        # Fallback: we at least have a camera object
        return True

def _placeholder_generator():
    """
    MJPEG placeholder when no camera is attached.
    Generates a simple image with guidance text. If NumPy is missing,
    we keep the connection open but yield empty chunks.
    """
    if cv2 is None:
        while True:
            time.sleep(1)
            yield b""

    try:
        import numpy as np  # Ensure your requirements are compatible with your OpenCV build
    except Exception:
        # No numpy -> yield empty chunks at a low rate
        while True:
            time.sleep(0.25)
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n\r\n")
    else:
        W, H = 640, 360
        last_log = 0.0
        while True:
            frame = np.zeros((H, W, 3), dtype=np.uint8)
            cv2.putText(frame, "NO CAMERA", (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, "Connect ribbon cable", (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2, cv2.LINE_AA)
            ok, buf = cv2.imencode(".jpg", frame)
            if ok:
                jpg = buf.tobytes()
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            time.sleep(0.1)
            now = time.time()
            if now - last_log > 5:
                logging.info("📷 Placeholder streaming (no camera attached).")
                last_log = now

def generate_frames():
    """
    MJPEG generator: reads frames, overlays motion (and tracker if enabled),
    yields JPEG frames. Falls back to placeholder when no camera and
    PLACEHOLDER_WHEN_NO_CAMERA is enabled.
    """
    global tracking_enabled

    if not camera_present():
        return _placeholder_generator() if PLACEHOLDER_WHEN_NO_CAMERA else (b"" for _ in iter(int, 1))

    assert camera is not None and motion_detector is not None
    while True:
        frame = camera.get_frame()
        if frame is None:
            # Camera not delivering frames; brief backoff avoids a tight loop.
            time.sleep(0.05)
            continue

        # Motion overlay
        frame, _ = motion_detector.detect_motion(frame)

        # Tracker overlay (optional)
        if tracking_enabled and tracker is not None:
            frame, _ = tracker.update_tracking(frame)

        # Encode JPEG
        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            logging.error("❌ JPEG encode failed.")
            continue

        jpg = buf.tobytes()
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")

# ----------------------- Routes -------------------------------

@app.get("/video_feed")
def video_feed():
    """
    MJPEG stream endpoint. If no camera and no placeholder is configured,
    return 503 so the UI can mark the stream as disconnected.
    """
    if not camera_present() and not PLACEHOLDER_WHEN_NO_CAMERA:
        return jsonify({"ok": False, "error": "camera_unavailable"}), 503

    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
            "X-Accel-Buffering": "no",
            "X-Content-Type-Options": "nosniff",
        },
    )

@app.get("/health")
def health():
    """
    Lightweight liveness/readiness probe consumed by the UI.
    - HTTP 200 when streaming is healthy (camera producing recent frames)
    - HTTP 503 when down OR absent (UI can still read JSON and show 'absent')
    """
    present = camera_present()
    connected = camera_connected()
    absent = (not present) and PLACEHOLDER_WHEN_NO_CAMERA

    # Expose useful metrics when available (safe defaults otherwise)
    fps = getattr(camera, "fps", 0.0) if present else 0.0
    frames_total = getattr(camera, "frames_total", 0) if present else 0
    drops_total = getattr(camera, "drops_total", 0) if present else 0

    status = "connected" if connected else ("absent" if absent else "down")

    payload = {
        "service": "video",
        "status": status,             # "connected" | "absent" | "down"
        "ok": connected,              # boolean convenience
        "cv2": cv2 is not None,
        "camera_present": present,
        "placeholder": absent,
        "device": CAMERA_DEVICE,
        "size": [CAMERA_WIDTH, CAMERA_HEIGHT],
        "fps": round(float(fps), 2),
        "frames_total": int(frames_total),
        "drops_total": int(drops_total),
        "ts": int(time.time() * 1000),
    }
    return jsonify(payload), (200 if connected else 503)

@app.post("/start_tracking")
def start_tracking():
    """
    Enable object tracking by selecting a ROI (requires a GUI/`DISPLAY`).
    Returns an error if no GUI is available, no frame is available,
    or OpenCV/camera is missing.
    """
    global tracking_enabled

    if cv2 is None:
        return "❌ OpenCV not available", 400
    if not camera_present():
        return "❌ Camera not available", 400

    frame = camera.get_frame()  # type: ignore[union-attr]
    if frame is None:
        return "❌ No frame available", 400

    try:
        if os.environ.get("DISPLAY"):
            bbox = cv2.selectROI("Select Object to Track", frame, fromCenter=False)
            if bbox and all(i > 0 for i in bbox):
                tracker.start_tracking(frame, bbox)  # type: ignore[union-attr]
                tracking_enabled = True
                return "Tracking started", 200
            return "❌ No object selected", 400
        return "❌ No GUI available for object selection", 500
    except Exception as e:
        logging.error(f"⚠️ Tracking error: {e}")
        return f"❌ Tracking failed: {e}", 500

# ----------------------- Entrypoint ---------------------------

if __name__ == "__main__":
    _log_startup()
    _init_video_stack()
    try:
        if SSL_ENABLED:
            logging.info("🔒 SSL enabled")
            app.run(host=BIND_HOST, port=PORT, ssl_context=(CERT_PATH, KEY_PATH), use_reloader=False)
        else:
            logging.info("⚡ Running without SSL")
            app.run(host=BIND_HOST, port=PORT, use_reloader=False)
    except Exception as e:
        logging.error(f"🔥 Error starting video server: {e}")
