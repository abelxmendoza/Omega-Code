"""
📌 File: video/video_server.py

Secure, hostname/proxy-friendly MJPEG video server for the robot.
- Hides raw IPs: never logs numeric addresses
- CORS restricted to allowed origins (hostnames/domains)
- Optional SSL
- Motion detection + object tracking preserved
"""
# 📌 File: video/video_server.py
# Secure, hostname/proxy-friendly MJPEG server (works with/without camera)

import os, time, warnings, logging
from typing import Optional

try:
    import cv2  # type: ignore
except Exception as e:  # pragma: no cover
    cv2 = None  # type: ignore
    warnings.warn(f"OpenCV not available ({e}). Video streaming disabled.", ImportWarning)

from flask import Flask, Response, jsonify
from flask_cors import CORS
from dotenv import load_dotenv, find_dotenv

# Local modules
from video.camera import Camera
from video.motion_detection import MotionDetector
from video.object_tracking import ObjectTracker

# ---------- Config ----------
load_dotenv(find_dotenv())

BIND_HOST = os.getenv("BIND_HOST", "0.0.0.0")
PORT = int(os.getenv("VIDEO_PORT", "5000"))

PUBLIC_BASE_URL = os.getenv("PUBLIC_BASE_URL")
PUBLIC_BASE_URL_ALT = os.getenv("PUBLIC_BASE_URL_ALT")

CERT_PATH = os.getenv("CERT_PATH") or None
KEY_PATH  = os.getenv("KEY_PATH") or None
SSL_ENABLED = bool(CERT_PATH and KEY_PATH and os.path.exists(CERT_PATH) and os.path.exists(KEY_PATH))

# CORS: comma-separated origins; if empty → "*"
_origins = [o.strip() for o in (os.getenv("ORIGIN_ALLOW", "")).split(",") if o.strip()]
CORS_ORIGINS = _origins or None
ALLOWED_ORIGINS = CORS_ORIGINS if CORS_ORIGINS else "*"

# Camera settings
CAMERA_DEVICE  = os.getenv("CAMERA_DEVICE", "/dev/video0")
CAMERA_WIDTH   = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT  = int(os.getenv("CAMERA_HEIGHT", "480"))
PLACEHOLDER_WHEN_NO_CAMERA = os.getenv("PLACEHOLDER_WHEN_NO_CAMERA", "0").lower() in ("1","true","yes")

# Logging
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logging.getLogger("werkzeug").setLevel(logging.WARNING)

# ---------- App ----------
app = Flask(__name__)
CORS(app, resources={
    r"/video_feed":     {"origins": ALLOWED_ORIGINS},
    r"/start_tracking": {"origins": ALLOWED_ORIGINS},
    r"/health":         {"origins": ALLOWED_ORIGINS},
})

# Globals
camera: Optional[Camera] = None
motion_detector: Optional[MotionDetector] = None
tracker: Optional[ObjectTracker] = None
tracking_enabled = False

def _log_startup():
    logging.info("🚀 Video server starting…")
    if PUBLIC_BASE_URL:
        base = PUBLIC_BASE_URL if "://" in PUBLIC_BASE_URL else (f"https://{PUBLIC_BASE_URL}" if SSL_ENABLED else f"http://{PUBLIC_BASE_URL}:{PORT}")
        logging.info(f"   🌐 Public: {base}/video_feed")
    if PUBLIC_BASE_URL_ALT:
        alt = PUBLIC_BASE_URL_ALT if "://" in PUBLIC_BASE_URL_ALT else f"http://{PUBLIC_BASE_URL_ALT}:{PORT}"
        logging.info(f"   🌐 Alt:    {alt}/video_feed")
    if not PUBLIC_BASE_URL and not PUBLIC_BASE_URL_ALT:
        logging.info("   ℹ️ Set PUBLIC_BASE_URL(_ALT) for friendly logs (no raw IPs).")
    logging.info(f"   🔐 CORS: {'* (dev fallback)' if ALLOWED_ORIGINS=='*' else f'{len(CORS_ORIGINS or [])} allowed origin(s)'}")

def _init_video_stack() -> bool:
    global camera, motion_detector, tracker
    if cv2 is None:
        logging.warning("OpenCV not installed — video pipeline disabled.")
        camera = motion_detector = tracker = None
        return False
    try:
        camera = Camera(device=CAMERA_DEVICE, width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
        motion_detector = MotionDetector()
        tracker = ObjectTracker()
        _ = camera.get_frame()  # probe; OK if None
        return True
    except Exception as e:
        logging.warning(f"Camera initialization failed ({e}). Running without camera.")
        camera = motion_detector = tracker = None
        return False

def camera_available() -> bool:
    return cv2 is not None and camera is not None

def _placeholder_generator():
    """MJPEG placeholder when no camera."""
    if cv2 is None:
        while True:
            time.sleep(1); yield b""
    import numpy as np  # requires numpy (pin numpy<2 with OpenCV 4.x)
    W, H = 640, 360; last_log = 0.0
    while True:
        frame = np.zeros((H, W, 3), dtype=np.uint8)
        cv2.putText(frame, "NO CAMERA", (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255,255,255), 3, cv2.LINE_AA)
        cv2.putText(frame, "Connect ribbon cable", (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200,200,200), 2, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", frame)
        if ok:
            jpg = buf.tobytes()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        time.sleep(0.1)
        if time.time() - last_log > 5:
            logging.info("📷 Placeholder streaming (no camera attached)."); last_log = time.time()

def generate_frames():
    """MJPEG generator with motion/tracking overlays."""
    global tracking_enabled
    if not camera_available():
        return _placeholder_generator() if PLACEHOLDER_WHEN_NO_CAMERA else (b"" for _ in iter(int,1))
    assert camera is not None and motion_detector is not None
    while True:
        frame = camera.get_frame()
        if frame is None: time.sleep(0.05); continue
        frame, _ = motion_detector.detect_motion(frame)
        if tracking_enabled and tracker is not None:
            frame, _ = tracker.update_tracking(frame)
        ok, buf = cv2.imencode(".jpg", frame)
        if not ok: logging.error("❌ JPEG encode failed."); continue
        jpg = buf.tobytes()
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")

@app.get("/video_feed")
def video_feed():
    if not camera_available() and not PLACEHOLDER_WHEN_NO_CAMERA:
        return jsonify({"ok": False, "error": "camera_unavailable"}), 503
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control":"no-store, no-cache, must-revalidate, max-age=0",
                             "Pragma":"no-cache","X-Accel-Buffering":"no","X-Content-Type-Options":"nosniff"})

@app.get("/health")
def health():
    ok = camera_available()
    return jsonify({"ok": ok, "cv2": cv2 is not None, "camera": ok,
                    "placeholder": PLACEHOLDER_WHEN_NO_CAMERA and not ok}), (200 if ok else 503)

@app.post("/start_tracking")
def start_tracking():
    global tracking_enabled
    if cv2 is None: return "❌ OpenCV not available", 400
    if not camera_available(): return "❌ Camera not available", 400
    frame = camera.get_frame()  # type: ignore[union-attr]
    if frame is None: return "❌ No frame available", 400
    try:
        if os.environ.get("DISPLAY"):
            bbox = cv2.selectROI("Select Object to Track", frame, fromCenter=False)
            if bbox and all(i > 0 for i in bbox):
                tracker.start_tracking(frame, bbox)  # type: ignore[union-attr]
                tracking_enabled = True; return "Tracking started", 200
            return "❌ No object selected", 400
        return "❌ No GUI available for object selection", 500
    except Exception as e:
        logging.error(f"⚠️ Tracking error: {e}")
        return f"❌ Tracking failed: {e}", 500

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
