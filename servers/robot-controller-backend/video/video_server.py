"""
📌 File: video/video_server.py

Secure, hostname/proxy-friendly MJPEG video server for the robot.
- Hides raw IPs: never logs numeric addresses
- CORS restricted to allowed origins (hostnames/domains)
- Optional SSL
- Motion detection + object tracking preserved
"""

import os
import time
import warnings
import logging
from typing import List, Optional

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    warnings.warn("OpenCV not installed. Video streaming disabled.", ImportWarning)

from flask import Flask, Response, jsonify
from flask_cors import CORS
from dotenv import load_dotenv

from video.camera import Camera
from video.motion_detection import MotionDetector
from video.object_tracking import ObjectTracker

# ---------- Config ----------
load_dotenv('.env')

# Where Flask binds internally. Keep this private; don't log it.
BIND_HOST = os.getenv("BIND_HOST", "0.0.0.0")
PORT = int(os.getenv("VIDEO_PORT", "5000"))

# Public, user-facing base URLs (hostnames only). Used ONLY for friendly logs.
# e.g. PUBLIC_BASE_URL=https://robot.ts.net
#      PUBLIC_BASE_URL_ALT=http://robot.local:5000
PUBLIC_BASE_URL = os.getenv("PUBLIC_BASE_URL")          # hostname / domain only
PUBLIC_BASE_URL_ALT = os.getenv("PUBLIC_BASE_URL_ALT")  # optional secondary

# SSL (optional)
CERT_PATH = os.getenv("CERT_PATH") or None
KEY_PATH = os.getenv("KEY_PATH") or None
SSL_ENABLED = bool(CERT_PATH and KEY_PATH and os.path.exists(CERT_PATH) and os.path.exists(KEY_PATH))

# CORS: comma-separated hostnames/domains (no IPs)
# Example: ORIGIN_ALLOW=http://localhost:3000,https://robot.ts.net,http://robot.local
_ORIGINS = [o.strip() for o in (os.getenv("ORIGIN_ALLOW", "")).split(",") if o.strip()]
CORS_ORIGINS: Optional[List[str]] = _ORIGINS or None

# Reduce noisy request logs that print client IPs
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logging.getLogger("werkzeug").setLevel(logging.WARNING)

# ---------- App ----------
app = Flask(__name__)
CORS(app, resources={r"/video_feed": {"origins": CORS_ORIGINS},
                     r"/start_tracking": {"origins": CORS_ORIGINS},
                     r"/health": {"origins": CORS_ORIGINS}})

# Camera + modules
camera = Camera(device="/dev/video0", width=640, height=480)
motion_detector = MotionDetector()
tracker = ObjectTracker()
tracking_enabled = False

def _log_startup():
    logging.info("🚀 Video server starting…")
    if PUBLIC_BASE_URL:
        scheme = "https" if SSL_ENABLED and PUBLIC_BASE_URL.startswith("https") else ("http" if "://" in PUBLIC_BASE_URL else None)
        base = PUBLIC_BASE_URL if "://" in PUBLIC_BASE_URL else (f"https://{PUBLIC_BASE_URL}" if SSL_ENABLED else f"http://{PUBLIC_BASE_URL}:{PORT}")
        logging.info(f"   🌐 Public: {base}/video_feed")
    if PUBLIC_BASE_URL_ALT:
        alt = PUBLIC_BASE_URL_ALT if "://" in PUBLIC_BASE_URL_ALT else f"http://{PUBLIC_BASE_URL_ALT}:{PORT}"
        logging.info(f"   🌐 Alt:    {alt}/video_feed")
    if not PUBLIC_BASE_URL and not PUBLIC_BASE_URL_ALT:
        logging.info("   ℹ️ Set PUBLIC_BASE_URL (and optionally _ALT) to a hostname for friendly logs.")
    # Never print bind IP

def generate_frames():
    """MJPEG generator with motion/tracking overlays."""
    global tracking_enabled

    if cv2 is None:
        logging.warning("OpenCV not installed. No video frames will be generated.")
        while True:
            time.sleep(1)
            yield b""

    while True:
        frame = camera.get_frame()
        if frame is None:
            time.sleep(0.01)
            continue

        # Motion detection
        frame, _motion = motion_detector.detect_motion(frame)

        # Object tracking
        if tracking_enabled:
            frame, _tracking = tracker.update_tracking(frame)

        # Encode JPEG
        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            logging.error("❌ Error encoding frame to JPEG.")
            continue

        jpg = buf.tobytes()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")

@app.get("/video_feed")
def video_feed():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.get("/health")
def health():
    # Minimal liveness check; frontend can poll this if desired
    healthy = (cv2 is not None) and (camera is not None)
    return jsonify({"ok": healthy}), (200 if healthy else 503)

@app.post("/start_tracking")
def start_tracking():
    """Enable object tracking by selecting a bounding box (requires GUI / DISPLAY)."""
    global tracking_enabled
    if cv2 is None:
        logging.error("❌ OpenCV not available")
        return "❌ OpenCV not available", 400

    frame = camera.get_frame()
    if frame is None:
        logging.error("❌ Camera not available")
        return "❌ Camera not available", 400

    try:
        if os.environ.get("DISPLAY"):
            logging.info("📌 Waiting for object selection (GUI)…")
            bbox = cv2.selectROI("Select Object to Track", frame, fromCenter=False)
            if bbox and all(i > 0 for i in bbox):
                tracker.start_tracking(frame, bbox)
                tracking_enabled = True
                logging.info("✅ Tracking started.")
                return "Tracking started", 200
            else:
                logging.warning("⚠️ No object selected.")
                return "❌ No object selected", 400
        else:
            logging.warning("⚠️ No DISPLAY; cannot use ROI selection.")
            return "❌ No GUI available for object selection", 500
    except Exception as e:
        logging.error(f"⚠️ Tracking error: {e}")
        return f"❌ Tracking failed: {e}", 500

if __name__ == "__main__":
    _log_startup()
    try:
        if SSL_ENABLED:
            logging.info("🔒 SSL enabled")
            app.run(host=BIND_HOST, port=PORT, ssl_context=(CERT_PATH, KEY_PATH))
        else:
            logging.info("⚡ Running without SSL")
            app.run(host=BIND_HOST, port=PORT)
    except Exception as e:
        logging.error(f"🔥 Error starting video server: {e}")
