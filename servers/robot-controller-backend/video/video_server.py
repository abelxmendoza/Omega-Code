# File: /Omega-Code/servers/robot-controller-backend/video/video_server.py
"""
Hardened MJPEG video server for the robot.

Adds:
- Background camera (re)init retries if startup fails
- Optional stall watchdog to recreate the camera if frames go stale
- Keeps /video_feed and /health usable while reconnecting (UI shows 'connecting')

Env:
  BIND_HOST                (default "0.0.0.0")
  VIDEO_PORT               (default "5000")
  ORIGIN_ALLOW             (comma-separated origins, else "*" dev fallback)
  PUBLIC_BASE_URL(_ALT)    (pretty logs only)
  CERT_PATH, KEY_PATH      (TLS if both exist)
  CAMERA_*                 (see camera.py)
  PLACEHOLDER_WHEN_NO_CAMERA = 0|1

  # New:
  STARTUP_RETRY_SEC        (default 5)    # retry camera init when it fails
  RESTART_ON_STALL         (default 1)    # 1=auto recreate on stall
  STALE_MS                 (default 2500) # what 'stale' means
  WATCHDOG_PERIOD_MS       (default 1500)
"""

import os
import time
import logging
from typing import Optional, List

try:
    import cv2  # noqa: F401
except Exception:
    cv2 = None  # type: ignore

from flask import Flask, Response, jsonify
from flask_cors import CORS
from dotenv import load_dotenv, find_dotenv

# Local
from .aruco_detection import ArucoDetector
from .camera import Camera
from .face_recognition import FaceRecognizer
from .motion_detection import MotionDetector
from .object_tracking import ObjectTracker

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

CAMERA_WIDTH   = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT  = int(os.getenv("CAMERA_HEIGHT", "480"))
PLACEHOLDER_WHEN_NO_CAMERA = os.getenv("PLACEHOLDER_WHEN_NO_CAMERA", "0").lower() in ("1", "true", "yes")

STARTUP_RETRY_SEC = int(os.getenv("STARTUP_RETRY_SEC", "5"))
RESTART_ON_STALL  = os.getenv("RESTART_ON_STALL", "1").lower() in ("1", "true", "yes")
STALE_MS          = int(os.getenv("STALE_MS", os.getenv("FRAME_STALE_MS", "2500")))
WATCHDOG_PERIOD_MS= int(os.getenv("WATCHDOG_PERIOD_MS", "1500"))

FACE_RECOGNITION_ENABLED = os.getenv("FACE_RECOGNITION", "1").lower() in ("1", "true", "yes", "on")
FACE_RECOGNITION_THRESHOLD = float(os.getenv("FACE_RECOGNITION_THRESHOLD", "0.6"))
KNOWN_FACES_DIR = os.getenv("KNOWN_FACES_DIR") or None

ARUCO_DETECTION_ENABLED = os.getenv("ARUCO_DETECTION", "1").lower() in ("1", "true", "yes", "on")

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logging.getLogger("werkzeug").setLevel(logging.WARNING)

app = Flask(__name__)
CORS(app, resources={
    r"/video_feed":     {"origins": ALLOWED_ORIGINS},
    r"/start_tracking": {"origins": ALLOWED_ORIGINS},
    r"/health":         {"origins": ALLOWED_ORIGINS},
    r"/face_recognition/reload": {"origins": ALLOWED_ORIGINS},
})

# Globals
camera: Optional[Camera] = None
motion_detector: Optional[MotionDetector] = None
tracker: Optional[ObjectTracker] = None
face_recognizer: Optional[FaceRecognizer] = None
aruco_detector: Optional[ArucoDetector] = None
tracking_enabled = False
_last_init_attempt = 0.0

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



def _log_startup() -> None:
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


def _create_camera() -> bool:
    global camera, motion_detector, tracker, _last_init_attempt
    _last_init_attempt = time.time()
    try:
        camera = Camera(width=CAMERA_WIDTH, height=CAMERA_HEIGHT)
        motion_detector = MotionDetector()
        tracker = ObjectTracker()
        logging.info("Initialization successful.")
        return True
    except Exception as e:
        logging.warning(f"Camera initialization failed ({e}). Running without camera.")
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
        if not camera_present():
            if PLACEHOLDER_WHEN_NO_CAMERA:
                yield from _placeholder_generator()
                return
            else:
                # no camera and no placeholder → end stream (UI will retry)
                yield b""
                return

        assert camera is not None and motion_detector is not None
        frame = camera.get_frame()
        if frame is None:
            time.sleep(0.03)
            continue

        # Motion overlay
        frame, _ = motion_detector.detect_motion(frame)

        # Tracker overlay (optional)
        if tracking_enabled and tracker is not None:
            frame, _ = tracker.update_tracking(frame)

        if FACE_RECOGNITION_ENABLED and face_recognizer and face_recognizer.active:
            frame, _ = face_recognizer.annotate(frame)

        if ARUCO_DETECTION_ENABLED and aruco_detector and aruco_detector.active:
            frame, _ = aruco_detector.annotate(frame)

        # Encode JPEG
        import cv2 as _cv2  # type: ignore
        ok, buf = _cv2.imencode(".jpg", frame)
        if not ok:
            logging.error("❌ JPEG encode failed.")
            continue

        jpg = buf.tobytes()
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")


@app.get("/video_feed")
def video_feed():
    """
    MJPEG stream endpoint.
    """
    # If there’s no camera yet and placeholder is disabled, tell the client to try later
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
        "ts": int(time.time() * 1000),
    }
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
            camera.stop()  # type: ignore[union-attr]
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
            app.run(host=BIND_HOST, port=PORT, ssl_context=(CERT_PATH, KEY_PATH), use_reloader=False)
        else:
            logging.info("⚡ Running without SSL")
            app.run(host=BIND_HOST, port=PORT, use_reloader=False)
    except Exception as e:
        logging.error(f"🔥 Error starting video server: {e}")
