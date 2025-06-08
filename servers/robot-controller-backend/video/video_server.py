"""
📌 File: video/video_server.py

💡 Summary:
This module provides a Flask-based video streaming server that integrates:
✅ Real-time video feed from the Raspberry Pi camera
✅ Motion detection using frame difference analysis (see: video/motion_detection.py)
✅ Object tracking with OpenCV (see: video/object_tracking.py)
✅ Secure access via SSL (if enabled)
✅ Local and Tailscale support for remote access
✅ Optimized to handle CORS and Mixed Content issues

🛠️ Features:
- Streams video via an MJPEG stream
- Highlights detected motion in real-time
- Allows object tracking on a selected region
- Supports SSL encryption for secure streaming
- Enables CORS for frontend access
- Runs efficiently on Raspberry Pi hardware
"""

import os
import time
import warnings
try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    warnings.warn("OpenCV not installed. Video streaming disabled.", ImportWarning)
import logging
from flask import Flask, Response, request
from flask_cors import CORS  # Added for CORS support
from dotenv import load_dotenv
from video.camera import Camera
from video.motion_detection import MotionDetector
from video.object_tracking import ObjectTracker

# Load environment variables
load_dotenv('.env')
PI_IP = os.getenv("PI_IP", "0.0.0.0")  # Default to 0.0.0.0 if not specified
TAILSCALE_IP_PI = os.getenv("TAILSCALE_IP_PI", None)

# SSL Configuration
CERT_PATH = os.getenv("CERT_PATH", None)
KEY_PATH = os.getenv("KEY_PATH", None)

# Determine the appropriate IP to use
HOST_IP = TAILSCALE_IP_PI if TAILSCALE_IP_PI else PI_IP

# Flask Application
app = Flask(__name__)
CORS(app)  # Enable CORS to allow frontend access

# Initialize camera and modules
camera = Camera(device="/dev/video0", width=640, height=480)  # Explicitly define the device
motion_detector = MotionDetector()
tracker = ObjectTracker()

# Tracking state
tracking_enabled = False

# Set up logging
logging.basicConfig(level=logging.INFO)


def generate_frames():
    """ 
    Video streaming generator function.
    Continuously captures frames, applies processing, and encodes them for streaming.
    """
    global tracking_enabled

    if cv2 is None:
        logging.warning("OpenCV not installed. No video frames will be generated.")
        while True:
            time.sleep(1)
            yield (b'')

    while True:
        frame = camera.get_frame()
        if frame is None:
            continue

        # Apply motion detection
        frame, motion_detected = motion_detector.detect_motion(frame)

        # Apply object tracking if enabled
        if tracking_enabled:
            frame, tracking_active = tracker.update_tracking(frame)

        # Encode frame in JPEG format
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            logging.error("❌ Error encoding frame to JPEG format.")
            continue

        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    """ 
    Streams the video feed with motion detection & tracking.
    The stream is accessible via an MJPEG-compatible player or browser.
    """
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/start_tracking', methods=['POST'])
def start_tracking():
    """ 
    Enables object tracking by selecting a bounding box. 
    Requires a GUI display to manually select a tracking region.
    """
    global tracking_enabled
    frame = camera.get_frame()

    if frame is None or cv2 is None:
        logging.error("❌ Camera or OpenCV not available")
        return "❌ Camera not available", 400

    try:
        logging.info("📌 Waiting for object selection...")

        # Ensure a GUI is available for selection
        if os.environ.get("DISPLAY") and cv2 is not None:
            bbox = cv2.selectROI("Select Object to Track", frame, fromCenter=False)
            if bbox and all(i > 0 for i in bbox):  # Validate bounding box
                tracker.start_tracking(frame, bbox)
                tracking_enabled = True
                logging.info("✅ Tracking started.")
                return "Tracking started", 200
            else:
                logging.warning("⚠️ No object selected.")
                return "❌ No object selected", 400
        else:
            logging.warning("⚠️ No DISPLAY environment found, cannot use ROI selection.")
            return "❌ No GUI available for object selection", 500

    except Exception as e:
        logging.error(f"⚠️ Error in object tracking: {e}")
        return f"❌ Tracking failed: {e}", 500


if __name__ == '__main__':
    logging.info("🚀 Video server starting...")
    logging.info(f"   🌐 Local: http://{PI_IP}:5000/video_feed")
    if TAILSCALE_IP_PI:
        logging.info(f"   🏴‍☠️ Tailscale: http://{TAILSCALE_IP_PI}:5000/video_feed")

    # Start Flask server with optional SSL
    try:
        if CERT_PATH and KEY_PATH and os.path.exists(CERT_PATH) and os.path.exists(KEY_PATH):
            logging.info("🔒 Running with SSL enabled!")
            app.run(host=HOST_IP, port=5000, ssl_context=(CERT_PATH, KEY_PATH))
        else:
            logging.info("⚡ Running without SSL.")
            app.run(host=HOST_IP, port=5000)
    except Exception as e:
        logging.error(f"🔥 Error starting video server: {e}")
