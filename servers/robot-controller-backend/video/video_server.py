"""
📌 Video Streaming with Motion Detection & Object Tracking

✅ Streams video from the Raspberry Pi camera
✅ Detects motion and highlights moving objects
✅ Allows object tracking
✅ Supports local & Tailscale access
"""

import os
import cv2
import logging
from flask import Flask, Response, request
from dotenv import load_dotenv
from video.camera import Camera
from video.motion_detection import MotionDetector
from video.object_tracking import ObjectTracker

# Load environment variables
load_dotenv('.env')
PI_IP = os.getenv("PI_IP", "0.0.0.0")  # Fixed closing quote
TAILSCALE_IP_PI = os.getenv("TAILSCALE_IP_PI", None)

# Flask App
app = Flask(__name__)
camera = Camera(device=0, width=640, height=480)
motion_detector = MotionDetector()
tracker = ObjectTracker()

# User selects a bounding box to track
tracking_enabled = False

# Set up logging
logging.basicConfig(level=logging.INFO)

def generate_frames():
    """ Video streaming generator function. """
    global tracking_enabled

    while True:
        frame = camera.get_frame()
        if frame is None:
            continue

        # Apply motion detection
        frame, motion_detected = motion_detector.detect_motion(frame)

        # Apply object tracking if enabled
        if tracking_enabled:
            frame, tracking_active = tracker.update_tracking(frame)

        # Encode frame
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """ Stream video with motion detection & tracking. """
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/start_tracking', methods=['POST'])
def start_tracking():
    """ Start tracking an object by selecting a bounding box. """
    global tracking_enabled
    frame = camera.get_frame()
    
    if frame is None:
        logging.error("❌ Camera not available")
        return "❌ Camera not available", 400

    try:
        logging.info("📌 Waiting for object selection...")

        # Check if running with a GUI
        if os.environ.get("DISPLAY"):
            bbox = cv2.selectROI("Select Object to Track", frame, fromCenter=False)
            if bbox and all(i > 0 for i in bbox):  # Ensure a valid bounding box
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

    # Start Flask server
    try:
        app.run(host="0.0.0.0", port=5000)
    except Exception as e:
        logging.error(f"Error starting video server: {e}")
