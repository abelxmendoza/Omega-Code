"""
📌 Video Streaming Server for Raspberry Pi Camera

This Flask server:
✅ Streams live video from the Raspberry Pi camera
✅ Allows capturing a snapshot via `/capture`
✅ Supports both local and Tailscale network access
✅ Uses OpenCV for efficient video processing
✅ Runs independently from the robot control system

🎯 Future Features:
- Motion detection
- Object tracking
- WebSocket integration for real-time control
"""

import os
import cv2
from flask import Flask, Response
from dotenv import load_dotenv
from video.camera import Camera  # Import modular camera class

# Load environment variables
load_dotenv('.env')

# Get Raspberry Pi & Tailscale IPs from .env
PI_IP = os.getenv('PI_IP', '0.0.0.0')  # Local IP
TAILSCALE_IP_PI = os.getenv('TAILSCALE_IP_PI', None)  # Tailscale IP

# SSL Configuration
CERT_PATH = os.getenv('CERT_PATH', None)
KEY_PATH = os.getenv('KEY_PATH', None)

# Flask App
app = Flask(__name__)
camera = Camera(device=0, width=640, height=480)  # Initialize camera

def generate_frames():
    """ Video streaming generator function. """
    while True:
        frame = camera.get_frame()
        if frame is None:
            continue
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """ Flask route for streaming video. """
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/capture')
def capture_image():
    """ Capture and save an image. """
    filename = camera.capture_image("snapshot.jpg")
    return f"Image saved: {filename}" if filename else "Failed to capture image"

if __name__ == '__main__':
    print(f"🚀 Video server running at:")
    print(f"   🌐 Local: http://YOUR_LOCAL_IP:5000/video_feed")
    if TAILSCALE_IP_PI:
        print(f"   🏴‍☠️ Tailscale: http://YOUR_TAILSCALE_IP:5000/video_feed")

    # Start Flask server
    if CERT_PATH and KEY_PATH:
        app.run(host="0.0.0.0", port=5000, ssl_context=(CERT_PATH, KEY_PATH))
    else:
        app.run(host="0.0.0.0", port=5000)
