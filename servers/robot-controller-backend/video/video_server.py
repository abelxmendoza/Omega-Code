import os
import cv2
from flask import Flask, Response
from dotenv import load_dotenv

# Load environment variables
load_dotenv('.env')

# Get IP addresses from environment variables
PI_IP = os.getenv('PI_IP', '0.0.0.0')  # Default to 0.0.0.0 if not set
TAILSCALE_IP_PI = os.getenv('TAILSCALE_IP_PI', None)  # Can be None if not set

# SSL Configuration
CERT_PATH = os.getenv('CERT_PATH', None)
KEY_PATH = os.getenv('KEY_PATH', None)

# Ensure Flask listens on all interfaces
HOST_IP = "0.0.0.0"

app = Flask(__name__)

class VideoStreaming:
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        # Open the camera
        self.capture = cv2.VideoCapture(0)  # Change this if using a different /dev/videoX
        if not self.capture.isOpened():
            raise RuntimeError("❌ Error: Could not open camera. Check your device settings.")

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    def generate(self):
        while True:
            ret, frame = self.capture.read()
            if not ret:
                continue

            # Convert to grayscale for face detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

            # Draw rectangles around detected faces
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            # Encode frame to JPEG and send it over HTTP
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

video_stream = VideoStreaming()

@app.route('/video_feed')
def video_feed():
    return Response(video_stream.generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print(f"🚀 Video server running at:")
    print(f"   🌐 Local (Omega1 Pi): http://{PI_IP}:5000/video_feed")
    if TAILSCALE_IP_PI:
        print(f"   🏴‍☠️ Tailscale: http://{TAILSCALE_IP_PI}:5000/video_feed")

    # Use SSL if certificate and key are available
    if CERT_PATH and KEY_PATH:
        print("🔒 Running with SSL enabled!")
        app.run(host=HOST_IP, port=5000, ssl_context=(CERT_PATH, KEY_PATH))
    else:
        print("⚡ Running without SSL.")
        app.run(host=HOST_IP, port=5000)
