# File: /Omega-Code/servers/robot-controller-backend/video/video_server.py

"""
This script sets up a video streaming server using Flask. It captures video from a camera,
detects faces in real-time, and streams the video to clients. The server runs on the 
Tailscale IP specified in the .env file.

Key functionalities:
1. Load environment variables, including the Tailscale IP address.
2. Initialize a Flask app to handle HTTP requests.
3. Capture video from a connected camera.
4. Detect faces in the video frames using Haar Cascades.
5. Stream the video frames to connected clients via a Flask route.
6. Capture a still image and save it to a file via a Flask route.
"""

# File: /Omega-Code/servers/robot-controller-backend/video/video_server.py

import os
import cv2
from flask import Flask, Response
from dotenv import load_dotenv

load_dotenv('.env')
TAILSCALE_IP_PI = os.getenv('TAILSCALE_IP_PI')

app = Flask(__name__)

class VideoStreaming:
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.capture = cv2.VideoCapture(0)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    def generate(self):
        while True:
            ret, frame = self.capture.read()
            if not ret:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
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
    app.run(host=TAILSCALE_IP_PI, port=5000)
