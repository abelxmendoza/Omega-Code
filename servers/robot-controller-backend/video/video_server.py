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

import os
import cv2
import socket
import struct
import numpy as np
from flask import Flask, Response, jsonify
from dotenv import load_dotenv

# Load environment variables from .env file in the current directory and parent directory
load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))
load_dotenv(os.path.join(os.path.dirname(__file__), '../../config/.env'))  # Load the parent .env file

# Access the Tailscale IP from environment variables
TAILSCALE_IP_PI = os.getenv('TAILSCALE_IP_PI')

# Initialize the Flask app
app = Flask(__name__)

class VideoStreaming:
    """
    A class to handle video streaming and face detection.
    """
    def __init__(self):
        # Load the Haar Cascade for face detection
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.video_Flag = True
        self.face_x = 0
        self.face_y = 0
        # Initialize the video capture object
        self.capture = cv2.VideoCapture(0)
        # Set lower resolution
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    def face_detect(self, img):
        """
        Detect faces in the given image and draw circles around them.

        Args:
            img (numpy.ndarray): The input image in which faces are to be detected.

        Returns:
            numpy.ndarray: The image with detected faces highlighted.
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3)
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                self.face_x = float(x + w / 2.0)
                self.face_y = float(y + h / 2.0)
                img = cv2.circle(img, (int(self.face_x), int(self.face_y)), int((w + h) / 4), (0, 255, 0), 2)
        else:
            self.face_x = 0
            self.face_y = 0
        return img

    def generate(self):
        """
        Generate video frames from the camera and yield them in a format suitable for HTTP streaming.

        Yields:
            bytes: The next frame in the video stream encoded as JPEG.
        """
        frame_counter = 0
        while True:
            ret, frame = self.capture.read()
            if not ret:
                continue
            # Process every 5th frame
            if frame_counter % 5 == 0:
                if self.video_Flag:
                    frame = self.face_detect(frame)
                    self.video_Flag = False
            frame_counter += 1
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Create an instance of the VideoStreaming class
video_stream = VideoStreaming()

@app.route('/video_feed')
def video_feed():
    """
    Flask route to provide video feed.
    
    Returns:
        Response: A Flask response object that streams video frames.
    """
    return Response(video_stream.generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Run the Flask app on the Tailscale IP and port 5000
    app.run(host=TAILSCALE_IP_PI, port=5000)
