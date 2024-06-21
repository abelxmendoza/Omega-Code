# File: /Omega-Code/servers/robot-controller-backend/video_server.py
import socket
import struct
import numpy as np
import cv2
from flask import Flask, Response, jsonify

app = Flask(__name__)

class VideoStreaming:
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.video_Flag = True
        self.face_x = 0
        self.face_y = 0
        self.capture = cv2.VideoCapture(0)  # Initialize the camera

    def face_detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
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
        while True:
            ret, frame = self.capture.read()
            if not ret:
                continue
            if self.video_Flag:
                frame = self.face_detect(frame)
                self.video_Flag = False
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
    app.run(host='0.0.0.0', port=5000)

