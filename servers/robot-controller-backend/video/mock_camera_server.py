#!/usr/bin/env python3
"""
Mock Camera Server
Provides a test video feed when no real camera is available
"""

import cv2
import numpy as np
import time
import threading
from flask import Flask, Response, jsonify
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

class MockCamera:
    """Mock camera that generates test patterns"""
    
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.running = False
        self.frame = None
        self.thread = None
        
    def start(self):
        """Start mock camera"""
        self.running = True
        self.thread = threading.Thread(target=self._generate_frames)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        """Stop mock camera"""
        self.running = False
        if self.thread:
            self.thread.join()
            
    def _generate_frames(self):
        """Generate test frames"""
        frame_count = 0
        while self.running:
            # Create a test pattern
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            
            # Add some moving elements
            center_x = self.width // 2
            center_y = self.height // 2
            
            # Moving circle
            angle = frame_count * 0.1
            circle_x = int(center_x + 100 * np.cos(angle))
            circle_y = int(center_y + 100 * np.sin(angle))
            cv2.circle(frame, (circle_x, circle_y), 30, (0, 255, 0), -1)
            
            # Moving rectangle
            rect_x = int(center_x + 150 * np.sin(angle * 0.7))
            rect_y = int(center_y + 80 * np.cos(angle * 0.7))
            cv2.rectangle(frame, (rect_x-20, rect_y-20), (rect_x+20, rect_y+20), (255, 0, 0), -1)
            
            # Add text
            cv2.putText(frame, f"Mock Camera Frame {frame_count}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(frame, f"Time: {time.strftime('%H:%M:%S')}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Add some noise
            noise = np.random.randint(0, 50, (self.height, self.width, 3), dtype=np.uint8)
            frame = cv2.add(frame, noise)
            
            self.frame = frame
            frame_count += 1
            time.sleep(1/30)  # 30 FPS
            
    def get_frame(self):
        """Get current frame"""
        return self.frame

# Global mock camera
mock_camera = MockCamera()

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    def generate():
        while True:
            frame = mock_camera.get_frame()
            if frame is not None:
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', frame)
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(1/30)

    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status():
    """Camera status"""
    return jsonify({
        'status': 'running',
        'camera_type': 'mock',
        'resolution': f"{mock_camera.width}x{mock_camera.height}",
        'fps': 30
    })

@app.route('/')
def index():
    """Main page"""
    return """
    <html>
    <head>
        <title>Mock Camera Server</title>
    </head>
    <body>
        <h1>Mock Camera Server</h1>
        <p>This is a test camera server for when no real camera is available.</p>
        <img src="/video_feed" width="640" height="480">
        <p><a href="/status">Status</a></p>
    </body>
    </html>
    """

if __name__ == '__main__':
    print("🎥 Starting Mock Camera Server...")
    print("📹 This provides a test video feed when no real camera is available")
    print("🌐 Access at: http://localhost:5000")
    print("📺 Video feed: http://localhost:5000/video_feed")
    
    mock_camera.start()
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n🛑 Stopping mock camera server...")
    finally:
        mock_camera.stop()
