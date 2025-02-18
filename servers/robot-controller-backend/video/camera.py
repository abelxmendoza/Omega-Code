"""
📌 Camera Module for Raspberry Pi

This module provides a `Camera` class that:
✅ Streams live video frames using OpenCV
✅ Captures images on demand
✅ Runs video processing on a background thread (prevents Flask from blocking)
✅ Is designed to be used in Flask-based video streaming servers

🎯 Future Features:
- Motion detection (to detect movement)
- Object tracking (to track specific objects)
"""

import cv2
import threading

class Camera:
    def __init__(self, device=0, width=640, height=480):
        self.device = device
        self.width = width
        self.height = height

        # Initialize OpenCV video capture
        self.capture = cv2.VideoCapture(self.device)
        if not self.capture.isOpened():
            raise RuntimeError(f"❌ Error: Could not open camera at {self.device}")

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # Frame buffer
        self.frame = None
        self.running = True
        self.lock = threading.Lock()

        # Start background frame capture thread
        self.thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.thread.start()

    def _capture_frames(self):
        """ Continuously captures frames in a background thread. """
        while self.running:
            ret, frame = self.capture.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def get_frame(self):
        """ Returns the latest frame captured. """
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def capture_image(self, filename="snapshot.jpg"):
        """ Captures a still image and saves it. """
        frame = self.get_frame()
        if frame is not None:
            cv2.imwrite(filename, frame)
            return filename
        return None

    def stop(self):
        """ Stops the camera capture and releases resources. """
        self.running = False
        self.thread.join()
        self.capture.release()

# Standalone Test (Run this to check if the camera works)
if __name__ == "__main__":
    cam = Camera()
    print("Press 'q' to quit.")
    while True:
        frame = cam.get_frame()
        if frame is not None:
            cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cam.stop()
    cv2.destroyAllWindows()
