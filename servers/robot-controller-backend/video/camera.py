"""
📌 File: video/camera.py

💡 Summary:
This module provides a `Camera` class for handling video capture on a Raspberry Pi.
It supports real-time streaming, capturing images, and ensures smooth frame retrieval 
through a background thread.

🛠️ Features:
✅ Streams live video frames using OpenCV
✅ Captures images on demand
✅ Runs video processing on a background thread (prevents Flask from blocking)
✅ Ensures frame format consistency (BGR)
✅ Uses MJPEG mode for improved compatibility
✅ Supports Motion Detection (see: video/motion_detection.py)
✅ Supports Object Tracking (see: video/object_tracking.py)

"""

import cv2
import threading
import time

class Camera:
    def __init__(self, device="/dev/video0", width=640, height=480):
        """
        Initializes the camera module.
        
        Parameters:
        - device (str/int): Camera device path or index.
        - width (int): Frame width.
        - height (int): Frame height.
        """
        self.device = device
        self.width = width
        self.height = height
        self.running = False
        self.frame = None
        self.lock = threading.Lock()

        # Initialize camera
        self.capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f"❌ Error: Could not open camera at {self.device}")

        # Set video properties
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # Ensure MJPEG mode
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # Start the frame capture thread
        self.running = True
        self.thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.thread.start()
        time.sleep(0.5)  # Give thread time to initialize

    def _capture_frames(self):
        """ Continuously captures frames in a background thread. """
        while self.running:
            ret, frame = self.capture.read()
            if not ret or frame is None:
                print("⚠️ Warning: Failed to capture frame.")
                continue

            # Ensure frame is 3-channel BGR format
            if len(frame.shape) == 2:  # Grayscale fix
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[-1] != 3:
                print("⚠️ Warning: Unexpected frame format, converting to BGR.")
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            with self.lock:
                self.frame = frame

    def get_frame(self):
        """ Returns the latest captured frame. """
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def capture_image(self, filename="snapshot.jpg"):
        """ Captures and saves an image. """
        frame = self.get_frame()
        if frame is not None:
            cv2.imwrite(filename, frame)
            return filename
        return None

    def stop(self):
        """ Stops the camera and releases resources. """
        self.running = False
        self.thread.join()
        self.capture.release()
        print("✅ Camera stopped and released.")

# Standalone Test (Run this script to test the camera)
if __name__ == "__main__":
    try:
        cam = Camera()
        print("🎥 Camera is running. Press 'q' to quit.")

        while True:
            frame = cam.get_frame()
            if frame is not None:
                cv2.imshow("Camera Feed", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
    except Exception as e:
        print(f"🔥 Error: {e}")
    finally:
        cam.stop()
        cv2.destroyAllWindows()
