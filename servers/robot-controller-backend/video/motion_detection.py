"""
📌 Motion Detection for Raspberry Pi Camera

✅ Detects motion by comparing consecutive frames
✅ Highlights areas where movement occurs
✅ Can trigger actions (e.g., alerts, logging)
"""

import cv2
import numpy as np

class MotionDetector:
    def __init__(self, sensitivity=25):
        self.previous_frame = None
        self.sensitivity = sensitivity  # Adjust this to control motion sensitivity

    def detect_motion(self, frame):
        """ Detect motion by comparing current frame with the previous one. """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        gray = cv2.GaussianBlur(gray, (21, 21), 0)  # Smooth to reduce noise

        if self.previous_frame is None:
            self.previous_frame = gray
            return frame, False  # No motion detected

        # Compute the absolute difference between the current frame and previous frame
        delta_frame = cv2.absdiff(self.previous_frame, gray)
        self.previous_frame = gray  # Update previous frame

        # Apply threshold to get regions with motion
        threshold_frame = cv2.threshold(delta_frame, self.sensitivity, 255, cv2.THRESH_BINARY)[1]
        threshold_frame = cv2.dilate(threshold_frame, None, iterations=2)

        # Find contours of moving objects
        contours, _ = cv2.findContours(threshold_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        motion_detected = False
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Ignore small movements
                motion_detected = True
                (x, y, w, h) = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw box around movement

        return frame, motion_detected
