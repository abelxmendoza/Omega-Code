"""
📌 Object Tracking for Raspberry Pi Camera

✅ Tracks an object in real-time using OpenCV's tracking algorithms
✅ Highlights the object with a bounding box
✅ Allows tracking a manually selected region
"""

import cv2

class ObjectTracker:
    def __init__(self, tracker_type="CSRT"):
        """ Initialize object tracker. Supported types: CSRT, KCF, MIL, MOSSE """
        OPENCV_OBJECT_TRACKERS = {
            "CSRT": cv2.TrackerCSRT_create,
            "KCF": cv2.TrackerKCF_create,
            "MIL": cv2.TrackerMIL_create,
            "MOSSE": cv2.TrackerMOSSE_create,
        }
        self.tracker = OPENCV_OBJECT_TRACKERS[tracker_type]()
        self.bounding_box = None  # Bounding box of the object being tracked
        self.tracking = False

    def start_tracking(self, frame, bounding_box):
        """ Initialize tracking with a selected bounding box. """
        self.bounding_box = bounding_box
        self.tracking = self.tracker.init(frame, bounding_box)

    def update_tracking(self, frame):
        """ Update object tracking and draw bounding box. """
        if not self.tracking:
            return frame, False

        success, box = self.tracker.update(frame)
        if success:
            (x, y, w, h) = [int(v) for v in box]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Blue box
            return frame, True
        else:
            self.tracking = False
            return frame, False

