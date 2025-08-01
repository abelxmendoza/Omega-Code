"""
📌 Object Tracking for Raspberry Pi Camera

✅ Tracks an object in real-time using OpenCV's tracking algorithms
✅ Highlights the object with a bounding box
✅ Allows tracking a manually selected region
"""

import warnings
try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    warnings.warn("OpenCV not installed. Object tracking disabled.", ImportWarning)

class ObjectTracker:
    def __init__(self, tracker_type="CSRT"):
        """ Initialize object tracker. Supported types: CSRT, KCF, MIL, MOSSE """
        if cv2 is None:
            warnings.warn("OpenCV not available. ObjectTracker disabled.", RuntimeWarning)
            self.tracker = None
        else:
            OPENCV_OBJECT_TRACKERS = {
                "CSRT": cv2.TrackerCSRT_create,
                "KCF": cv2.TrackerKCF_create,
                "MIL": cv2.TrackerMIL_create,
                "MOSSE": cv2.legacy.TrackerMOSSE_create,
            }
            self.tracker = OPENCV_OBJECT_TRACKERS[tracker_type]()
        self.bounding_box = None  # Bounding box of the object being tracked
        self.tracking = False

    def start_tracking(self, frame, bounding_box):
        """ Initialize tracking with a selected bounding box. """
        if self.tracker is None:
            return
        self.bounding_box = bounding_box
        self.tracking = self.tracker.init(frame, bounding_box)

    def update_tracking(self, frame):
        """ Update object tracking and draw bounding box. """
        if self.tracker is None or not self.tracking:
            return frame, False

        success, box = self.tracker.update(frame)
        if success:
            (x, y, w, h) = [int(v) for v in box]
            if cv2 is not None:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Blue box
            return frame, True
        else:
            self.tracking = False
            return frame, False

