import warnings
try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore
    warnings.warn("OpenCV not installed. Camera test disabled.", ImportWarning)

def test_camera():
    if cv2 is None:
        print("OpenCV not installed. Skipping camera test.")
        return
    device = '/dev/video0'  # Correct device based on your previous outputs
    gst_str = f"v4l2src device={device} ! videoconvert ! appsink"
    
    # Open the camera using GStreamer pipeline
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print(f"❌ Error: Could not open camera at {device}")
        return

    print(f"✅ Successfully opened camera at {device}")
    print("🎥 Press 'q' to quit.")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("❌ Error: Could not read frame. Exiting...")
            break

        cv2.imshow('Camera Feed', frame)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release camera and close window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera()
