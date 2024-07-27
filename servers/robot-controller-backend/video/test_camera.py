import cv2

def test_camera():
    # List of video device paths
    video_devices = ['/dev/video10', '/dev/video11', '/dev/video12', '/dev/video13', '/dev/video14', '/dev/video15', '/dev/video16']

    for device in video_devices:
        cap = cv2.VideoCapture(device)
        if cap.isOpened():
            print(f"Successfully opened camera at {device}")
            print("Press 'q' to quit.")
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Error: Could not read frame.")
                    break
                cv2.imshow('Camera Feed', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            cap.release()
            cv2.destroyAllWindows()
            return
        else:
            print(f"Error: Could not open camera at {device}")

    print("Error: Could not open any camera.")

if __name__ == "__main__":
    test_camera()

