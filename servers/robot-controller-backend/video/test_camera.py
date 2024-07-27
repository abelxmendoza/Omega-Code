import cv2

def test_camera():
    device = '/dev/video14'
    cap = cv2.VideoCapture(device)

    if not cap.isOpened():
        print(f"Error: Could not open camera at {device}")
        return

    # Set the resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

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

if __name__ == "__main__":
    test_camera()
