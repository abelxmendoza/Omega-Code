#!/usr/bin/env python3
"""
Camera Test Script
Tests different camera backends and devices to find available cameras
"""

import cv2
import sys
import os

def test_camera_devices():
    """Test different camera devices"""
    print("🔍 Testing camera devices...")
    
    # Test different camera indices
    for i in range(5):
        print(f"Testing camera index {i}...")
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"✅ Camera {i} is working! Resolution: {frame.shape}")
                cap.release()
                return i
            cap.release()
        else:
            print(f"❌ Camera {i} not available")
    
    # Test V4L2 devices
    v4l2_devices = ["/dev/video0", "/dev/video1", "/dev/video2"]
    for device in v4l2_devices:
        if os.path.exists(device):
            print(f"Testing V4L2 device {device}...")
            cap = cv2.VideoCapture(device)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print(f"✅ V4L2 device {device} is working! Resolution: {frame.shape}")
                    cap.release()
                    return device
                cap.release()
            else:
                print(f"❌ V4L2 device {device} not working")
        else:
            print(f"❌ V4L2 device {device} does not exist")
    
    return None

def test_picamera2():
    """Test Picamera2 backend"""
    print("🔍 Testing Picamera2 backend...")
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        picam2.start()
        frame = picam2.capture_array()
        print(f"✅ Picamera2 is working! Resolution: {frame.shape}")
        picam2.stop()
        picam2.close()
        return True
    except Exception as e:
        print(f"❌ Picamera2 not available: {e}")
        return False

def main():
    print("🎥 Camera Detection Test")
    print("=" * 50)
    
    # Test OpenCV
    print(f"OpenCV version: {cv2.__version__}")
    
    # Test Picamera2
    picam2_ok = test_picamera2()
    
    # Test V4L2/OpenCV cameras
    camera_device = test_camera_devices()
    
    print("\n📋 Summary:")
    print(f"Picamera2: {'✅ Available' if picam2_ok else '❌ Not available'}")
    print(f"V4L2/OpenCV: {'✅ Available' if camera_device else '❌ Not available'}")
    
    if camera_device:
        print(f"Working camera: {camera_device}")
        print("\n💡 To use this camera, set environment variable:")
        if isinstance(camera_device, int):
            print(f"export CAMERA_DEVICE={camera_device}")
        else:
            print(f"export CAMERA_DEVICE={camera_device}")
    
    if not picam2_ok and not camera_device:
        print("\n⚠️  No cameras found!")
        print("💡 Possible solutions:")
        print("1. Connect a USB camera")
        print("2. Enable camera permissions")
        print("3. Install camera drivers")
        print("4. Use a virtual camera for testing")

if __name__ == "__main__":
    main()
