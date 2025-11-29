# Raspberry Pi Camera Troubleshooting Guide

## Issue 1: Port Already in Use

**Error**: `[Errno 98] Address already in use`

**Solution**:
```bash
# Find what's using port 5000
sudo lsof -i :5000
# or
sudo netstat -tulpn | grep :5000

# Kill the process (replace PID with actual process ID)
sudo kill -9 <PID>

# Or kill all Python processes (if safe to do so)
pkill -f video_server.py

# Or use a different port
export VIDEO_PORT=5001
python3 video_server.py
```

## Issue 2: Camera Not Detected (V4L2 Errors)

**Error**: `Could not open V4L2 device: /dev/video0`

**Solutions**:

### Option A: Use Picamera2 (Recommended for Pi Camera Module)

The Pi Camera Module (CSI ribbon) should use Picamera2, not V4L2:

```bash
# Set camera backend to picamera2
export CAMERA_BACKEND=picamera2

# Or in your .env file
echo "CAMERA_BACKEND=picamera2" >> .env

# Run video server
python3 video_server.py
```

### Option B: Check Camera Module Connection

```bash
# Check if camera is detected
libcamera-hello --list-cameras

# Test camera
libcamera-hello -t 5000

# Check camera info
vcgencmd get_camera
# Should show: supported=1 detected=1
```

### Option C: Enable Camera Interface

```bash
# Enable camera interface
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable

# Reboot
sudo reboot
```

### Option D: Check USB Webcam (if using USB camera)

```bash
# List video devices
ls -la /dev/video*

# Test with v4l2-ctl
sudo apt install v4l-utils
v4l2-ctl --list-devices

# Check camera capabilities
v4l2-ctl --device=/dev/video0 --all
```

## Issue 3: OpenCV Tracker Not Available

**Error**: `module 'cv2' has no attribute 'TrackerCSRT_create'`

**Solution**: This is now fixed! The code will automatically:
- Check which trackers are available
- Fall back to available trackers (KCF, MOSSE, MIL)
- Disable tracking gracefully if none are available

**To verify OpenCV version**:
```bash
python3 -c "import cv2; print(cv2.__version__)"
```

**To install OpenCV with contrib modules** (if needed):
```bash
# For Pi OS (Debian-based)
sudo apt update
sudo apt install python3-opencv python3-opencv-contrib

# Or build from source (takes longer)
# See: https://docs.opencv.org/master/d6/d15/tutorial_building_tegra_cuda.html
```

## Issue 4: Mock Camera Fallback

If no real camera is available, the server will try to use a mock camera. The mock camera:
- Generates test patterns
- Works without hardware
- Useful for testing

**To force mock camera**:
```bash
export CAMERA_BACKEND=mock
python3 video_server.py
```

## Quick Fix Script

Create a script `fix_pi_camera.sh`:

```bash
#!/bin/bash
# Fix common Pi camera issues

echo "🔍 Checking camera setup..."

# Kill existing video server
echo "1. Killing existing video server processes..."
pkill -f video_server.py
sleep 2

# Check camera detection
echo "2. Checking camera detection..."
if command -v libcamera-hello &> /dev/null; then
    libcamera-hello --list-cameras
else
    echo "   libcamera-hello not found. Install with: sudo apt install libcamera-apps"
fi

# Check camera interface
echo "3. Checking camera interface..."
vcgencmd get_camera

# Set environment for Picamera2
echo "4. Setting camera backend to picamera2..."
export CAMERA_BACKEND=picamera2

# Check if Picamera2 is installed
echo "5. Checking Picamera2 installation..."
python3 -c "from picamera2 import Picamera2; print('✅ Picamera2 available')" 2>/dev/null || echo "❌ Picamera2 not installed. Install with: pip3 install picamera2"

echo ""
echo "✅ Setup check complete!"
echo "Run: python3 video_server.py"
```

Make it executable:
```bash
chmod +x fix_pi_camera.sh
./fix_pi_camera.sh
```

## Recommended Pi Camera Setup

1. **Use Picamera2 for CSI Camera Module**:
   ```bash
   export CAMERA_BACKEND=picamera2
   ```

2. **Use V4L2 for USB Webcam**:
   ```bash
   export CAMERA_BACKEND=v4l2
   export CAMERA_DEVICE=/dev/video0
   ```

3. **Auto-detect** (default):
   ```bash
   export CAMERA_BACKEND=auto
   # Will try picamera2 first, then v4l2
   ```

## Environment Variables

Add to `.env` file in `servers/robot_controller_backend/`:

```bash
# Camera backend (auto, picamera2, v4l2)
CAMERA_BACKEND=picamera2

# Camera device (for V4L2)
CAMERA_DEVICE=/dev/video0

# Resolution
CAMERA_WIDTH=640
CAMERA_HEIGHT=480
CAMERA_FPS=30

# Video server port
VIDEO_PORT=5000

# Disable features if needed
FACE_RECOGNITION=0  # Disable face recognition on Pi (CPU intensive)
```

## Testing Camera

```bash
# Test with libcamera
libcamera-hello -t 5000

# Test with Python
python3 << EOF
from picamera2 import Picamera2
import time

picam2 = Picamera2()
picam2.start()
time.sleep(2)
frame = picam2.capture_array()
print(f"Camera working! Frame shape: {frame.shape}")
picam2.stop()
EOF
```

## Still Having Issues?

1. **Check logs**: Look for specific error messages
2. **Verify hardware**: Ensure camera is properly connected
3. **Check permissions**: May need to add user to video group:
   ```bash
   sudo usermod -a -G video $USER
   # Log out and back in
   ```
4. **Update system**:
   ```bash
   sudo apt update
   sudo apt upgrade
   sudo reboot
   ```

---

**Last Updated**: 2024

