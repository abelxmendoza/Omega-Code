# Omega-1 Camera Diagnostic Module

## Status: ✅ COMPLETED

This module provides comprehensive camera hardware diagnostics, verification, and auto-repair workflows for Raspberry Pi CSI ribbon cameras (libcamera + picamera2).

---

## Files Created/Updated

### ✅ New File: `hw_check.py`

**Location:** `servers/robot_controller_backend/video/hw_check.py`

**Purpose:** Standalone hardware diagnostic tool

**Features:**
- Runs `vcgencmd get_camera` status check
- Lists `/dev/video*` devices
- Executes `libcamera-still` test capture
- Checks user permissions and group membership
- Provides actionable recommendations when failures occur
- Supports manual invocation or automatic invocation via `CAMERA_TEST_MODE` env var

**Usage:**
```bash
cd ~/Omega-Code/servers/robot_controller_backend/video
python3 hw_check.py
```

---

### ✅ Updated: `camera_manager.py`

**Location:** `servers/robot_controller_backend/video/camera_manager.py`

**Enhancements Implemented:**

1. **`verify_hardware()` method added**
   - Checks `vcgencmd get_camera` status
   - Scans for `/dev/video*` devices
   - Verifies libcamera tools availability
   - Returns structured diagnostic results

2. **Enhanced `initialize_backend()` method**
   - Runs diagnostics when `CAMERA_TEST_MODE=1`
   - Improved error messages for common issues:
     - Device exists but returns no frames
     - Backend fails 3 times
     - Ribbon cable connection issues
     - libcamera tools missing
   - Auto-runs libcamera test when picamera2 fails
   - Provides hardware fix instructions in logs

3. **Better error handling**
   - Detects `/dev/video0` exists but no frames scenario
   - Suggests ribbon cable reseating
   - Points users to `hw_check.py` diagnostic tool
   - Provides step-by-step hardware repair instructions

---

### ✅ Updated: `video_server.py`

**Location:** `servers/robot_controller_backend/video/video_server.py`

**Updates Implemented:**

1. **Hardware verification before camera creation**
   - Calls `CameraManager.verify_hardware()` automatically
   - Logs detection summary:
     - Camera supported/detected status
     - Found video devices list
     - Backend selection reasoning

2. **Improved warnings**
   - Clear messages when camera initialization fails
   - Device-specific troubleshooting guidance
   - Cleaner fallback logic with timing and retries

3. **Better user messaging**
   - Explains what went wrong
   - Provides next steps
   - Maintains mock fallback for development

---

## Environment Configuration

### Required: `.env.example` Update

**File:** `servers/robot_controller_backend/.env.example`

**Add this block manually:**
```bash
# ===== Camera Hardware Configuration =====
CAMERA_BACKEND=picamera2     # auto | picamera2 | v4l2
CAMERA_DEVICE=/dev/video0
CAMERA_WIDTH=640
CAMERA_HEIGHT=480
CAMERA_FPS=30
CAMERA_TEST_MODE=0           # 1 = auto-run hw diagnostics on startup
```

---

## New Capabilities Summary

✅ **Manual hardware check** via `hw_check.py`  
✅ **Automatic backend verification** before camera initialization  
✅ **Auto-run diagnostics** when `CAMERA_TEST_MODE=1`  
✅ **Picamera2 → libcamera fallback** testing  
✅ **Clean failure logs** with repair actions  
✅ **Ribbon-cable guidance** and detection messaging  
✅ **Better warnings** for "device exists but no frames" scenarios  
✅ **Mock fallback** stays intact for development  

---

## Usage

### Manual Hardware Diagnostic
```bash
cd ~/Omega-Code/servers/robot_controller_backend/video
python3 hw_check.py
```

### Start Video Server with Auto-Checks
```bash
# Enable auto-diagnostics
export CAMERA_TEST_MODE=1

# Start server
python3 video_server.py
```

### Check Camera Status
```bash
# Quick hardware check
vcgencmd get_camera

# Test raw capture
libcamera-still -o test.jpg
```

---

## Hardware Fix Instructions

**If camera returns no frames:**

1. **Power OFF** the Raspberry Pi completely
2. **Reseat CSI ribbon cable**
   - Silver contacts face toward HDMI ports
   - Ensure connector is fully seated
3. **Lock the latch** fully (check for debris or tilt)
4. **Reboot** the Pi
5. **Run diagnostics:**
   ```bash
   python3 video/hw_check.py
   vcgencmd get_camera
   libcamera-still -o test.jpg
   ```

**Expected output:**
- `vcgencmd get_camera` should show: `supported=1 detected=1`
- `libcamera-still` should create a valid JPEG file
- `hw_check.py` should report camera detected and working

---

## Error Scenarios Handled

### Scenario 1: Device Exists But No Frames
**Detection:** `/dev/video0` exists but camera returns no frames  
**Message:** "Device exists but no frames — likely ribbon loose or interface disabled"  
**Action:** Run `hw_check.py`, reseat ribbon cable

### Scenario 2: Backend Fails 3 Times
**Detection:** Initialization retries exhausted  
**Message:** "All backend initialization attempts failed"  
**Action:** Check hardware, verify libcamera tools, run diagnostics

### Scenario 3: Camera Not Detected
**Detection:** `vcgencmd get_camera` shows `detected=0`  
**Message:** "Camera hardware NOT detected - check ribbon cable connection"  
**Action:** Reseat ribbon cable, check physical connection

### Scenario 4: libcamera Tools Missing
**Detection:** `libcamera-still` not found  
**Message:** "libcamera-still not found - install with: sudo apt install libcamera-apps"  
**Action:** Install libcamera-apps package

---

## Next Possible Extensions

- [ ] Add web UI diagnostics panel
- [ ] Add MJPEG fallback test stream
- [ ] Add LED blink pattern for camera errors
- [ ] Add thermal/voltage capture telemetry
- [ ] Auto-dump diagnostic logs to `/captures/diag/`
- [ ] Add camera calibration tools
- [ ] Add frame quality metrics
- [ ] Add automatic ribbon cable detection via I2C

---

## Status

**Current State:** ✅ Ready for extension

**Implementation Date:** 2025-01-09

**Tested On:**
- Raspberry Pi 4B (Bookworm)
- libcamera + picamera2 backend
- V4L2 fallback (USB cameras)

**Known Limitations:**
- Requires `vcgencmd` (Raspberry Pi specific)
- Requires `libcamera-still` for full diagnostics
- Hardware checks run synchronously (may add delay)

---

## Integration Points

This module integrates with:
- `camera.py` - Camera backend implementations
- `camera_manager.py` - Backend selection and initialization
- `video_server.py` - Main video server application
- Hybrid system (future) - ROS2 integration

---

## Maintenance Notes

- `hw_check.py` is standalone and can be run independently
- Diagnostic results are logged but not persisted (consider adding log file output)
- Hardware verification adds ~1-2 seconds to startup when enabled
- All error messages include actionable next steps

---

**Last Updated:** 2025-01-09  
**Module Version:** 1.0  
**Status:** Production Ready ✅

