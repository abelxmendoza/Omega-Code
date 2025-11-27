# Video Server - Complete Feature List

## 🎉 All Features Implemented!

The video server now includes **ALL** requested improvements and more!

---

## ✅ Core Features

### 1. **Video Recording** 📹
- Record video streams to MP4 files
- Hardware-aware encoding (H.264 on Jetson, XVID on Pi)
- Automatic file rotation when size limit reached
- Start/stop via API
- File management (auto-cleanup old files)

**Endpoints:**
- `POST /recording/start` - Start recording (optional filename)
- `POST /recording/stop` - Stop recording
- `GET /recording/status` - Get recording status

**Example:**
```bash
# Start recording
curl -X POST http://localhost:5000/recording/start \
  -H "Content-Type: application/json" \
  -d '{"filename": "my_recording.mp4"}'

# Check status
curl http://localhost:5000/recording/status

# Stop recording
curl -X POST http://localhost:5000/recording/stop
```

---

### 2. **Multiple Resolution Streams** 📐
- Simultaneous streams at different resolutions/qualities
- Low: 320x240 @ quality 60 (for slow connections)
- Medium: 640x480 @ quality 75 (default)
- High: 1280x720 @ quality 90 (for high bandwidth)

**Endpoints:**
- `GET /video_feed` - Main stream (configurable)
- `GET /video_feed_low` - Low quality stream
- `GET /video_feed_medium` - Medium quality stream
- `GET /video_feed_high` - High quality stream

**Usage:**
```html
<!-- Low bandwidth -->
<img src="http://pi:5000/video_feed_low">

<!-- High quality -->
<img src="http://pi:5000/video_feed_high">
```

---

### 3. **WebSocket Support** 🔌
- Real-time metrics broadcasting
- Low-latency status updates
- Multiple client support
- Automatic reconnection

**Setup:**
```bash
pip install flask-socketio
```

**Usage:**
```javascript
// Connect to WebSocket
const socket = io('http://pi:5000');

socket.on('connect', () => {
  console.log('Connected!');
  socket.emit('subscribe_metrics');
});

socket.on('metrics', (data) => {
  console.log('Metrics:', data);
});
```

---

### 4. **Frame Overlays** 🖼️
- Timestamp overlay
- FPS counter (color-coded)
- Telemetry overlay (battery, temperature, CPU)
- Customizable via API

**Endpoints:**
- `POST /overlays/config` - Configure overlays

**Example:**
```bash
curl -X POST http://localhost:5000/overlays/config \
  -H "Content-Type: application/json" \
  -d '{
    "show_timestamp": true,
    "show_fps": true,
    "show_telemetry": true,
    "telemetry": {
      "battery": 85.5,
      "temperature": 45.2,
      "cpu_usage": 35.0
    }
  }'
```

---

### 5. **Multi-Camera Support** 📷
- Switch between multiple cameras at runtime
- List available cameras
- No restart required

**Endpoints:**
- `GET /camera/list` - List available cameras
- `POST /camera/switch` - Switch to different camera

**Example:**
```bash
# List cameras
curl http://localhost:5000/camera/list

# Switch camera
curl -X POST http://localhost:5000/camera/switch \
  -H "Content-Type: application/json" \
  -d '{"device": "/dev/video1"}'
```

---

### 6. **Frame Buffering** 📦
- Thread-safe frame buffer
- Reduces frame drops
- Smooth playback for slow clients
- Statistics tracking

**Features:**
- Configurable buffer size (default: 5 frames)
- Drop tracking
- Age monitoring

---

### 7. **ROS2 Integration** 🤖
- Publish frames to ROS2 topics
- Compressed image support (efficient)
- Raw image support
- Background publishing thread

**Topics:**
- `/omega/camera/image_raw` - Raw uncompressed images
- `/omega/camera/image_raw/compressed` - JPEG compressed (recommended)

**Setup:**
```bash
# Enable ROS2
export ENABLE_ROS2=1

# Ensure ROS2 is sourced
source /opt/ros/rolling/setup.bash
```

**View frames:**
```bash
ros2 topic echo /omega/camera/image_raw/compressed
```

---

## 🚀 Hardware Optimizations

All modules are hardware-aware:

| Feature | Pi 4B | Jetson | Mac/Linux |
|---------|-------|--------|-----------|
| **JPEG Quality** | 75 | 90 | 85 |
| **Max FPS** | 30 | 60 | 20 |
| **Recording Codec** | XVID | H.264 | mp4v |
| **Frame Skipping** | Yes (CPU >75%) | No | No |
| **Adaptive Quality** | Yes | No | No |
| **Motion Detection** | Optimized | High Quality | Balanced |
| **Object Tracking** | KCF (fast) | CSRT (quality) | KCF |

---

## 📊 Performance Features

### Metrics Endpoint
- `GET /metrics` - Comprehensive performance metrics
- Includes: FPS, frame counts, errors, buffer stats, recording status

### Health Endpoint Enhanced
- `GET /health` - Extended health check
- Includes: Hardware info, recording status, buffer stats, overlay config

---

## 🎛️ Configuration

### Environment Variables

```bash
# Core Settings
VIDEO_PORT=5000
BIND_HOST=0.0.0.0
JPEG_QUALITY=85          # Auto-adjusted by hardware
MAX_FPS_LIMIT=0          # 0 = unlimited, auto-adjusted

# Features
ENABLE_RECORDING=1       # Enable video recording
ENABLE_OVERLAYS=1        # Enable frame overlays
ENABLE_FRAME_BUFFER=1    # Enable frame buffering
ENABLE_MULTI_RESOLUTION=0 # Enable multi-res streams
ENABLE_ROS2=0            # Enable ROS2 publishing
ENABLE_METRICS=1         # Enable metrics endpoint

# Recording
RECORDING_DIR=/tmp/omega_recordings
MAX_FILE_SIZE_MB=100     # Max recording file size
MAX_RECORDING_FILES=10   # Max files to keep

# Camera
CAMERA_DEVICE=/dev/video0
CAMERA_WIDTH=640
CAMERA_HEIGHT=480
CAMERA_FPS=30

# Features (auto=hardware-aware)
FACE_RECOGNITION=auto    # auto | 1 | 0
ARUCO_DETECTION=auto     # auto | 1 | 0
```

---

## 📝 API Reference

### Video Streams
- `GET /video_feed` - Main MJPEG stream
- `GET /video_feed_low` - Low quality (320x240)
- `GET /video_feed_medium` - Medium quality (640x480)
- `GET /video_feed_high` - High quality (1280x720)

### Control
- `GET /health` - Health check
- `GET /metrics` - Performance metrics
- `GET /snapshot` - Capture single frame (`?save=1&quality=85`)
- `POST /config` - Update runtime config

### Recording
- `POST /recording/start` - Start recording
- `POST /recording/stop` - Stop recording
- `GET /recording/status` - Recording status

### Camera Management
- `GET /camera/list` - List available cameras
- `POST /camera/switch` - Switch camera device

### Overlays
- `POST /overlays/config` - Configure frame overlays

### Features
- `POST /start_tracking` - Start object tracking
- `POST /face_recognition/reload` - Reload known faces

---

## 🔧 New Modules

### `video_recorder.py`
- Hardware-aware video recording
- MP4 file output
- Automatic rotation
- File management

### `frame_overlays.py`
- Timestamp overlay
- FPS counter
- Telemetry display
- Customizable

### `frame_buffer.py`
- Thread-safe buffering
- Frame queue management
- Statistics tracking

### `websocket_server.py`
- Real-time metrics
- Multi-client support
- Flask-SocketIO integration

### `ros2_integration.py`
- ROS2 frame publishing
- Compressed image support
- Background thread

---

## 🎯 Usage Examples

### Start Recording
```bash
curl -X POST http://localhost:5000/recording/start \
  -H "Content-Type: application/json" \
  -d '{"filename": "test_recording.mp4"}'
```

### Switch Camera
```bash
curl -X POST http://localhost:5000/camera/switch \
  -H "Content-Type: application/json" \
  -d '{"device": "/dev/video1"}'
```

### Configure Overlays
```bash
curl -X POST http://localhost:5000/overlays/config \
  -H "Content-Type: application/json" \
  -d '{
    "show_timestamp": true,
    "show_fps": true,
    "show_telemetry": true,
    "telemetry": {"battery": 85, "temperature": 45}
  }'
```

### Get Metrics
```bash
curl http://localhost:5000/metrics | jq
```

---

## 🏆 Summary

**✅ All Features Implemented:**
1. ✅ Video recording (MP4)
2. ✅ Multiple resolution streams
3. ✅ WebSocket support
4. ✅ Frame overlays
5. ✅ Multi-camera support
6. ✅ Frame buffering
7. ✅ ROS2 integration
8. ✅ Hardware optimizations
9. ✅ Performance monitoring
10. ✅ Enhanced API endpoints

**🚀 Ready for Production!**

The video server is now a **complete, production-ready** solution with all requested features and more!

