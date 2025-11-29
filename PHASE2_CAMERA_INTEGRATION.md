# Phase 2: Camera Integration

Complete camera integration with ROS2 for Omega Robot.

## 🎯 Goals

- Stream camera frames to ROS2 topics
- Display camera feed in web app
- Support both raw and compressed images
- Integrate with existing camera backend

## ✅ Implemented

### 1. Camera Publisher Node

**File**: `ros/src/omega_robot/omega_robot/camera_publisher.py`

**Features**:
- Publishes to `/camera/image_raw` (sensor_msgs/Image)
- Publishes to `/camera/image_raw/compressed` (sensor_msgs/CompressedImage)
- Integrates with existing `video/camera.py` backend
- Supports Picamera2 and OpenCV/V4L2
- Configurable resolution and FPS
- Placeholder frames when camera unavailable

**Usage**:
```bash
ros2 run omega_robot camera_publisher
```

**Parameters**:
- `width` (default: 640)
- `height` (default: 480)
- `fps` (default: 30)
- `publish_compressed` (default: true)

### 2. Web Bridge Support

**Updated**: `servers/robot_controller_backend/api/ros_web_bridge.py`

**Features**:
- Supports `Image` message type
- Supports `CompressedImage` message type
- Base64 encoding for web transmission
- Efficient JPEG compression

### 3. React Component

**File**: `ui/robot-controller-ui/src/components/ros/CameraViewer.tsx`

**Features**:
- Real-time camera feed display
- FPS counter
- Connection status
- Automatic reconnection
- Canvas-based rendering

**Usage**:
```tsx
import { CameraViewer } from '@/components/ros';

<CameraViewer 
  topic="/camera/image_raw/compressed"
  width={640}
  height={480}
/>
```

## 📡 Topics

### Camera Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | Image | Raw uncompressed image |
| `/camera/image_raw/compressed` | CompressedImage | JPEG compressed (recommended) |

### Message Formats

**CompressedImage** (Recommended for web):
```python
CompressedImage
  header:
    stamp: {sec, nanosec}
    frame_id: "camera_frame"
  format: "jpeg"
  data: [jpeg_bytes]  # Base64 encoded for web
```

**Image** (Raw):
```python
Image
  header:
    stamp: {sec, nanosec}
    frame_id: "camera_frame"
  height: 480
  width: 640
  encoding: "rgb8"
  data: [pixel_data]
```

## 🚀 Quick Start

### 1. Install Dependencies

```bash
# On Lenovo or Pi
sudo apt install ros-rolling-cv-bridge
```

### 2. Build

```bash
cd ~/omega_ws
colcon build --packages-select omega_robot
source install/setup.bash
```

### 3. Launch

```bash
# Launch with camera
ros2 launch omega_robot robot_full.launch.py

# Or just camera
ros2 run omega_robot camera_publisher
```

### 4. View in Web App

```tsx
// In your React component
<CameraViewer topic="/camera/image_raw/compressed" />
```

## 🔧 Configuration

### Camera Backend

The node uses the existing `video/camera.py` backend which supports:

- **Picamera2** (preferred for Pi CSI cameras)
- **OpenCV/V4L2** (USB webcams)
- **Mock camera** (for development)

### Environment Variables

```bash
CAMERA_BACKEND=auto          # auto | picamera2 | v4l2
CAMERA_DEVICE=/dev/video0    # For V4L2
CAMERA_WIDTH=640
CAMERA_HEIGHT=480
CAMERA_FPS=30
```

### ROS2 Parameters

```bash
ros2 run omega_robot camera_publisher \
  --ros-args \
  -p width:=1280 \
  -p height:=720 \
  -p fps:=30 \
  -p publish_compressed:=true
```

## 📊 Performance

### Recommended Settings

**For Web Streaming**:
- Resolution: 640x480
- FPS: 15-30
- Format: CompressedImage (JPEG)
- Quality: 85% (good balance)

**For Processing**:
- Resolution: 1280x720
- FPS: 30
- Format: Image (raw)
- For computer vision tasks

### Network Considerations

- **CompressedImage**: ~50-200 KB/frame (JPEG)
- **Image**: ~900 KB/frame (640x480 RGB)
- **Bandwidth**: CompressedImage uses ~90% less bandwidth

## 🎨 Web Integration

### Subscribe to Camera

```javascript
const ws = new WebSocket('ws://localhost:8000/api/ros/bridge');

// Subscribe to compressed image
ws.send(JSON.stringify({
  type: "subscribe",
  topic: "/camera/image_raw/compressed",
  msg_type: "CompressedImage"
}));

// Receive and display
ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  if (msg.type === "ros2_message") {
    const imageData = msg.data.data; // Base64 JPEG
    const img = document.createElement('img');
    img.src = `data:image/jpeg;base64,${imageData}`;
    document.body.appendChild(img);
  }
};
```

### React Component

```tsx
import { CameraViewer } from '@/components/ros';

function RobotDashboard() {
  return (
    <div>
      <CameraViewer 
        topic="/camera/image_raw/compressed"
        width={640}
        height={480}
      />
    </div>
  );
}
```

## 🔍 Testing

### Test Camera Node

```bash
# Run camera publisher
ros2 run omega_robot camera_publisher

# In another terminal, view image
ros2 run rqt_image_view rqt_image_view /camera/image_raw/compressed
```

### Test from Command Line

```bash
# Check topics
ros2 topic list | grep camera

# Get topic info
ros2 topic info /camera/image_raw/compressed

# Echo topic (shows metadata)
ros2 topic echo /camera/image_raw/compressed --once
```

### Test Web Integration

```bash
# Start backend
cd servers/robot_controller_backend
python main_api.py

# Open browser console and connect
const ws = new WebSocket('ws://localhost:8000/api/ros/bridge');
ws.send(JSON.stringify({
  type: "subscribe",
  topic: "/camera/image_raw/compressed",
  msg_type: "CompressedImage"
}));
```

## 🐛 Troubleshooting

### cv_bridge Not Found

```bash
sudo apt install ros-rolling-cv-bridge
```

### Camera Not Initializing

```bash
# Check camera backend
python3 -c "from video.camera import Camera; c = Camera(); print('OK')"

# Check permissions
ls -l /dev/video0
sudo chmod 666 /dev/video0  # If needed
```

### No Frames Published

```bash
# Check camera is working
ros2 topic echo /camera/image_raw/compressed --once

# Check node logs
ros2 node info /camera_publisher
```

### Web Display Issues

- Check WebSocket connection
- Verify base64 decoding
- Check browser console for errors
- Ensure topic is subscribed

## 📈 Next Steps

### Phase 2 Enhancements

- [ ] Camera control service (zoom, focus, exposure)
- [ ] Multiple camera support
- [ ] Image annotation overlay
- [ ] Recording to bag files
- [ ] Motion detection integration

### Phase 3 Integration

- [ ] Computer vision processing
- [ ] Object detection on camera feed
- [ ] Face recognition
- [ ] ArUco marker detection
- [ ] SLAM with camera

---

**Status**: Phase 2 Complete ✅  
**Next**: Phase 3 - Autonomous Behaviors  
**Last Updated**: 2024

