# Hybrid System Mode API Documentation

Complete API documentation for the Omega Hybrid Vision System mode management and latency benchmarking.

## Table of Contents

1. [System Mode Management](#system-mode-management)
2. [Latency Benchmarking](#latency-benchmarking)
3. [Frontend Integration](#frontend-integration)
4. [Hybrid System Integration](#hybrid-system-integration)
5. [Thermal/CPU Watchdog](#thermalcpu-watchdog)

---

## System Mode Management

### System Modes (0-7)

| Mode | Name | Description |
|------|------|-------------|
| 0 | Camera Only | Raw MJPEG stream without processing |
| 1 | Motion Detection | Detect motion in video frames |
| 2 | Tracking | Object tracking using KCF/CSRT |
| 3 | Face Detection | Light face detection using Haar cascades |
| 4 | ArUco Detection | Detect ArUco fiducial markers |
| 5 | Recording Only | Record video without processing |
| 6 | Orin-Enhanced Detection | Hybrid mode with YOLOv8 (requires Orin) |
| 7 | Orin Navigation Mode | Full navigation with Orin AI brain (requires Orin) |

### Endpoints

#### `GET /api/system/mode/list`

List all available system modes.

**Response:**
```json
{
  "ok": true,
  "modes": {
    "0": {
      "mode": 0,
      "name": "CAMERA_ONLY",
      "description": "Camera Only - Raw MJPEG stream without processing",
      "available": true
    },
    ...
  },
  "current_mode": 0
}
```

#### `GET /api/system/mode/status`

Get current system mode status.

**Response:**
```json
{
  "ok": true,
  "mode": 0,
  "description": "Camera Only - Raw MJPEG stream without processing",
  "manual_override": false,
  "mode_name": "CAMERA_ONLY",
  "hybrid_mode": "pi_only",
  "orin_available": false,
  "thermal_temp": 45.2,
  "cpu_load": 25.5,
  "throttling": false
}
```

#### `POST /api/system/mode/set`

Set system mode.

**Request:**
```json
{
  "mode": 3
}
```

**Response:**
```json
{
  "ok": true,
  "message": "System mode set to 3",
  "mode": 3,
  "description": "Face Detection - Light face detection using Haar cascades",
  "manual_override": true,
  "mode_name": "FACE_DETECTION"
}
```

#### `GET /api/system/mode/history`

Get mode change history.

**Query Parameters:**
- `limit` (optional): Maximum number of history entries (default: 10)

**Response:**
```json
{
  "ok": true,
  "history": [
    {
      "mode": 3,
      "timestamp": 1704067200.123,
      "manual": true,
      "previous_mode": 0
    }
  ],
  "count": 1
}
```

---

## Latency Benchmarking

### Pi-Only Latency

Measures frame processing latency on the Pi (capture → encode → network).

#### `GET /latency`

Get Pi-only latency metrics.

**Response:**
```json
{
  "ok": true,
  "type": "pi_only",
  "timestamps_ns": {
    "capture_timestamp_ns": 1704067200123456789,
    "encode_start_ns": 1704067200123457890,
    "encode_end_ns": 1704067200123458901
  },
  "latencies_ms": {
    "capture_to_encode_ms": 1.10,
    "encode_duration_ms": 1.01,
    "total_processing_ms": 2.11
  },
  "ts": 1704067200123
}
```

**Metrics:**
- `capture_to_encode_ms`: Time from frame capture to encode start
- `encode_duration_ms`: JPEG encoding duration
- `total_processing_ms`: Total processing time (capture → encode end)

### Pi ↔ Orin Round-Trip Latency

Measures round-trip latency for hybrid system (Pi → Orin → Pi).

#### `GET /latency/hybrid`

Get hybrid round-trip latency metrics.

**Response:**
```json
{
  "ok": true,
  "type": "hybrid",
  "round_trip_ms": {
    "min": 45.2,
    "max": 125.8,
    "avg": 78.5,
    "count": 150
  },
  "inference_ms": {
    "min": 12.3,
    "max": 45.6,
    "avg": 25.4,
    "count": 150
  },
  "ts": 1704067200123
}
```

**Metrics:**
- `round_trip_ms`: Complete round-trip latency (Pi → Orin → Pi)
  - `min`: Minimum latency
  - `max`: Maximum latency
  - `avg`: Average latency
  - `count`: Number of samples
- `inference_ms`: Inference-only duration on Orin
  - `min`: Minimum inference time
  - `max`: Maximum inference time
  - `avg`: Average inference time
  - `count`: Number of samples

**Note:** Returns `503` if hybrid system is not available (Orin not connected).

---

## Frontend Integration

### System Mode Dashboard

**Component:** `SystemModeDashboard.tsx`

**Features:**
- Mode buttons (0-7) with visual indicators
- Real-time status polling (1 second interval)
- Thermal/CPU monitoring display
- Throttling warnings
- Manual override indicator

**Usage:**
```tsx
import SystemModeDashboard from '@/components/SystemModeDashboard';

<SystemModeDashboard />
```

### Latency Dashboard

**Component:** `LatencyDashboard.tsx`

**Features:**
- Pi-only latency metrics
- Hybrid round-trip latency metrics
- Real-time updates (500ms interval)
- Color-coded latency indicators
- Inference time display

**Usage:**
```tsx
import LatencyDashboard from '@/components/LatencyDashboard';

<LatencyDashboard />
```

### API Calls

**Set Mode:**
```typescript
const response = await fetch('/api/system/mode/set', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ mode: 3 }),
});
const data = await response.json();
```

**Get Status:**
```typescript
const response = await fetch('/api/system/mode/status');
const status = await response.json();
```

**Get Latency:**
```typescript
// Pi-only
const response = await fetch('/api/video-proxy/latency');
const latency = await response.json();

// Hybrid
const response = await fetch('/api/video-proxy/latency/hybrid');
const hybridLatency = await response.json();
```

---

## Hybrid System Integration

### System Mode Detection

The hybrid system automatically detects:
- **Pi-only mode**: Default when Orin is not available
- **Pi+Orin hybrid mode**: When Orin is detected (NVMe installed)
- **Orin-only mode**: Future mode (not yet implemented)

### Manual Mode Override

Manual mode override allows forcing a specific system mode (0-7) regardless of auto-detection.

**Backend:**
```python
from video.hybrid_system import get_hybrid_system_manager

hybrid_manager = get_hybrid_system_manager()
hybrid_manager.set_manual_mode(3)  # Force mode 3
hybrid_manager.clear_manual_mode()  # Clear override
effective_mode = hybrid_manager.get_effective_mode()  # Get current mode
```

**API:**
```bash
# Set manual mode
curl -X POST http://localhost:8000/api/system/mode/set \
  -H "Content-Type: application/json" \
  -d '{"mode": 3}'
```

### Hybrid Communication

**Pi → Orin Topics:**
- `/omega/camera/compressed` - Compressed JPEG frames (with UUID in frame_id)
- `/omega/events/aruco` - ArUco marker detections
- `/omega/events/tracking` - Tracking bounding boxes
- `/omega/events/motion` - Motion detection events
- `/omega/telemetry` - System telemetry

**Orin → Pi Topics:**
- `/omega/brain/detections` - Detection results (with UUID echo and inference duration)
- `/omega/brain/tracking` - Tracking results
- `/omega/brain/navigation` - Navigation commands
- `/omega/brain/commands` - Control commands

---

## Thermal/CPU Watchdog

### Automatic Throttling

The system automatically throttles modules when:
- CPU temperature > 70°C
- CPU load > 75%

**Throttle Priority Order:**
1. Motion detection
2. Tracking
3. ArUco detection
4. Face detection

### Auto-Switch Mode (TODO)

When throttling activates, the system can automatically switch to low-power mode (mode 1).

**Implementation Status:** Placeholder ready, needs integration with `system_state.py`

**Future Behavior:**
- Auto-switch to mode 1 (Motion Detection) when thermal > 70°C
- Auto-switch to mode 1 when CPU load > 75%
- Only auto-switch if no manual override is set
- Send WebSocket event `thermal_throttle` when throttling activates

---

## Architecture

### Backend Files

- `servers/robot-controller-backend/video/system_state.py` - System state management
- `servers/robot-controller-backend/api/system_mode_routes.py` - API endpoints
- `servers/robot-controller-backend/video/hybrid_system.py` - Hybrid system manager
- `servers/robot-controller-backend/video/pi_sensor_hub.py` - Pi sensor hub node
- `servers/robot-controller-backend/video/frame_overlays.py` - Frame overlays with latency timestamps
- `servers/robot-controller-backend/video/video_server.py` - Video server with latency endpoints

### Frontend Files

- `ui/robot-controller-ui/src/components/SystemModeDashboard.tsx` - System mode control UI
- `ui/robot-controller-ui/src/components/LatencyDashboard.tsx` - Latency visualization UI
- `ui/robot-controller-ui/src/pages/index.tsx` - Main dashboard page

### ROS2 Files

- `ros/src/omega_robot/omega_robot/orin_ai_brain.py` - Orin AI brain node with UUID echo

---

## Usage Examples

### Switch to Face Detection Mode

```bash
curl -X POST http://localhost:8000/api/system/mode/set \
  -H "Content-Type: application/json" \
  -d '{"mode": 3}'
```

### Monitor Latency

```bash
# Pi-only latency
curl http://localhost:5000/latency

# Hybrid round-trip latency
curl http://localhost:5000/latency/hybrid
```

### Get Current Status

```bash
curl http://localhost:8000/api/system/mode/status
```

---

## Error Handling

### Common Errors

**Mode Not Available:**
```json
{
  "ok": false,
  "error": "Invalid mode: 8. Must be 0-7"
}
```

**Hybrid System Not Available:**
```json
{
  "ok": false,
  "error": "hybrid_system_not_available",
  "message": "Hybrid system or Pi sensor hub not initialized"
}
```

**Frame Overlay Not Available:**
```json
{
  "ok": false,
  "error": "frame_overlay_not_available"
}
```

---

## Performance Considerations

- **Status Polling**: Frontend polls status every 1 second
- **Latency Polling**: Frontend polls latency every 500ms
- **UUID History**: Pi sensor hub maintains up to 1000 UUID entries
- **Mode History**: System state maintains up to 100 mode change entries

---

## Future Enhancements

1. **WebSocket Events**: Real-time mode updates via WebSocket
2. **Auto-Switch Mode**: Automatic mode switching based on thermal/CPU conditions
3. **Mode Presets**: Save/load mode configurations
4. **Latency Charts**: Historical latency visualization
5. **Performance Profiling**: Detailed performance breakdowns

---

## Troubleshooting

### Mode Not Switching

1. Check API endpoint: `curl http://localhost:8000/api/system/mode/status`
2. Verify backend is running: Check logs for errors
3. Check manual override: Status will show `manual_override: true`

### Latency Not Available

1. **Pi-only latency**: Ensure frame overlays are enabled
2. **Hybrid latency**: Ensure Orin is connected and ROS2 is running
3. Check endpoint: `curl http://localhost:5000/latency`

### Hybrid System Not Detected

1. Check NVMe: `ls /dev/nvme0n1`
2. Check Jetson hardware: `cat /proc/device-tree/model`
3. Force enable: `export OMEGA_ORIN_AVAILABLE=1`

---

**Last Updated:** 2024
**Version:** 1.0

