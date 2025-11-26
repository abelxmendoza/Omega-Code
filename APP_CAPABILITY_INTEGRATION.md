# App-Wide Capability Integration Guide

## Overview

The capability detection system has been integrated throughout the entire application - backend, frontend, and video server. The system automatically adapts features based on detected hardware capabilities.

## Architecture

### Backend (Python)

**`servers/robot-controller-backend/api/capability_service.py`**
- Detects system capabilities (Jetson, CUDA, ROS2)
- Provides singleton service for capability queries
- Integrates with ROS2 capability profile if available
- Falls back to local detection

**`servers/robot-controller-backend/api/capability_routes.py`**
- REST API endpoints:
  - `GET /api/capabilities` - Get full capability profile
  - `GET /api/capabilities/check?feature=X` - Check specific feature
  - `GET /api/capabilities/resolution` - Get max resolution
  - `GET /api/capabilities/fps` - Get max FPS
  - `GET /api/capabilities/profile` - Get profile mode

**Video Server Integration**
- `video/video_server.py` now uses capability service for default resolution/FPS
- Automatically adapts camera settings based on profile

### Frontend (React/TypeScript)

**`src/hooks/useCapabilities.ts`**
- React hook to fetch and use capabilities
- Auto-refreshes every 30 seconds
- Provides convenience getters: `isMLCapable`, `isSLAMCapable`, etc.

**`src/context/CapabilityContext.tsx`**
- React context provider for app-wide capability access
- Wraps entire app in `_app.tsx`
- Provides hooks: `useCapabilityContext()`, `useIsMLCapable()`, etc.

**`src/components/capability/CapabilityStatus.tsx`**
- Displays current profile mode and capabilities
- Shows badges for enabled features
- Color-coded by profile (green=Jetson, blue=Lenovo, gray=Mac)

**`src/components/capability/CapabilityGate.tsx`**
- Conditional rendering component
- `<CapabilityGate feature="ml_capable">` - Only renders if ML available
- `<ProfileGate mode="jetson">` - Only renders in Jetson mode

## Usage Examples

### Backend Usage

```python
from api.capability_service import get_capability_service

service = get_capability_service()

# Check capabilities
if service.is_ml_capable():
    # Use GPU processing
    pass

# Get max resolution
width, height = service.get_max_resolution()
fps = service.get_max_fps()
```

### Frontend Usage

#### Using Hook Directly

```tsx
import { useCapabilities } from '@/hooks/useCapabilities';

function MyComponent() {
  const { isMLCapable, isSLAMCapable, profileMode } = useCapabilities();
  
  return (
    <div>
      {isMLCapable && <GPUFeatures />}
      {isSLAMCapable && <SLAMFeatures />}
    </div>
  );
}
```

#### Using Context

```tsx
import { useCapabilityContext } from '@/context/CapabilityContext';

function MyComponent() {
  const { isMLCapable, maxResolution } = useCapabilityContext();
  
  return <div>Max resolution: {maxResolution}</div>;
}
```

#### Using Capability Gate

```tsx
import { CapabilityGate, ProfileGate } from '@/components/capability';

function Dashboard() {
  return (
    <div>
      {/* Only show YOLO if ML capable */}
      <CapabilityGate feature="ml_capable">
        <YOLODetection />
      </CapabilityGate>
      
      {/* Only show SLAM in Jetson mode */}
      <ProfileGate mode="jetson">
        <SLAMVisualization />
      </ProfileGate>
      
      {/* Show fallback if not available */}
      <CapabilityGate 
        feature="face_recognition" 
        fallback={<p>Face recognition requires Jetson</p>}
      >
        <FaceRecognition />
      </CapabilityGate>
    </div>
  );
}
```

#### Displaying Status

```tsx
import { CapabilityStatus } from '@/components/capability';

function Header() {
  return (
    <header>
      <CapabilityStatus />
    </header>
  );
}
```

## API Endpoints

### Get Full Profile

```bash
curl http://localhost:8000/api/capabilities
```

Response:
```json
{
  "ok": true,
  "capabilities": {
    "profile_mode": "jetson",
    "ml_capable": true,
    "slam_capable": true,
    "max_resolution": "1920x1080",
    "max_fps": 60,
    ...
  }
}
```

### Check Specific Feature

```bash
curl http://localhost:8000/api/capabilities/check?feature=ml_capable
```

Response:
```json
{
  "ok": true,
  "feature": "ml_capable",
  "available": true
}
```

## Integration Points

### Video Server

The video server automatically uses capability-aware defaults:

```python
# video_server.py now uses:
CAMERA_WIDTH = capability_service.get_max_resolution()[0]
CAMERA_HEIGHT = capability_service.get_max_resolution()[1]
CAMERA_FPS = capability_service.get_max_fps()
```

### Header Component

The Header component displays capability status in the top bar, showing:
- Profile mode (Omega/Dev/Light)
- Enabled features (ML/GPU, SLAM)
- Max resolution and FPS

### Component Gating

Components can be conditionally rendered:

```tsx
// Only show advanced features if ML capable
<CapabilityGate feature="ml_capable">
  <AdvancedVisionFeatures />
</CapabilityGate>

// Only show in Jetson mode
<ProfileGate mode="jetson">
  <GPUAcceleratedFeatures />
</ProfileGate>
```

## Capability Features

| Feature | Light Mode | Dev Mode | Omega Mode |
|---------|-----------|----------|------------|
| `ml_capable` | ❌ | ❌ | ✅ |
| `slam_capable` | ❌ | ✅ | ✅ |
| `tracking` | ✅ | ✅ | ✅ |
| `aruco` | ✅ | ✅ | ✅ |
| `motion_detection` | ✅ | ✅ | ✅ |
| `face_recognition` | ❌ | ✅ (slow) | ✅ (fast) |
| `yolo` | ❌ | ❌ | ✅ |

## Testing

### Test Backend API

```bash
# Start backend
cd servers/robot-controller-backend
python main_api.py

# Test endpoints
curl http://localhost:8000/api/capabilities
curl http://localhost:8000/api/capabilities/check?feature=ml_capable
```

### Test Frontend

```bash
# Start frontend
cd ui/robot-controller-ui
npm run dev

# Open browser console
# Capabilities should be logged on load
# Check Network tab for API calls
```

## Troubleshooting

### Capabilities Not Loading

1. Check backend is running: `curl http://localhost:8000/api/capabilities`
2. Check CORS settings in `main_api.py`
3. Check browser console for errors
4. Verify `NEXT_PUBLIC_API_URL` is set correctly

### Wrong Profile Detected

1. Check ROS2 capability file: `cat /tmp/omega_capabilities.json`
2. Run detection script: `./scripts/apply_profile.sh`
3. Force refresh: `curl http://localhost:8000/api/capabilities?refresh=true`

### Components Not Showing/Hiding

1. Check capability context is provided (in `_app.tsx`)
2. Verify feature name matches capability profile key
3. Check browser console for errors
4. Use React DevTools to inspect capability context

## Next Steps

- [ ] Add capability-aware UI themes
- [ ] Show capability warnings in UI
- [ ] Add capability-based feature recommendations
- [ ] Create capability comparison view
- [ ] Add capability history/trends

---

**Status**: ✅ Fully Integrated  
**Last Updated**: 2024

