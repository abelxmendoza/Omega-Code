# Viewing the Capability UI Component

## Quick Access

### Option 1: View in Main App
1. Open `http://localhost:3000` in your browser
2. Look at the **top right of the header**
3. You'll see the capability status badge with an **info icon (ℹ️)**
4. **Click the info icon** to open the detailed modal

### Option 2: Direct Component Test Page
Create a test page to see just the component:

```bash
# The component is already integrated in the Header
# Just open http://localhost:3000 and look at the header
```

## What You'll See

### In the Header (Top Right)
```
[●] Light Mode  [ML/GPU] [SLAM]  640x480 @ 20fps  [ℹ️]
```

### When You Click the Info Icon
A modal opens showing:

1. **Current Profile Card**
   - Profile name and icon (💻 for MacBook)
   - Hardware specifications grid:
     - Resolution: 640x480
     - Max FPS: 20
     - CPU Cores: [your CPU count]
     - GPU: None (or Available if Jetson)

2. **Limitations Section**
   - List of what's not available
   - Yellow warning icon

3. **Recommendations Section**
   - Suggestions for upgrades
   - Blue info icon

4. **Feature Comparison Table**
   - Side-by-side comparison of all 3 profiles
   - Checkmarks (✅) for available features
   - X marks (❌) for unavailable features
   - Notes for conditional features

5. **Current Capabilities Grid**
   - Visual grid showing what you have
   - Green checkmarks for available
   - Red X marks for unavailable

## Browser DevTools Inspection

### Network Tab
- Filter for "capabilities"
- Should see: `GET /api/capabilities`
- Status: 200 OK
- Response: JSON with your capability profile

### Elements Tab
1. Right-click on the capability status in header
2. Select "Inspect"
3. You'll see the component structure:
   - `CapabilityInfoModal` wrapper
   - `DialogTrigger` button
   - `CapabilityStatus` component inside

### Console Tab
- Should see capability context loading
- No errors (CSP warnings are harmless)

## Visual Guide

```
┌─────────────────────────────────────────────────┐
│  Header                                          │
│  ... Status | Battery | [●] Light Mode [ℹ️]     │ ← Click here
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│  ⚡ System Capabilities & Hardware Information   │
│                                                  │
│  ┌──────────────────────────────────────────┐  │
│  │ 💻 Light Mode                            │  │
│  │ MacBook + Raspberry Pi          [MAC]    │  │
│  │                                          │  │
│  │ [📷 Resolution] [⚡ FPS] [💻 CPU] [🎮 GPU]│  │
│  │                                          │  │
│  │ ⚠️ Current Limitations:                  │  │
│  │ • No GPU acceleration                    │  │
│  │ • No ML inference                        │  │
│  │                                          │  │
│  │ 💡 Recommendations:                     │  │
│  │ • Use Jetson for full autonomy           │  │
│  └──────────────────────────────────────────┘  │
│                                                  │
│  Feature Comparison                             │
│  ┌─────────────┬──────────┬──────────┬────────┐ │
│  │ Feature     │ Light    │ Dev     │ Omega  │ │
│  ├─────────────┼──────────┼──────────┼────────┤ │
│  │ Motion Det. │    ✅    │    ✅    │   ✅   │ │
│  │ YOLO        │    ❌    │    ❌    │   ✅   │ │
│  │ SLAM        │    ❌    │    ✅    │   ✅   │ │
│  └─────────────┴──────────┴──────────┴────────┘ │
└─────────────────────────────────────────────────┘
```

## Troubleshooting

### Component Not Visible?
1. Check if Header component is rendering
2. Verify `CapabilityProvider` is wrapping the app in `_app.tsx`
3. Check browser console for errors

### Modal Not Opening?
1. Check Network tab for `/api/capabilities` request
2. Verify backend is running on port 8000
3. Check console for Dialog component errors

### Wrong Profile Shown?
1. Check backend API: `curl http://localhost:8000/api/capabilities`
2. Verify capability detection logic
3. Check `/tmp/omega_capabilities.json` file

## Testing Different Profiles

### Simulate Jetson Profile
```bash
# Create test profile file
cat > /tmp/omega_capabilities.json <<EOF
{
  "profile_mode": "jetson",
  "ml_capable": true,
  "slam_capable": true,
  "max_resolution": "1920x1080",
  "max_fps": 60,
  "gpu_available": true,
  "gpu_name": "NVIDIA Tegra Orin"
}
EOF
```

Then refresh the page - you should see "Omega Mode" with all features enabled!

