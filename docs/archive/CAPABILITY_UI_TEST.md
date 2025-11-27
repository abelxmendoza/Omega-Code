# Capability UI Testing Guide

## Quick Start

### 1. Start Backend API

```bash
cd servers/robot-controller-backend
python main_api.py
```

The API should be available at `http://localhost:8000`

### 2. Start Frontend Dev Server

```bash
cd ui/robot-controller-ui
npm run dev
```

The UI should be available at `http://localhost:3000`

### 3. Test Capability API

```bash
# Test capability endpoint
curl http://localhost:8000/api/capabilities

# Should return JSON with capabilities
```

## What to Look For in Browser

### In the Header

1. **Capability Status Badge**
   - Should show current profile mode (Light/Dev/Omega)
   - Shows enabled features (ML/GPU, SLAM)
   - Shows max resolution and FPS
   - Has an info icon (ℹ️) next to it

2. **Click the Info Icon**
   - Opens a detailed modal
   - Shows current profile card with:
     - Profile name and description
     - Hardware specs (Resolution, FPS, CPU, GPU)
     - Limitations list
     - Recommendations
   - Feature comparison table
   - Current capabilities summary

### Browser DevTools

Open Chrome/Firefox DevTools (F12) and check:

1. **Network Tab**
   - Look for request to `/api/capabilities`
   - Should return 200 OK
   - Response should contain capability profile

2. **Console Tab**
   - Should see capability context loading
   - No errors about missing capabilities

3. **Elements Tab**
   - Inspect the capability status element
   - Should see the modal trigger button
   - Click it to see modal structure

## Testing Different Profiles

### Simulate MacBook Profile

The backend will auto-detect, but you can test by:

1. Check what profile is detected:
   ```bash
   curl http://localhost:8000/api/capabilities | jq '.capabilities.profile_mode'
   ```

2. Should show `"mac"` on MacBook

### Simulate Jetson Profile

If you have Jetson hardware, it should auto-detect. Otherwise, you can manually set:

```bash
# Create profile file
echo '{"profile_mode": "jetson", "ml_capable": true, "slam_capable": true, ...}' > /tmp/omega_capabilities.json
```

## UI Features to Test

1. **Capability Status in Header**
   - ✅ Visible in header
   - ✅ Shows correct profile
   - ✅ Shows enabled features
   - ✅ Info icon clickable

2. **Capability Info Modal**
   - ✅ Opens when clicking info icon
   - ✅ Shows current profile card
   - ✅ Shows hardware specs
   - ✅ Shows limitations (if any)
   - ✅ Shows recommendations
   - ✅ Feature comparison table renders
   - ✅ Current capabilities grid shows correct status

3. **Feature Comparison Table**
   - ✅ All features listed
   - ✅ Checkmarks/X marks show correctly
   - ✅ Notes appear for conditional features
   - ✅ Responsive layout

4. **Responsive Design**
   - ✅ Works on desktop
   - ✅ Works on mobile (test responsive breakpoints)

## Troubleshooting

### Modal Not Opening

- Check browser console for errors
- Verify Dialog component is imported correctly
- Check if capability context is loading

### Capabilities Not Loading

- Check Network tab for API call
- Verify backend is running
- Check CORS settings
- Verify `NEXT_PUBLIC_API_URL` is set correctly

### Wrong Profile Detected

- Check backend logs
- Verify capability detection logic
- Check `/tmp/omega_capabilities.json` file

## Expected Behavior

### MacBook Profile (Light Mode)
- Profile: Light Mode
- ML/GPU: ❌ Not shown
- SLAM: ❌ Not shown
- Max Resolution: 640x480
- Max FPS: 20
- Limitations: Should show list
- Recommendations: Should suggest upgrades

### Lenovo Profile (Dev Mode)
- Profile: Dev Mode
- ML/GPU: ❌ Not shown
- SLAM: ✅ Shown
- Max Resolution: 1280x720
- Max FPS: 25
- Limitations: Should show CPU-based limitations

### Jetson Profile (Omega Mode)
- Profile: Omega Mode
- ML/GPU: ✅ Shown
- SLAM: ✅ Shown
- Max Resolution: 1920x1080
- Max FPS: 60
- Limitations: None (or empty list)
- Recommendations: Full capabilities message

