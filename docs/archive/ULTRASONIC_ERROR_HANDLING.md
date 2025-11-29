# Ultrasonic Sensor Error Handling & Debugging Guide

## Overview

Enhanced error handling and debugging messages have been added to help diagnose and fix ultrasonic sensor issues quickly.

## Backend Improvements (Go Server)

### Enhanced Error Messages

The `measureDistance()` function now provides detailed error messages:

1. **Timeout Errors:**
   - Shows initial echo pin state
   - Indicates how long it waited
   - Provides troubleshooting hints

2. **Wiring Errors:**
   - Detects if echo pin is stuck HIGH
   - Identifies trigger pin failures
   - Suggests specific GPIO pins to check

3. **Range Errors:**
   - Validates distance range (2-400cm)
   - Explains when readings are too close (<2cm)
   - Shows pulse duration for debugging

### Error Tracking

- **Error Count:** Tracks total errors vs successes
- **Consecutive Errors:** Monitors repeated failures
- **Recovery Detection:** Logs when sensor recovers
- **Diagnostic Hints:** Auto-suggests troubleshooting after 5 consecutive errors

### Example Error Messages

```
[ULTRA] ⚠️  Measurement failed: timeout waiting for echo HIGH (initial state: Low, waited 100ms). 
        Check: 1) Sensor power (5V), 2) Echo wiring (GPIO22/Pin15), 3) Sensor may be faulty
        (errors: 10/15, consecutive: 5)

[ULTRA] 🔧 Troubleshooting: Check sensor power (5V), wiring (GPIO27/22), 
        and run: go run test_ultrasonic_hardware.go
```

## Frontend Improvements (React UI)

### Error Display

The UI now shows:

1. **Error Banner:**
   - Red error box with detailed message
   - Troubleshooting checklist (after 3+ consecutive errors)
   - Quick diagnostic commands

2. **Status Indicators:**
   - Connection status (connecting/connected/disconnected)
   - Error count and consecutive errors
   - Last successful reading timestamp

3. **Visual Feedback:**
   - Status dot changes color based on health
   - Invalid readings highlighted in yellow
   - Error messages with actionable steps

### Error Tracking

- **Error Count:** Total errors since connection
- **Consecutive Errors:** Current streak of failures
- **Last Success:** Timestamp of last valid reading
- **Recovery Detection:** Shows when sensor recovers

### Console Logging

Enhanced console messages:

```
[ULTRASONIC] ✅ WebSocket connected: ws://100.93.225.61:8080/ultrasonic
[ULTRASONIC] ✅ Welcome message received - sensor ready
[ULTRASONIC] ⚠️  Sensor error: timeout waiting for echo HIGH...
[ULTRASONIC] Error count: 5, Consecutive: 5
[ULTRASONIC] 🔧 Hardware issue detected! Check: 1) Power (5V), 2) Wiring (GPIO27/22), 3) Run diagnostic
[ULTRASONIC] ✅ Sensor recovered after 5 errors
[ULTRASONIC] 📏 Distance: 25 cm (0.25m)
```

## UI Error Display Examples

### Minor Error (1-2 errors)
```
⚠️ Sensor Error:
timeout waiting for echo HIGH (initial state: Low, waited 100ms)
```

### Major Error (3+ consecutive)
```
⚠️ Sensor Error:
timeout waiting for echo HIGH... (5 consecutive errors - sensor may need hardware check)

🔧 Troubleshooting:
• Check sensor power: VCC → 5V, GND → GND
• Verify wiring: Trigger → GPIO27 (Pin13), Echo → GPIO22 (Pin15)
• Run diagnostic: go run test_ultrasonic_hardware.go
• Check GPIO permissions: sudo usermod -a -G gpio $USER
```

### Connection Error
```
❌ All connection attempts failed. Check server is running on port 8080.
```

## Debugging Workflow

### Step 1: Check Console Logs

Look for:
- Connection status
- Error messages with details
- Error counts and patterns

### Step 2: Check UI Error Display

- Red error banner shows current issue
- Troubleshooting checklist appears after 3+ errors
- Status dot indicates connection health

### Step 3: Run Diagnostic Script

```bash
cd servers/robot_controller_backend/sensors
go run test_ultrasonic_hardware.go
```

This will:
- Test GPIO pin configuration
- Verify trigger pulse generation
- Check echo pin response
- Measure pulse duration
- Provide hardware-specific diagnostics

### Step 4: Check Hardware

Based on error messages:
- **"timeout waiting for echo HIGH"** → Check power and echo wiring
- **"echo stuck HIGH"** → Check echo pin wiring
- **"trigger pin failed"** → Check trigger pin wiring
- **"distance out of range"** → Sensor may be faulty or object too close/far

## Error Message Reference

| Error Message | Meaning | Solution |
|--------------|---------|----------|
| `timeout waiting for echo HIGH` | Echo pin never goes HIGH after trigger | Check power (5V), echo wiring (GPIO22), sensor may be faulty |
| `timeout waiting for echo LOW` | Echo pin stuck HIGH | Check echo wiring, may be shorted or wrong pin |
| `trigger pin failed` | Can't control trigger pin | Check trigger wiring (GPIO27), GPIO permissions |
| `distance out of range` | Reading < 2cm or > 400cm | Object too close/far, or sensor fault |
| `distance too close (<2cm)` | Reading < 2cm (likely noise) | Move object away, check for interference |

## Status Indicators

### Status Dot Colors

- 🟢 **Green (connected):** WebSocket connected, sensor responding
- 🟡 **Yellow (connecting):** Attempting to connect
- 🔴 **Red (disconnected):** Connection failed or sensor errors

### UI Status Text

- `✅ Sensor active • Last reading: 2:30:45 PM` - Working normally
- `⚠️ Sensor Error: timeout...` - Hardware issue detected
- `❌ Connection error` - WebSocket connection failed

## Best Practices

1. **Monitor Console:** Check browser console for detailed logs
2. **Check Error Banner:** UI shows current issues prominently
3. **Run Diagnostic:** Use `test_ultrasonic_hardware.go` for hardware testing
4. **Check Wiring:** Verify all connections match error message suggestions
5. **Test Power:** Ensure sensor has stable 5V supply
6. **Verify GPIO:** Check permissions and pin assignments

## Summary

✅ **Detailed error messages** with troubleshooting hints  
✅ **Error tracking** (count, consecutive, recovery)  
✅ **Visual error display** in UI with actionable steps  
✅ **Enhanced console logging** for debugging  
✅ **Automatic diagnostic suggestions** after repeated errors  
✅ **Connection status indicators** for quick health check  

The system now provides comprehensive diagnostics to quickly identify and fix sensor issues!

