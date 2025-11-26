# Ultrasonic Sensor Troubleshooting Guide

## Problem: "timeout waiting for echo"

If you see repeated `⚠️ timeout waiting for echo` messages, the sensor is not responding. This guide will help you diagnose and fix the issue.

## Quick Diagnostic

Run the hardware test script:

```bash
cd servers/robot-controller-backend/sensors
go run test_ultrasonic_hardware.go
```

This will test:
1. GPIO pin configuration
2. Trigger pulse generation
3. Echo pin response
4. Distance measurement

## Common Causes & Solutions

### 1. **Power Issues** ⚡

**Symptoms:** Echo never goes HIGH, sensor appears "dead"

**Solutions:**
- ✅ Verify sensor is powered: **VCC → 5V**, **GND → GND**
- ✅ Check power supply can provide enough current (HC-SR04 needs ~15mA)
- ✅ Use a multimeter to verify 5V at VCC pin
- ✅ Try a different power source (USB power bank, separate 5V supply)

### 2. **Wiring Issues** 🔌

**Symptoms:** Intermittent readings or constant timeouts

**Wiring Checklist:**
```
HC-SR04 Pin → Raspberry Pi
─────────────────────────────
VCC         → 5V (Pin 2 or 4)
GND         → GND (Pin 6, 9, 14, 20, etc.)
Trigger     → GPIO27 (Pin 13) ✅
Echo        → GPIO22 (Pin 15) ✅
```

**Solutions:**
- ✅ Double-check all connections are secure
- ✅ Verify pins match the code (GPIO27/GPIO22)
- ✅ Check for loose wires or cold solder joints
- ✅ Try swapping trigger/echo wires (sometimes mislabeled)
- ✅ Use a breadboard for stable connections

### 3. **GPIO Permissions** 🔐

**Symptoms:** "Permission denied" errors or no response

**Solutions:**
```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER

# Log out and back in, or:
newgrp gpio

# Verify permissions
groups | grep gpio
```

### 4. **Wrong GPIO Pins** 📍

**Current Configuration:**
- Trigger: GPIO27 (Physical Pin 13)
- Echo: GPIO22 (Physical Pin 15)

**To Change Pins:**

Edit `main_ultrasonic.go`:
```go
// Around line 321-322
trigger := rpi.P1_13 // GPIO27 - change to your pin
echo := rpi.P1_15    // GPIO22 - change to your pin
```

**Common Alternatives:**
- GPIO17 (Pin 11) + GPIO18 (Pin 12)
- GPIO23 (Pin 16) + GPIO24 (Pin 18)

### 5. **Faulty Sensor** 🔧

**Symptoms:** No response even with correct wiring and power

**Solutions:**
- ✅ Test with a multimeter:
  - VCC-GND should show ~5V
  - Trigger should respond to pulses
- ✅ Try a different HC-SR04 sensor
- ✅ Test sensor on Arduino first to verify it works
- ✅ Check sensor datasheet for specifications

### 6. **Software Issues** 💻

**Symptoms:** Code runs but sensor doesn't respond

**Solutions:**
- ✅ Verify `periph.io` is installed: `go get periph.io/x/host/v3`
- ✅ Check for conflicting GPIO access (other programs using pins)
- ✅ Restart the Pi to clear GPIO state
- ✅ Try the Python version: `python3 ultrasonic_sensor.py`

## Testing Without Hardware (Simulation Mode)

If you want to test the WebSocket server without hardware:

**Option 1: Mock Mode (Future Enhancement)**
```bash
# This would require adding a simulation flag
ULTRA_SIM_MODE=1 go run main_ultrasonic.go
```

**Option 2: Use Python Version with Simulation**
```bash
cd sensors
FORCE_SIM=1 python3 ultrasonic_ws_server.py
```

## Step-by-Step Debugging

1. **Check Power**
   ```bash
   # Measure voltage at sensor VCC pin
   # Should read ~5V
   ```

2. **Check GPIO State**
   ```bash
   # If gpio command available:
   gpio readall
   
   # Or check sysfs:
   cat /sys/kernel/debug/gpio
   ```

3. **Test Trigger Pin**
   ```bash
   # Manually toggle trigger (requires root or gpio group)
   echo 27 > /sys/class/gpio/export
   echo out > /sys/class/gpio/gpio27/direction
   echo 1 > /sys/class/gpio/gpio27/value
   sleep 0.00002
   echo 0 > /sys/class/gpio/gpio27/value
   ```

4. **Test Echo Pin**
   ```bash
   # Monitor echo pin
   echo 22 > /sys/class/gpio/export
   echo in > /sys/class/gpio/gpio22/direction
   watch -n 0.1 cat /sys/class/gpio/gpio22/value
   ```

5. **Run Diagnostic Script**
   ```bash
   go run test_ultrasonic_hardware.go
   ```

## Expected Behavior

**Working Sensor:**
```
✅ Echo pin goes HIGH within ~100µs after trigger
✅ Echo pulse duration corresponds to distance
✅ Readings are consistent (±2cm)
✅ Range: 2cm to 400cm
```

**Faulty Sensor/Wiring:**
```
❌ Echo never goes HIGH (timeout)
❌ Echo stays HIGH forever (wiring issue)
❌ Inconsistent readings (loose connection)
❌ Readings always 0 or max (sensor fault)
```

## Still Not Working?

1. **Check Pi Model Compatibility**
   - Raspberry Pi 5: ✅ Supported (uses periph.io)
   - Raspberry Pi 4: ✅ Supported
   - Older models: May need different GPIO library

2. **Verify Sensor Model**
   - HC-SR04: ✅ Supported
   - HC-SR05: May need different timing
   - Other models: Check datasheet

3. **Check System Logs**
   ```bash
   dmesg | grep -i gpio
   journalctl -u your-service | grep -i error
   ```

4. **Try Alternative Libraries**
   - Python version: `python3 ultrasonic_sensor.py`
   - Different Go library: `github.com/stianeikeland/go-rpio`

## Quick Fix Checklist

- [ ] Sensor has power (5V measured at VCC)
- [ ] All wires connected securely
- [ ] Correct GPIO pins (27/22)
- [ ] User in gpio group
- [ ] No other programs using GPIO
- [ ] Sensor not damaged (test on Arduino)
- [ ] Pi rebooted after GPIO changes
- [ ] Diagnostic script run successfully

## Contact & Resources

- HC-SR04 Datasheet: Search online for "HC-SR04 datasheet"
- Raspberry Pi GPIO Pinout: https://pinout.xyz
- periph.io Documentation: https://periph.io

