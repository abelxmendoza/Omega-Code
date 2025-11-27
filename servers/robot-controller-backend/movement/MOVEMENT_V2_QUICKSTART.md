# Movement V2 Quick Start Guide

## What is Movement V2?

Movement V2 transforms Omega-1's movement from instant PWM changes to smooth, PID-controlled, safety-monitored movement. All improvements are backward compatible.

## Key Features

✅ **Smooth Ramping** — Acceleration/deceleration curves (linear, exponential, S-curve)  
✅ **PID Speed Control** — Maintains target speed despite load variations  
✅ **Watchdog Timer** — Auto-stops motors if no commands received  
✅ **Thermal Safety** — Throttles/stops motors on overheating  
✅ **Movement Profiles** — Smooth, aggressive, precision modes  
✅ **Backward Compatible** — All existing commands work unchanged  

## Installation

All Movement V2 modules are already created! Just integrate them into `movement_ws_server.py`.

### Step 1: Review Integration Guide

See `MOVEMENT_V2_INTEGRATION.md` for detailed integration instructions.

### Step 2: Enable Movement V2

Add to `.env` or environment:

```bash
MOVEMENT_V2_ENABLED=1
```

### Step 3: Configure (Optional)

```bash
# Ramping
MOVEMENT_ACCEL_RATE=150.0
MOVEMENT_DECEL_RATE=200.0
MOVEMENT_RAMP_TYPE=linear

# Profile
MOVEMENT_DEFAULT_PROFILE=smooth

# Watchdog
MOVEMENT_WATCHDOG_TIMEOUT=2.0

# Thermal Safety
MOVEMENT_THERMAL_MAX_TEMP=75.0
MOVEMENT_THERMAL_MAX_CURRENT=2.5
```

### Step 4: Test

```bash
# Start server
python3 movement_ws_server.py

# Connect via WebSocket
# Send: {"command": "forward", "speed": 2000}
# Should see smooth acceleration!
```

## Fast Wins (Implement Today)

### 1. Add Ramping (5 minutes)

**File:** `movement_ws_server.py`

```python
# Import (after line 117)
from .movement_ramp import MovementRamp, RampType

# Initialize (after line 290)
movement_ramp = MovementRamp(accel_rate=150.0, decel_rate=200.0)

# Modify do_move() (around line 440)
async def do_move(fn_name: str, speed: int):
    async with motor_lock:
        movement_ramp.set_target(float(speed))
        current_pwm = movement_ramp.update()
        fn = getattr(motor, fn_name, None)
        # ... existing logic ...
        _call_motor(fn, int(current_pwm))
```

**Result:** Smooth acceleration immediately!

### 2. Add Watchdog (5 minutes)

```python
# Import
from .movement_watchdog import MovementWatchdog

# Initialize
movement_watchdog = MovementWatchdog(timeout_sec=2.0)

# In do_move()
movement_watchdog.kick()  # Reset timer

# Background task
async def watchdog_task():
    while True:
        await asyncio.sleep(0.1)
        if movement_watchdog.should_stop():
            await do_stop()

# In main()
asyncio.create_task(watchdog_task())
```

**Result:** Auto-stop on inactivity immediately!

## Module Overview

| Module | Purpose | Status |
|--------|---------|--------|
| `movement_ramp.py` | Smooth acceleration/deceleration | ✅ Ready |
| `movement_pid.py` | PID speed control | ✅ Ready |
| `movement_watchdog.py` | Auto-stop timer | ✅ Ready |
| `movement_profiles.py` | Movement styles | ✅ Ready |
| `thermal_safety.py` | Thermal protection | ✅ Ready |
| `odometry.py` | Position tracking | ✅ Ready (placeholder) |
| `movement_config.py` | Configuration | ✅ Ready |
| `utils/timing.py` | Timing helpers | ✅ Ready |

## New Commands

```json
// Switch profile
{"command": "set-profile", "profile": "aggressive"}

// Set acceleration rate
{"command": "set-accel-rate", "rate": 200.0}

// Get Movement V2 status
{"command": "get-movement-status"}
```

## Architecture

See `MOVEMENT_V2_ARCHITECTURE.md` for:
- Full architecture diagram
- Code flow diagrams
- Timeline plan
- Testing strategy

## Integration Guide

See `MOVEMENT_V2_INTEGRATION.md` for:
- Step-by-step integration instructions
- Code snippets for each integration point
- Environment variable reference
- Testing procedures

## Status

✅ **All modules created**  
✅ **Documentation complete**  
⏳ **Integration into movement_ws_server.py** (see integration guide)  
⏳ **Testing** (after integration)  

## Next Steps

1. Review `MOVEMENT_V2_INTEGRATION.md`
2. Integrate Movement V2 into `movement_ws_server.py`
3. Test ramping behavior
4. Test watchdog timeout
5. Tune PID parameters
6. Test thermal safety
7. Deploy to hardware

## Support

- **Architecture:** See `MOVEMENT_V2_ARCHITECTURE.md`
- **Integration:** See `MOVEMENT_V2_INTEGRATION.md`
- **Code:** See individual module files

---

**Movement V2: Smooth, Safe, Professional.**

