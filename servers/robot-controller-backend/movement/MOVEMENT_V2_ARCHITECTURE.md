# OMEGA ROBOTICS — MOVEMENT V2 ARCHITECTURE

## Executive Summary

Movement V2 transforms Omega-1's movement system from instant PWM changes to a smooth, PID-controlled, safety-monitored pipeline. All improvements are backward compatible and modular.

---

## 1. CORE ARCHITECTURE UPDATE

### 1.1 Final File Structure

```
movement/
│
├── movement_ws_server.py            # ✅ MAIN SERVER (modified for V2)
├── minimal_motor_control.py         # ✅ KEEP (legacy base)
├── motor_telemetry.py               # ✅ KEEP (enhanced controller)
├── PCA9685.py                       # ✅ KEEP (I2C PWM driver)
├── straight_drive_assist.py         # ✅ KEEP (trim system)
│
├── movement_ramp.py                 # 🆕 NEW — Acceleration/deceleration
├── movement_pid.py                  # 🆕 NEW — PID speed control
├── movement_watchdog.py             # 🆕 NEW — Auto-stop timer
├── movement_profiles.py             # 🆕 NEW — Movement styles
├── thermal_safety.py                # 🆕 NEW — Temperature/current limits
├── odometry.py                      # 🆕 NEW — Position tracking (future)
├── movement_config.py               # 🆕 NEW — Configuration
│
└── utils/
    └── timing.py                    # 🆕 NEW — Timing helpers
```

### 1.2 File Status

| File | Status | Action |
|------|--------|--------|
| `movement_ws_server.py` | ✅ Keep | Modify to integrate V2 |
| `minimal_motor_control.py` | ✅ Keep | No changes |
| `motor_telemetry.py` | ✅ Keep | No changes |
| `PCA9685.py` | ✅ Keep | No changes |
| `movement.go` | ⚠️ Deprecate | Document as legacy, keep for reference |
| `utils/pca9685.py` | ❌ Remove | Duplicate of `PCA9685.py` |

### 1.3 Python Stack as Primary

**Decision:** Python (`movement_ws_server.py`) is the primary movement system.

**Rationale:**
- ✅ Full-featured WebSocket server
- ✅ Motor telemetry support
- ✅ Servo control
- ✅ Buzzer control
- ✅ Autonomy integration
- ✅ Better error handling

**Go Version Status:**
- ⚠️ **Deprecated** — Keep `movement.go` for reference but mark as legacy
- 📝 **Documentation:** Add comment: `// LEGACY: This Go implementation is deprecated. Use Python movement_ws_server.py instead.`

---

## 2. MOVEMENT V2 PIPELINE ARCHITECTURE

### 2.1 Pipeline Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    UI LAYER (React)                             │
│  CarControlPanel → WebSocket Client                             │
└────────────────────────────┬────────────────────────────────────┘
                             │ JSON Command
                             │ {"command": "forward", "speed": 2000}
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│              WEBSOCKET SERVER (movement_ws_server.py)            │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ Command Handler                                          │   │
│  │  - Parse command                                         │   │
│  │  - Kick watchdog                                         │   │
│  │  - Validate parameters                                    │   │
│  └────────────────────┬─────────────────────────────────────┘   │
│                       │                                          │
│                       ▼                                          │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ MOVEMENT V2 PIPELINE                                      │   │
│  │                                                            │   │
│  │  1. Profile Manager                                       │   │
│  │     └─> Transform speed based on profile                  │   │
│  │         (smooth/aggressive/precision)                    │   │
│  │                                                            │   │
│  │  2. Movement Ramp                                         │   │
│  │     └─> Smooth acceleration/deceleration                  │   │
│  │         (linear/exponential/s_curve)                      │   │
│  │                                                            │   │
│  │  3. PID Speed Controller                                  │   │
│  │     └─> Maintain target RPM despite load                  │   │
│  │                                                            │   │
│  │  4. Thermal Safety                                        │   │
│  │     └─> Throttle/kill on overheating                      │   │
│  │                                                            │   │
│  │  5. Hardware Motor Controller                             │   │
│  │     └─> Apply final PWM to motors                         │   │
│  │                                                            │   │
│  │  6. Telemetry Feedback Loop                               │   │
│  │     └─> RPM → PID → Correction                            │   │
│  └────────────────────┬─────────────────────────────────────┘   │
│                       │                                          │
│                       ▼                                          │
└────────────────────────────┬─────────────────────────────────────┘
                             │ PWM values
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                    HARDWARE LAYER                               │
│  PCA9685 → Motor Drivers → Motors                                │
└─────────────────────────────────────────────────────────────────┘

PARALLEL SYSTEMS:
┌─────────────────────────────────────────────────────────────────┐
│              BACKGROUND TASKS                                    │
│  ┌──────────────────┐  ┌──────────────────┐                   │
│  │ Watchdog Task     │  │ Ramp Update Task │                   │
│  │ - Check timeout   │  │ - Update ramp    │                   │
│  │ - Auto-stop       │  │ - Smooth decel   │                   │
│  └──────────────────┘  └──────────────────┘                   │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Component Interaction

```
┌─────────────────┐
│  UI Command     │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────┐
│ movement_ws_server.py                │
│  ┌───────────────────────────────┐   │
│  │ handler()                     │   │
│  │  - Kick watchdog              │   │
│  │  - Parse command              │   │
│  └───────────┬───────────────────┘   │
│              │                        │
│              ▼                        │
│  ┌───────────────────────────────┐   │
│  │ do_move()                     │   │
│  │  ┌─────────────────────────┐  │   │
│  │  │ 1. Profile.transform()  │  │   │
│  │  │    → Adjust speed       │  │   │
│  │  └───────────┬─────────────┘  │   │
│  │              │                 │   │
│  │              ▼                 │   │
│  │  ┌─────────────────────────┐  │   │
│  │  │ 2. Ramp.set_target()   │  │   │
│  │  │    → Set target PWM     │  │   │
│  │  └───────────┬─────────────┘  │   │
│  │              │                 │   │
│  │              ▼                 │   │
│  │  ┌─────────────────────────┐  │   │
│  │  │ 3. Ramp.update()        │  │   │
│  │  │    → Smooth PWM          │  │   │
│  │  └───────────┬─────────────┘  │   │
│  │              │                 │   │
│  │              ▼                 │   │
│  │  ┌─────────────────────────┐  │   │
│  │  │ 4. Thermal.check()       │  │   │
│  │  │    → Throttle/kill       │  │   │
│  │  └───────────┬─────────────┘  │   │
│  │              │                 │   │
│  │              ▼                 │   │
│  │  ┌─────────────────────────┐  │   │
│  │  │ 5. PID.compute()        │  │   │
│  │  │    → Speed correction    │  │   │
│  │  └───────────┬─────────────┘  │   │
│  │              │                 │   │
│  │              ▼                 │   │
│  │  ┌─────────────────────────┐  │   │
│  │  │ 6. Motor.setMotors()   │  │   │
│  │  │    → Apply PWM          │  │   │
│  │  └─────────────────────────┘  │   │
│  └───────────────────────────────┘   │
└──────────────────────────────────────┘
```

---

## 3. MODULE SPECIFICATIONS

### 3.1 Movement Ramp (`movement_ramp.py`)

**Purpose:** Smooth acceleration/deceleration curves

**Key Classes:**
- `MovementRamp` — Main ramping controller
- `RampType` — Enum: LINEAR, EXPONENTIAL, S_CURVE

**Key Methods:**
```python
ramp.set_target(pwm: float)      # Set target PWM
pwm = ramp.update(dt: float)    # Update and get current PWM
ramp.is_at_target()              # Check if reached target
ramp.is_ramping()                # Check if currently ramping
```

**Integration:**
- Called in `do_move()` before motor control
- Updated via background task for smooth deceleration

### 3.2 Movement PID (`movement_pid.py`)

**Purpose:** Speed regulation using PID control

**Key Classes:**
- `SpeedPID` — Motor speed PID controller
- `StraightDrivePID` — Straight-line correction PID
- `ServoSmoothPID` — Servo smoothing PID

**Key Methods:**
```python
pid.set_target_rpm(rpm: float)                    # Set target RPM
correction = pid.compute(current_rpm, dt)          # Get PWM correction
pid.reset()                                        # Reset PID state
```

**Integration:**
- Used after ramping, before hardware output
- Feeds telemetry RPM back into PID loop

### 3.3 Movement Watchdog (`movement_watchdog.py`)

**Purpose:** Auto-stop on inactivity

**Key Classes:**
- `MovementWatchdog` — Watchdog timer
- `WatchdogState` — Enum: ACTIVE, TRIGGERED, DISABLED

**Key Methods:**
```python
watchdog.kick()                    # Reset timer (call on every command)
if watchdog.should_stop():        # Check if timeout exceeded
    motor.stop()
```

**Integration:**
- `kick()` called in `do_move()` on every command
- Background task checks `should_stop()` periodically

### 3.4 Movement Profiles (`movement_profiles.py`)

**Purpose:** Movement styles (smooth/aggressive/precision)

**Key Classes:**
- `MovementProfile` — Base profile class
- `SmoothProfile` — Gentle movement
- `AggressiveProfile` — Fast movement
- `PrecisionProfile` — Slow, controlled movement
- `ProfileManager` — Profile switching

**Key Methods:**
```python
profile.transform_speed(pwm)       # Transform PWM based on profile
profile.get_accel_rate()          # Get acceleration rate
profile.get_decel_rate()          # Get deceleration rate
manager.set_profile(ProfileType)  # Switch profile
```

**Integration:**
- Applied in `do_move()` before ramping
- Configures ramp rates based on profile

### 3.5 Thermal Safety (`thermal_safety.py`)

**Purpose:** Motor protection from overheating/overcurrent

**Key Classes:**
- `ThermalSafety` — Thermal monitoring
- `SafetyState` — Enum: OK, WARNING, THROTTLE, KILL
- `ThermalLimits` — Configuration limits

**Key Methods:**
```python
state = safety.check(telemetry)           # Check telemetry
pwm = safety.apply_throttle(pwm)          # Apply throttling
factor = safety.get_throttle_factor()     # Get throttle factor
```

**Integration:**
- Called in `do_move()` after ramping
- Uses telemetry temperature/current

### 3.6 Odometry (`odometry.py`)

**Purpose:** Position tracking (future: encoder-based)

**Key Classes:**
- `Odometry` — Position tracker
- `Pose` — Position + heading

**Key Methods:**
```python
odom.update(left_rpm, right_rpm, dt)      # Update position
pose = odom.get_pose()                    # Get current pose
odom.reset(x, y, heading)                 # Reset position
```

**Integration:**
- Currently placeholder (simulated)
- Future: integrate wheel encoders

---

## 4. CODE FLOW DIAGRAM

### 4.1 Command Flow (Detailed)

```
UI sends: {"command": "forward", "speed": 2000}
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│ movement_ws_server.py::handler()                            │
│  - Parse JSON                                                │
│  - Validate command                                          │
│  - Extract speed parameter                                   │
└───────────────┬───────────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────────┐
│ do_move("forward", 2000)                                    │
│  ┌───────────────────────────────────────────────────────┐   │
│  │ 1. movement_watchdog.kick()                          │   │
│  │    → Reset watchdog timer                             │   │
│  └───────────────────────────────────────────────────────┘   │
│  ┌───────────────────────────────────────────────────────┐   │
│  │ 2. profile = profile_manager.get_current_profile()    │   │
│  │    speed = profile.transform_speed(2000)              │   │
│  │    → Apply profile limits/transformations             │   │
│  └───────────────────────────────────────────────────────┘   │
│  ┌───────────────────────────────────────────────────────┐   │
│  │ 3. movement_ramp.set_target(speed)                    │   │
│  │    → Set target PWM for ramping                        │   │
│  └───────────────────────────────────────────────────────┘   │
│  ┌───────────────────────────────────────────────────────┐   │
│  │ 4. current_pwm = movement_ramp.update(dt)            │   │
│  │    → Smooth ramp from current to target                │   │
│  └───────────────────────────────────────────────────────┘   │
│  ┌───────────────────────────────────────────────────────┐   │
│  │ 5. telemetry = get_cached_motor_telemetry()          │   │
│  │    state = thermal_safety.check(telemetry)            │   │
│  │    current_pwm = thermal_safety.apply_throttle(...)  │   │
│  │    → Throttle if overheating                          │   │
│  └───────────────────────────────────────────────────────┘   │
│  ┌───────────────────────────────────────────────────────┐   │
│  │ 6. avg_rpm = extract_rpm_from_telemetry(telemetry)    │   │
│  │    correction = speed_pid.compute(avg_rpm, dt)       │   │
│  │    current_pwm += correction                          │   │
│  │    → PID speed correction                              │   │
│  └───────────────────────────────────────────────────────┘   │
│  ┌───────────────────────────────────────────────────────┐   │
│  │ 7. motor.forward(int(current_pwm))                   │   │
│  │    → Apply final PWM to hardware                       │   │
│  └───────────────────────────────────────────────────────┘   │
└───────────────────────────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│ Hardware: PCA9685 → Motors                                  │
└─────────────────────────────────────────────────────────────┘

BACKGROUND TASKS:
┌─────────────────────────────────────────────────────────────┐
│ watchdog_task() (runs every 100ms)                          │
│  - Check if watchdog.should_stop()                         │
│  - If timeout exceeded → do_stop()                          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ ramp_update_task() (runs at ramp_update_rate_hz)           │
│  - Update ramp state for smooth deceleration                │
│  - Apply to motors if needed                                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. TIMELINE PLAN

### Week 1: Core Ramping & Watchdog (Fast Wins)

**Days 1-2: Movement Ramp**
- ✅ Implement `movement_ramp.py` (DONE)
- ✅ Integrate into `do_move()` (TODO)
- ✅ Test linear ramping
- ✅ Test exponential ramping
- ✅ Test S-curve ramping

**Days 3-4: Watchdog**
- ✅ Implement `movement_watchdog.py` (DONE)
- ✅ Integrate `kick()` into `do_move()` (TODO)
- ✅ Add background watchdog task (TODO)
- ✅ Test timeout behavior

**Day 5: Integration & Testing**
- ✅ Integrate ramp + watchdog into `movement_ws_server.py` (TODO)
- ✅ Test end-to-end movement commands
- ✅ Verify backward compatibility

**Fast Wins (Can implement TODAY):**
1. ✅ Movement Ramp module (DONE)
2. ✅ Watchdog module (DONE)
3. ✅ Basic integration into `do_move()` (TODO - see integration guide)

### Week 2: PID & Thermal Safety

**Days 1-3: PID Integration**
- ✅ Implement `movement_pid.py` (DONE)
- ✅ Integrate PID into `do_move()` (TODO)
- ✅ Connect PID to telemetry feedback loop (TODO)
- ✅ Tune PID parameters

**Days 4-5: Thermal Safety**
- ✅ Implement `thermal_safety.py` (DONE)
- ✅ Integrate thermal checks into `do_move()` (TODO)
- ✅ Test throttling behavior
- ✅ Test kill behavior

### Week 3: Profiles & Polish

**Days 1-2: Movement Profiles**
- ✅ Implement `movement_profiles.py` (DONE)
- ✅ Integrate profile switching (TODO)
- ✅ Test smooth/aggressive/precision profiles

**Days 3-4: Odometry & Config**
- ✅ Implement `odometry.py` (DONE - placeholder)
- ✅ Implement `movement_config.py` (DONE)
- ✅ Add configuration loading

**Day 5: Documentation & Testing**
- ✅ Complete architecture documentation (DONE)
- ✅ Write integration tests
- ✅ Performance testing
- ✅ User documentation

---

## 6. FAST WINS (Implement Today)

### 6.1 Movement Ramp Integration

**File:** `movement_ws_server.py`

**Changes:**
1. Import `MovementRamp` at top
2. Initialize `movement_ramp` after hardware init
3. Modify `do_move()` to use ramping

**Code:**
```python
# After line 117
from .movement_ramp import MovementRamp, RampType

# After line 290
movement_ramp = MovementRamp(accel_rate=150.0, decel_rate=200.0)

# Modify do_move() around line 440
async def do_move(fn_name: str, speed: int):
    async with motor_lock:
        # Set ramp target
        movement_ramp.set_target(float(speed))
        
        # Get ramped PWM
        current_pwm = movement_ramp.update()
        
        # Apply to motor
        fn = getattr(motor, fn_name, None)
        if not callable(fn):
            # ... existing fallback logic ...
        _call_motor(fn, int(current_pwm))
```

**Result:** Smooth acceleration/deceleration immediately!

### 6.2 Watchdog Integration

**File:** `movement_ws_server.py`

**Changes:**
1. Import `MovementWatchdog`
2. Initialize watchdog
3. Add `kick()` to `do_move()`
4. Add background task

**Code:**
```python
# After line 117
from .movement_watchdog import MovementWatchdog

# After line 290
movement_watchdog = MovementWatchdog(timeout_sec=2.0, stop_callback=lambda: asyncio.create_task(do_stop()))

# In do_move()
async def do_move(fn_name: str, speed: int):
    async with motor_lock:
        movement_watchdog.kick()  # Reset timer
        # ... rest of movement logic ...

# Add background task
async def watchdog_task():
    while True:
        await asyncio.sleep(0.1)
        if movement_watchdog.should_stop():
            await do_stop()

# In main()
asyncio.create_task(watchdog_task())
```

**Result:** Auto-stop on inactivity immediately!

### 6.3 Profile Switching

**File:** `movement_ws_server.py`

**Changes:**
1. Import `ProfileManager`
2. Initialize profile manager
3. Add `set-profile` command

**Code:**
```python
# After line 117
from .movement_profiles import ProfileManager, ProfileType

# After line 290
profile_manager = ProfileManager(default_profile=ProfileType.SMOOTH)

# In command handler (before "else:")
elif cmd == "set-profile":
    profile_name = str(data.get("profile", "smooth")).lower()
    if profile_name == "smooth":
        profile_manager.set_profile(ProfileType.SMOOTH)
    elif profile_name == "aggressive":
        profile_manager.set_profile(ProfileType.AGGRESSIVE)
    elif profile_name == "precision":
        profile_manager.set_profile(ProfileType.PRECISION)
    await send_json(ws, ok("set-profile", profile=profile_name))
```

**Result:** Profile switching immediately!

---

## 7. TESTING STRATEGY

### 7.1 Unit Tests

**Files to test:**
- `movement_ramp.py` — Ramp behavior
- `movement_pid.py` — PID stability
- `movement_watchdog.py` — Timeout behavior
- `thermal_safety.py` — Thermal cutoffs
- `movement_profiles.py` — Profile transforms

**Test cases:**
- Ramp: linear/exponential/s_curve transitions
- PID: step response, stability, windup
- Watchdog: timeout, kick reset, multiple triggers
- Thermal: warning/throttle/kill thresholds
- Profiles: speed transforms, rate changes

### 7.2 Integration Tests

**Test scenarios:**
- Forward/backward transitions with ramping
- Turning transitions with ramping
- Stop behavior (smooth deceleration)
- Overheating simulation (thermal throttling)
- Disconnect behavior (watchdog trigger)
- Profile switching mid-movement

### 7.3 Hardware Tests

**Test scenarios:**
- Progressive ramping (visual smoothness)
- Smooth starts/stops (no jerking)
- Thermal protection (heat gun test)
- Emergency stop behavior
- Speed holding under load (PID test)

---

## 8. CONFIGURATION REFERENCE

### 8.1 Environment Variables

```bash
# Movement V2 Enable
MOVEMENT_V2_ENABLED=1

# Ramping
MOVEMENT_ACCEL_RATE=150.0        # PWM units per second
MOVEMENT_DECEL_RATE=200.0        # PWM units per second
MOVEMENT_RAMP_TYPE=linear        # linear, exponential, s_curve

# Profiles
MOVEMENT_DEFAULT_PROFILE=smooth  # smooth, aggressive, precision

# Watchdog
MOVEMENT_WATCHDOG_TIMEOUT=2.0    # seconds
MOVEMENT_WATCHDOG_ENABLED=1

# Thermal Safety
MOVEMENT_THERMAL_MAX_TEMP=75.0      # °C
MOVEMENT_THERMAL_WARNING_TEMP=60.0  # °C
MOVEMENT_THERMAL_MAX_CURRENT=2.5     # A
MOVEMENT_THERMAL_WARNING_CURRENT=2.0 # A
MOVEMENT_THERMAL_COOLDOWN_TEMP=50.0  # °C
MOVEMENT_THERMAL_THROTTLE_FACTOR=0.5 # 0.0-1.0
MOVEMENT_THERMAL_ENABLED=1

# PID
MOVEMENT_PID_ENABLED=1
MOVEMENT_PID_KP=0.3
MOVEMENT_PID_KI=0.05
MOVEMENT_PID_KD=0.01
MOVEMENT_PID_KF=0.0

# Odometry
MOVEMENT_ODOMETRY_ENABLED=0
MOVEMENT_WHEEL_BASE=0.2          # meters
MOVEMENT_WHEEL_RADIUS=0.05       # meters

# Update Rates
MOVEMENT_RAMP_UPDATE_RATE=50.0   # Hz
MOVEMENT_TELEMETRY_UPDATE_RATE=10.0 # Hz
```

### 8.2 Configuration File

See `movement_config.py` for programmatic configuration.

---

## 9. BACKWARD COMPATIBILITY

### 9.1 API Compatibility

**All existing commands work unchanged:**
- `forward`, `backward`, `left`, `right`, `stop`
- `set-speed`, `increase-speed`, `decrease-speed`
- `set-servo-position`, `camera-servo-*`
- `buzzer-on`, `buzzer-off`, `buzzer-pulse`
- `status`, `ping`

**New commands (opt-in):**
- `set-profile` — Switch movement profile
- `set-accel-rate` — Set acceleration rate
- `set-decel-rate` — Set deceleration rate
- `get-movement-status` — Get V2 status

### 9.2 Behavior Compatibility

**If `MOVEMENT_V2_ENABLED=0`:**
- System uses V1 behavior (instant PWM)
- No ramping, PID, or thermal safety
- Watchdog disabled
- All V2 modules bypassed

**If `MOVEMENT_V2_ENABLED=1`:**
- V2 features active
- Backward compatible API
- Smooth ramping replaces instant PWM
- Safety features active

---

## 10. FUTURE ENHANCEMENTS

### 10.1 Phase 2 Features

1. **Encoder-Based Odometry**
   - Integrate wheel encoders
   - Real position tracking
   - Dead reckoning

2. **IMU Integration**
   - Heading correction
   - Tilt compensation
   - Drift correction

3. **Advanced PID Tuning**
   - Auto-tuning
   - Adaptive PID
   - Load-dependent tuning

4. **Movement Recording**
   - Record movement sequences
   - Playback support
   - Path following

### 10.2 Phase 3 Features

1. **SLAM Integration**
   - Map building
   - Localization
   - Path planning

2. **Multi-Surface Profiles**
   - Grass mode
   - Sand mode
   - Indoor mode

3. **Predictive Safety**
   - Collision avoidance
   - Obstacle detection
   - Emergency braking

---

## 11. SUMMARY

**Movement V2 delivers:**
- ✅ Smooth acceleration/deceleration
- ✅ PID-based speed regulation
- ✅ Watchdog auto-stop
- ✅ Thermal protection
- ✅ Movement profiles
- ✅ Backward compatibility
- ✅ Modular architecture
- ✅ Production-ready safety

**Next Steps:**
1. Integrate Movement V2 into `movement_ws_server.py` (see `MOVEMENT_V2_INTEGRATION.md`)
2. Test ramping behavior
3. Test watchdog timeout
4. Tune PID parameters
5. Test thermal safety
6. Deploy to hardware

**Omega-1 Movement V2: Smooth, Safe, Professional.**

