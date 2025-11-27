# Movement V2 Integration Guide

This document shows how Movement V2 modules are integrated into `movement_ws_server.py`.

## Integration Points

### 1. Import Movement V2 Modules

Add after line 117 (after MotorTelemetryController import):

```python
# Movement V2 imports
try:
    from .movement_ramp import MovementRamp, RampType
    from .movement_pid import SpeedPID, PIDTuning
    from .movement_watchdog import MovementWatchdog, WatchdogState
    from .movement_profiles import ProfileManager, ProfileType
    from .thermal_safety import ThermalSafety, ThermalLimits, SafetyState
    from .odometry import Odometry
    from .movement_config import get_config, MovementV2Config
    from .utils.timing import RateLimiter, Timer
    MOVEMENT_V2_AVAILABLE = True
except ImportError as e:
    logger.warning(f"Movement V2 modules not available: {e}")
    MOVEMENT_V2_AVAILABLE = False
```

### 2. Initialize Movement V2 Components

Add after line 290 (after MOTOR_TELEMETRY initialization):

```python
# ---------- Movement V2 initialization ----------

MOVEMENT_V2_ENABLED = os.getenv("MOVEMENT_V2_ENABLED", "1") == "1" and MOVEMENT_V2_AVAILABLE

if MOVEMENT_V2_ENABLED:
    config = get_config()
    
    # Initialize Movement V2 components
    movement_ramp = MovementRamp(
        accel_rate=config.accel_rate,
        decel_rate=config.decel_rate,
        ramp_type=RampType.LINEAR if config.ramp_type == "linear" else 
                  RampType.EXPONENTIAL if config.ramp_type == "exponential" else 
                  RampType.S_CURVE
    )
    
    speed_pid = SpeedPID(
        tuning=PIDTuning(
            kp=config.pid_kp,
            ki=config.pid_ki,
            kd=config.pid_kd,
            kf=config.pid_kf
        )
    ) if config.pid_enabled else None
    
    profile_manager = ProfileManager(
        default_profile=ProfileType.SMOOTH if config.default_profile == "smooth" else
                        ProfileType.AGGRESSIVE if config.default_profile == "aggressive" else
                        ProfileType.PRECISION
    )
    
    thermal_safety = ThermalSafety(
        limits=ThermalLimits(
            max_temp=config.thermal_max_temp,
            warning_temp=config.thermal_warning_temp,
            max_current=config.thermal_max_current,
            warning_current=config.thermal_warning_current,
            cooldown_temp=config.thermal_cooldown_temp,
            throttle_factor=config.thermal_throttle_factor
        ),
        enabled=config.thermal_enabled
    )
    
    movement_watchdog = MovementWatchdog(
        timeout_sec=config.watchdog_timeout_sec,
        stop_callback=lambda: asyncio.create_task(do_stop()),
        enabled=config.watchdog_enabled
    )
    
    odometry = Odometry(
        wheel_base=config.wheel_base,
        wheel_radius=config.wheel_radius
    ) if config.odometry_enabled else None
    
    # Rate limiters
    ramp_rate_limiter = RateLimiter(rate_hz=config.ramp_update_rate_hz)
    telemetry_rate_limiter = RateLimiter(rate_hz=config.telemetry_update_rate_hz)
    
    log("Movement V2 initialized: ramp, PID, watchdog, profiles, thermal safety")
else:
    movement_ramp = None
    speed_pid = None
    profile_manager = None
    thermal_safety = None
    movement_watchdog = None
    odometry = None
    ramp_rate_limiter = None
    telemetry_rate_limiter = None
    log("Movement V2 disabled - using V1 behavior")
```

### 3. Modify `do_move()` Function

Replace the existing `do_move()` function (around line 440) with:

```python
async def do_move(fn_name: str, speed: int):
    """Execute movement command with Movement V2 pipeline."""
    async with motor_lock:
        # Kick watchdog
        if movement_watchdog:
            movement_watchdog.kick()
        
        # Get current profile
        profile = profile_manager.get_current_profile() if profile_manager else None
        
        # Transform speed through profile
        if profile:
            speed = int(profile.transform_speed(speed))
            # Update ramp rates from profile
            movement_ramp.set_rates(
                accel_rate=profile.get_accel_rate(),
                decel_rate=profile.get_decel_rate()
            )
            movement_ramp.set_ramp_type(
                RampType.LINEAR if profile.get_ramp_type() == "linear" else
                RampType.EXPONENTIAL if profile.get_ramp_type() == "exponential" else
                RampType.S_CURVE
            )
        
        # Set ramp target
        if movement_ramp:
            movement_ramp.set_target(float(speed))
        
        # Get motor function
        fn = getattr(motor, fn_name, None)
        if not callable(fn):
            if fn_name == "left":
                fn = getattr(motor, "left", None)
            elif fn_name == "right":
                fn = getattr(motor, "right", None)
        if not callable(fn):
            raise RuntimeError(f"motor missing method: {fn_name}")
        
        # Apply Movement V2 pipeline
        if movement_ramp and ramp_rate_limiter:
            # Use ramped PWM
            if ramp_rate_limiter.should_update():
                current_pwm = movement_ramp.update()
                
                # Apply thermal safety throttling
                if thermal_safety and MOTOR_TELEMETRY:
                    telemetry = get_cached_motor_telemetry()
                    safety_state = thermal_safety.check(telemetry)
                    current_pwm = thermal_safety.apply_throttle(current_pwm)
                    
                    if safety_state == SafetyState.KILL:
                        log("THERMAL SAFETY: Motor kill triggered")
                        motor.stop()
                        return
                
                # Apply PID correction if enabled
                if speed_pid and speed_pid.enabled and MOTOR_TELEMETRY:
                    telemetry = get_cached_motor_telemetry()
                    # Get average RPM from telemetry
                    avg_rpm = sum(
                        telemetry.get(m, {}).get('speed', 0) 
                        for m in ['frontLeft', 'frontRight', 'rearLeft', 'rearRight']
                    ) / 4.0 if telemetry else 0.0
                    
                    # Estimate target RPM from PWM (rough conversion)
                    target_rpm = (current_pwm / 4095.0) * 300.0  # Assume 300 RPM max
                    speed_pid.set_target_rpm(target_rpm)
                    
                    # Compute PID correction
                    dt = 1.0 / config.ramp_update_rate_hz if config else 0.02
                    correction = speed_pid.compute(current_rpm=avg_rpm, dt=dt)
                    current_pwm = max(0, min(4095, current_pwm + correction))
                
                # Apply to motor
                _call_motor(fn, int(current_pwm))
        else:
            # Fallback to V1 behavior (direct PWM)
            _call_motor(fn, speed)
```

### 4. Add Watchdog Background Task

Add after line 276 (after STRAIGHT_ASSIST initialization):

```python
# Movement V2 background tasks
async def watchdog_task():
    """Background task to check watchdog timer."""
    if not movement_watchdog or not movement_watchdog.enabled:
        return
    
    while True:
        try:
            await asyncio.sleep(0.1)  # Check every 100ms
            if movement_watchdog.should_stop():
                log("Watchdog triggered - stopping motors")
                await do_stop()
        except Exception as e:
            logger.error(f"Watchdog task error: {e}")

async def ramp_update_task():
    """Background task to update ramp state."""
    if not movement_ramp or not ramp_rate_limiter:
        return
    
    while True:
        try:
            await asyncio.sleep(1.0 / config.ramp_update_rate_hz if config else 0.02)
            if ramp_rate_limiter.should_update():
                # Ramp update happens in do_move(), but we can also update here
                # for smooth deceleration when stopped
                if movement_ramp.is_ramping():
                    current_pwm = movement_ramp.update()
                    # Could apply to motors here if needed
        except Exception as e:
            logger.error(f"Ramp update task error: {e}")
```

### 5. Add New Movement V2 Commands

Add in the command handler (around line 880, before `else:`):

```python
                    # -------- MOVEMENT V2 COMMANDS --------
                    elif cmd == "set-profile":
                        profile_name = str(data.get("profile", "smooth")).lower()
                        if profile_manager:
                            if profile_name == "smooth":
                                profile_manager.set_profile(ProfileType.SMOOTH)
                            elif profile_name == "aggressive":
                                profile_manager.set_profile(ProfileType.AGGRESSIVE)
                            elif profile_name == "precision":
                                profile_manager.set_profile(ProfileType.PRECISION)
                            else:
                                await send_json(ws, err(f"unknown-profile:{profile_name}"))
                                continue
                            await send_json(ws, ok("set-profile", profile=profile_name))
                        else:
                            await send_json(ws, err("movement-v2-not-enabled"))
                    
                    elif cmd == "set-accel-rate":
                        if movement_ramp:
                            rate = float(data.get("rate", 150.0))
                            movement_ramp.set_rates(accel_rate=rate)
                            await send_json(ws, ok("set-accel-rate", rate=rate))
                        else:
                            await send_json(ws, err("movement-v2-not-enabled"))
                    
                    elif cmd == "set-decel-rate":
                        if movement_ramp:
                            rate = float(data.get("rate", 200.0))
                            movement_ramp.set_rates(decel_rate=rate)
                            await send_json(ws, ok("set-decel-rate", rate=rate))
                        else:
                            await send_json(ws, err("movement-v2-not-enabled"))
                    
                    elif cmd == "get-movement-status":
                        status = {
                            "type": "movement-status",
                            "v2_enabled": MOVEMENT_V2_ENABLED,
                            "ramp": {
                                "current": movement_ramp.get_current() if movement_ramp else 0,
                                "target": movement_ramp.get_target() if movement_ramp else 0,
                                "is_ramping": movement_ramp.is_ramping() if movement_ramp else False,
                            } if movement_ramp else None,
                            "watchdog": movement_watchdog.get_status() if movement_watchdog else None,
                            "thermal": thermal_safety.get_status() if thermal_safety else None,
                            "profile": profile_manager.get_current_profile().get_config() if profile_manager else None,
                            "pid_enabled": speed_pid.enabled if speed_pid else False,
                            "odometry": odometry.get_status() if odometry else None,
                            "ts": _now_ms(),
                        }
                        await send_json(ws, status)
```

### 6. Update Status Command

Modify the `status` command handler (around line 861) to include Movement V2 status:

```python
                    elif cmd == "status":
                        status_payload = {
                            "type": "status",
                            "speed": current_speed,
                            "motors": get_cached_motor_telemetry(),
                            "servo": {
                                "horizontal": current_horizontal_angle,
                                "vertical": current_vertical_angle,
                                "min": SERVO_MIN,
                                "max": SERVO_MAX,
                            },
                            "buzzer": buzzer_on_state,
                            "autonomy": AUTONOMY.status(),
                            "straightAssist": STRAIGHT_ASSIST.status(),
                            "ts": _now_ms(),
                            "sim": SIM_MODE,
                        }
                        
                        # Add Movement V2 status if enabled
                        if MOVEMENT_V2_ENABLED:
                            status_payload["movementV2"] = {
                                "enabled": True,
                                "ramp": {
                                    "current": movement_ramp.get_current() if movement_ramp else 0,
                                    "target": movement_ramp.get_target() if movement_ramp else 0,
                                    "is_ramping": movement_ramp.is_ramping() if movement_ramp else False,
                                } if movement_ramp else None,
                                "watchdog": movement_watchdog.get_status() if movement_watchdog else None,
                                "thermal": thermal_safety.get_status() if thermal_safety else None,
                                "profile": profile_manager.get_current_profile().get_config() if profile_manager else None,
                                "pid_enabled": speed_pid.enabled if speed_pid else False,
                            }
                        
                        await send_json(ws, status_payload)
```

### 7. Start Background Tasks

Modify `main()` function (around line 903) to start background tasks:

```python
async def main():
    log(f"listening on ws://0.0.0.0:{PORT}{'' if PATH=='/' else PATH}")
    log(f"ORIGIN_ALLOW={','.join(sorted(_ALLOWED_ORIGINS)) or '(none)'} "
        f"ALLOW_NO_ORIGIN={ALLOW_NO_ORIGIN} SIM_MODE={SIM_MODE}")
    
    # Start Movement V2 background tasks
    if MOVEMENT_V2_ENABLED:
        asyncio.create_task(watchdog_task())
        asyncio.create_task(ramp_update_task())
        log("Movement V2 background tasks started")
    
    async with websockets.serve(
        handler,
        "0.0.0.0",
        PORT,
        ping_interval=None,
        max_size=64 * 1024,
        max_queue=64,
    ):
        await asyncio.Future()  # run forever
```

## Environment Variables

Add these to `.env` or environment:

```bash
# Movement V2
MOVEMENT_V2_ENABLED=1
MOVEMENT_ACCEL_RATE=150.0
MOVEMENT_DECEL_RATE=200.0
MOVEMENT_RAMP_TYPE=linear
MOVEMENT_DEFAULT_PROFILE=smooth
MOVEMENT_WATCHDOG_TIMEOUT=2.0
MOVEMENT_WATCHDOG_ENABLED=1
MOVEMENT_THERMAL_MAX_TEMP=75.0
MOVEMENT_THERMAL_WARNING_TEMP=60.0
MOVEMENT_THERMAL_MAX_CURRENT=2.5
MOVEMENT_PID_ENABLED=1
MOVEMENT_PID_KP=0.3
MOVEMENT_PID_KI=0.05
MOVEMENT_PID_KD=0.01
```

## Testing

1. Start the server: `python3 movement_ws_server.py`
2. Connect via WebSocket
3. Send commands:
   - `{"command": "forward", "speed": 2000}` - Should ramp smoothly
   - `{"command": "set-profile", "profile": "aggressive"}` - Switch profile
   - `{"command": "get-movement-status"}` - Check V2 status
   - `{"command": "status"}` - Full status including V2

## Backward Compatibility

Movement V2 is fully backward compatible:
- If `MOVEMENT_V2_ENABLED=0`, system uses V1 behavior
- All existing commands work unchanged
- New V2 features are opt-in via new commands

