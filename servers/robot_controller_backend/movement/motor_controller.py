"""
MotorController — dead-zone aware, rate-limited, ramped motor control.

Architecture
------------
  WS handler            sets target via set_speed() / stop()
      │
      ▼
  MotorController       ramps current → target at UPDATE_HZ
      │                 enforces MIN/MAX PWM dead-zone
      ▼
  driver (MotorTelemetryController or Motor)
      │
      ▼
  PCA9685 @ 500 Hz

Calling set_speed() or stop() is safe from any coroutine or thread.
The actual hardware write only happens inside tick(), which runs in
a single background asyncio loop.
"""

import asyncio
import time
import logging
from typing import Optional

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Calibrated limits (tune these per your supply voltage and motor)
# ---------------------------------------------------------------------------
_DEFAULT_MIN_PWM  = 1100   # lowest duty where all 4 wheels spin reliably
_DEFAULT_BASE_PWM = 1279   # known-good mid-point
_DEFAULT_MAX_PWM  = 1600   # ceiling before supply-sag instability
_DEFAULT_RAMP     = 20     # PWM units per tick  (at 50 Hz → 1000 units/s)
_DEFAULT_HZ       = 50     # control loop rate


class MotorController:
    """
    Three-layer motor controller:

    1. Command layer  — accept 0–1 speed intent + direction string
    2. Control layer  — dead-zone lift, ramp, hard clamp
    3. Hardware layer — write to driver only from tick()
    """

    def __init__(
        self,
        driver,
        min_pwm:  int   = _DEFAULT_MIN_PWM,
        max_pwm:  int   = _DEFAULT_MAX_PWM,
        ramp_step: int  = _DEFAULT_RAMP,
        update_hz: int  = _DEFAULT_HZ,
    ):
        self.driver     = driver
        self.MIN_PWM    = min_pwm
        self.MAX_PWM    = max_pwm
        self.RAMP_STEP  = ramp_step
        self.UPDATE_HZ  = update_hz

        # Signed targets: positive = forward, negative = backward
        self._target_left:  float = 0.0
        self._target_right: float = 0.0
        self._current_left:  float = 0.0
        self._current_right: float = 0.0

        self._last_ts = time.monotonic()

    # ── Public command interface ─────────────────────────────────────── #

    def set_speed(self, percent: float, direction: str = "forward") -> None:
        """
        Set target motion.

        percent   : 0.0 – 1.0  (percentage of max stable speed)
        direction : "forward" | "backward" | "pivot_left" | "pivot_right" |
                    "left" | "right" | "stop"
        """
        direction = direction.lower()

        if direction in ("stop", "") or percent <= 0:
            self._target_left  = 0.0
            self._target_right = 0.0
            return

        pwm = float(self._map(percent))

        if direction == "forward":
            self._target_left  =  pwm
            self._target_right =  pwm
        elif direction == "backward":
            self._target_left  = -pwm
            self._target_right = -pwm
        elif direction in ("pivot_left", "left"):
            self._target_left  = -pwm
            self._target_right =  pwm
        elif direction in ("pivot_right", "right"):
            self._target_left  =  pwm
            self._target_right = -pwm

    def set_motion(self, speed: float, turn: float) -> None:
        """
        Arcade-drive: speed 0–1 forward, turn -1–1 (negative = left).
        Useful for joystick / gamepad control.
        """
        base = float(self._map(speed))
        diff = turn * (self.MAX_PWM - self.MIN_PWM) * 0.3
        self._target_left  = float(self._clamp(int(base - diff)))
        self._target_right = float(self._clamp(int(base + diff)))

    def stop(self) -> None:
        """Immediate stop — resets ramp state."""
        self._target_left  = 0.0
        self._target_right = 0.0
        self._current_left  = 0.0
        self._current_right = 0.0
        try:
            self.driver.stop()
        except Exception as e:
            log.warning("stop() failed: %s", e)

    # ── Control loop ─────────────────────────────────────────────────── #

    def tick(self) -> None:
        """
        Advance ramp one step and write to hardware.
        Must be called at UPDATE_HZ from an asyncio background task.
        """
        now = time.monotonic()
        dt  = now - self._last_ts
        if dt < 1.0 / self.UPDATE_HZ * 0.8:   # guard: skip if too early
            return
        self._last_ts = now

        self._current_left  = self._ramp(self._current_left,  self._target_left)
        self._current_right = self._ramp(self._current_right, self._target_right)

        self._write(int(self._current_left), int(self._current_right))

    # ── Internal helpers ─────────────────────────────────────────────── #

    def _map(self, pct: float) -> int:
        """Map 0.0–1.0 percent to MIN_PWM–MAX_PWM, never in dead zone."""
        pct = max(0.0, min(1.0, float(pct)))
        if pct < 0.01:
            return 0
        return int(self.MIN_PWM + pct * (self.MAX_PWM - self.MIN_PWM))

    def _clamp(self, pwm: int) -> int:
        """Hard clamp — result is 0 or in [MIN_PWM, MAX_PWM]."""
        if abs(pwm) < self.MIN_PWM // 2:
            return 0
        sign  = 1 if pwm >= 0 else -1
        value = max(self.MIN_PWM, min(self.MAX_PWM, abs(pwm)))
        return sign * value

    def _ramp(self, current: float, target: float) -> float:
        if current < target:
            return min(current + self.RAMP_STEP, target)
        if current > target:
            return max(current - self.RAMP_STEP, target)
        return target

    def _write(self, left: int, right: int) -> None:
        """Write signed PWM to hardware. Never output dead-zone values."""
        left  = self._safe(left)
        right = self._safe(right)

        try:
            if left == 0 and right == 0:
                self.driver.stop()
            elif left == right:
                # Symmetric — use telemetry-aware call if available
                if hasattr(self.driver, 'set_pwm_all'):
                    self.driver.set_pwm_all(left)
                elif left > 0:
                    self.driver.forward(left)
                else:
                    self.driver.backward(-left)
            else:
                # Differential
                if hasattr(self.driver, 'set_left_right'):
                    self.driver.set_left_right(left, right)
                elif left > 0 and right < 0:
                    self.driver.pivot_left(right)
                elif left < 0 and right > 0:
                    self.driver.pivot_right(left)
                else:
                    self.driver.forward(max(abs(left), abs(right)))
        except Exception as e:
            log.warning("_write(%d, %d) failed: %s", left, right, e)

    def _safe(self, pwm: int) -> int:
        """Enforce dead zone: never output a value in (0, MIN_PWM)."""
        if pwm == 0:
            return 0
        sign  = 1 if pwm > 0 else -1
        value = abs(pwm)
        if value < self.MIN_PWM:
            return 0                          # below dead zone → coast to stop
        return sign * min(value, self.MAX_PWM)

    # ── Telemetry / status ────────────────────────────────────────────── #

    @property
    def target_pwm(self) -> int:
        """Absolute target (left side) for status reporting."""
        return int(abs(self._target_left))

    @property
    def current_pwm(self) -> int:
        """Absolute current PWM being applied (left side)."""
        return int(abs(self._current_left))


# ---------------------------------------------------------------------------
# Background asyncio loop — start once in main()
# ---------------------------------------------------------------------------

async def motor_loop(controller: MotorController, hz: int = _DEFAULT_HZ) -> None:
    """Run controller.tick() at `hz` Hz until cancelled."""
    interval = 1.0 / hz
    log.info("motor_loop started at %d Hz", hz)
    try:
        while True:
            controller.tick()
            await asyncio.sleep(interval)
    except asyncio.CancelledError:
        controller.stop()
        log.info("motor_loop stopped")
