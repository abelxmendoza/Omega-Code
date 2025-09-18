"""Utility for keeping the robot driving in a straight line.

The helper tracks left/right trim offsets that are applied to the motor
controller in order to compensate for mechanical differences between wheels.
It exposes small convenience methods so higher-level components (such as the
WebSocket server) can enable/disable the assist feature or nudge the
compensation in response to observed drift.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

DEFAULT_STEP = 25
DEFAULT_MAX_TRIM = 800
ABSOLUTE_MAX_TRIM = 4095


class StraightDriveAssist:
    """Maintain trim offsets that help the robot drive straight."""

    def __init__(self, motor: Any, *, enabled: bool = True, step: int = DEFAULT_STEP,
                 max_trim: int = DEFAULT_MAX_TRIM) -> None:
        self.motor = motor
        self.enabled = bool(enabled)
        self.step = self._sanitize_positive(step, fallback=DEFAULT_STEP)
        self.max_trim = self._sanitize_positive(max_trim, fallback=DEFAULT_MAX_TRIM)
        self.left_trim = 0
        self.right_trim = 0
        self._supports_trim = hasattr(motor, "set_trim")
        self._max_pwm = self._detect_max_pwm()
        self._last_applied = 0.0
        self._apply()

    # ------------------------------------------------------------------
    def status(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled,
            "leftTrim": self.left_trim,
            "rightTrim": self.right_trim,
            "step": self.step,
            "maxTrim": self.max_trim,
            "supportsMotorTrim": self._supports_trim,
            "lastApplied": self._last_applied,
        }

    # ------------------------------------------------------------------
    def set_enabled(self, enabled: bool) -> Dict[str, Any]:
        self.enabled = bool(enabled)
        self._apply()
        return self.status()

    def set_step(self, value: int) -> Dict[str, Any]:
        self.step = self._sanitize_positive(value, fallback=self.step)
        return self.status()

    def set_max_trim(self, value: int) -> Dict[str, Any]:
        self.max_trim = self._sanitize_positive(value, fallback=self.max_trim)
        self.left_trim = self._clamp(self.left_trim)
        self.right_trim = self._clamp(self.right_trim)
        self._apply()
        return self.status()

    def set_trim(self, *, left: Optional[int] = None, right: Optional[int] = None) -> Dict[str, Any]:
        if left is not None:
            self.left_trim = self._clamp(int(left))
        if right is not None:
            self.right_trim = self._clamp(int(right))
        self._apply()
        return self.status()

    def reset(self) -> Dict[str, Any]:
        self.left_trim = 0
        self.right_trim = 0
        self._apply()
        return self.status()

    def nudge(self, direction: str, amount: Optional[int] = None) -> Dict[str, Any]:
        if not direction:
            raise ValueError("direction is required")
        step = self._sanitize_positive(amount if amount is not None else self.step,
                                       fallback=self.step)
        token = direction.strip().lower()
        if token in {"left", "l"}:
            # Robot veers left -> speed up right side
            self.right_trim = self._clamp(self.right_trim + step)
        elif token in {"right", "r"}:
            # Robot veers right -> speed up left side
            self.left_trim = self._clamp(self.left_trim + step)
        elif token in {"center", "straight", "none", "neutral", "balance"}:
            self.left_trim = self._relax(self.left_trim, step)
            self.right_trim = self._relax(self.right_trim, step)
        elif token in {"reset", "zero"}:
            return self.reset()
        else:
            raise ValueError(f"unknown direction '{direction}'")
        self._apply()
        return self.status()

    # ------------------------------------------------------------------
    def _apply(self) -> None:
        self._last_applied = time.time()
        left = self.left_trim if self.enabled else 0
        right = self.right_trim if self.enabled else 0
        if self._supports_trim:
            try:
                self.motor.set_trim(left=left, right=right)
            except TypeError:
                self.motor.set_trim(left, right)  # type: ignore[call-arg]
        else:
            if hasattr(self.motor, "left_trim"):
                setattr(self.motor, "left_trim", left)
            if hasattr(self.motor, "right_trim"):
                setattr(self.motor, "right_trim", right)

    def _relax(self, value: int, amount: int) -> int:
        if value > 0:
            return max(0, value - amount)
        if value < 0:
            return min(0, value + amount)
        return 0

    def _clamp(self, value: int) -> int:
        limit = min(self.max_trim, self._max_pwm)
        return max(-limit, min(limit, int(value)))

    def _detect_max_pwm(self) -> int:
        for attr in ("MAX_PWM", "max_pwm", "maxSpeed", "max_speed"):
            candidate = getattr(self.motor, attr, None)
            if isinstance(candidate, (int, float)) and candidate > 0:
                return int(min(candidate, ABSOLUTE_MAX_TRIM))
        return ABSOLUTE_MAX_TRIM

    @staticmethod
    def _sanitize_positive(value: Optional[int], *, fallback: int) -> int:
        try:
            ivalue = int(value) if value is not None else int(fallback)
        except Exception:
            ivalue = int(fallback)
        return max(1, abs(ivalue))


__all__ = ["StraightDriveAssist"]
