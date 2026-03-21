"""
Raspberry Pi Motor Driver — Freenove 4WD channel mapping
=========================================================
Channel layout (matches official Freenove motor.py exactly):

  Wheel            Forward ch   Backward ch
  ---------------  -----------  ------------
  Left  upper      ch1          ch0
  Left  lower      ch2          ch3
  Right upper      ch7          ch6
  Right lower      ch5          ch4

STOP = set BOTH channels of a wheel to 4095 (not 0).

set_motor_model(duty1, duty2, duty3, duty4)
  duty1 = left upper   (positive = forward, negative = backward, 0 = stop)
  duty2 = left lower
  duty3 = right upper
  duty4 = right lower

set_pwm(left_pwm, right_pwm)
  Convenience wrapper: applies left_pwm to both left wheels,
  right_pwm to both right wheels.  Signed: positive = forward.
"""

from .base_motor_driver import BaseMotorDriver
from .pca9685_real import PCA9685
from servers.robot_controller_backend.hardware.pwm_singleton import get_pca

# Freenove channel assignments
_CH = {
    'left_upper_fwd':  1,
    'left_upper_bwd':  0,
    'left_lower_fwd':  2,
    'left_lower_bwd':  3,
    'right_upper_fwd': 7,
    'right_upper_bwd': 6,
    'right_lower_fwd': 5,
    'right_lower_bwd': 4,
}

_STOP_DUTY = 4095   # Freenove stop value (both channels high)
_MAX_DUTY  = 4095


class PiMotorDriver(BaseMotorDriver):
    """
    Real PCA9685 motor control on Raspberry Pi using Freenove channel mapping.
    """

    __slots__ = ("pca",)

    def __init__(self, trim_left: int = 0, trim_right: int = 0):
        super().__init__(trim_left, trim_right)
        self.pca = get_pca(PCA9685, 0x40, debug=False)
        self.pca.set_pwm_freq(50)

    # ------------------------------------------------------------------ #
    # Core Freenove-style API                                             #
    # ------------------------------------------------------------------ #

    def set_motor_model(self, duty1: int, duty2: int, duty3: int, duty4: int) -> None:
        """
        Drive all four wheels independently.

        Args:
            duty1: Left  upper wheel  (positive=fwd, negative=bwd, 0=stop)
            duty2: Left  lower wheel
            duty3: Right upper wheel
            duty4: Right lower wheel
        """
        self._drive_wheel(duty1, _CH['left_upper_fwd'],  _CH['left_upper_bwd'])
        self._drive_wheel(duty2, _CH['left_lower_fwd'],  _CH['left_lower_bwd'])
        self._drive_wheel(duty3, _CH['right_upper_fwd'], _CH['right_upper_bwd'])
        self._drive_wheel(duty4, _CH['right_lower_fwd'], _CH['right_lower_bwd'])

    def _drive_wheel(self, duty: int, fwd_ch: int, bwd_ch: int) -> None:
        """Apply a signed duty to one wheel's forward/backward channel pair."""
        duty = max(-_MAX_DUTY, min(_MAX_DUTY, int(duty)))
        if duty > 0:
            # Forward: zero the backward channel, apply duty to forward channel
            self.pca.set_motor_pwm(bwd_ch, 0)
            self.pca.set_motor_pwm(fwd_ch, duty)
        elif duty < 0:
            # Backward: zero the forward channel, apply abs(duty) to backward channel
            self.pca.set_motor_pwm(fwd_ch, 0)
            self.pca.set_motor_pwm(bwd_ch, abs(duty))
        else:
            # Stop: both channels to 4095
            self.pca.set_motor_pwm(fwd_ch, _STOP_DUTY)
            self.pca.set_motor_pwm(bwd_ch, _STOP_DUTY)

    # ------------------------------------------------------------------ #
    # BaseMotorDriver interface                                            #
    # ------------------------------------------------------------------ #

    def set_pwm(self, left_pwm: int, right_pwm: int) -> None:
        """
        Set left and right motor pairs with signed duty values.

        Args:
            left_pwm:  Signed duty for both left  wheels (-4095…4095)
            right_pwm: Signed duty for both right wheels (-4095…4095)

        Positive = forward, negative = backward, 0 = stop.
        Trim offsets are applied before clamping.
        """
        left  = max(-_MAX_DUTY, min(_MAX_DUTY, left_pwm  + self.trim_left))
        right = max(-_MAX_DUTY, min(_MAX_DUTY, right_pwm + self.trim_right))
        self.set_motor_model(left, left, right, right)

    def stop(self) -> None:
        """Stop all four wheels immediately (both channels → 4095)."""
        self.set_motor_model(0, 0, 0, 0)

    def cleanup(self) -> None:
        """Stop motors and release resources."""
        self.stop()
