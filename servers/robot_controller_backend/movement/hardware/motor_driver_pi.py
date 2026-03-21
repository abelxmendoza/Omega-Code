"""
Raspberry Pi Motor Driver — mirrors Freenove Ordinary_Car exactly
==================================================================
Channel layout (matches official Freenove motor.py):

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

_MAX_DUTY = 4095


class PiMotorDriver(BaseMotorDriver):
    """
    Real PCA9685 motor control on Raspberry Pi — mirrors Freenove Ordinary_Car.
    """

    def __init__(self, trim_left: int = 0, trim_right: int = 0):
        super().__init__(trim_left, trim_right)
        self.pca9685 = PCA9685()
        self.pca9685.set_pwm_freq(50)

    # ------------------------------------------------------------------ #
    # Per-wheel methods (Freenove Ordinary_Car style)                     #
    # ------------------------------------------------------------------ #

    def left_upper_wheel(self, duty: int) -> None:
        if duty > 0:
            self.pca9685.set_motor_pwm(0, 0)
            self.pca9685.set_motor_pwm(1, duty)
        elif duty < 0:
            self.pca9685.set_motor_pwm(1, 0)
            self.pca9685.set_motor_pwm(0, abs(duty))
        else:
            self.pca9685.set_motor_pwm(0, 4095)
            self.pca9685.set_motor_pwm(1, 4095)

    def left_lower_wheel(self, duty: int) -> None:
        if duty > 0:
            self.pca9685.set_motor_pwm(3, 0)
            self.pca9685.set_motor_pwm(2, duty)
        elif duty < 0:
            self.pca9685.set_motor_pwm(2, 0)
            self.pca9685.set_motor_pwm(3, abs(duty))
        else:
            self.pca9685.set_motor_pwm(2, 4095)
            self.pca9685.set_motor_pwm(3, 4095)

    def right_upper_wheel(self, duty: int) -> None:
        if duty > 0:
            self.pca9685.set_motor_pwm(6, 0)
            self.pca9685.set_motor_pwm(7, duty)
        elif duty < 0:
            self.pca9685.set_motor_pwm(7, 0)
            self.pca9685.set_motor_pwm(6, abs(duty))
        else:
            self.pca9685.set_motor_pwm(6, 4095)
            self.pca9685.set_motor_pwm(7, 4095)

    def right_lower_wheel(self, duty: int) -> None:
        if duty > 0:
            self.pca9685.set_motor_pwm(4, 0)
            self.pca9685.set_motor_pwm(5, duty)
        elif duty < 0:
            self.pca9685.set_motor_pwm(5, 0)
            self.pca9685.set_motor_pwm(4, abs(duty))
        else:
            self.pca9685.set_motor_pwm(4, 4095)
            self.pca9685.set_motor_pwm(5, 4095)

    # ------------------------------------------------------------------ #
    # High-level API                                                       #
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
        duty1 = max(-_MAX_DUTY, min(_MAX_DUTY, int(duty1)))
        duty2 = max(-_MAX_DUTY, min(_MAX_DUTY, int(duty2)))
        duty3 = max(-_MAX_DUTY, min(_MAX_DUTY, int(duty3)))
        duty4 = max(-_MAX_DUTY, min(_MAX_DUTY, int(duty4)))
        self.left_upper_wheel(duty1)
        self.left_lower_wheel(duty2)
        self.right_upper_wheel(duty3)
        self.right_lower_wheel(duty4)

    def set_pwm(self, left_pwm: int, right_pwm: int) -> None:
        """
        Set left and right motor pairs with signed duty values.

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
