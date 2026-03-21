"""
Real PCA9685 Hardware Driver

Hardware implementation for Raspberry Pi using smbus (Freenove-compatible).
Method names match the official Freenove PCA9685.py:
  set_pwm_freq(freq)          -- was setPWMFreq
  set_motor_pwm(channel, duty) -- was setMotorPwm
"""

import time
import math

# Prefer system smbus (matches Freenove), fall back to smbus2 if not present
try:
    import smbus
    SMBUS_AVAILABLE = True
except ImportError:
    try:
        import smbus2 as smbus
        SMBUS_AVAILABLE = True
    except ImportError:
        SMBUS_AVAILABLE = False


class PCA9685:
    """
    Raspi PCA9685 16-Channel PWM Servo Driver

    Real hardware implementation using I2C via smbus.
    Method names match Freenove's official PCA9685.py.
    """

    # Registers
    __SUBADR1      = 0x02
    __SUBADR2      = 0x03
    __SUBADR3      = 0x04
    __MODE1        = 0x00
    __PRESCALE     = 0xFE
    __LED0_ON_L    = 0x06
    __LED0_ON_H    = 0x07
    __LED0_OFF_L   = 0x08
    __LED0_OFF_H   = 0x09
    __ALLLED_ON_L  = 0xFA
    __ALLLED_ON_H  = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address: int = 0x40, debug: bool = False):
        if not SMBUS_AVAILABLE:
            raise ImportError(
                'smbus not available. Install with: sudo apt install python3-smbus'
            )
        self.bus     = smbus.SMBus(1)
        self.address = address
        self.debug   = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg: int, value: int) -> None:
        """Write an 8-bit value to the specified register."""
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg: int) -> int:
        """Read an unsigned byte from the I2C device."""
        return self.bus.read_byte_data(self.address, reg)

    # ------------------------------------------------------------------ #
    # Freenove-compatible primary method names                            #
    # ------------------------------------------------------------------ #

    def set_pwm_freq(self, freq: int) -> None:
        """Set the PWM frequency (Hz)."""
        prescaleval  = 25000000.0 / 4096.0 / float(freq) - 1.0
        prescale     = int(math.floor(prescaleval + 0.5))
        oldmode      = self.read(self.__MODE1)
        self.write(self.__MODE1, (oldmode & 0x7F) | 0x10)   # sleep
        self.write(self.__PRESCALE, prescale)
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        """Set a single PWM channel with explicit on/off counts."""
        self.write(self.__LED0_ON_L  + 4 * channel, on  & 0xFF)
        self.write(self.__LED0_ON_H  + 4 * channel, on  >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        """Set motor PWM duty cycle on a single channel (on=0, off=duty)."""
        self.set_pwm(channel, 0, duty)

    def set_servo_pulse(self, channel: int, pulse: int) -> None:
        """Set servo pulse width in microseconds (PWM freq must be 50 Hz)."""
        self.set_pwm(channel, 0, int(pulse * 4096 / 20000))

    # ------------------------------------------------------------------ #
    # Legacy aliases (kept so existing callers don't break)               #
    # ------------------------------------------------------------------ #

    def setPWMFreq(self, freq: int) -> None:
        self.set_pwm_freq(freq)

    def setPWM(self, channel: int, on: int, off: int) -> None:
        self.set_pwm(channel, on, off)

    def setMotorPwm(self, channel: int, duty: int) -> None:
        self.set_motor_pwm(channel, duty)

    def setServoPulse(self, channel: int, pulse: int) -> None:
        self.set_servo_pulse(channel, pulse)
