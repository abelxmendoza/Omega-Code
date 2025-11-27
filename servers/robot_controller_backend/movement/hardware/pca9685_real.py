"""
Real PCA9685 Hardware Driver

Hardware implementation for Raspberry Pi using smbus2.
"""

import time
import math

try:
    import smbus2 as smbus
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False


class PCA9685:
    """
    Raspi PCA9685 16-Channel PWM Servo Driver
    
    Real hardware implementation using I2C via smbus2.
    """
    
    # Registers
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD
    
    def __init__(self, address: int = 0x40, debug: bool = False):
        """
        Initialize PCA9685.
        
        Args:
            address: I2C address (default 0x40)
            debug: Enable debug output
        """
        if not SMBUS_AVAILABLE:
            raise ImportError("smbus2 not available. Install with: pip install smbus2")
        
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)
    
    def write(self, reg: int, value: int) -> None:
        """Write an 8-bit value to the specified register/address."""
        self.bus.write_byte_data(self.address, reg, value)
    
    def read(self, reg: int) -> int:
        """Read an unsigned byte from the I2C device."""
        return self.bus.read_byte_data(self.address, reg)
    
    def setPWMFreq(self, freq: int) -> None:
        """Set the PWM frequency."""
        prescaleval = 25000000.0  # 25MHz
        prescaleval /= 4096.0  # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = math.floor(prescaleval + 0.5)
        
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.write(self.__MODE1, newmode)  # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)
    
    def setPWM(self, channel: int, on: int, off: int) -> None:
        """Set a single PWM channel."""
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)
    
    def setMotorPwm(self, channel: int, duty: int) -> None:
        """Set motor PWM (convenience method)."""
        self.setPWM(channel, 0, duty)
    
    def setServoPulse(self, channel: int, pulse: int) -> None:
        """Set the Servo Pulse. The PWM frequency must be 50HZ."""
        pulse = pulse * 4096 / 20000  # PWM frequency is 50HZ, period is 20000us
        self.setPWM(channel, 0, int(pulse))

