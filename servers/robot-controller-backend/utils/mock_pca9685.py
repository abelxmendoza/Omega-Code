# File: /Omega-Code/servers/robot-controller-backend/utils/mock_pca9685.py
"""
This module provides a mock implementation of the PCA9685 class for testing purposes
on non-Raspberry Pi environments. It mimics the behavior of the actual PCA9685 module.
"""

class PCA9685:
    def __init__(self, address, debug=False):
        """
        Initialize the mock PCA9685 at the given I2C address.

        :param address: I2C address of the PCA9685
        :param debug: If True, print debug information
        """
        self.address = address
        self.debug = debug
        self.frequency = None
        self.pulse_length = {}
        if self.debug:
            print(f"Initialized mock PCA9685 at address {address} with debug={debug}")

    def setPWMFreq(self, freq):
        """
        Set the PWM frequency for the mock PCA9685.

        :param freq: Frequency in Hz
        """
        self.frequency = freq
        if self.debug:
            print(f"Set PWM frequency to {freq}")

    def setServoPulse(self, channel, pulse):
        """
        Set the servo pulse width for the specified channel.

        :param channel: Channel number
        :param pulse: Pulse width
        """
        self.pulse_length[channel] = pulse
        if self.debug:
            print(f"Set servo pulse on channel {channel} to {pulse}")
