# File: /Omega-Code/servers/robot-controller-backend/adc.py

"""
This script interfaces with ADC chips (PCF8591 and ADS7830) using the I2C protocol.
It reads analog values from different channels, converts them to voltage, and prints the values in a loop.

Key functionalities:
1. Initialize I2C communication with PCF8591 and ADS7830 chips.
2. Read analog values from specified channels and convert them to voltage.
3. Continuously print the values from the ADC channels.
"""

import time
from smbus2 import SMBus

class Adc:
    """
    A class to handle ADC operations for PCF8591 and ADS7830 chips.
    """
    def __init__(self):
        # Get I2C bus
        self.bus = SMBus(1)
        
        # I2C address of the device
        self.ADDRESS = 0x48
        
        # PCF8591 Command
        self.PCF8591_CMD = 0x40  # Command
        
        # ADS7830 Command 
        self.ADS7830_CMD = 0x84  # Single-Ended Inputs
        
        # Determine the type of ADC chip
        for i in range(3):
            aa = self.bus.read_byte_data(self.ADDRESS, 0xf4)
            if aa < 150:
                self.Index = "PCF8591"
            else:
                self.Index = "ADS7830" 

    def analogReadPCF8591(self, chn):  
        """
        Read ADC value from PCF8591.

        Args:
            chn (int): The channel to read from (0, 1, 2, or 3).

        Returns:
            int: The ADC value.
        """
        value = [0] * 9
        for i in range(9):
            value[i] = self.bus.read_byte_data(self.ADDRESS, self.PCF8591_CMD + chn)
        value = sorted(value)
        return value[4]   
        
    def analogWritePCF8591(self, value):  
        """
        Write DAC value to PCF8591.

        Args:
            value (int): The value to write.
        """
        self.bus.write_byte_data(self.ADDRESS, self.PCF8591_CMD, value)
        
    def recvPCF8591(self, channel):  
        """
        Read and convert ADC value from PCF8591.

        Args:
            channel (int): The channel to read from (0, 1, 2, or 3).

        Returns:
            float: The voltage value.
        """
        while True:
            value1 = self.analogReadPCF8591(channel)
            value2 = self.analogReadPCF8591(channel)
            if value1 == value2:
                break
        voltage = value1 / 256.0 * 3.3  # calculate the voltage value
        voltage = round(voltage, 2)
        return voltage

    def recvADS7830(self, channel):
        """
        Read and convert ADC value from ADS7830.

        Args:
            channel (int): The channel to read from (0, 1, 2, or 3).

        Returns:
            float: The voltage value.
        """
        COMMAND_SET = self.ADS7830_CMD | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)
        self.bus.write_byte(self.ADDRESS, COMMAND_SET)
        while True:
            value1 = self.bus.read_byte(self.ADDRESS)
            value2 = self.bus.read_byte(self.ADDRESS)
            if value1 == value2:
                break
        voltage = value1 / 255.0 * 3.3  # calculate the voltage value
        voltage = round(voltage, 2)
        return voltage
        
    def recvADC(self, channel):
        """
        Read ADC value from the determined ADC chip (PCF8591 or ADS7830).

        Args:
            channel (int): The channel to read from (0, 1, 2, or 3).

        Returns:
            float: The voltage value.
        """
        if self.Index == "PCF8591":
            data = self.recvPCF8591(channel)
        elif self.Index == "ADS7830":
            data = self.recvADS7830(channel)
        return data

    def i2cClose(self):
        """
        Close the I2C bus.
        """
        self.bus.close()

def loop():
    """
    Main loop to read and print ADC values continuously.
    """
    adc = Adc()
    while True:
        Left_IDR = adc.recvADC(0)
        print(Left_IDR)
        Right_IDR = adc.recvADC(1)
        print(Right_IDR)
        Power = adc.recvADC(2) * 3
        print(Power)
        time.sleep(1)
        print('----')

def destroy():
    """
    Placeholder for cleanup actions.
    """
    pass

# Main program logic follows:
if __name__ == '__main__':
    print('Program is starting ... ')
    try:
        loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the destroy() function will be executed.
        destroy()
