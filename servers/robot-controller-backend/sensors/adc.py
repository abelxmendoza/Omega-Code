#!/usr/bin/env python3
# File: /Omega-Code/servers/robot-controller-backend/adc.py
"""
Omega 1 ADC Utility
-------------------
This script reads analog voltages from supported ADC chips via I2C (PCF8591 and ADS7830),
autodetects the chip type, and continuously displays readings from channels 0–2.

🔌 Hardware Requirements:
- I2C enabled on Raspberry Pi (bus 1)
- PCF8591 or ADS7830 connected
- smbus2 installed

🧪 Features:
- Autodetects ADC model
- Converts ADC values to voltages
- Continuous output loop with Ctrl+C exit
"""

import time
from smbus2 import SMBus
import sys

class Adc:
    def __init__(self, address=0x48):
        self.bus = SMBus(1)
        self.ADDRESS = address
        self.PCF8591_CMD = 0x40
        self.ADS7830_CMD = 0x84
        self.Index = self.detect_chip_type()

    def detect_chip_type(self):
        """
        Attempts to detect if the connected chip is a PCF8591 or ADS7830.
        """
        try:
            for _ in range(3):
                read = self.bus.read_byte_data(self.ADDRESS, 0xf4)
                if read < 150:
                    return "PCF8591"
            return "ADS7830"
        except Exception as e:
            print(f"❌ Failed to detect ADC chip: {e}")
            sys.exit(1)

    def analog_read_pcf8591(self, chn):
        values = [self.bus.read_byte_data(self.ADDRESS, self.PCF8591_CMD + chn) for _ in range(9)]
        return sorted(values)[4]  # Median filtering

    def recv_pcf8591(self, chn):
        while True:
            v1 = self.analog_read_pcf8591(chn)
            v2 = self.analog_read_pcf8591(chn)
            if v1 == v2:
                return round(v1 / 256.0 * 3.3, 2)

    def recv_ads7830(self, chn):
        cmd = self.ADS7830_CMD | ((((chn << 2) | (chn >> 1)) & 0x07) << 4)
        self.bus.write_byte(self.ADDRESS, cmd)
        while True:
            v1 = self.bus.read_byte(self.ADDRESS)
            v2 = self.bus.read_byte(self.ADDRESS)
            if v1 == v2:
                return round(v1 / 255.0 * 3.3, 2)

    def recv_adc(self, chn):
        if self.Index == "PCF8591":
            return self.recv_pcf8591(chn)
        elif self.Index == "ADS7830":
            return self.recv_ads7830(chn)
        else:
            raise ValueError("Unsupported ADC type")

    def close(self):
        self.bus.close()

def loop():
    adc = Adc()
    print(f"📡 Detected ADC Chip: {adc.Index}\n")
    try:
        while True:
            readings = {
                "Left_IDR": adc.recv_adc(0),
                "Right_IDR": adc.recv_adc(1),
                "Power": round(adc.recv_adc(2) * 3, 2)
            }
            for label, val in readings.items():
                print(f"{label}: {val} V")
            print("-" * 30)
            time.sleep(1)
    except KeyboardInterrupt:
        print("🛑 Stopping ADC loop.")
        adc.close()

if __name__ == "__main__":
    loop()
