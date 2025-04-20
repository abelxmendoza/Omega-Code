#!/usr/bin/env python3
# File: /Omega-Code/servers/robot-controller-backend/sensors/ultrasonic_sensor.py

"""
Ultrasonic Sensor Control (Pi 5-Compatible – using lgpio)

This script controls an ultrasonic sensor using the lgpio library.
It measures distances and helps with obstacle detection.

Class:
- Ultrasonic: Manages the ultrasonic sensor operations using accurate pulse timing.

Hardware:
- Raspberry Pi 5
- HC-SR04 Ultrasonic Sensor
- GPIO Trigger (e.g. 27), Echo (e.g. 22)
"""

import time
import lgpio

TRIG = 27
ECHO = 22

class Ultrasonic:
    def __init__(self):
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, TRIG, 0)
        lgpio.gpio_claim_input(self.h, ECHO)
        time.sleep(0.5)

    def get_distance(self):
        # Send 10μs pulse
        lgpio.gpio_write(self.h, TRIG, 1)
        time.sleep(10e-6)
        lgpio.gpio_write(self.h, TRIG, 0)

        # Wait for echo HIGH
        tick_start = lgpio.tick()
        timeout = tick_start + 1000000  # 1 second timeout in μs

        while lgpio.gpio_read(self.h, ECHO) == 0:
            if lgpio.tick() - tick_start > 1000000:
                print("⚠️ Timeout waiting for ECHO to go HIGH")
                return -1
        start = lgpio.tick()

        while lgpio.gpio_read(self.h, ECHO) == 1:
            if lgpio.tick() - start > 1000000:
                print("⚠️ Timeout waiting for ECHO to go LOW")
                return -1
        end = lgpio.tick()

        pulse_len = lgpio.tick_diff(start, end)
        distance_cm = pulse_len / 58.0
        return round(distance_cm, 2)

    def cleanup(self):
        lgpio.gpiochip_close(self.h)

if __name__ == "__main__":
    sensor = Ultrasonic()
    try:
        for _ in range(3):
            dist = sensor.get_distance()
            if dist != -1:
                print(f"📏 Distance: {dist} cm")
            else:
                print("❌ Measurement failed")
            time.sleep(1)
    finally:
        sensor.cleanup()
