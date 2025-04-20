#!/usr/bin/env python3
# File: /Omega-Code/servers/robot-controller-backend/sensors/ultrasonic_sensor.py

"""
Ultrasonic Sensor Control (Pi 5-Compatible – using pigpio)

This script controls an ultrasonic sensor using the pigpio library.
It measures distances and helps with obstacle detection.

Class:
- Ultrasonic: Manages the ultrasonic sensor operations using accurate pulse timing.

Hardware:
- Raspberry Pi 5
- HC-SR04 Ultrasonic Sensor
- GPIO Trigger (e.g. 27), Echo (e.g. 22)
- pigpio daemon must be running: `sudo pigpiod`
"""

import time
import pigpio

TRIG = 27
ECHO = 22

class Ultrasonic:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise IOError("❌ pigpio daemon not running. Start it with: sudo pigpiod")
        self.pi.set_mode(TRIG, pigpio.OUTPUT)
        self.pi.set_mode(ECHO, pigpio.INPUT)
        self.pi.write(TRIG, 0)
        time.sleep(0.5)

    def get_distance(self):
        self.pi.gpio_trigger(TRIG, 10)  # Send 10μs pulse
        start_time = time.time()
        timeout = start_time + 1

        # Wait for ECHO to go HIGH
        while self.pi.read(ECHO) == 0:
            if time.time() > timeout:
                print("⚠️ Timeout waiting for ECHO to go HIGH")
                return -1
        start_tick = self.pi.get_current_tick()

        # Wait for ECHO to go LOW
        while self.pi.read(ECHO) == 1:
            if time.time() > timeout:
                print("⚠️ Timeout waiting for ECHO to go LOW")
                return -1
        end_tick = self.pi.get_current_tick()

        duration = pigpio.tickDiff(start_tick, end_tick)  # microseconds
        distance_cm = duration / 58.0  # Convert to cm
        return round(distance_cm, 2)

if __name__ == "__main__":
    sensor = Ultrasonic()
    for _ in range(3):
        dist = sensor.get_distance()
        if dist != -1:
            print(f"📏 Distance: {dist} cm")
        else:
            print("❌ Measurement failed")
        time.sleep(1)
