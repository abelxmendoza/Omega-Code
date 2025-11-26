#!/usr/bin/env python3
# File: /Omega-Code/servers/robot-controller-backend/sensors/ultrasonic_sensor.py

"""
Ultrasonic Sensor Control (Pi 5-Compatible – using lgpio)

🔧 Purpose: Python utility class for HC-SR04 ultrasonic sensor
   - Used by: ultrasonic_ws_server.py, ultrasonic_sensor_runner.py
   - Not meant to be run directly (import and use in other scripts)
   - Provides: Ultrasonic class for sensor control

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

# Constants - cached for performance
TRIG = 27
ECHO = 22
TIMEOUT_NS = 1_000_000_000  # 1 second in nanoseconds
US_TO_CM_DIVISOR = 58.0  # Microseconds to cm conversion factor
NS_TO_US_DIVISOR = 1000.0  # Nanoseconds to microseconds

class Ultrasonic:
    def __init__(self):
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, TRIG, 0)
        lgpio.gpio_claim_input(self.h, ECHO)
        time.sleep(0.5)

    def get_distance(self):
        """
        Optimized distance measurement with reduced time calls and efficient polling.
        Returns distance in cm, or -1 on error.
        """
        # Trigger pulse sequence
        lgpio.gpio_write(self.h, TRIG, 0)
        time.sleep(0.000002)  # 2 µs
        lgpio.gpio_write(self.h, TRIG, 1)
        time.sleep(0.00002)   # 20 µs
        lgpio.gpio_write(self.h, TRIG, 0)

        # Wait for echo HIGH with optimized polling
        wait_start = time.monotonic_ns()
        deadline = wait_start + TIMEOUT_NS
        poll_interval = 10e-6  # Start with 10µs polling
        
        while lgpio.gpio_read(self.h, ECHO) == 0:
            now = time.monotonic_ns()
            if now >= deadline:
                print("⚠️ Timeout waiting for ECHO to go HIGH")
                return -1
            # Adaptive polling: shorter delays initially, longer as we approach timeout
            if now - wait_start < TIMEOUT_NS // 2:
                time.sleep(poll_interval)
            else:
                time.sleep(poll_interval * 5)  # Slower polling near timeout
        
        # Capture pulse start time
        start = time.monotonic_ns()
        pulse_deadline = start + TIMEOUT_NS
        
        # Wait for echo LOW with optimized polling
        while lgpio.gpio_read(self.h, ECHO) == 1:
            now = time.monotonic_ns()
            if now >= pulse_deadline:
                print("⚠️ Timeout waiting for ECHO to go LOW")
                return -1
            time.sleep(poll_interval)
        
        # Calculate distance using cached constants
        end = time.monotonic_ns()
        pulse_len_us = (end - start) / NS_TO_US_DIVISOR
        distance_cm = pulse_len_us / US_TO_CM_DIVISOR
        
        # Range validation
        if distance_cm < 2 or distance_cm > 400:
            return -1
        
        return round(distance_cm, 2)

    def close(self):
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
        sensor.close()
