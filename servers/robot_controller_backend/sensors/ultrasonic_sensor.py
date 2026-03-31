#!/usr/bin/env python3
# File: /Omega-Code/servers/robot_controller_backend/sensors/ultrasonic_sensor.py

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
import warnings
from gpiozero import DistanceSensor, PWMSoftwareFallback, DistanceSensorNoEcho

TRIG = 27
ECHO = 22


class Ultrasonic:
    def __init__(self, trigger_pin: int = TRIG, echo_pin: int = ECHO, max_distance: float = 3.0):
        warnings.filterwarnings("ignore", category=DistanceSensorNoEcho)
        warnings.filterwarnings("ignore", category=PWMSoftwareFallback)
        self.sensor = DistanceSensor(
            echo=echo_pin, trigger=trigger_pin, max_distance=max_distance
        )
        time.sleep(0.2)

    def get_distance(self):
        """Returns distance in cm, or -1 on error."""
        try:
            distance_cm = self.sensor.distance * 100
            distance_cm = round(float(distance_cm), 1)
            if distance_cm < 2 or distance_cm > 400:
                return -1
            return distance_cm
        except Exception:
            return -1

    def close(self):
        self.sensor.close()

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
