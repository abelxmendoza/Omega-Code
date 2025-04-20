#!/usr/bin/env python3
# File: /Omega-Code/servers/robot-controller-backend/sensors/ultrasonic_sensor_runner.py

"""
Ultrasonic Sensor Runner (Pi 5-Compatible – using pigpio)

This script runs the ultrasonic sensor in a separate thread using the pigpio library.
It continuously measures and prints distance readings for monitoring or debugging.

Requirements:
- pigpio daemon running: `sudo pigpiod`
"""

import threading
import time
import pigpio
from ultrasonic_sensor import Ultrasonic

ultrasonic = Ultrasonic()
stop_event = threading.Event()

def run_ultrasonic():
    """
    Continuously measures and prints the distance using the ultrasonic sensor.
    """
    try:
        while not stop_event.is_set():
            distance = ultrasonic.get_distance()
            if distance == -1:
                print("❌ Error: Check sensor wiring or power.")
            else:
                print(f"📏 Distance: {distance} cm")
            time.sleep(1)
    except Exception as e:
        print(f"⚠️ Runner interrupted: {e}")
    finally:
        ultrasonic.pi.stop()  # Properly shut down pigpio interface

if __name__ == "__main__":
    ultrasonic_thread = threading.Thread(target=run_ultrasonic)
    ultrasonic_thread.start()
    try:
        time.sleep(10)  # Run for 10 seconds
    except KeyboardInterrupt:
        print("🔴 Keyboard interrupt received.")
    print("🛑 Stopping ultrasonic thread")
    stop_event.set()
    ultrasonic_thread.join()
    ultrasonic.pi.stop()
