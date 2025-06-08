#!/usr/bin/env python3
# File: /Omega-Code/servers/robot-controller-backend/sensors/ultrasonic_sensor_runner.py

"""
Ultrasonic Sensor Runner (Pi 5-Compatible – using lgpio)

This script runs the ultrasonic sensor in a separate thread using the lgpio library.
It continuously measures and prints distance readings for monitoring or debugging.

Requirements:
- lgpio installed (`pip install lgpio`)
- Script must be run with sufficient permissions (e.g., `newgrp gpio` or via `sudo`)
"""

import threading
import time
from ultrasonic_sensor import Ultrasonic

stop_event = threading.Event()

def run_ultrasonic():
    """
    Continuously measures and prints the distance using the ultrasonic sensor.
    """
    ultrasonic = Ultrasonic()
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
        ultrasonic.close()
        print("🧹 GPIO closed")

if __name__ == "__main__":
    ultrasonic_thread = threading.Thread(target=run_ultrasonic)
    ultrasonic_thread.start()
    try:
        time.sleep(10)  # Run for 10 seconds or until interrupted
    except KeyboardInterrupt:
        print("🔴 Keyboard interrupt received.")
    print("🛑 Stopping ultrasonic thread")
    stop_event.set()
    ultrasonic_thread.join()
