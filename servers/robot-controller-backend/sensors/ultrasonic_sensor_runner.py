# File: /Omega-Code/servers/robot-controller-backend/sensors/ultrasonic_sensor_runner.py

"""
Ultrasonic Sensor Runner

This script runs the ultrasonic sensor in a separate thread, measures distances,
and prints the distance measurements.

Functions:
- run_ultrasonic: Continuously measures and prints the distance using the ultrasonic sensor.
"""

import threading
import time
from ultrasonic_sensor import Ultrasonic
import RPi.GPIO as GPIO

ultrasonic = Ultrasonic()
stop_event = threading.Event()

def run_ultrasonic():
    """
    Continuously measures and prints the distance using the ultrasonic sensor.
    """
    try:
        while not stop_event.is_set():
            distance = ultrasonic.get_distance()
            print(f'Distance: {distance} cm')
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == "__main__":
    ultrasonic_thread = threading.Thread(target=run_ultrasonic)
    ultrasonic_thread.start()
    time.sleep(10)
    print("Stopping ultrasonic thread")
    stop_event.set()
    ultrasonic_thread.join()
    GPIO.cleanup()  # Ensure GPIO is cleaned up when the thread stops
