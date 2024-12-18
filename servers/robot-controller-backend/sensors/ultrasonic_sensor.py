# File: /Omega-Code/servers/robot-controller-backend/sensors/ultrasonic_sensor.py

"""
Ultrasonic Sensor Control

This script controls an ultrasonic sensor to measure distances and helps with obstacle detection.

Class:
- Ultrasonic: Manages the ultrasonic sensor operations and motor control based on distance measurements.

Functions:
- send_trigger_pulse: Sends a trigger pulse to the sensor.
- wait_for_echo: Waits for an echo response from the sensor.
- get_distance: Measures the distance using the sensor.
"""

import time
import RPi.GPIO as GPIO

class Ultrasonic:
    def __init__(self):
        """
        Initializes the Ultrasonic sensor.
        Sets up GPIO pins and configurations.
        """
        GPIO.setwarnings(False)
        self.trigger_pin = 27
        self.echo_pin = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        self.check_gpio_state()

    def check_gpio_state(self):
        """
        Validates the initial GPIO pin states.
        """
        trigger_state = GPIO.input(self.trigger_pin)
        echo_state = GPIO.input(self.echo_pin)
        print(f"Initial Trigger Pin State: {trigger_state}")
        print(f"Initial Echo Pin State: {echo_state}")
        if echo_state not in [GPIO.HIGH, GPIO.LOW]:
            print("Warning: Echo pin not responding correctly. Check connections.")

    def send_trigger_pulse(self):
        """
        Sends a trigger pulse to the ultrasonic sensor.
        """
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00015)
        GPIO.output(self.trigger_pin, False)

    def wait_for_echo(self, value, timeout):
        """
        Waits for the echo response from the sensor.

        Parameters:
        value (bool): The expected value from the echo pin.
        timeout (int): The maximum count to wait for the response.

        Returns:
        bool: True if echo received, False otherwise.
        """
        count = timeout
        while GPIO.input(self.echo_pin) != value and count > 0:
            count -= 1
        return count > 0

    def get_distance(self):
        """
        Measures the distance using the ultrasonic sensor.

        Returns:
        int: The measured distance in centimeters, or -1 if an error occurs.
        """
        distance_cm = []
        for i in range(3):
            self.send_trigger_pulse()
            if not self.wait_for_echo(True, 10000):
                print("Error: No echo received (HIGH). Check connections or sensor power.")
                return -1
            start = time.time()
            if not self.wait_for_echo(False, 10000):
                print("Error: No echo received (LOW). Check connections or sensor power.")
                return -1
            finish = time.time()
            pulse_len = finish - start
            if pulse_len <= 0 or pulse_len > 0.04:  # Check for valid pulse length
                print("Error: Invalid pulse length detected. Possible connection issue.")
                return -1
            distance_cm.append(pulse_len / 0.000058)  # Calculate distance in cm
        distance_cm = sorted(distance_cm)
        return int(distance_cm[1])  # Return the median of the three readings
