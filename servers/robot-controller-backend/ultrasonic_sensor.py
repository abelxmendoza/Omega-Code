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
        """
        count = timeout
        while GPIO.input(self.echo_pin) != value and count > 0:
            count -= 1

    def get_distance(self):
        """
        Measures the distance using the ultrasonic sensor.

        Returns:
        int: The measured distance in centimeters.
        """
        distance_cm = [0] * 5
        for i in range(3):
            self.send_trigger_pulse()
            self.wait_for_echo(True, 10000)
            start = time.time()
            self.wait_for_echo(False, 10000)
            finish = time.time()
            pulse_len = finish - start
            distance_cm[i] = pulse_len / 0.000058
        distance_cm = sorted(distance_cm)
        return int(distance_cm[2])

ultrasonic = Ultrasonic()

# Main program logic follows:
if __name__ == '__main__':
    print('Program is starting ... ')
    try:
        while True:
            distance = ultrasonic.get_distance()
            print(f'Distance: {distance} cm')
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
