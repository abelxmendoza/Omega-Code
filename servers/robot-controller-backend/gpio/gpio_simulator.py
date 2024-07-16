# gpio_simulator.py

import RPi.GPIO as GPIO
import time
import sys

# Setup pin and state based on arguments
pin = int(sys.argv[1])
state = sys.argv[2]

GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin, GPIO.OUT)

if state == 'HIGH':
    GPIO.output(pin, GPIO.HIGH)
elif state == 'LOW':
    GPIO.output(pin, GPIO.LOW)
else:
    print("Invalid state")
