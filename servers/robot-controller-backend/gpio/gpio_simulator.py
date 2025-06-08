# File: /Omega-Code/servers/robot-controller-backend/gpio/gpio_simulator.py
"""
This script simulates GPIO actions using the lgpio library.
It provides basic GPIO operations like setting pin mode, outputting values, and cleanup.
"""

import lgpio
import sys
import time

# Setup pin and state based on arguments
pin = int(sys.argv[1])
state = sys.argv[2].upper()

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, pin, 0)

if state == 'HIGH':
    lgpio.gpio_write(h, pin, 1)
elif state == 'LOW':
    lgpio.gpio_write(h, pin, 0)
else:
    print("Invalid state")

# Brief delay to allow observation when testing
time.sleep(0.1)
lgpio.gpiochip_close(h)
