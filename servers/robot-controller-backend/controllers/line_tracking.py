"""
Line Tracking

This script uses infrared sensors to follow a line and controls the robot's motors based on sensor input.

Class:
- Line_Tracking: Manages the infrared sensors and motor control for line tracking.

Functions:
- run: Continuously reads sensor inputs and controls the motors to follow a line.
"""

import time
from Motor import *
import lgpio

class Line_Tracking:
    def __init__(self):
        """
        Initializes the Line Tracking sensors.
        Sets up GPIO pins and configurations.
        """
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(self.h, self.IR01)
        lgpio.gpio_claim_input(self.h, self.IR02)
        lgpio.gpio_claim_input(self.h, self.IR03)

    def close(self):
        """Release the GPIO handle."""
        lgpio.gpiochip_close(self.h)

    def run(self):
        """
        Continuously reads sensor inputs and controls the motors to follow a line.
        """
        while True:
            self.LMR = 0x00
            if lgpio.gpio_read(self.h, self.IR01):
                self.LMR = (self.LMR | 4)
            if lgpio.gpio_read(self.h, self.IR02):
                self.LMR = (self.LMR | 2)
            if lgpio.gpio_read(self.h, self.IR03):
                self.LMR = (self.LMR | 1)
            if self.LMR == 2:
                PWM.setMotorModel(800, 800, 800, 800)
            elif self.LMR == 4:
                PWM.setMotorModel(-1500, -1500, 2500, 2500)
            elif self.LMR == 6:
                PWM.setMotorModel(-2000, -2000, 4000, 4000)
            elif self.LMR == 1:
                PWM.setMotorModel(2500, 2500, -1500, -1500)
            elif self.LMR == 3:
                PWM.setMotorModel(4000, 4000, -2000, -2000)
            elif self.LMR == 7:
                # pass
                PWM.setMotorModel(0, 0, 0, 0)

infrared = Line_Tracking()

# Main program logic follows:
if __name__ == '__main__':
    print('Program is starting ... ')
    try:
        infrared.run()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program will be executed.
        PWM.setMotorModel(0, 0, 0, 0)
    finally:
        infrared.close()
