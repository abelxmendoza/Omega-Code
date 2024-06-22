# File: /Omega-Code/servers/robot-controller-backend/led_control.py

"""
This script controls an LED strip using the rpi_ws281x library.
It allows setting colors, running various light patterns, and controlling the LEDs through command-line arguments.

Key functionalities:
1. Initialize the LED strip with specified configuration.
2. Set the color and patterns of the LEDs.
3. Provide various lighting effects such as color wipe, theater chase, and rainbow.
"""

import sys
import time
from rpi_ws281x import *

# LED strip configuration:
LED_COUNT      = 8       # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (must support PWM).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800kHz).
LED_DMA        = 10      # DMA channel to use for generating signal (try 10).
LED_BRIGHTNESS = 255     # Brightness of the LEDs (0-255).
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift).
LED_CHANNEL    = 0       # Channel number.

class Led:
    """
    A class to handle LED operations for the LED strip.
    """
    def __init__(self):
        self.ORDER = "RGB"  # Order of the colors (RGB or GRB, etc.)
        self.strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()

    def LED_TYPR(self, order, R_G_B):
        """
        Convert color value based on the LED strip order.

        Args:
            order (str): The order of the colors.
            R_G_B (int): The RGB color value.

        Returns:
            int: The converted color value.
        """
        B = R_G_B & 255
        G = R_G_B >> 8 & 255
        R = R_G_B >> 16 & 255 
        Led_type = ["GRB", "GBR", "RGB", "RBG", "BRG", "BGR"]
        color = [Color(G, R, B), Color(G, B, R), Color(R, G, B), Color(R, B, G), Color(B, R, G), Color(B, G, R)]
        if order in Led_type:
            return color[Led_type.index(order)]

    def colorWipe(self, color, wait_ms=50):
        """
        Wipe color across display a pixel at a time.

        Args:
            color (int): The color value.
            wait_ms (int, optional): The wait time in milliseconds. Defaults to 50.
        """
        color = self.LED_TYPR(self.ORDER, color)
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def theaterChase(self, color, wait_ms=50, iterations=10):
        """
        Movie theater light style chaser animation.

        Args:
            color (int): The color value.
            wait_ms (int, optional): The wait time in milliseconds. Defaults to 50.
            iterations (int, optional): Number of iterations. Defaults to 10.
        """
        color = self.LED_TYPR(self.ORDER, color)
        for j in range(iterations):
            for q in range(3):
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i + q, color)
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i + q, 0)

    def wheel(self, pos):
        """
        Generate rainbow colors across 0-255 positions.

        Args:
            pos (int): Position in the color wheel.

        Returns:
            int: The color value.
        """
        if pos < 0 or pos > 255:
            r = g = b = 0
        elif pos < 85:
            r = pos * 3
            g = 255 - pos * 3
            b = 0
        elif pos < 170:
            pos -= 85
            r = 255 - pos * 3
            g = 0
            b = pos * 3
        else:
            pos -= 170
            r = 0
            g = pos * 3
            b = 255 - pos * 3
        return self.LED_TYPR(self.ORDER, Color(r, g, b))

    def rainbow(self, wait_ms=20, iterations=1):
        """
        Draw rainbow that fades across all pixels at once.

        Args:
            wait_ms (int, optional): The wait time in milliseconds. Defaults to 20.
            iterations (int, optional): Number of iterations. Defaults to 1.
        """
        for j in range(256 * iterations):
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, self.wheel((i + j) & 255))
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def rainbowCycle(self, wait_ms=20, iterations=5):
        """
        Draw rainbow that uniformly distributes itself across all pixels.

        Args:
            wait_ms (int, optional): The wait time in milliseconds. Defaults to 20.
            iterations (int, optional): Number of iterations. Defaults to 5.
        """
        for j in range(256 * iterations):
            for i in range(self.strip.numPixels()):
                self.strip.setPixelColor(i, self.wheel((int(i * 256 / self.strip.numPixels()) + j) & 255))
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def theaterChaseRainbow(self, wait_ms=50):
        """
        Rainbow movie theater light style chaser animation.

        Args:
            wait_ms (int, optional): The wait time in milliseconds. Defaults to 50.
        """
        for j in range(256):
            for q in range(3):
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i + q, self.wheel((i + j) % 255))
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i + q, 0)

    def ledIndex(self, index, R, G, B):
        """
        Set color for specific LEDs based on index.

        Args:
            index (int): The LED index.
            R (int): Red color value.
            G (int): Green color value.
            B (int): Blue color value.
        """
        color = self.LED_TYPR(self.ORDER, Color(R, G, B))
        for i in range(8):
            if index & 0x01 == 1:
                self.strip.setPixelColor(i, color)
                self.strip.show()
            index = index >> 1

    def setLed(self, color, mode, pattern, interval):
        """
        Set LED color and pattern.

        Args:
            color (int): The color value.
            mode (str): The mode of the LED ('single', 'multi', 'two').
            pattern (str): The pattern of the LED.
            interval (int): The interval time in milliseconds.
        """
        if mode == 'single':
            self.colorWipe(Color(color))
        elif mode == 'multi':
            self.theaterChase(Color(color), wait_ms=interval)
        elif mode == 'two':
            self.rainbow(wait_ms=interval)
        else:
            self.colorWipe(Color(0, 0, 0), 10)

led = Led()                 

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print("Usage: python3 led_control.py <color> <mode> <pattern> <interval>")
        sys.exit(1)

    color = int(sys.argv[1], 16)
    mode = sys.argv[2]
    pattern = sys.argv[3]
    interval = int(sys.argv[4])

    led.setLed(color, mode, pattern, interval)
