#!/usr/bin/env python3
import sys
from rpi_ws281x import Adafruit_NeoPixel, Color, WS2811_STRIP_GRB

# LED strip configuration
LED_COUNT = 16
LED_PIN = 18
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_INVERT = False
LED_BRIGHTNESS = 255
LED_CHANNEL = 0

def set_color(r, g, b):
    try:
        strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        strip.begin()
        
        for i in range(LED_COUNT):
            strip.setPixelColor(i, Color(r, g, b))
        strip.show()
        print(f'LEDs set to RGB({r}, {g}, {b})')
        
    except Exception as e:
        print(f'Error: {e}')
        sys.exit(1)

if __name__ == '__main__':
    if len(sys.argv) == 4:
        try:
            r = int(sys.argv[1])
            g = int(sys.argv[2])
            b = int(sys.argv[3])
            set_color(r, g, b)
        except ValueError:
            print('Error: RGB values must be integers')
            sys.exit(1)
    else:
        print('Usage: sudo python3 rgb_led_test.py <r> <g> <b>')
        sys.exit(1)
