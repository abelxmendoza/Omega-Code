#!/usr/bin/env python3
from rpi_ws281x import PixelStrip, Color
import sys

LED_COUNT = 16
LED_PIN = 18
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 255
LED_INVERT = False
LED_CHANNEL = 0

strip = PixelStrip(
    LED_COUNT,
    LED_PIN,
    LED_FREQ_HZ,
    LED_DMA,
    LED_INVERT,
    LED_BRIGHTNESS,
    LED_CHANNEL
)
strip.begin()

def set_all(color):
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
    strip.show()

if len(sys.argv) != 2:
    print("usage: lights.py {purple|off}")
    sys.exit(1)

cmd = sys.argv[1].lower()

if cmd == "purple":
    set_all(Color(128, 0, 128))
elif cmd == "off":
    set_all(Color(0, 0, 0))
else:
    print("unknown command:", cmd)
    print("usage: lights.py {purple|off}")
