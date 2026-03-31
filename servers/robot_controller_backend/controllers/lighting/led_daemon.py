"""
LED Daemon — runs as root (sudo), reads JSON commands from stdin, controls hardware.

Usage (internal — called by LedController.SubprocessLedController):
  sudo python3 led_daemon.py

Protocol (newline-delimited JSON on stdin):
  {"cmd": "color", "r": 255, "g": 0, "b": 0}
  {"cmd": "off"}
  {"cmd": "brightness", "value": 128}
  {"cmd": "pattern", "name": "rainbow"}
  {"cmd": "quit"}

Responds with {"ok": true} or {"ok": false, "error": "..."} on stdout.
"""

import sys
import json
import traceback

NUM_PIXELS = 16
PIN = 18
BRIGHTNESS = 255
FREQ_HZ = 800000
DMA = 10

try:
    from rpi_ws281x import Adafruit_NeoPixel, Color, WS2811_STRIP_GRB
    strip = Adafruit_NeoPixel(NUM_PIXELS, PIN, FREQ_HZ, DMA, False, BRIGHTNESS, 0, WS2811_STRIP_GRB)
    strip.begin()
    sys.stdout.write(json.dumps({"ok": True, "msg": "LED strip ready"}) + "\n")
    sys.stdout.flush()
except Exception as e:
    sys.stdout.write(json.dumps({"ok": False, "error": str(e)}) + "\n")
    sys.stdout.flush()
    sys.exit(1)


def set_all(r, g, b):
    c = Color(r, g, b)
    for i in range(NUM_PIXELS):
        strip.setPixelColor(i, c)
    strip.show()


def rainbow_cycle():
    import time
    for j in range(256):
        for i in range(NUM_PIXELS):
            pos = (i * 256 // NUM_PIXELS + j) & 255
            if pos < 85:
                c = Color(pos * 3, 255 - pos * 3, 0)
            elif pos < 170:
                pos -= 85
                c = Color(255 - pos * 3, 0, pos * 3)
            else:
                pos -= 170
                c = Color(0, pos * 3, 255 - pos * 3)
            strip.setPixelColor(i, c)
        strip.show()
        time.sleep(0.01)


for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    try:
        cmd = json.loads(line)
        name = cmd.get("cmd", "")

        if name == "color":
            set_all(int(cmd.get("r", 0)), int(cmd.get("g", 0)), int(cmd.get("b", 0)))
            sys.stdout.write(json.dumps({"ok": True}) + "\n")

        elif name == "off":
            set_all(0, 0, 0)
            sys.stdout.write(json.dumps({"ok": True}) + "\n")

        elif name == "brightness":
            strip.setBrightness(int(cmd.get("value", 255)))
            strip.show()
            sys.stdout.write(json.dumps({"ok": True}) + "\n")

        elif name == "pattern":
            pattern = cmd.get("name", "")
            if pattern == "rainbow":
                rainbow_cycle()
                sys.stdout.write(json.dumps({"ok": True}) + "\n")
            elif pattern == "pulse":
                import time
                r, g, b = int(cmd.get("r", 0)), int(cmd.get("g", 0)), int(cmd.get("b", 255))
                for v in list(range(0, 256, 8)) + list(range(255, -1, -8)):
                    c = Color(r * v // 255, g * v // 255, b * v // 255)
                    for i in range(NUM_PIXELS):
                        strip.setPixelColor(i, c)
                    strip.show()
                    time.sleep(0.01)
                sys.stdout.write(json.dumps({"ok": True}) + "\n")
            else:
                sys.stdout.write(json.dumps({"ok": False, "error": f"unknown pattern: {pattern}"}) + "\n")

        elif name == "quit":
            set_all(0, 0, 0)
            sys.stdout.write(json.dumps({"ok": True}) + "\n")
            sys.stdout.flush()
            break

        else:
            sys.stdout.write(json.dumps({"ok": False, "error": f"unknown cmd: {name}"}) + "\n")

    except Exception as e:
        sys.stdout.write(json.dumps({"ok": False, "error": str(e)}) + "\n")

    sys.stdout.flush()
