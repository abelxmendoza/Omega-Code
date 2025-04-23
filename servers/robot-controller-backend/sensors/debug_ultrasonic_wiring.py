#!/usr/bin/env python3
# File: sensors/debug_ultrasonic_wiring.py

"""
Ultrasonic Sensor Wiring Debugger
---------------------------------
Tests the responsiveness of an HC-SR04 ultrasonic sensor wired to GPIO.

✅ Purpose:
- Verifies if the ECHO pin responds to a 10µs pulse on TRIG
- Detects HIGH/LOW state changes on ECHO during a 100ms observation window

🛠 Requirements:
- Raspberry Pi 5 (or compatible)
- lgpio installed (`pip install lgpio`)
- TRIG: GPIO27 (pin 13), ECHO: GPIO22 (pin 15)
- Must have access to /dev/gpiochip0 (part of gpio group or run with sudo)
"""

import time
import lgpio

TRIG = 27  # GPIO27 (physical pin 13)
ECHO = 22  # GPIO22 (physical pin 15)

print("🌡️ Ultrasonic Wiring Debug Start")

# Try to open the GPIO chip
try:
    h = lgpio.gpiochip_open(0)
except Exception as e:
    print(f"❌ Failed to open GPIO chip: {e}")
    exit(1)

try:
    # Setup GPIO modes
    lgpio.gpio_claim_output(h, TRIG, 0)
    lgpio.gpio_claim_input(h, ECHO)

    time.sleep(0.05)  # Allow line to settle
    idle = lgpio.gpio_read(h, ECHO)
    print(f"🔍 ECHO pin idle state: {'HIGH (1)' if idle else 'LOW (0)'}")

    # Fire 10µs ultrasonic pulse
    print("⚡ Triggering ultrasonic pulse...")
    lgpio.gpio_write(h, TRIG, 1)
    time.sleep(10e-6)
    lgpio.gpio_write(h, TRIG, 0)

    print("⏱️ Watching ECHO pin for 100ms...")
    start_ns = time.monotonic_ns()
    changes = []
    last_state = idle

    while (time.monotonic_ns() - start_ns) < 100_000_000:  # 100ms in ns
        current_state = lgpio.gpio_read(h, ECHO)
        if current_state != last_state:
            delta_ms = (time.monotonic_ns() - start_ns) / 1e6
            changes.append((delta_ms, current_state))
            last_state = current_state

    if changes:
        print("✅ ECHO changed during pulse:")
        for t_ms, val in changes:
            print(f"  -> {round(t_ms, 2)} ms: {'HIGH' if val else 'LOW'}")
    else:
        print("❌ ECHO pin did not change. Sensor may be disconnected, reversed, or faulty.")

finally:
    lgpio.gpiochip_close(h)
    print("🔚 Done.")
