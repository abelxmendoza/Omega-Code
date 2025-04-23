#!/usr/bin/env python3
# File: sensors/debug_ultrasonic_wiring.py

import time
import lgpio

TRIG = 27  # GPIO27 (physical pin 13)
ECHO = 22  # GPIO22 (physical pin 15)

h = lgpio.gpiochip_open(0)

try:
    print("🌡️ Ultrasonic Wiring Debug Start")

    # Setup pins
    lgpio.gpio_claim_output(h, TRIG, 0)
    lgpio.gpio_claim_input(h, ECHO)

    # Check idle state of ECHO
    idle = lgpio.gpio_read(h, ECHO)
    print(f"🔍 ECHO pin idle state: {'HIGH (1)' if idle else 'LOW (0)'}")

    print("⚡ Triggering ultrasonic pulse...")
    lgpio.gpio_write(h, TRIG, 1)
    time.sleep(10e-6)
    lgpio.gpio_write(h, TRIG, 0)

    # Watch ECHO for 100 ms
    print("⏱️ Watching ECHO pin for 100ms...")
    changes = []
    last_state = idle
    start = time.time()

    while time.time() - start < 0.1:
        state = lgpio.gpio_read(h, ECHO)
        if state != last_state:
            changes.append((time.time(), state))
            last_state = state

    if changes:
        print("✅ ECHO changed during pulse:")
        for t, val in changes:
            print(f"  -> {round((t - start)*1000, 2)} ms: {'HIGH' if val else 'LOW'}")
    else:
        print("❌ ECHO pin did not change (stuck?). Possible bad wiring.")

finally:
    lgpio.gpiochip_close(h)
    print("🔚 Done.")
