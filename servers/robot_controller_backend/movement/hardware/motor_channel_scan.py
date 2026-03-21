#!/usr/bin/env python3
"""
Motor Channel Scanner — identifies which PCA9685 channel pair drives each motor.

Run this directly on the Pi to find the correct channel assignments:
    python3 motor_channel_scan.py

The script steps through each channel pair (0/1, 2/3, 4/5, 6/7) and asks you
to identify which physical motor spins.  Record the results and update the
channel constants in motor_driver_pi.py accordingly.
"""

import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../../..'))

from pca9685_real import PCA9685

DUTY    = 2000   # ~49% — enough to overcome static friction
HOLD_S  = 1.5    # how long to hold each state

# Channel pairs to test: (ch_a, ch_b, label)
PAIRS = [
    (0, 1, "pair 0/1  (expected: left_upper  / front-left)"),
    (2, 3, "pair 2/3  (expected: left_lower  / rear-left)"),
    (4, 5, "pair 4/5  (expected: right_lower / rear-right)"),
    (6, 7, "pair 6/7  (expected: right_upper / front-right)"),
]


def stop_all(pca: PCA9685) -> None:
    """Brake all channels: both sides HIGH = H-bridge brake."""
    for ch in range(8):
        pca.set_motor_pwm(ch, 4095)


def drive_pair(pca: PCA9685, ch_a: int, ch_b: int, duty: int) -> None:
    """
    Drive a single H-bridge pair.
      duty > 0: ch_a = 0, ch_b = PWM  (one direction)
      duty < 0: ch_a = PWM, ch_b = 0  (other direction)
    """
    if duty > 0:
        pca.set_motor_pwm(ch_a, 0)
        pca.set_motor_pwm(ch_b, abs(duty))
    else:
        pca.set_motor_pwm(ch_b, 0)
        pca.set_motor_pwm(ch_a, abs(duty))


def pause(msg: str) -> None:
    input(f"\n  {msg}\n  Press ENTER to continue...")


def main() -> None:
    pca = PCA9685()
    pca.set_pwm_freq(50)

    print("\n=== Omega-1 Motor Channel Scanner ===")
    print("Place the robot on a stand so wheels can spin freely.")
    pause("Ready to begin?")

    results: dict[str, str] = {}

    for ch_a, ch_b, label in PAIRS:
        print(f"\n--- Testing {label} ---")

        # Direction A: ch_b active
        print(f"  Direction A: ch{ch_a}=0, ch{ch_b}=PWM  ({HOLD_S}s) ...")
        stop_all(pca)
        time.sleep(0.1)
        drive_pair(pca, ch_a, ch_b, DUTY)
        time.sleep(HOLD_S)
        stop_all(pca)

        time.sleep(0.5)

        # Direction B: ch_a active
        print(f"  Direction B: ch{ch_b}=0, ch{ch_a}=PWM  ({HOLD_S}s) ...")
        drive_pair(pca, ch_a, ch_b, -DUTY)
        time.sleep(HOLD_S)
        stop_all(pca)

        motor = input(
            f"\n  Which motor moved? (e.g. 'front-left', 'rear-right', 'none'): "
        ).strip()
        results[label] = motor
        time.sleep(0.5)

    # Summary
    print("\n=== Results ===")
    for label, motor in results.items():
        print(f"  {label}  ->  {motor}")

    print("\nUpdate the channel assignments in motor_driver_pi.py to match.")
    print("Each wheel method should have:  A channel (zero side)  and  B channel (PWM side)")

    stop_all(pca)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
