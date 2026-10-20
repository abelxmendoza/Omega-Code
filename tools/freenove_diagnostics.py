#!/usr/bin/env python3
"""Read-only PCA9685 diagnostics for Freenove-compatible Raspberry Pi robots.

No PWM writes, motor movements, or controller initialization are performed.
"""
import argparse
import json
from pathlib import Path

DEFAULT_ADDRESS = 0x40
MODE1 = 0x00
PRESCALE = 0xFE

def inspect_controller(bus, address=DEFAULT_ADDRESS):
    """Return diagnostic data using only I2C register reads."""
    if not 0x03 <= address <= 0x77:
        raise ValueError("I2C address must be between 0x03 and 0x77")
    mode = bus.read_byte_data(address, MODE1)
    prescale = bus.read_byte_data(address, PRESCALE)
    return {
        "status": "reachable",
        "address": f"0x{address:02x}",
        "mode1": f"0x{mode:02x}",
        "prescale": prescale,
        "sleeping": bool(mode & 0x10),
        "estimated_pwm_hz": round(25_000_000 / (4096 * (prescale + 1)), 2),
        "note": "I2C reachability does not verify motor power, wiring, or wheel movement.",
    }

def diagnose(bus_factory, address=DEFAULT_ADDRESS, bus_number=1):
    try:
        bus = bus_factory(bus_number)
    except (OSError, PermissionError, FileNotFoundError) as exc:
        return {"status": "error", "stage": "open_bus", "detail": str(exc),
                "suggestion": "Enable I2C, check /dev/i2c-* and user permissions."}
    try:
        try:
            return inspect_controller(bus, address)
        except (OSError, IOError) as exc:
            return {"status": "error", "stage": "read_controller", "detail": str(exc),
                    "address": f"0x{address:02x}",
                    "suggestion": "Check board power, I2C wiring, address and connector seating."}
    finally:
        close = getattr(bus, "close", None)
        if callable(close):
            close()

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=DEFAULT_ADDRESS)
    parser.add_argument("--mock", action="store_true", help="Run without hardware using a read-only mock")
    args = parser.parse_args(argv)
    if args.mock:
        class MockBus:
            def __init__(self, number):
                self.number = number
            def read_byte_data(self, address, register):
                return {MODE1: 0x01, PRESCALE: 121}[register]
            def close(self):
                pass
        factory = MockBus
    else:
        try:
            import smbus2
        except ImportError:
            try:
                import smbus as smbus2
            except ImportError:
                print(json.dumps({"status": "error", "stage": "dependency",
                                  "suggestion": "Install python3-smbus or smbus2."}, indent=2))
                return 2
        factory = smbus2.SMBus
    try:
        result = diagnose(factory, args.address, args.bus)
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "reachable" else 2

if __name__ == "__main__":
    raise SystemExit(main())
