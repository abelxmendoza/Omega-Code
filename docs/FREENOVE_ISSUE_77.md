# Freenove issue #77: PCA9685 not detected

Issue: https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/issues/77

## Evidence from the report

- Hardware: Raspberry Pi Zero 2 W and Freenove FNK43A.
- `test.py Motor` fails in `pca9685.py` while writing MODE1 at I2C address `0x40`.
- The reported `i2cdetect -y 1` scan shows no responding devices.
- A Freenove maintainer recommended checking connector seating and whether the car battery is installed and switched on.

## Diagnosis

The failure occurs **before any motor movement command**. No I2C acknowledgement is observed, so changing motor duty cycles or PWM channel mappings is not a demonstrated fix. The root cause remains unverified: possible explanations include controller power, wiring, connector seating, address, bus selection, or faulty hardware.

## Read-only triage

1. Confirm the Freenove board's supported power requirements, battery installation, and switch state using the official kit instructions. Do not probe live circuitry unless qualified.
2. Check that I2C is enabled and `/dev/i2c-1` exists.
3. Run `i2cdetect -y 1` and check for a device at `0x40` (or the actual configured address).
4. Run `python3 tools/freenove_diagnostics.py --bus 1 --address 0x40`. This only reads two registers.
5. If no device acknowledges, investigate hardware connectivity/power rather than running motor motion tests.

## Limitations

Our mocked I2C tests verify **software behavior only**. They do not reproduce a Raspberry Pi Zero 2 W, Freenove FNK43A electrical interface, or actual motor behavior. Do not claim issue #77 is fixed. The diagnostic is useful only when paired with clear instructions and accurate error reporting.

## Upstream contribution scope

Propose a short troubleshooting section or a small optional read-only diagnostic, subject to Freenove maintainer interest. Avoid changing Freenove's PWM driver without evidence of a software defect.
