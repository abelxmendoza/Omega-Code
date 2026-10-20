# Freenove-compatible hardware diagnostics

This is an **experimental, read-only** diagnostic for a Raspberry Pi connected to a PCA9685 PWM controller. It is intended as a small, reviewable candidate for an upstream Freenove contribution, not an official Freenove utility.

## Run without hardware

```bash
python3 tools/freenove_diagnostics.py --mock
python3 -m unittest discover -s tests -p 'test_freenove_diagnostics.py'
```

## Run on a Raspberry Pi

Enable I2C and install `python3-smbus` or `smbus2`. With the robot secured and motors disabled:

```bash
python3 tools/freenove_diagnostics.py --bus 1 --address 0x40
```

The utility **only reads** MODE1 and PRESCALE registers. It does not move wheels, command servos, or alter PWM configuration. A successful read only proves that the controller responded on I2C; it cannot establish that motors, motor power, battery, or wiring are functional.

## Upstream plan

1. Validate on Omega1's Raspberry Pi 4 and confirm actual PCA9685 address.
2. Compare expected behavior with the Freenove kit and its PCB revision.
3. Discuss the utility with Freenove maintainers before proposing a focused PR.
4. Keep the advanced ROS 2 / FastAPI stack in Omega-Code.

## License review

Do not copy Freenove source files into this utility without checking attribution, license compatibility, and upstream acceptance. This initial implementation uses register-level PCA9685 behavior and standard SMBus interfaces; it is not a transplant of the Freenove driver.
