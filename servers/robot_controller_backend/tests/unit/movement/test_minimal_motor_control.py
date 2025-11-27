import importlib
import os
import sys
import types

import pytest

BACKEND_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
if BACKEND_ROOT not in sys.path:
    sys.path.insert(0, BACKEND_ROOT)


@pytest.fixture()
def motor_module(monkeypatch):
    instances = []

    class FakePCA9685:
        def __init__(self, *args, **kwargs):
            self.calls = []
            self.freq = None
            instances.append(self)

        def setPWMFreq(self, freq):
            self.freq = freq

        def setMotorPwm(self, channel, duty):
            self.calls.append((channel, duty))

    fake_module = types.ModuleType("PCA9685")
    fake_module.PCA9685 = FakePCA9685
    monkeypatch.setitem(sys.modules, "PCA9685", fake_module)
    sys.modules.pop("movement.minimal_motor_control", None)
    module = importlib.import_module("movement.minimal_motor_control")
    return module, instances


def test_forward_uses_trim_offsets(motor_module):
    module, instances = motor_module
    motor = module.Motor()
    pwm = instances[0]

    motor.set_trim(left=100, right=200)
    motor.forward(1000)

    # Forward motion should bias left/right channels according to trim
    assert pwm.calls == [
        (0, 1100), (1, 0),
        (4, 1100), (5, 0),
        (2, 1200), (3, 0),
        (6, 1200), (7, 0),
    ]


def test_backward_respects_trim_and_clamps(motor_module):
    module, instances = motor_module
    motor = module.Motor()
    pwm = instances[0]

    motor.set_trim(left=5000, right=50)  # left should clamp to MAX_PWM
    assert motor.get_trim()["left"] == motor.MAX_PWM

    motor.backward(1000)
    assert pwm.calls[-8:] == [
        (0, 0), (1, motor.MAX_PWM),
        (4, 0), (5, motor.MAX_PWM),
        (2, 0), (3, 1050),
        (6, 0), (7, 1050),
    ]


def test_stop_command_zeroes_channels(motor_module):
    module, instances = motor_module
    motor = module.Motor()
    pwm = instances[0]

    motor.forward(800)
    motor.stop()

    # Last 8 calls correspond to stop command
    assert pwm.calls[-8:] == [
        (0, 0), (1, 0),
        (4, 0), (5, 0),
        (2, 0), (3, 0),
        (6, 0), (7, 0),
    ]
