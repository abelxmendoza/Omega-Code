"""
Hardware Abstraction Layer

Multi-platform motor driver abstraction with automatic platform detection.
"""

from .base_motor_driver import BaseMotorDriver
from .hardware_factory import get_motor_driver
from .motor_driver_pi import PiMotorDriver
from .motor_driver_orin import OrinMotorDriver
from .motor_driver_mac import MacMotorDriver
from .motor_driver_linux import LinuxMotorDriver
from .motor_driver_sim import SimMotorDriver
from .motor_driver_noop import NoopMotorDriver
from .pca9685_real import PCA9685 as PCA9685Real
from .pca9685_mock import PCA9685 as PCA9685Mock

__all__ = [
    "BaseMotorDriver",
    "get_motor_driver",
    "PiMotorDriver",
    "OrinMotorDriver",
    "MacMotorDriver",
    "LinuxMotorDriver",
    "SimMotorDriver",
    "NoopMotorDriver",
    "PCA9685Real",
    "PCA9685Mock",
]

