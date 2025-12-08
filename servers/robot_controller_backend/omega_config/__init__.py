"""
OmegaOS Configuration Module

Provides unified configuration management for the entire robot system.
"""

from .config_manager import ConfigManager, get_config_manager, RobotConfig

__all__ = [
    'ConfigManager',
    'get_config_manager',
    'RobotConfig',
]

