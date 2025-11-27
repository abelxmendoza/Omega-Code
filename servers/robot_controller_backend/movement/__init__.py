"""
Movement V2 - Omega Robotics Movement System

This package provides the Movement V2 system with smooth ramping, PID control,
thermal safety, watchdog, profiles, and odometry support.
"""

from .movement_ramp import MovementRamp, RampType
from .movement_pid import SpeedPID, StraightDrivePID, ServoSmoothPID, PIDTuning, PID_AVAILABLE
from .movement_watchdog import MovementWatchdog, WatchdogState
from .movement_profiles import (
    MovementProfile,
    SmoothProfile,
    AggressiveProfile,
    PrecisionProfile,
    ProfileManager,
    ProfileType,
    ProfileConfig
)
from .thermal_safety import ThermalSafety, SafetyState, ThermalLimits
from .odometry import Odometry, Pose
from .movement_config import (
    MovementV2Config,
    load_config,
    get_config,
    reload_config,
    get,
    save,
    update_from_profile
)

__all__ = [
    # Ramp
    "MovementRamp",
    "RampType",
    
    # PID
    "SpeedPID",
    "StraightDrivePID",
    "ServoSmoothPID",
    "PIDTuning",
    "PID_AVAILABLE",
    
    # Watchdog
    "MovementWatchdog",
    "WatchdogState",
    
    # Profiles
    "MovementProfile",
    "SmoothProfile",
    "AggressiveProfile",
    "PrecisionProfile",
    "ProfileManager",
    "ProfileType",
    "ProfileConfig",
    
    # Thermal Safety
    "ThermalSafety",
    "SafetyState",
    "ThermalLimits",
    
    # Odometry
    "Odometry",
    "Pose",
    
    # Config
    "MovementV2Config",
    "load_config",
    "get_config",
    "reload_config",
    "get",
    "save",
    "update_from_profile",
]

__version__ = "2.0.0"
