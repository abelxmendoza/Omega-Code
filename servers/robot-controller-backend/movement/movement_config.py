"""
Movement V2 - Configuration Module

Centralized configuration for Movement V2 system.
"""

import os
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict


@dataclass
class MovementV2Config:
    """Movement V2 configuration"""
    # Ramping
    accel_rate: float = 150.0
    decel_rate: float = 200.0
    ramp_type: str = "linear"  # linear, exponential, s_curve
    
    # Profiles
    default_profile: str = "smooth"  # smooth, aggressive, precision
    
    # Watchdog
    watchdog_timeout_sec: float = 2.0
    watchdog_enabled: bool = True
    
    # Thermal Safety
    thermal_max_temp: float = 75.0
    thermal_warning_temp: float = 60.0
    thermal_max_current: float = 2.5
    thermal_warning_current: float = 2.0
    thermal_cooldown_temp: float = 50.0
    thermal_throttle_factor: float = 0.5
    thermal_enabled: bool = True
    
    # PID
    pid_enabled: bool = True
    pid_kp: float = 0.3
    pid_ki: float = 0.05
    pid_kd: float = 0.01
    pid_kf: float = 0.0
    
    # Odometry
    odometry_enabled: bool = False
    wheel_base: float = 0.2  # meters
    wheel_radius: float = 0.05  # meters
    
    # Speed limits
    max_speed: int = 4095
    min_speed: int = 0
    
    # Update rates
    ramp_update_rate_hz: float = 50.0  # Ramp update frequency
    telemetry_update_rate_hz: float = 10.0  # Telemetry update frequency


def load_config() -> MovementV2Config:
    """
    Load Movement V2 configuration from environment variables or defaults.
    
    Returns:
        MovementV2Config instance
    """
    def _env_float(key: str, default: float) -> float:
        val = os.getenv(key)
        return float(val) if val else default
    
    def _env_int(key: str, default: int) -> int:
        val = os.getenv(key)
        return int(val) if val else default
    
    def _env_bool(key: str, default: bool) -> bool:
        val = os.getenv(key)
        if val is None:
            return default
        return val.lower() in ('1', 'true', 'yes', 'on', 'enable', 'enabled')
    
    return MovementV2Config(
        # Ramping
        accel_rate=_env_float("MOVEMENT_ACCEL_RATE", 150.0),
        decel_rate=_env_float("MOVEMENT_DECEL_RATE", 200.0),
        ramp_type=os.getenv("MOVEMENT_RAMP_TYPE", "linear"),
        
        # Profiles
        default_profile=os.getenv("MOVEMENT_DEFAULT_PROFILE", "smooth"),
        
        # Watchdog
        watchdog_timeout_sec=_env_float("MOVEMENT_WATCHDOG_TIMEOUT", 2.0),
        watchdog_enabled=_env_bool("MOVEMENT_WATCHDOG_ENABLED", True),
        
        # Thermal Safety
        thermal_max_temp=_env_float("MOVEMENT_THERMAL_MAX_TEMP", 75.0),
        thermal_warning_temp=_env_float("MOVEMENT_THERMAL_WARNING_TEMP", 60.0),
        thermal_max_current=_env_float("MOVEMENT_THERMAL_MAX_CURRENT", 2.5),
        thermal_warning_current=_env_float("MOVEMENT_THERMAL_WARNING_CURRENT", 2.0),
        thermal_cooldown_temp=_env_float("MOVEMENT_THERMAL_COOLDOWN_TEMP", 50.0),
        thermal_throttle_factor=_env_float("MOVEMENT_THERMAL_THROTTLE_FACTOR", 0.5),
        thermal_enabled=_env_bool("MOVEMENT_THERMAL_ENABLED", True),
        
        # PID
        pid_enabled=_env_bool("MOVEMENT_PID_ENABLED", True),
        pid_kp=_env_float("MOVEMENT_PID_KP", 0.3),
        pid_ki=_env_float("MOVEMENT_PID_KI", 0.05),
        pid_kd=_env_float("MOVEMENT_PID_KD", 0.01),
        pid_kf=_env_float("MOVEMENT_PID_KF", 0.0),
        
        # Odometry
        odometry_enabled=_env_bool("MOVEMENT_ODOMETRY_ENABLED", False),
        wheel_base=_env_float("MOVEMENT_WHEEL_BASE", 0.2),
        wheel_radius=_env_float("MOVEMENT_WHEEL_RADIUS", 0.05),
        
        # Speed limits
        max_speed=_env_int("MOVEMENT_MAX_SPEED", 4095),
        min_speed=_env_int("MOVEMENT_MIN_SPEED", 0),
        
        # Update rates
        ramp_update_rate_hz=_env_float("MOVEMENT_RAMP_UPDATE_RATE", 50.0),
        telemetry_update_rate_hz=_env_float("MOVEMENT_TELEMETRY_UPDATE_RATE", 10.0),
    )


# Global config instance
_config: Optional[MovementV2Config] = None


def get_config() -> MovementV2Config:
    """Get Movement V2 configuration (singleton)"""
    global _config
    if _config is None:
        _config = load_config()
    return _config


def reload_config() -> MovementV2Config:
    """Reload configuration from environment"""
    global _config
    _config = load_config()
    return _config


def get(section: str, key: str, default: Any = None) -> Any:
    """
    Get configuration value by section and key.
    
    Args:
        section: Configuration section (e.g., "ramp", "pid", "thermal")
        key: Configuration key within section
        default: Default value if not found
    
    Returns:
        Configuration value or default
    """
    config = get_config()
    
    # Map sections to config attributes
    section_map = {
        "ramp": {
            "accel_rate": config.accel_rate,
            "decel_rate": config.decel_rate,
            "ramp_type": config.ramp_type,
            "update_hz": config.ramp_update_rate_hz,
        },
        "pid": {
            "enabled": config.pid_enabled,
            "kp": config.pid_kp,
            "ki": config.pid_ki,
            "kd": config.pid_kd,
            "kf": config.pid_kf,
        },
        "profiles": {
            "default": config.default_profile,
        },
        "thermal": {
            "enabled": config.thermal_enabled,
            "max_temp": config.thermal_max_temp,
            "warning_temp": config.thermal_warning_temp,
            "max_current": config.thermal_max_current,
            "warning_current": config.thermal_warning_current,
            "cooldown_temp": config.thermal_cooldown_temp,
            "throttle_factor": config.thermal_throttle_factor,
        },
        "watchdog": {
            "enabled": config.watchdog_enabled,
            "timeout_sec": config.watchdog_timeout_sec,
        },
        "odometry": {
            "enabled": config.odometry_enabled,
            "wheel_base": config.wheel_base,
            "wheel_radius": config.wheel_radius,
        },
        "limits": {
            "max_speed": config.max_speed,
            "min_speed": config.min_speed,
        },
    }
    
    if section in section_map and key in section_map[section]:
        return section_map[section][key]
    
    return default


def save(config: MovementV2Config, filepath: Optional[str] = None) -> None:
    """
    Save configuration to file (JSON format).
    
    Args:
        config: Configuration to save
        filepath: Path to save file (default: movement_config.json in movement dir)
    """
    import json
    from pathlib import Path
    
    if filepath is None:
        filepath = Path(__file__).parent / "movement_config.json"
    
    config_dict = asdict(config)
    with open(filepath, 'w') as f:
        json.dump(config_dict, f, indent=2)


def update_from_profile(profile_object: Any) -> None:
    """
    Update configuration from a movement profile.
    
    Args:
        profile_object: MovementProfile instance with get_config() method
    """
    global _config
    config = get_config()
    
    if hasattr(profile_object, 'get_config'):
        profile_config = profile_object.get_config()
        
        # Update ramp rates from profile
        if 'accel_rate' in profile_config:
            config.accel_rate = profile_config['accel_rate']
        if 'decel_rate' in profile_config:
            config.decel_rate = profile_config['decel_rate']
        if 'ramp_type' in profile_config:
            config.ramp_type = profile_config['ramp_type']
        if 'max_speed' in profile_config:
            config.max_speed = int(profile_config['max_speed'])
        
        _config = config


__all__ = [
    "MovementV2Config",
    "load_config",
    "get_config",
    "reload_config",
    "get",
    "save",
    "update_from_profile"
]

