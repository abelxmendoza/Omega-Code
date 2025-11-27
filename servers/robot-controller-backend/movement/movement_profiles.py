"""
Movement V2 - Movement Profiles Module

Defines different movement styles: smooth, aggressive, precision.
Each profile adjusts acceleration rates, speed limits, and turning behavior.
"""

import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class ProfileType(Enum):
    """Movement profile types"""
    SMOOTH = "smooth"
    AGGRESSIVE = "aggressive"
    PRECISION = "precision"
    CUSTOM = "custom"


@dataclass
class ProfileConfig:
    """Configuration for a movement profile"""
    name: str
    accel_rate: float = 150.0      # PWM units per second
    decel_rate: float = 200.0      # PWM units per second
    max_speed: float = 4095.0      # Maximum PWM
    turn_ratio: float = 0.5         # Turn speed ratio (0.0-1.0)
    pivot_enabled: bool = True      # Allow pivot turns
    ramp_type: str = "linear"       # linear/exponential/s_curve


class MovementProfile:
    """Optimized movement profile with __slots__ for memory efficiency"""
    __slots__ = ('config', 'name')
    
    def __init__(self, config: ProfileConfig):
        """Initialize movement profile"""
        self.config = config
        self.name = config.name
    
    def transform_speed(self, pwm: float) -> float:
        """
        Transform PWM speed based on profile limits.
        
        Args:
            pwm: Input PWM value
        
        Returns:
            Transformed PWM value
        """
        return max(0, min(self.config.max_speed, float(pwm)))
    
    def get_accel_rate(self) -> float:
        """Get acceleration rate for this profile"""
        return self.config.accel_rate
    
    def get_decel_rate(self) -> float:
        """Get deceleration rate for this profile"""
        return self.config.decel_rate
    
    def get_turn_ratio(self) -> float:
        """Get turn speed ratio for this profile"""
        return self.config.turn_ratio
    
    def can_pivot(self) -> bool:
        """Check if pivot turns are allowed"""
        return self.config.pivot_enabled
    
    def get_ramp_type(self) -> str:
        """Get ramp type for this profile"""
        return self.config.ramp_type
    
    def get_config(self) -> Dict[str, Any]:
        """Get profile configuration as dict"""
        return {
            "name": self.config.name,
            "accel_rate": self.config.accel_rate,
            "decel_rate": self.config.decel_rate,
            "max_speed": self.config.max_speed,
            "turn_ratio": self.config.turn_ratio,
            "pivot_enabled": self.config.pivot_enabled,
            "ramp_type": self.config.ramp_type
        }
    
    def apply_to_config(self, movement_config: Any) -> None:
        """
        Apply profile settings to a MovementV2Config object.
        
        Args:
            movement_config: MovementV2Config instance to update
        """
        if hasattr(movement_config, 'accel_rate'):
            movement_config.accel_rate = self.config.accel_rate
        if hasattr(movement_config, 'decel_rate'):
            movement_config.decel_rate = self.config.decel_rate
        if hasattr(movement_config, 'ramp_type'):
            movement_config.ramp_type = self.config.ramp_type
        if hasattr(movement_config, 'max_speed'):
            movement_config.max_speed = int(self.config.max_speed)


class SmoothProfile(MovementProfile):
    """
    Smooth movement profile: gentle acceleration and soft steering.
    
    Best for:
    - Indoor navigation
    - Smooth video recording
    - Precise positioning
    """
    
    def __init__(self):
        config = ProfileConfig(
            name="smooth",
            accel_rate=100.0,      # Slow acceleration
            decel_rate=150.0,      # Moderate deceleration
            max_speed=3000.0,      # Moderate max speed
            turn_ratio=0.6,        # Gentle turns
            pivot_enabled=False,   # No sharp pivots
            ramp_type="s_curve"    # Smooth S-curve ramping
        )
        super().__init__(config)


class AggressiveProfile(MovementProfile):
    """
    Aggressive movement profile: fast acceleration and sharp turns.
    
    Best for:
    - Outdoor navigation
    - Fast traversal
    - Quick maneuvers
    """
    
    def __init__(self):
        config = ProfileConfig(
            name="aggressive",
            accel_rate=300.0,      # Fast acceleration
            decel_rate=400.0,      # Fast deceleration
            max_speed=4095.0,      # Full speed
            turn_ratio=0.3,        # Sharp turns
            pivot_enabled=True,    # Allow pivots
            ramp_type="exponential"  # Fast exponential ramping
        )
        super().__init__(config)
    
    def transform_speed(self, pwm: float) -> float:
        """Aggressive profile allows full speed range"""
        return max(0, min(self.config.max_speed, float(pwm)))


class PrecisionProfile(MovementProfile):
    """
    Precision movement profile: very slow and controlled.
    
    Best for:
    - Tight spaces
    - Fine positioning
    - Obstacle avoidance
    """
    
    def __init__(self):
        config = ProfileConfig(
            name="precision",
            accel_rate=50.0,       # Very slow acceleration
            decel_rate=75.0,       # Very slow deceleration
            max_speed=1500.0,      # Low max speed
            turn_ratio=0.8,        # Very gentle turns
            pivot_enabled=False,   # No pivots
            ramp_type="linear"     # Linear ramping for predictability
        )
        super().__init__(config)
    
    def transform_speed(self, pwm: float) -> float:
        """Precision profile limits speed more aggressively"""
        # Scale down input PWM to precision range
        scaled = float(pwm) * (self.config.max_speed / 4095.0)
        return max(0, min(self.config.max_speed, scaled))


class ProfileManager:
    """Optimized profile manager with __slots__ for memory efficiency"""
    __slots__ = ('profiles', 'current_profile')
    
    def __init__(self, default_profile: ProfileType = ProfileType.SMOOTH):
        """Initialize profile manager"""
        self.profiles: Dict[str, MovementProfile] = {
            ProfileType.SMOOTH.value: SmoothProfile(),
            ProfileType.AGGRESSIVE.value: AggressiveProfile(),
            ProfileType.PRECISION.value: PrecisionProfile()
        }
        self.current_profile = self.profiles[default_profile.value]
        logger.info(f"Profile manager initialized with default: {default_profile.value}")
    
    def set_profile(self, profile_type: ProfileType) -> MovementProfile:
        """
        Switch to a different movement profile.
        
        Args:
            profile_type: Profile type to switch to
        
        Returns:
            The new active profile
        """
        if profile_type == ProfileType.CUSTOM:
            logger.warning("Cannot switch to CUSTOM profile without configuration")
            return self.current_profile
        
        profile_name = profile_type.value
        if profile_name in self.profiles:
            self.current_profile = self.profiles[profile_name]
            logger.info(f"Switched to profile: {profile_name}")
            return self.current_profile
        else:
            logger.warning(f"Unknown profile: {profile_name}")
            return self.current_profile
    
    def get_profile(self, profile_type: Optional[ProfileType] = None) -> MovementProfile:
        """
        Get movement profile.
        
        Args:
            profile_type: Profile type (None = current profile)
        
        Returns:
            Movement profile
        """
        if profile_type is None:
            return self.current_profile
        
        profile_name = profile_type.value
        return self.profiles.get(profile_name, self.current_profile)
    
    def get_current_profile(self) -> MovementProfile:
        """Get current active profile"""
        return self.current_profile
    
    def add_custom_profile(self, name: str, config: ProfileConfig) -> None:
        """Add a custom movement profile"""
        self.profiles[name] = MovementProfile(config)
        logger.info(f"Added custom profile: {name}")
    
    def list_profiles(self) -> list[str]:
        """List available profile names"""
        return list(self.profiles.keys())


__all__ = [
    "MovementProfile",
    "SmoothProfile",
    "AggressiveProfile",
    "PrecisionProfile",
    "ProfileManager",
    "ProfileType",
    "ProfileConfig"
]

