"""
OmegaOS Configuration Manager

Unified configuration management for the entire robot system.
- Loads configuration from YAML files
- Validates configuration
- Saves persistent state
- Provides API for config access
"""

import os
import yaml
import json
import logging
from pathlib import Path
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, asdict
from datetime import datetime

log = logging.getLogger(__name__)

# Configuration directory
CONFIG_DIR = Path(__file__).parent
CONFIG_FILE = CONFIG_DIR / "config.yaml"
PROFILE_FILE = CONFIG_DIR / "robot_profile.yaml"
HARDWARE_FILE = CONFIG_DIR / "hardware_map.yaml"
STATE_FILE = CONFIG_DIR / "persistent_state.json"

# Backup directory
BACKUP_DIR = CONFIG_DIR / "backups"
BACKUP_DIR.mkdir(exist_ok=True)


@dataclass
class RobotConfig:
    """Main robot configuration"""
    robot: Dict[str, Any]
    network: Dict[str, Any]
    services: Dict[str, Any]
    camera: Dict[str, Any]
    movement: Dict[str, Any]
    lighting: Dict[str, Any]
    logging: Dict[str, Any]
    security: Dict[str, Any]
    telemetry: Dict[str, Any]


class ConfigManager:
    """Manages robot-wide configuration"""
    
    def __init__(self):
        self.config: Optional[RobotConfig] = None
        self.profile: Optional[Dict[str, Any]] = None
        self.hardware: Optional[Dict[str, Any]] = None
        self.state: Optional[Dict[str, Any]] = None
        self._load_all()
    
    def _load_yaml(self, file_path: Path) -> Optional[Dict[str, Any]]:
        """Load YAML file safely"""
        try:
            if not file_path.exists():
                log.warning(f"Config file not found: {file_path}")
                return None
            
            with open(file_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            log.error(f"Failed to load {file_path}: {e}")
            return None
    
    def _save_yaml(self, file_path: Path, data: Dict[str, Any]) -> bool:
        """Save YAML file safely"""
        try:
            with open(file_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            return True
        except Exception as e:
            log.error(f"Failed to save {file_path}: {e}")
            return False
    
    def _load_json(self, file_path: Path) -> Optional[Dict[str, Any]]:
        """Load JSON file safely"""
        try:
            if not file_path.exists():
                return {}
            
            with open(file_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            log.error(f"Failed to load {file_path}: {e}")
            return {}
    
    def _save_json(self, file_path: Path, data: Dict[str, Any]) -> bool:
        """Save JSON file safely"""
        try:
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=2)
            return True
        except Exception as e:
            log.error(f"Failed to save {file_path}: {e}")
            return False
    
    def _load_all(self):
        """Load all configuration files"""
        # Load main config
        config_data = self._load_yaml(CONFIG_FILE)
        if config_data:
            self.config = RobotConfig(**config_data)
        else:
            log.error("Failed to load main config, using defaults")
            self.config = self._get_default_config()
        
        # Load profile
        self.profile = self._load_yaml(PROFILE_FILE)
        
        # Load hardware map
        self.hardware = self._load_yaml(HARDWARE_FILE)
        
        # Load persistent state
        self.state = self._load_json(STATE_FILE)
    
    def _get_default_config(self) -> RobotConfig:
        """Get default configuration"""
        return RobotConfig(
            robot={"name": "Omega-1", "profile": "pi4b", "version": "1.0"},
            network={"default_mode": "ap"},
            services={"autostart": [], "restart_policies": {}},
            camera={"backend": "picamera2", "width": 640, "height": 480, "fps": 30},
            movement={"default_profile": "smooth"},
            lighting={"default_pattern": "omega_signature", "brightness": 0.5},
            logging={"level": "INFO"},
            security={"api_auth_enabled": False, "allowed_origins": []},
            telemetry={"enabled": True, "update_interval_ms": 1000}
        )
    
    def get_config(self) -> Dict[str, Any]:
        """Get full configuration"""
        if self.config is None:
            return {}
        return asdict(self.config)
    
    def get_section(self, section: str) -> Optional[Dict[str, Any]]:
        """Get a specific configuration section"""
        if self.config is None:
            return None
        config_dict = asdict(self.config)
        return config_dict.get(section)
    
    def update_section(self, section: str, data: Dict[str, Any]) -> bool:
        """Update a configuration section"""
        if self.config is None:
            return False
        
        config_dict = asdict(self.config)
        if section not in config_dict:
            log.error(f"Unknown config section: {section}")
            return False
        
        # Sanitize sensitive data before updating
        sanitized_data = self._sanitize_sensitive_data(section, data)
        config_dict[section].update(sanitized_data)
        
        # Reconstruct config object
        self.config = RobotConfig(**config_dict)
        
        # Save to file
        return self.save_config()
    
    def _sanitize_sensitive_data(self, section: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Sanitize sensitive data before storing.
        Prevents logging of passwords/secrets.
        """
        sanitized = data.copy()
        
        # Mask passwords in network section
        if section == "network":
            if "password" in sanitized and sanitized["password"]:
                # Don't log the actual password
                log.info(f"Updating network password (masked)")
                # Keep the password value, just don't log it
        
        # Mask API keys in security section
        if section == "security":
            if "api_key" in sanitized and sanitized["api_key"]:
                log.info(f"Updating API key (masked)")
        
        return sanitized
    
    def save_config(self) -> bool:
        """Save configuration to file"""
        if self.config is None:
            return False
        
        # Create backup
        self._create_backup()
        
        # Convert to dict
        config_dict = asdict(self.config)
        
        # Save to YAML
        return self._save_yaml(CONFIG_FILE, config_dict)
    
    def _create_backup(self):
        """Create backup of current config"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_file = BACKUP_DIR / f"config_{timestamp}.yaml"
            
            if CONFIG_FILE.exists():
                import shutil
                shutil.copy2(CONFIG_FILE, backup_file)
                log.info(f"Created config backup: {backup_file}")
        except Exception as e:
            log.error(f"Failed to create backup: {e}")
    
    def get_profile(self, profile_name: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """Get robot profile"""
        if self.profile is None:
            return None
        
        if profile_name is None:
            profile_name = self.config.robot.get("profile", "pi4b") if self.config else "pi4b"
        
        profiles = self.profile.get("profiles", {})
        return profiles.get(profile_name)
    
    def get_hardware_map(self) -> Optional[Dict[str, Any]]:
        """Get hardware mapping"""
        return self.hardware
    
    def get_state(self) -> Dict[str, Any]:
        """Get persistent state"""
        return self.state or {}
    
    def update_state(self, key: str, value: Any) -> bool:
        """Update persistent state"""
        if self.state is None:
            self.state = {}
        
        self.state[key] = value
        self.state["last_updated"] = datetime.now().isoformat()
        
        return self._save_json(STATE_FILE, self.state)
    
    def validate_config(self) -> tuple[bool, List[str]]:
        """Validate configuration"""
        errors = []
        
        if self.config is None:
            errors.append("Configuration not loaded")
            return False, errors
        
        # Validate robot section
        if not self.config.robot.get("name"):
            errors.append("Robot name is required")
        
        if self.config.robot.get("profile") not in ["pi4b", "jetson", "dev"]:
            errors.append(f"Invalid robot profile: {self.config.robot.get('profile')}")
        
        # Validate network section
        if self.config.network.get("default_mode") not in ["ap", "client"]:
            errors.append("Network default_mode must be 'ap' or 'client'")
        
        # Validate camera section
        if self.config.camera.get("backend") not in ["picamera2", "v4l2", "auto", "mock"]:
            errors.append("Invalid camera backend")
        
        # Validate movement section
        if self.config.movement.get("default_profile") not in ["smooth", "aggressive", "precision"]:
            errors.append("Invalid movement profile")
        
        return len(errors) == 0, errors
    
    def export_config(self) -> Dict[str, Any]:
        """Export full configuration as JSON"""
        return {
            "config": self.get_config(),
            "profile": self.profile,
            "hardware": self.hardware,
            "state": self.state,
            "version": "1.0",
            "exported_at": datetime.now().isoformat()
        }
    
    def import_config(self, data: Dict[str, Any]) -> tuple[bool, List[str]]:
        """Import configuration from JSON"""
        errors = []
        
        try:
            # Validate structure
            if "config" not in data:
                errors.append("Missing 'config' section")
                return False, errors
            
            # Create backup before import
            self._create_backup()
            
            # Update config
            config_dict = data["config"]
            self.config = RobotConfig(**config_dict)
            
            # Update profile if provided
            if "profile" in data:
                self.profile = data["profile"]
                self._save_yaml(PROFILE_FILE, self.profile)
            
            # Update hardware if provided
            if "hardware" in data:
                self.hardware = data["hardware"]
                self._save_yaml(HARDWARE_FILE, self.hardware)
            
            # Update state if provided
            if "state" in data:
                self.state = data["state"]
                self._save_json(STATE_FILE, self.state)
            
            # Save main config
            if not self.save_config():
                errors.append("Failed to save imported config")
                return False, errors
            
            # Validate
            valid, validation_errors = self.validate_config()
            if not valid:
                errors.extend(validation_errors)
                return False, errors
            
            log.info("Configuration imported successfully")
            return True, []
            
        except Exception as e:
            errors.append(f"Import failed: {e}")
            log.error(f"Config import error: {e}")
            return False, errors


# Global config manager instance
_config_manager: Optional[ConfigManager] = None


def get_config_manager() -> ConfigManager:
    """Get or create global config manager instance"""
    global _config_manager
    if _config_manager is None:
        _config_manager = ConfigManager()
    return _config_manager

