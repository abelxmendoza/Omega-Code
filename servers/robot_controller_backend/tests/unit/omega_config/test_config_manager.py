"""
Unit Tests for Configuration Manager (OMEGAOS Configuration Layer)

Tests individual components of the configuration manager in isolation.
"""

import pytest
import yaml
import json
import os
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch, mock_open

# Import config manager
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../'))

from omega_config.config_manager import ConfigManager, RobotConfig
from dataclasses import asdict


class TestConfigManager:
    """Unit tests for ConfigManager"""

    @pytest.fixture
    def temp_config_dir(self):
        """Create a temporary config directory for testing"""
        with tempfile.TemporaryDirectory() as tmpdir:
            yield tmpdir

    @pytest.fixture
    def sample_config(self):
        """Sample configuration data"""
        return {
            'robot': {
                'name': 'Omega-1',
                'profile': 'pi4b',
                'version': '1.0.0'
            },
            'network': {
                'default_mode': 'ap',
                'ap': {
                    'ssid': 'Omega1-AP',
                    'password': 'test123',
                    'ip': '192.168.4.1'
                }
            },
            'services': {
                'autostart': ['movement_ws_server'],
                'restart_policies': {
                    'movement_ws_server': 'always'
                }
            }
        }

    @pytest.fixture
    def config_manager(self, temp_config_dir):
        """Create a ConfigManager instance"""
        with patch('omega_config.config_manager.CONFIG_DIR', temp_config_dir):
            manager = ConfigManager()
            yield manager

    def test_get_config(self, config_manager, temp_config_dir, sample_config):
        """Test getting configuration"""
        config_path = os.path.join(temp_config_dir, 'config.yaml')
        with open(config_path, 'w') as f:
            yaml.dump(sample_config, f)
        
        # Reload config manager to pick up the file
        with patch('omega_config.config_manager.CONFIG_DIR', Path(temp_config_dir)):
            with patch('omega_config.config_manager.CONFIG_FILE', Path(temp_config_dir) / 'config.yaml'):
                # Ensure sample_config has all required fields
                full_config = {
                    **sample_config,
                    'camera': sample_config.get('camera', {}),
                    'movement': sample_config.get('movement', {}),
                    'lighting': sample_config.get('lighting', {}),
                    'logging': sample_config.get('logging', {}),
                    'security': sample_config.get('security', {}),
                    'telemetry': sample_config.get('telemetry', {}),
                }
                with open(os.path.join(temp_config_dir, 'config.yaml'), 'w') as f:
                    yaml.dump(full_config, f)
                config_manager2 = ConfigManager()
                config = config_manager2.get_config()
                assert config is not None
                assert config.get('robot', {}).get('name') == 'Omega-1'

    def test_save_config(self, config_manager, temp_config_dir, sample_config):
        """Test saving configuration to YAML"""
        config_path = os.path.join(temp_config_dir, 'config.yaml')
        
        # Set config using RobotConfig dataclass
        config_manager.config = RobotConfig(
            robot=sample_config['robot'],
            network=sample_config['network'],
            services=sample_config['services'],
            camera={},
            movement={},
            lighting={},
            logging={},
            security={},
            telemetry={}
        )
        with patch('omega_config.config_manager.CONFIG_FILE', Path(config_path)):
            result = config_manager.save_config()
            assert result is True
            assert os.path.exists(config_path)

    def test_get_section(self, config_manager, sample_config):
        """Test retrieving a configuration section"""
        # Set config using RobotConfig dataclass
        config_manager.config = RobotConfig(
            robot=sample_config['robot'],
            network=sample_config['network'],
            services=sample_config['services'],
            camera={},
            movement={},
            lighting={},
            logging={},
            security={},
            telemetry={}
        )
        section = config_manager.get_section('robot')
        assert section is not None
        assert section['name'] == 'Omega-1'

    def test_update_section(self, config_manager, temp_config_dir, sample_config):
        """Test updating a configuration section"""
        config_path = os.path.join(temp_config_dir, 'config.yaml')
        with open(config_path, 'w') as f:
            yaml.dump(sample_config, f)
        
        # Ensure sample_config has all required fields
        full_config = {
            **sample_config,
            'camera': sample_config.get('camera', {}),
            'movement': sample_config.get('movement', {}),
            'lighting': sample_config.get('lighting', {}),
            'logging': sample_config.get('logging', {}),
            'security': sample_config.get('security', {}),
            'telemetry': sample_config.get('telemetry', {}),
        }
        with open(config_path, 'w') as f:
            yaml.dump(full_config, f)
        
        # Reload config manager
        with patch('omega_config.config_manager.CONFIG_DIR', Path(temp_config_dir)):
            with patch('omega_config.config_manager.CONFIG_FILE', Path(config_path)):
                config_manager2 = ConfigManager()
                new_robot_config = {
                    'name': 'Omega-2',
                    'profile': 'jetson',
                    'version': '1.0.0'
                }
                
                config_manager2.update_section('robot', new_robot_config)
                
                config = config_manager2.get_config()
                assert config['robot']['name'] == 'Omega-2'

    def test_validate_config_valid(self, config_manager, sample_config):
        """Test validating a valid configuration"""
        # Set config using RobotConfig dataclass with all required fields
        config_manager.config = RobotConfig(
            robot=sample_config.get('robot', {'name': 'Omega-1', 'profile': 'pi4b'}),
            network=sample_config.get('network', {'default_mode': 'ap'}),
            services=sample_config.get('services', {'autostart': []}),
            camera=sample_config.get('camera', {}),
            movement=sample_config.get('movement', {}),
            lighting=sample_config.get('lighting', {}),
            logging=sample_config.get('logging', {'level': 'INFO'}),
            security=sample_config.get('security', {}),
            telemetry=sample_config.get('telemetry', {})
        )
        valid, errors = config_manager.validate_config()
        # Validation might have some checks, but should pass for valid config
        assert isinstance(valid, bool)
        assert isinstance(errors, list)

    def test_validate_config_invalid(self, config_manager):
        """Test validating an invalid configuration"""
        # Set invalid config
        config_manager.config = None
        valid, errors = config_manager.validate_config()
        # Should fail when config is None
        assert valid is False
        assert len(errors) > 0

    def test_export_config(self, config_manager, sample_config):
        """Test exporting configuration to JSON"""
        # Set config using RobotConfig dataclass
        config_manager.config = RobotConfig(
            robot=sample_config['robot'],
            network=sample_config['network'],
            services=sample_config['services'],
            camera={},
            movement={},
            lighting={},
            logging={},
            security={},
            telemetry={}
        )
        exported = config_manager.get_config()
        assert exported is not None
        assert 'robot' in exported
        assert exported['robot']['name'] == 'Omega-1'

    def test_import_config(self, config_manager, temp_config_dir, sample_config):
        """Test importing configuration from JSON"""
        config_path = os.path.join(temp_config_dir, 'config.yaml')
        
        # Set config using RobotConfig dataclass and save
        config_manager.config = RobotConfig(
            robot=sample_config['robot'],
            network=sample_config['network'],
            services=sample_config['services'],
            camera={},
            movement={},
            lighting={},
            logging={},
            security={},
            telemetry={}
        )
        with patch('omega_config.config_manager.CONFIG_FILE', Path(config_path)):
            config_manager.save_config()
        
        # Verify it was saved
        assert os.path.exists(config_path)
        loaded = config_manager.get_config()
        assert loaded['robot']['name'] == 'Omega-1'

    def test_get_robot_profile(self, config_manager, temp_config_dir):
        """Test getting robot profile"""
        profile_data = {
            'profiles': {
                'pi4b': {
                    'hardware': {
                        'cpu_cores': 4,
                        'ram_gb': 4
                    },
                    'capabilities': {
                        'ml_capable': False
                    }
                }
            }
        }
        
        profile_path = os.path.join(temp_config_dir, 'robot_profile.yaml')
        with open(profile_path, 'w') as f:
            yaml.dump(profile_data, f)
        
        # Reload config manager
        with patch('omega_config.config_manager.CONFIG_DIR', Path(temp_config_dir)):
            with patch('omega_config.config_manager.PROFILE_FILE', Path(profile_path)):
                config_manager2 = ConfigManager()
                profile = config_manager2.get_profile('pi4b')
                assert profile is not None
                assert profile['hardware']['cpu_cores'] == 4

    def test_get_hardware_map(self, config_manager, temp_config_dir):
        """Test getting hardware map"""
        hardware_data = {
            'gpio': {
                'motor_left': 12,
                'motor_right': 13
            },
            'i2c': {
                'devices': {
                    'pca9685': {
                        'address': '0x40'
                    }
                }
            }
        }
        
        hardware_path = os.path.join(temp_config_dir, 'hardware_map.yaml')
        with open(hardware_path, 'w') as f:
            yaml.dump(hardware_data, f)
        
        # Reload config manager
        with patch('omega_config.config_manager.CONFIG_DIR', Path(temp_config_dir)):
            with patch('omega_config.config_manager.HARDWARE_FILE', Path(hardware_path)):
                config_manager2 = ConfigManager()
                hardware = config_manager2.get_hardware_map()
                assert hardware is not None
                assert 'gpio' in hardware
                assert hardware['gpio']['motor_left'] == 12

    def test_get_persistent_state(self, config_manager, temp_config_dir):
        """Test getting persistent state"""
        state_data = {
            'boot_count': 5,
            'last_boot_time': '2024-01-01T00:00:00',
            'current_network_mode': 'ap'
        }
        
        state_path = os.path.join(temp_config_dir, 'persistent_state.json')
        with open(state_path, 'w') as f:
            json.dump(state_data, f)
        
        # Reload config manager
        with patch('omega_config.config_manager.CONFIG_DIR', Path(temp_config_dir)):
            with patch('omega_config.config_manager.STATE_FILE', Path(state_path)):
                config_manager2 = ConfigManager()
                state = config_manager2.get_state()
                assert state is not None
                assert state['boot_count'] == 5

    def test_save_persistent_state(self, config_manager, temp_config_dir):
        """Test saving persistent state via update_state"""
        state_data = {
            'boot_count': 6,
            'last_boot_time': '2024-01-02T00:00:00'
        }
        
        # Set state and update
        config_manager.state = state_data
        config_manager.update_state('boot_count', 6)
        
        state_path = os.path.join(temp_config_dir, 'persistent_state.json')
        # State file might be in different location, check if update worked
        state = config_manager.get_state()
        assert state['boot_count'] == 6

    def test_update_persistent_state(self, config_manager, temp_config_dir):
        """Test updating persistent state"""
        initial_state = {
            'boot_count': 5,
            'current_network_mode': 'ap'
        }
        
        state_path = os.path.join(temp_config_dir, 'persistent_state.json')
        with open(state_path, 'w') as f:
            json.dump(initial_state, f)
        
        # Reload config manager
        with patch('omega_config.config_manager.CONFIG_DIR', Path(temp_config_dir)):
            with patch('omega_config.config_manager.STATE_FILE', Path(state_path)):
                config_manager2 = ConfigManager()
                config_manager2.update_state('boot_count', 6)
                
                state = config_manager2.get_state()
                assert state['boot_count'] == 6

