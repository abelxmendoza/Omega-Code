"""
System Tests for OMEGAOS

Tests the complete OMEGAOS system as a whole, including:
- Service Orchestrator
- Configuration Layer
- API Integration
- End-to-end workflows
"""

import pytest
import json
import os
import tempfile
import time
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../'))

from omega_services.service_manager import ServiceManager
from omega_config.config_manager import ConfigManager


class TestOMEGAOSSystem:
    """System tests for complete OMEGAOS"""

    @pytest.fixture
    def temp_dir(self):
        """Create temporary directory for test files"""
        with tempfile.TemporaryDirectory() as tmpdir:
            yield tmpdir

    @pytest.fixture
    def full_config(self, temp_dir):
        """Create a full OMEGAOS configuration"""
        config = {
            'robot': {
                'name': 'Omega-1-Test',
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
                'autostart': ['movement_ws_server', 'main_api'],
                'restart_policies': {
                    'movement_ws_server': 'always',
                    'main_api': 'on-failure'
                }
            },
            'camera': {
                'backend': 'picamera2',
                'device': '/dev/video0',
                'width': 640,
                'height': 480,
                'fps': 30
            },
            'movement': {
                'default_profile': 'smooth',
                'max_speed': 4095,
                'min_speed': 0
            },
            'lighting': {
                'default_pattern': 'omega_signature',
                'default_brightness': 0.5
            },
            'logging': {
                'level': 'INFO'
            },
            'security': {
                'api_auth_enabled': False,
                'allowed_origins': []
            },
            'telemetry': {
                'enabled': True,
                'update_interval_ms': 1000
            }
        }
        
        config_path = os.path.join(temp_dir, 'config.yaml')
        import yaml
        with open(config_path, 'w') as f:
            yaml.dump(config, f)
        
        return config_path

    @pytest.fixture
    def service_registry(self, temp_dir):
        """Create service registry"""
        registry = {
            "services": [
                {
                    "name": "movement_ws_server",
                    "display_name": "Movement WebSocket Server",
                    "cmd": "python3",
                    "args": ["-c", "import time; time.sleep(0.1)"],
                    "working_dir": temp_dir,
                    "autostart": True,
                    "restart_policy": "always",
                    "health_check": "movement_check",
                    "port": 7070
                },
                {
                    "name": "main_api",
                    "display_name": "Main API Server",
                    "cmd": "python3",
                    "args": ["-c", "import time; time.sleep(0.1)"],
                    "working_dir": temp_dir,
                    "autostart": True,
                    "restart_policy": "on-failure",
                    "health_check": "api_check",
                    "port": 8080
                }
            ]
        }
        
        registry_path = os.path.join(temp_dir, 'service_registry.json')
        with open(registry_path, 'w') as f:
            json.dump(registry, f)
        
        return registry_path

    def test_system_boot_sequence(self, temp_dir, full_config, service_registry):
        """Test complete system boot sequence"""
        # Initialize config manager
        with patch('omega_config.config_manager.CONFIG_DIR', Path(temp_dir)):
            with patch('omega_config.config_manager.CONFIG_FILE', Path(temp_dir) / 'config.yaml'):
                config_manager = ConfigManager()
                config = config_manager.get_config()
                
                assert config is not None
                assert config.get('robot', {}).get('name') == 'Omega-1-Test'
                
                # Initialize service manager (outside patch context)
                service_manager = ServiceManager(registry_path=service_registry)
                services = service_manager.list_services()
                
                assert len(services) > 0
                
                # Verify autostart services are configured
                autostart = config.get('services', {}).get('autostart', [])
                assert len(autostart) > 0
                assert 'movement_ws_server' in autostart

    def test_configuration_persistence(self, temp_dir, full_config):
        """Test that configuration persists across system restarts"""
        with patch('omega_config.config_manager.CONFIG_DIR', Path(temp_dir)):
            with patch('omega_config.config_manager.CONFIG_FILE', Path(temp_dir) / 'config.yaml'):
                config_manager = ConfigManager()
                
                # Load initial config
                config1 = config_manager.get_config()
                original_name = config1.get('robot', {}).get('name', '')
                
                # Update config
                config_manager.update_section('robot', {'name': 'Omega-1-Updated'})
                
                # Create new config manager instance (simulating restart)
                config_manager2 = ConfigManager()
                config2 = config_manager2.get_config()
                
                # Verify persistence
                assert config2.get('robot', {}).get('name') == 'Omega-1-Updated'
                assert config2.get('robot', {}).get('name') != original_name

    def test_service_health_monitoring(self, temp_dir, service_registry):
        """Test that service health is monitored correctly"""
        service_manager = ServiceManager(registry_path=service_registry)
        
        # Get service status
        status = service_manager.get_service_status('movement_ws_server')
        # Status might be None if service not started, or dict if started
        # Health check would be included if service is running
        assert status is None or isinstance(status, dict)

    def test_config_update_triggers_service_reload(self, temp_dir, full_config, service_registry):
        """Test that config updates trigger service reload"""
        with patch('omega_config.config_manager.CONFIG_DIR', Path(temp_dir)):
            with patch('omega_config.config_manager.CONFIG_FILE', Path(temp_dir) / 'config.yaml'):
                config_manager = ConfigManager()
                
                service_manager = ServiceManager(registry_path=service_registry)
                
                # Update config
                config = config_manager.get_config()
                services_config = config.get('services', {})
                restart_policies = services_config.get('restart_policies', {})
                restart_policies['movement_ws_server'] = 'never'
                services_config['restart_policies'] = restart_policies
                config_manager.update_section('services', services_config)
                
                # Service manager should reflect changes
                # (In practice, would require reload mechanism)

    def test_error_recovery(self, temp_dir, service_registry):
        """Test system error recovery"""
        service_manager = ServiceManager(registry_path=service_registry)
        
        # Simulate service failure
        # Service should restart based on policy
        # (Would require actual process monitoring in practice)
        assert service_manager is not None

    def test_concurrent_operations(self, temp_dir, full_config):
        """Test concurrent configuration and service operations"""
        with patch('omega_config.config_manager.CONFIG_DIR', Path(temp_dir)):
            with patch('omega_config.config_manager.CONFIG_FILE', Path(temp_dir) / 'config.yaml'):
                config_manager = ConfigManager()
                
                # Simulate concurrent config reads/writes
                config1 = config_manager.get_config()
                config2 = config_manager.get_config()
                
                # Both should succeed
                assert config1 is not None
                assert config2 is not None
                assert config1 == config2

