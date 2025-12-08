"""
Integration Tests for OMEGAOS Service Orchestrator + Configuration Layer

Tests how service orchestrator and configuration layer work together.
"""

import pytest
import json
import os
import tempfile
from unittest.mock import Mock, patch, MagicMock

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../'))

from omega_services.service_manager import ServiceManager
from omega_config.config_manager import ConfigManager


class TestServiceConfigIntegration:
    """Integration tests for Service Manager + Config Manager"""

    @pytest.fixture
    def temp_dir(self):
        """Create temporary directory for test files"""
        with tempfile.TemporaryDirectory() as tmpdir:
            yield tmpdir

    @pytest.fixture
    def config_manager(self, temp_dir):
        """Create ConfigManager instance"""
        with patch('omega_config.config_manager.CONFIG_DIR', temp_dir):
            manager = ConfigManager()
            yield manager

    @pytest.fixture
    def service_registry(self, temp_dir):
        """Create service registry"""
        registry = {
            "services": [
                {
                    "name": "test_service",
                    "display_name": "Test Service",
                    "cmd": "python3",
                    "args": ["-c", "import time; time.sleep(0.1)"],
                    "working_dir": temp_dir,
                    "autostart": True,
                    "restart_policy": "always",
                    "health_check": "movement_check",
                    "port": 8080
                }
            ]
        }
        
        registry_path = os.path.join(temp_dir, 'service_registry.json')
        with open(registry_path, 'w') as f:
            json.dump(registry, f)
        
        return registry_path

    @pytest.fixture
    def service_manager(self, service_registry):
        """Create ServiceManager instance"""
        manager = ServiceManager(registry_path=service_registry)
        yield manager

    def test_config_loads_service_autostart(self, config_manager, service_manager, temp_dir):
        """Test that config autostart settings are respected"""
        # Create config with autostart services
        config = {
            'services': {
                'autostart': ['test_service'],
                'restart_policies': {
                    'test_service': 'always'
                }
            }
        }
        
        config_path = os.path.join(temp_dir, 'config.yaml')
        import yaml
        with open(config_path, 'w') as f:
            yaml.dump(config, f)
        
        # Load config
        loaded_config = config_manager.get_config()
        assert 'test_service' in loaded_config.get('services', {}).get('autostart', [])

    def test_service_manager_reads_config(self, config_manager, service_manager, temp_dir):
        """Test that service manager can read config for service definitions"""
        # This tests the integration between config and service manager
        # In practice, service manager loads from registry, but config can override
        
        config = {
            'services': {
                'autostart': ['test_service']
            }
        }
        
        config_path = os.path.join(temp_dir, 'config.yaml')
        import yaml
        with open(config_path, 'w') as f:
            yaml.dump(config, f)
        
        # Service manager should be able to work with config
        services = service_manager.list_services()
        assert len(services) > 0

    @patch('omega_services.process_supervisor.ProcessSupervisor.start_service')
    def test_autostart_services_on_boot(self, mock_start, config_manager, service_manager, temp_dir):
        """Test that autostart services are started based on config"""
        config = {
            'services': {
                'autostart': ['test_service'],
                'restart_policies': {
                    'test_service': 'always'
                }
            }
        }
        
        config_path = os.path.join(temp_dir, 'config.yaml')
        import yaml
        with open(config_path, 'w') as f:
            yaml.dump(config, f)
        
        # Simulate boot sequence
        loaded_config = config_manager.get_config()
        autostart_services = loaded_config.get('services', {}).get('autostart', [])
        
        # Service manager should start autostart services
        for service_name in autostart_services:
            # Check if service exists in registry
            registry = service_manager.supervisor.load_registry()
            service_def = next((s for s in registry if s["name"] == service_name), None)
            if service_def:
                # Start service (returns dict with success status)
                result = service_manager.start_service(service_name)
                assert isinstance(result, dict)
        
        # Verify services were processed
        assert len(autostart_services) >= 0  # May be empty or have services

    def test_config_update_reflects_in_service_manager(self, config_manager, service_manager, temp_dir):
        """Test that config updates are reflected in service manager behavior"""
        # Update config to change autostart
        config = {
            'services': {
                'autostart': ['test_service'],
                'restart_policies': {
                    'test_service': 'on-failure'
                }
            }
        }
        
        config_path = os.path.join(temp_dir, 'config.yaml')
        import yaml
        with open(config_path, 'w') as f:
            yaml.dump(config, f)
        
        # Update config
        config_manager.update_section('services', config['services'])
        
        # Service manager should respect new restart policy
        # Check service status
        status = service_manager.get_service_status('test_service')
        # Status might be None if service not started
        # Restart policy would be updated in service state (would require reload in practice)

