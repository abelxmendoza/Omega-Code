"""
Unit Tests for Service Manager (OMEGAOS Service Orchestrator)

Tests individual components of the service manager in isolation.
"""

import pytest
import json
import os
import tempfile
from unittest.mock import Mock, patch, MagicMock
from pathlib import Path

# Import service manager components
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../'))

from omega_services.service_manager import ServiceManager
from omega_services.process_supervisor import ProcessSupervisor, ServiceState
from omega_services.health_checks import check_port, ros_core_check, sensor_check


class TestServiceManager:
    """Unit tests for ServiceManager"""

    @pytest.fixture
    def temp_registry(self):
        """Create a temporary service registry for testing"""
        registry = {
            "services": [
                {
                    "name": "test_service",
                    "display_name": "Test Service",
                    "cmd": "python3",
                    "args": ["-c", "print('test')"],
                    "working_dir": "/tmp",
                    "autostart": True,
                    "restart_policy": "always",
                    "health_check": "movement_check",
                    "port": 8080,
                    "description": "Test service"
                }
            ]
        }
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(registry, f)
            temp_path = f.name
        
        yield temp_path
        
        # Cleanup
        if os.path.exists(temp_path):
            os.unlink(temp_path)

    @pytest.fixture
    def service_manager(self, temp_registry):
        """Create a ServiceManager instance for testing"""
        manager = ServiceManager(registry_path=temp_registry)
        yield manager

    def test_load_service_registry(self, service_manager, temp_registry):
        """Test loading service registry from JSON"""
        registry = service_manager.supervisor.load_registry()
        assert registry is not None
        assert len(registry) > 0
        assert registry[0]['name'] == 'test_service'

    def test_get_service_status(self, service_manager):
        """Test retrieving a service status by name"""
        status = service_manager.get_service_status('test_service')
        # Status might be None if service not started, or dict if started
        assert status is None or isinstance(status, dict)

    def test_get_nonexistent_service_status(self, service_manager):
        """Test retrieving a non-existent service status"""
        status = service_manager.get_service_status('nonexistent')
        assert status is None

    def test_list_services(self, service_manager):
        """Test listing all services"""
        # Mock supervisor methods
        with patch.object(service_manager.supervisor, 'load_registry') as mock_load_registry:
            with patch.object(service_manager, 'get_service_status') as mock_get_status:
                mock_load_registry.return_value = [{
                    'name': 'test_service',
                    'display_name': 'Test Service',
                    'health_check': 'movement_check'  # String, not dict
                }]
                mock_get_status.return_value = {
                    'name': 'test_service',
                    'status': 'stopped',
                    'pid': None
                }
                
                services = service_manager.list_services()
                assert isinstance(services, list)
                assert len(services) > 0
                # Each service should be a dict with at least 'name'
                assert 'name' in services[0]

    @patch('omega_services.process_supervisor.ProcessSupervisor.start_service')
    def test_start_service(self, mock_start, service_manager):
        """Test starting a service"""
        mock_start.return_value = True
        
        result = service_manager.start_service('test_service')
        assert isinstance(result, dict)
        assert result.get('success') is True

    @patch('omega_services.process_supervisor.ProcessSupervisor.stop_service')
    def test_stop_service(self, mock_stop, service_manager):
        """Test stopping a service"""
        mock_stop.return_value = True
        
        result = service_manager.stop_service('test_service')
        assert isinstance(result, dict)
        assert result.get('success') is True

    @patch('omega_services.process_supervisor.ProcessSupervisor.restart_service')
    def test_restart_service(self, mock_restart, service_manager):
        """Test restarting a service"""
        mock_restart.return_value = True
        
        result = service_manager.restart_service('test_service')
        assert isinstance(result, dict)
        assert result.get('success') is True

    def test_get_service_status(self, service_manager):
        """Test getting service status"""
        with patch.object(service_manager.supervisor, 'get_service_status') as mock_get_status:
            mock_get_status.return_value = {
                'status': 'running',
                'pid': 1234
            }
            
            status = service_manager.get_service_status('test_service')
            assert status is not None
            assert 'status' in status

    def test_get_logs(self, service_manager):
        """Test retrieving service logs"""
        from pathlib import Path
        with patch('omega_services.utils.get_log_file') as mock_get_log:
            mock_get_log.return_value = Path('/tmp/test.log')
            with patch('builtins.open', create=True) as mock_open:
                mock_open.return_value.__enter__.return_value.readlines.return_value = ['test log']
                
                logs = service_manager.get_logs('test_service')
                assert logs is not None
                assert 'stdout' in logs


class TestProcessSupervisor:
    """Unit tests for ProcessSupervisor"""

    @pytest.fixture
    def temp_registry(self):
        """Create a temporary service registry for testing"""
        registry = {
            "services": [
                {
                    "name": "test",
                    "cmd": "python3",
                    "args": ["-c", "print('test')"],
                }
            ]
        }
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(registry, f)
            temp_path = f.name
        
        yield temp_path
        
        # Cleanup
        if os.path.exists(temp_path):
            os.unlink(temp_path)

    @pytest.fixture
    def supervisor(self, temp_registry):
        """Create a ProcessSupervisor instance"""
        return ProcessSupervisor(registry_path=temp_registry)

    @patch('subprocess.Popen')
    def test_start_service(self, mock_popen, supervisor):
        """Test starting a service process"""
        mock_process = Mock()
        mock_process.pid = 1234
        mock_popen.return_value = mock_process
        
        service_def = {
            'name': 'test',
            'cmd': 'python3',
            'args': ['-c', 'print("test")'],
            'working_dir': '/tmp'
        }
        
        result = supervisor.start_service(service_def)
        assert result is True
        assert 'test' in supervisor.services
        assert supervisor.services['test'].pid == 1234

    def test_stop_service(self, supervisor):
        """Test stopping a service"""
        # Setup a mock service state
        mock_state = ServiceState(
            name='test',
            pid=1234,
            status='running',
            crash_count=0
        )
        supervisor.services['test'] = mock_state
        supervisor.processes['test'] = Mock()  # Mock process
        
        result = supervisor.stop_service('test')
        assert result is True

    def test_get_status(self, supervisor):
        """Test getting service status"""
        mock_state = ServiceState(
            name='test',
            pid=1234,
            status='running',
            crash_count=0
        )
        supervisor.services['test'] = mock_state
        
        status = supervisor.get_service_status('test')
        assert status is not None
        assert status['pid'] == 1234
        assert status['status'] == 'running'


class TestHealthChecks:
    """Unit tests for health check functions"""

    @patch('socket.socket')
    def test_check_port_success(self, mock_socket):
        """Test successful port check"""
        mock_sock = Mock()
        mock_socket.return_value = mock_sock
        mock_sock.connect_ex.return_value = 0
        
        result = check_port('localhost', 8080)
        assert result is True

    @patch('socket.socket')
    def test_check_port_failure(self, mock_socket):
        """Test failed port check"""
        mock_sock = Mock()
        mock_socket.return_value = mock_sock
        mock_sock.connect_ex.return_value = 1
        
        result = check_port('localhost', 8080)
        assert result is False

    @patch('subprocess.run')
    def test_ros_core_check_success(self, mock_run):
        """Test successful ROS check"""
        mock_run.return_value = Mock(returncode=0, stdout='topic1\ntopic2')
        
        result = ros_core_check()
        assert result['healthy'] is True

    @patch('subprocess.run')
    def test_ros_core_check_failure(self, mock_run):
        """Test failed ROS check"""
        mock_run.return_value = Mock(returncode=1, stdout='')
        
        result = ros_core_check()
        assert result['healthy'] is False

    @patch('os.path.exists')
    def test_sensor_check_success(self, mock_exists):
        """Test successful sensor check"""
        mock_exists.return_value = True
        
        result = sensor_check()
        assert result['healthy'] is True

    @patch('os.path.exists')
    def test_sensor_check_failure(self, mock_exists):
        """Test failed sensor check"""
        mock_exists.return_value = False
        
        result = sensor_check()
        assert result['healthy'] is False

