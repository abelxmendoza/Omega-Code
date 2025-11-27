"""
Hybrid System Manager Unit Tests
Tests for hybrid system functionality
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))


@pytest.fixture
def hybrid_system_manager():
    """Create a hybrid system manager instance for testing."""
    with patch('video.hybrid_system.rclpy') as mock_rclpy:
        with patch('video.hybrid_system.init_pi_sensor_hub') as mock_init:
            mock_init.return_value = None
            from video.hybrid_system import HybridSystemManager
            manager = HybridSystemManager()
            return manager


class TestHybridSystemManager:
    """Test HybridSystemManager class."""

    def test_initialization(self, hybrid_system_manager):
        """Test hybrid system manager initialization."""
        assert hybrid_system_manager is not None
        assert hasattr(hybrid_system_manager, 'thermal_monitor')
        assert hasattr(hybrid_system_manager, 'cpu_monitor')

    def test_set_manual_mode(self, hybrid_system_manager):
        """Test setting manual mode override."""
        hybrid_system_manager.set_manual_mode(3)
        assert hybrid_system_manager.get_effective_mode() == 3

    def test_set_manual_mode_invalid_low(self, hybrid_system_manager):
        """Test setting invalid manual mode (too low)."""
        initial_mode = hybrid_system_manager.get_effective_mode()
        hybrid_system_manager.set_manual_mode(-1)
        # Should not change
        assert hybrid_system_manager.get_effective_mode() == initial_mode

    def test_set_manual_mode_invalid_high(self, hybrid_system_manager):
        """Test setting invalid manual mode (too high)."""
        initial_mode = hybrid_system_manager.get_effective_mode()
        hybrid_system_manager.set_manual_mode(8)
        # Should not change
        assert hybrid_system_manager.get_effective_mode() == initial_mode

    def test_clear_manual_mode(self, hybrid_system_manager):
        """Test clearing manual mode override."""
        hybrid_system_manager.set_manual_mode(3)
        assert hybrid_system_manager.get_effective_mode() == 3
        
        hybrid_system_manager.clear_manual_mode()
        assert hybrid_system_manager.get_effective_mode() is None

    def test_get_effective_mode(self, hybrid_system_manager):
        """Test getting effective mode."""
        # Initially should be None
        assert hybrid_system_manager.get_effective_mode() is None
        
        # After setting manual mode
        hybrid_system_manager.set_manual_mode(5)
        assert hybrid_system_manager.get_effective_mode() == 5

    def test_check_and_auto_switch_mode(self, hybrid_system_manager):
        """Test automatic mode switching based on thermal/CPU."""
        # Mock thermal throttling
        hybrid_system_manager.thermal_monitor.should_throttle.return_value = True
        hybrid_system_manager.thermal_monitor.get_temperature.return_value = 75.0
        
        # Should check conditions
        hybrid_system_manager.check_and_auto_switch_mode()
        
        # Verify throttling was checked
        assert hybrid_system_manager.thermal_monitor.should_throttle.called

    def test_should_throttle_modules(self, hybrid_system_manager):
        """Test module throttling check."""
        hybrid_system_manager.thermal_monitor.should_throttle.return_value = False
        hybrid_system_manager.cpu_monitor.should_throttle.return_value = False
        
        assert hybrid_system_manager.should_throttle_modules() is False
        
        hybrid_system_manager.thermal_monitor.should_throttle.return_value = True
        assert hybrid_system_manager.should_throttle_modules() is True

    def test_get_throttle_priority(self, hybrid_system_manager):
        """Test throttle priority order."""
        priority = hybrid_system_manager.get_throttle_priority()
        assert isinstance(priority, list)
        assert len(priority) > 0
        assert "motion" in priority

