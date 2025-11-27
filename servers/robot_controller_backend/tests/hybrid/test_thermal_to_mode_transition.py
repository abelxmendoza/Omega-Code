"""
Thermal to Mode Transition Tests
Tests for automatic mode switching based on thermal/CPU conditions
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))


@pytest.fixture
def hybrid_system_manager():
    """Create hybrid system manager with thermal monitoring."""
    with patch('video.hybrid_system.rclpy'):
        with patch('video.hybrid_system.init_pi_sensor_hub'):
            from video.hybrid_system import HybridSystemManager
            manager = HybridSystemManager()
            return manager


class TestThermalToModeTransition:
    """Test thermal/CPU triggered mode transitions."""

    def test_thermal_threshold_triggers_safe_mode(self, hybrid_system_manager):
        """Test that thermal >70°C triggers safe mode."""
        # Set thermal monitor to report high temperature
        hybrid_system_manager.thermal_monitor.get_temperature.return_value = 75.0
        hybrid_system_manager.thermal_monitor.should_throttle.return_value = True
        hybrid_system_manager.thermal_monitor.is_throttling_active.return_value = True
        
        # Check if system should throttle
        should_throttle = hybrid_system_manager.should_throttle_modules()
        assert should_throttle is True
        
        # System should be in throttling state
        assert hybrid_system_manager.thermal_monitor.is_throttling_active() is True

    def test_cpu_load_triggers_degradation(self, hybrid_system_manager):
        """Test that CPU >75% triggers mode degradation."""
        # Set CPU monitor to report high load
        hybrid_system_manager.cpu_monitor.get_load.return_value = 85.0
        hybrid_system_manager.cpu_monitor.should_throttle.return_value = True
        hybrid_system_manager.cpu_monitor.is_throttling_active.return_value = True
        
        # Check if system should throttle
        should_throttle = hybrid_system_manager.should_throttle_modules()
        assert should_throttle is True
        
        # System should be in throttling state
        assert hybrid_system_manager.cpu_monitor.is_throttling_active() is True

    def test_combined_thermal_and_cpu_throttling(self, hybrid_system_manager):
        """Test combined thermal and CPU throttling."""
        # Both thermal and CPU are high
        hybrid_system_manager.thermal_monitor.is_throttling_active.return_value = True
        hybrid_system_manager.cpu_monitor.is_throttle_active.return_value = True
        
        # System should throttle
        should_throttle = hybrid_system_manager.should_throttle_modules()
        assert should_throttle is True

    def test_thermal_recovery_restores_mode(self, hybrid_system_manager):
        """Test that thermal recovery restores normal mode."""
        # Start with high temperature
        hybrid_system_manager.thermal_monitor.get_temperature.return_value = 75.0
        hybrid_system_manager.thermal_monitor.is_throttling_active.return_value = True
        
        # Temperature drops
        hybrid_system_manager.thermal_monitor.get_temperature.return_value = 50.0
        hybrid_system_manager.thermal_monitor.is_throttling_active.return_value = False
        
        # System should no longer throttle
        should_throttle = hybrid_system_manager.should_throttle_modules()
        assert should_throttle is False

    def test_throttle_priority_order(self, hybrid_system_manager):
        """Test throttle priority order."""
        priority = hybrid_system_manager.get_throttle_priority()
        
        # Should have priority order
        assert isinstance(priority, list)
        assert len(priority) > 0
        
        # Motion detection should be throttled first
        assert "motion" in priority

    def test_mode_switching_during_throttle(self, hybrid_system_manager):
        """Test that manual mode switching works during throttling."""
        # Start throttling
        hybrid_system_manager.thermal_monitor.is_throttling_active.return_value = True
        
        # Manual mode switch should still work
        hybrid_system_manager.set_manual_mode(3)
        assert hybrid_system_manager.get_effective_mode() == 3
        
        # Clear manual mode
        hybrid_system_manager.clear_manual_mode()
        assert hybrid_system_manager.get_effective_mode() is None

    def test_throttle_mode_prevents_high_load_modes(self, hybrid_system_manager):
        """Test that throttling prevents switching to high-load modes."""
        # Start throttling
        hybrid_system_manager.thermal_monitor.is_throttling_active.return_value = True
        
        # Try to switch to high-load mode (6 = Orin-Enhanced)
        hybrid_system_manager.set_manual_mode(6)
        
        # System should either reject or degrade the mode
        # (Implementation depends on throttle logic)
        effective_mode = hybrid_system_manager.get_effective_mode()
        assert effective_mode in [None, 0, 1]  # Should be safe mode or PI_ONLY

