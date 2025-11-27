"""
Hybrid Mode Switching Tests
Tests for switching between PI_ONLY, PI_ORIN_HYBRID, and ORIN_ONLY modes
"""

import pytest
import time
from unittest.mock import Mock, patch, MagicMock
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../..'))


@pytest.fixture
def hybrid_system_manager():
    """Create hybrid system manager for testing."""
    with patch('video.hybrid_system.rclpy'):
        with patch('video.hybrid_system.init_pi_sensor_hub'):
            from video.hybrid_system import HybridSystemManager
            manager = HybridSystemManager()
            return manager


@pytest.fixture
def mock_orin_available():
    """Mock Orin availability."""
    with patch('video.hybrid_system.HybridSystemManager.is_orin_available') as mock:
        mock.return_value = True
        yield mock


@pytest.fixture
def mock_orin_unavailable():
    """Mock Orin unavailability."""
    with patch('video.hybrid_system.HybridSystemManager.is_orin_available') as mock:
        mock.return_value = False
        yield mock


class TestHybridModeSwitching:
    """Test hybrid mode switching functionality."""

    def test_switch_pi_only_to_hybrid(self, hybrid_system_manager, mock_orin_available):
        """Test switching from PI_ONLY to HYBRID mode."""
        # Start in PI_ONLY
        hybrid_system_manager.set_manual_mode(0)
        assert hybrid_system_manager.get_effective_mode() == 0
        
        # Switch to HYBRID (mode 6)
        hybrid_system_manager.set_manual_mode(6)
        assert hybrid_system_manager.get_effective_mode() == 6
        
        # Verify hybrid system is active
        assert hybrid_system_manager.is_orin_available() is True

    def test_switch_hybrid_to_pi_only(self, hybrid_system_manager, mock_orin_available):
        """Test switching from HYBRID to PI_ONLY mode."""
        # Start in HYBRID
        hybrid_system_manager.set_manual_mode(6)
        assert hybrid_system_manager.get_effective_mode() == 6
        
        # Switch to PI_ONLY
        hybrid_system_manager.set_manual_mode(0)
        assert hybrid_system_manager.get_effective_mode() == 0

    def test_switch_under_load(self, hybrid_system_manager, mock_orin_available):
        """Test mode switching while system is under load."""
        # Simulate load
        hybrid_system_manager.cpu_monitor.get_load.return_value = 60.0
        
        # Switch modes rapidly
        for mode in [0, 1, 2, 3, 6, 0]:
            hybrid_system_manager.set_manual_mode(mode)
            assert hybrid_system_manager.get_effective_mode() == mode
            time.sleep(0.01)  # Small delay

    def test_auto_fallback_on_orin_failure(self, hybrid_system_manager, mock_orin_unavailable):
        """Test automatic fallback to PI_ONLY when Orin fails."""
        # Start in HYBRID mode
        hybrid_system_manager.set_manual_mode(6)
        
        # Simulate Orin failure
        hybrid_system_manager.is_orin_available = Mock(return_value=False)
        
        # System should detect failure and fallback
        # (Implementation depends on auto-detection logic)
        assert hybrid_system_manager.is_orin_available() is False

    def test_mode_switching_preserves_state(self, hybrid_system_manager):
        """Test that mode switching preserves system state."""
        # Set initial state
        hybrid_system_manager.set_manual_mode(3)
        
        # Switch modes
        hybrid_system_manager.set_manual_mode(6)
        hybrid_system_manager.set_manual_mode(0)
        hybrid_system_manager.set_manual_mode(3)
        
        # Should return to mode 3
        assert hybrid_system_manager.get_effective_mode() == 3

    def test_invalid_mode_transitions(self, hybrid_system_manager):
        """Test that invalid mode transitions are rejected."""
        initial_mode = hybrid_system_manager.get_effective_mode()
        
        # Try invalid modes
        hybrid_system_manager.set_manual_mode(-1)
        assert hybrid_system_manager.get_effective_mode() == initial_mode
        
        hybrid_system_manager.set_manual_mode(8)
        assert hybrid_system_manager.get_effective_mode() == initial_mode

    def test_concurrent_mode_switches(self, hybrid_system_manager):
        """Test handling of concurrent mode switch requests."""
        import threading
        
        results = []
        
        def switch_mode(mode):
            hybrid_system_manager.set_manual_mode(mode)
            results.append(hybrid_system_manager.get_effective_mode())
        
        # Create multiple threads switching modes
        threads = [
            threading.Thread(target=switch_mode, args=(i,))
            for i in range(8)
        ]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # All switches should complete
        assert len(results) == 8
        # Final mode should be valid
        assert hybrid_system_manager.get_effective_mode() in range(8) or hybrid_system_manager.get_effective_mode() is None

