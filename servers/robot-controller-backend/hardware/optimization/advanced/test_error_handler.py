"""
Comprehensive Tests for Error Handler
- Unit tests for all error handling functionality
- Recovery strategy tests
- Alert system tests
- Performance tests
"""

import unittest
import time
import threading
import logging
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the parent directory to the path to import the modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from error_handler import (
    HardwareErrorHandler, ErrorInfo, RecoveryStrategy, 
    ErrorSeverity, ErrorCategory, error_handler_decorator
)

class TestHardwareErrorHandler(unittest.TestCase):
    """Test cases for Hardware Error Handler"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.error_handler = HardwareErrorHandler()
        
    def tearDown(self):
        """Clean up after tests"""
        if hasattr(self.error_handler, 'stop_monitoring'):
            self.error_handler.stop_monitoring()
    
    def test_error_handler_initialization(self):
        """Test error handler initialization"""
        self.assertIsNotNone(self.error_handler)
        self.assertIsNotNone(self.error_handler.error_history)
        self.assertIsInstance(self.error_handler.recovery_strategies, dict)
        self.assertIsInstance(self.error_handler.error_stats, dict)
        self.assertFalse(self.error_handler.running)
    
    def test_error_info_creation(self):
        """Test error info creation"""
        error_info = ErrorInfo(
            timestamp=time.time(),
            component="test_component",
            error_type="ValueError",
            severity=ErrorSeverity.MEDIUM,
            category=ErrorCategory.SOFTWARE,
            message="Test error message",
            details={"key": "value"},
            stack_trace="Traceback...",
            recovery_action="retry"
        )
        
        self.assertEqual(error_info.component, "test_component")
        self.assertEqual(error_info.error_type, "ValueError")
        self.assertEqual(error_info.severity, ErrorSeverity.MEDIUM)
        self.assertEqual(error_info.category, ErrorCategory.SOFTWARE)
        self.assertEqual(error_info.message, "Test error message")
    
    def test_recovery_strategy_creation(self):
        """Test recovery strategy creation"""
        def test_recovery_action(error_info):
            return True
        
        strategy = RecoveryStrategy(
            strategy_id="test_strategy",
            component="test_component",
            error_patterns=["test", "error"],
            recovery_action=test_recovery_action,
            max_attempts=3,
            backoff_delay=1.0
        )
        
        self.assertEqual(strategy.strategy_id, "test_strategy")
        self.assertEqual(strategy.component, "test_component")
        self.assertEqual(strategy.max_attempts, 3)
        self.assertEqual(strategy.backoff_delay, 1.0)
    
    def test_handle_error_success(self):
        """Test successful error handling"""
        test_error = ValueError("Test error message")
        
        result = self.error_handler.handle_error(
            component="test_component",
            error=test_error,
            severity=ErrorSeverity.MEDIUM,
            category=ErrorCategory.SOFTWARE,
            details={"test": "data"}
        )
        
        self.assertIsInstance(result, bool)
        
        # Check that error was recorded
        self.assertGreater(len(self.error_handler.error_history), 0)
        
        # Check statistics
        stats = self.error_handler.get_error_stats()
        self.assertGreater(stats["total_errors"], 0)
    
    def test_handle_error_with_recovery(self):
        """Test error handling with recovery"""
        test_error = Exception("GPIO error")
        
        result = self.error_handler.handle_error(
            component="gpio",
            error=test_error,
            severity=ErrorSeverity.HIGH,
            category=ErrorCategory.HARDWARE
        )
        
        self.assertIsInstance(result, bool)
        
        # Check recovery statistics
        stats = self.error_handler.get_error_stats()
        self.assertGreaterEqual(stats["recovery_attempts"], 0)
    
    def test_add_recovery_strategy(self):
        """Test adding recovery strategy"""
        def custom_recovery_action(error_info):
            return True
        
        strategy = RecoveryStrategy(
            strategy_id="custom_strategy",
            component="custom_component",
            error_patterns=["custom"],
            recovery_action=custom_recovery_action
        )
        
        self.error_handler.add_recovery_strategy(strategy)
        
        # Check that strategy was added
        self.assertIn("custom_strategy", self.error_handler.recovery_strategies)
    
    def test_gpio_recovery_action(self):
        """Test GPIO recovery action"""
        error_info = ErrorInfo(
            timestamp=time.time(),
            component="gpio",
            error_type="GPIOError",
            severity=ErrorSeverity.HIGH,
            category=ErrorCategory.HARDWARE,
            message="GPIO pin error"
        )
        
        result = self.error_handler._gpio_recovery_action(error_info)
        self.assertTrue(result)
    
    def test_dma_recovery_action(self):
        """Test DMA recovery action"""
        error_info = ErrorInfo(
            timestamp=time.time(),
            component="dma",
            error_type="DMAError",
            severity=ErrorSeverity.HIGH,
            category=ErrorCategory.HARDWARE,
            message="DMA transfer error"
        )
        
        result = self.error_handler._dma_recovery_action(error_info)
        self.assertTrue(result)
    
    def test_cpu_recovery_action(self):
        """Test CPU recovery action"""
        error_info = ErrorInfo(
            timestamp=time.time(),
            component="cpu",
            error_type="CPUError",
            severity=ErrorSeverity.HIGH,
            category=ErrorCategory.HARDWARE,
            message="CPU frequency error"
        )
        
        result = self.error_handler._cpu_recovery_action(error_info)
        self.assertTrue(result)
    
    def test_memory_recovery_action(self):
        """Test memory recovery action"""
        error_info = ErrorInfo(
            timestamp=time.time(),
            component="memory",
            error_type="MemoryError",
            severity=ErrorSeverity.HIGH,
            category=ErrorCategory.MEMORY,
            message="Memory allocation error"
        )
        
        result = self.error_handler._memory_recovery_action(error_info)
        self.assertTrue(result)
    
    def test_error_statistics(self):
        """Test error statistics collection"""
        # Generate errors of different severities and categories
        errors = [
            (ValueError("Low error"), ErrorSeverity.LOW, ErrorCategory.SOFTWARE),
            (RuntimeError("Medium error"), ErrorSeverity.MEDIUM, ErrorCategory.SOFTWARE),
            (Exception("High error"), ErrorSeverity.HIGH, ErrorCategory.HARDWARE),
            (MemoryError("Critical error"), ErrorSeverity.CRITICAL, ErrorCategory.MEMORY)
        ]
        
        for error, severity, category in errors:
            self.error_handler.handle_error("test_component", error, severity, category)
        
        # Check statistics
        stats = self.error_handler.get_error_stats()
        
        self.assertIn("total_errors", stats)
        self.assertIn("errors_by_severity", stats)
        self.assertIn("errors_by_category", stats)
        self.assertIn("errors_by_component", stats)
        self.assertIn("recovery_attempts", stats)
        self.assertIn("successful_recoveries", stats)
        self.assertIn("failed_recoveries", stats)
        self.assertIn("recovery_success_rate", stats)
        
        self.assertGreaterEqual(stats["total_errors"], 4)
    
    def test_recent_errors(self):
        """Test getting recent errors"""
        # Generate some errors
        for i in range(5):
            error = ValueError(f"Error {i}")
            self.error_handler.handle_error("test_component", error, ErrorSeverity.MEDIUM, ErrorCategory.SOFTWARE)
        
        # Get recent errors
        recent_errors = self.error_handler.get_recent_errors(3)
        
        self.assertLessEqual(len(recent_errors), 3)
        self.assertGreater(len(recent_errors), 0)
    
    def test_clear_error_history(self):
        """Test clearing error history"""
        # Generate some errors
        for i in range(3):
            error = ValueError(f"Error {i}")
            self.error_handler.handle_error("test_component", error, ErrorSeverity.MEDIUM, ErrorCategory.SOFTWARE)
        
        # Clear history
        self.error_handler.clear_error_history()
        
        # Check that history is cleared
        self.assertEqual(len(self.error_handler.error_history), 0)
    
    def test_alert_system(self):
        """Test alert system"""
        # Add alert callback
        alert_callback = Mock()
        self.error_handler.add_alert_callback(alert_callback)
        
        # Generate critical error to trigger alert
        error = Exception("Critical error")
        self.error_handler.handle_error("test_component", error, ErrorSeverity.CRITICAL, ErrorCategory.HARDWARE)
        
        # Check that alert callback was called
        self.assertTrue(alert_callback.called)
    
    def test_start_stop_monitoring(self):
        """Test start/stop monitoring"""
        # Test start
        self.error_handler.start_monitoring()
        self.assertTrue(self.error_handler.running)
        
        # Test stop
        self.error_handler.stop_monitoring()
        self.assertFalse(self.error_handler.running)
    
    def test_concurrent_error_handling(self):
        """Test concurrent error handling"""
        threads = []
        
        # Generate errors concurrently
        for i in range(5):
            thread = threading.Thread(
                target=self._generate_error,
                args=(f"thread_{i}",)
            )
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join(timeout=1.0)
        
        # Check that all errors were handled
        stats = self.error_handler.get_error_stats()
        self.assertGreaterEqual(stats["total_errors"], 5)
    
    def _generate_error(self, thread_name):
        """Helper method to generate errors"""
        error = Exception(f"Error from {thread_name}")
        self.error_handler.handle_error(thread_name, error, ErrorSeverity.MEDIUM, ErrorCategory.SOFTWARE)

class TestErrorHandlerDecorator(unittest.TestCase):
    """Test cases for error handler decorator"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.error_handler = HardwareErrorHandler()
    
    def test_error_handler_decorator_success(self):
        """Test error handler decorator with successful execution"""
        @error_handler_decorator("test_component", ErrorSeverity.MEDIUM, ErrorCategory.SOFTWARE)
        def test_function():
            return "success"
        
        result = test_function()
        self.assertEqual(result, "success")
    
    def test_error_handler_decorator_error(self):
        """Test error handler decorator with error"""
        @error_handler_decorator("test_component", ErrorSeverity.HIGH, ErrorCategory.SOFTWARE)
        def test_function():
            raise ValueError("Test error")
        
        with self.assertRaises(ValueError):
            test_function()
        
        # Check that error was handled
        stats = self.error_handler.get_error_stats()
        self.assertGreater(stats["total_errors"], 0)
    
    def test_error_handler_decorator_with_args(self):
        """Test error handler decorator with arguments"""
        @error_handler_decorator("test_component", ErrorSeverity.MEDIUM, ErrorCategory.SOFTWARE)
        def test_function(arg1, arg2, kwarg1=None):
            return arg1 + arg2
        
        result = test_function(1, 2, kwarg1="test")
        self.assertEqual(result, 3)
    
    def test_error_handler_decorator_with_error_and_args(self):
        """Test error handler decorator with error and arguments"""
        @error_handler_decorator("test_component", ErrorSeverity.HIGH, ErrorCategory.SOFTWARE)
        def test_function(arg1, arg2):
            raise RuntimeError(f"Error with args: {arg1}, {arg2}")
        
        with self.assertRaises(RuntimeError):
            test_function("arg1", "arg2")
        
        # Check that error was handled
        stats = self.error_handler.get_error_stats()
        self.assertGreater(stats["total_errors"], 0)

class TestErrorRecoveryStrategies(unittest.TestCase):
    """Test cases for error recovery strategies"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.error_handler = HardwareErrorHandler()
    
    def test_default_recovery_strategies(self):
        """Test default recovery strategies"""
        # Check that default strategies are initialized
        self.assertIn("gpio_recovery", self.error_handler.recovery_strategies)
        self.assertIn("dma_recovery", self.error_handler.recovery_strategies)
        self.assertIn("cpu_recovery", self.error_handler.recovery_strategies)
        self.assertIn("memory_recovery", self.error_handler.recovery_strategies)
    
    def test_custom_recovery_strategy(self):
        """Test custom recovery strategy"""
        recovery_called = False
        
        def custom_recovery_action(error_info):
            nonlocal recovery_called
            recovery_called = True
            return True
        
        strategy = RecoveryStrategy(
            strategy_id="custom_test",
            component="custom_component",
            error_patterns=["custom"],
            recovery_action=custom_recovery_action
        )
        
        self.error_handler.add_recovery_strategy(strategy)
        
        # Trigger error that matches pattern
        error = Exception("Custom error message")
        self.error_handler.handle_error("custom_component", error, ErrorSeverity.HIGH, ErrorCategory.SOFTWARE)
        
        # Check that recovery was called
        self.assertTrue(recovery_called)
    
    def test_recovery_strategy_with_callbacks(self):
        """Test recovery strategy with callbacks"""
        success_callback_called = False
        failure_callback_called = False
        
        def success_callback(error_info, strategy):
            nonlocal success_callback_called
            success_callback_called = True
        
        def failure_callback(error_info, strategy):
            nonlocal failure_callback_called
            failure_callback_called = True
        
        def recovery_action(error_info):
            return True  # Always succeed
        
        strategy = RecoveryStrategy(
            strategy_id="callback_test",
            component="callback_component",
            error_patterns=["callback"],
            recovery_action=recovery_action,
            success_callback=success_callback,
            failure_callback=failure_callback
        )
        
        self.error_handler.add_recovery_strategy(strategy)
        
        # Trigger error
        error = Exception("Callback error")
        self.error_handler.handle_error("callback_component", error, ErrorSeverity.HIGH, ErrorCategory.SOFTWARE)
        
        # Check that success callback was called
        self.assertTrue(success_callback_called)
        self.assertFalse(failure_callback_called)

if __name__ == '__main__':
    # Set up logging
    logging.basicConfig(level=logging.INFO)
    
    # Run tests
    unittest.main(verbosity=2)
