"""
Comprehensive Tests for CPU Accelerator
- Unit tests for all CPU functionality
- Error handling tests
- Performance tests
- Integration tests
"""

import unittest
import time
import threading
import logging
from unittest.mock import Mock, patch, MagicMock
import sys
import os
import numpy as np

# Add the parent directory to the path to import the modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from cpu_accelerator import CPUAccelerator, CPUComputeTask, CPUOptimization
from error_handler import HardwareErrorHandler, ErrorSeverity, ErrorCategory

class TestCPUAccelerator(unittest.TestCase):
    """Test cases for CPU Accelerator"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.cpu_accelerator = CPUAccelerator()
        self.error_handler = HardwareErrorHandler()
        
    def tearDown(self):
        """Clean up after tests"""
        if hasattr(self.cpu_accelerator, 'stop'):
            self.cpu_accelerator.stop()
    
    def test_cpu_accelerator_initialization(self):
        """Test CPU accelerator initialization"""
        self.assertIsNotNone(self.cpu_accelerator)
        self.assertGreater(self.cpu_accelerator.cpu_cores, 0)
        self.assertIsNotNone(self.cpu_accelerator.executor)
        self.assertFalse(self.cpu_accelerator.running)
    
    def test_cpu_compute_task_creation(self):
        """Test CPU compute task creation"""
        task = CPUComputeTask(
            task_id="test_task",
            operation="matrix_multiply",
            input_data={"matrix_a": [[1, 2], [3, 4]], "matrix_b": [[5, 6], [7, 8]]},
            output_format="array",
            priority=1,
            core_affinity=0
        )
        
        self.assertEqual(task.task_id, "test_task")
        self.assertEqual(task.operation, "matrix_multiply")
        self.assertEqual(task.priority, 1)
        self.assertEqual(task.core_affinity, 0)
    
    def test_submit_compute_task_success(self):
        """Test successful compute task submission"""
        task = CPUComputeTask(
            task_id="test_submit",
            operation="matrix_multiply",
            input_data={"matrix_a": [[1, 2], [3, 4]], "matrix_b": [[5, 6], [7, 8]]},
            output_format="array"
        )
        
        task_id = self.cpu_accelerator.submit_compute_task(task)
        self.assertEqual(task_id, "test_submit")
        
        # Check that task is tracked
        self.assertGreater(self.cpu_accelerator.performance_stats["total_tasks"], 0)
    
    def test_matrix_multiply_operation(self):
        """Test matrix multiplication operation"""
        data = {
            "matrix_a": [[1, 2], [3, 4]],
            "matrix_b": [[5, 6], [7, 8]]
        }
        
        result = self.cpu_accelerator._matrix_multiply(data)
        self.assertIsNotNone(result)
        self.assertIsInstance(result, list)
        
        # Expected result: [[19, 22], [43, 50]]
        expected = [[19, 22], [43, 50]]
        self.assertEqual(result, expected)
    
    def test_image_process_operation(self):
        """Test image processing operation"""
        # Create test image data
        image_data = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16]]
        
        data = {
            "image_data": image_data,
            "operation": "resize"
        }
        
        result = self.cpu_accelerator._image_process(data)
        self.assertIsNotNone(result)
        self.assertIsInstance(result, list)
        
        # Should be downsampled (every other pixel)
        self.assertLess(len(result), len(image_data))
    
    def test_sensor_fusion_operation(self):
        """Test sensor fusion operation"""
        data = {
            "sensor_data": [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
        }
        
        result = self.cpu_accelerator._sensor_fusion(data)
        self.assertIsNotNone(result)
        self.assertIsInstance(result, list)
        
        # Should be fused data
        self.assertEqual(len(result), 3)
    
    def test_data_compression_operation(self):
        """Test data compression operation"""
        data = {
            "input_data": [1, 0, 2, 0, 3, 0, 4],
            "compression_type": "simple"
        }
        
        result = self.cpu_accelerator._data_compression(data)
        self.assertIsNotNone(result)
        self.assertIsInstance(result, list)
        
        # Should remove zeros
        self.assertNotIn(0, result)
    
    def test_generic_compute_operation(self):
        """Test generic compute operation"""
        # Test with list
        result = self.cpu_accelerator._generic_compute([1, 2, 3])
        self.assertEqual(result, [2, 4, 6])
        
        # Test with number
        result = self.cpu_accelerator._generic_compute(5)
        self.assertEqual(result, 10)
        
        # Test with string
        result = self.cpu_accelerator._generic_compute("test")
        self.assertEqual(result, "test")
    
    def test_performance_stats(self):
        """Test CPU performance statistics"""
        # Submit some tasks
        for i in range(3):
            task = CPUComputeTask(
                task_id=f"perf_test_{i}",
                operation="matrix_multiply",
                input_data={"matrix_a": [[1, 2], [3, 4]], "matrix_b": [[5, 6], [7, 8]]},
                output_format="array"
            )
            self.cpu_accelerator.submit_compute_task(task)
            time.sleep(0.01)  # Small delay
        
        # Get performance stats
        stats = self.cpu_accelerator.get_performance_stats()
        
        self.assertIn("total_tasks", stats)
        self.assertIn("completed_tasks", stats)
        self.assertIn("failed_tasks", stats)
        self.assertIn("avg_execution_time", stats)
        self.assertIn("performance_score", stats)
        self.assertIn("cpu_cores", stats)
        self.assertIn("max_workers", stats)
        
        self.assertGreaterEqual(stats["total_tasks"], 3)
    
    def test_error_handling_invalid_task(self):
        """Test error handling with invalid task"""
        # Test with invalid task data
        task = CPUComputeTask(
            task_id="invalid_task",
            operation="matrix_multiply",
            input_data=None,  # Invalid data
            output_format="array"
        )
        
        task_id = self.cpu_accelerator.submit_compute_task(task)
        self.assertEqual(task_id, "invalid_task")
        
        # Should handle gracefully
        time.sleep(0.1)  # Allow task to complete
        stats = self.cpu_accelerator.get_performance_stats()
        self.assertGreaterEqual(stats["total_tasks"], 1)
    
    def test_error_handling_matrix_error(self):
        """Test error handling with matrix operation errors"""
        # Test with invalid matrix data
        data = {
            "matrix_a": [[1, 2]],  # 1x2 matrix
            "matrix_b": [[5, 6], [7, 8]]  # 2x2 matrix - incompatible
        }
        
        result = self.cpu_accelerator._matrix_multiply(data)
        # Should handle error gracefully
        self.assertIsNone(result)
    
    def test_error_handling_image_error(self):
        """Test error handling with image processing errors"""
        # Test with invalid image data
        data = {
            "image_data": None,  # Invalid data
            "operation": "resize"
        }
        
        result = self.cpu_accelerator._image_process(data)
        # Should handle error gracefully
        self.assertIsNone(result)
    
    def test_concurrent_tasks(self):
        """Test concurrent CPU tasks"""
        tasks = []
        threads = []
        
        # Create multiple tasks
        for i in range(4):
            task = CPUComputeTask(
                task_id=f"concurrent_test_{i}",
                operation="matrix_multiply",
                input_data={"matrix_a": [[1, 2], [3, 4]], "matrix_b": [[5, 6], [7, 8]]},
                output_format="array"
            )
            tasks.append(task)
        
        # Submit tasks concurrently
        for task in tasks:
            thread = threading.Thread(
                target=self.cpu_accelerator.submit_compute_task,
                args=(task,)
            )
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join(timeout=1.0)
        
        # Check that all tasks were submitted
        stats = self.cpu_accelerator.get_performance_stats()
        self.assertGreaterEqual(stats["total_tasks"], 4)
    
    def test_cpu_optimization_settings(self):
        """Test CPU optimization settings"""
        # Test SIMD settings
        self.assertIsInstance(self.cpu_accelerator.simd_enabled, bool)
        
        # Test cache optimization
        self.assertIsInstance(self.cpu_accelerator.cache_optimized, bool)
        
        # Test frequency scaling
        self.assertIsInstance(self.cpu_accelerator.frequency_scaling, bool)
    
    def test_start_stop_functionality(self):
        """Test CPU accelerator start/stop functionality"""
        # Test start
        self.cpu_accelerator.start()
        self.assertTrue(self.cpu_accelerator.running)
        
        # Test stop
        self.cpu_accelerator.stop()
        self.assertFalse(self.cpu_accelerator.running)
    
    def test_executor_shutdown(self):
        """Test executor shutdown"""
        # Start accelerator
        self.cpu_accelerator.start()
        
        # Submit a task
        task = CPUComputeTask(
            task_id="shutdown_test",
            operation="matrix_multiply",
            input_data={"matrix_a": [[1, 2], [3, 4]], "matrix_b": [[5, 6], [7, 8]]},
            output_format="array"
        )
        self.cpu_accelerator.submit_compute_task(task)
        
        # Stop accelerator
        self.cpu_accelerator.stop()
        
        # Executor should be shut down
        self.assertTrue(self.cpu_accelerator.executor._shutdown)

class TestCPUErrorHandling(unittest.TestCase):
    """Test cases for CPU error handling"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.cpu_accelerator = CPUAccelerator()
        self.error_handler = HardwareErrorHandler()
    
    def test_error_handler_integration(self):
        """Test error handler integration with CPU"""
        # Test that error handler is properly integrated
        self.assertIsNotNone(self.error_handler)
        
        # Test error handling
        try:
            raise ValueError("Test CPU error")
        except Exception as e:
            result = self.error_handler.handle_error("cpu", e, ErrorSeverity.MEDIUM, ErrorCategory.SOFTWARE)
            self.assertIsInstance(result, bool)
    
    def test_cpu_recovery_strategies(self):
        """Test CPU recovery strategies"""
        # Test CPU recovery
        try:
            raise Exception("CPU frequency error")
        except Exception as e:
            result = self.error_handler.handle_error("cpu", e, ErrorSeverity.HIGH, ErrorCategory.HARDWARE)
            self.assertIsInstance(result, bool)
    
    def test_performance_monitoring(self):
        """Test performance monitoring with errors"""
        # Generate some errors
        for i in range(3):
            try:
                raise Exception(f"CPU test error {i}")
            except Exception as e:
                self.error_handler.handle_error("cpu", e, ErrorSeverity.MEDIUM, ErrorCategory.SOFTWARE)
        
        # Check statistics
        stats = self.error_handler.get_error_stats()
        self.assertIn("total_errors", stats)
        self.assertIn("errors_by_component", stats)
        
        self.assertGreaterEqual(stats["total_errors"], 3)

class TestCPUPerformance(unittest.TestCase):
    """Test cases for CPU performance"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.cpu_accelerator = CPUAccelerator()
    
    def tearDown(self):
        """Clean up after tests"""
        if hasattr(self.cpu_accelerator, 'stop'):
            self.cpu_accelerator.stop()
    
    def test_matrix_multiply_performance(self):
        """Test matrix multiplication performance"""
        # Create larger matrices for performance test
        size = 100
        matrix_a = [[i + j for j in range(size)] for i in range(size)]
        matrix_b = [[i * j for j in range(size)] for i in range(size)]
        
        data = {
            "matrix_a": matrix_a,
            "matrix_b": matrix_b
        }
        
        start_time = time.time()
        result = self.cpu_accelerator._matrix_multiply(data)
        execution_time = time.time() - start_time
        
        self.assertIsNotNone(result)
        self.assertLess(execution_time, 5.0)  # Should complete within 5 seconds
    
    def test_image_process_performance(self):
        """Test image processing performance"""
        # Create larger image for performance test
        size = 200
        image_data = [[i + j for j in range(size)] for i in range(size)]
        
        data = {
            "image_data": image_data,
            "operation": "resize"
        }
        
        start_time = time.time()
        result = self.cpu_accelerator._image_process(data)
        execution_time = time.time() - start_time
        
        self.assertIsNotNone(result)
        self.assertLess(execution_time, 2.0)  # Should complete within 2 seconds
    
    def test_sensor_fusion_performance(self):
        """Test sensor fusion performance"""
        # Create larger sensor data for performance test
        num_sensors = 10
        sensor_data = [[i + j for j in range(100)] for i in range(num_sensors)]
        
        data = {
            "sensor_data": sensor_data
        }
        
        start_time = time.time()
        result = self.cpu_accelerator._sensor_fusion(data)
        execution_time = time.time() - start_time
        
        self.assertIsNotNone(result)
        self.assertLess(execution_time, 1.0)  # Should complete within 1 second

if __name__ == '__main__':
    # Set up logging
    logging.basicConfig(level=logging.INFO)
    
    # Run tests
    unittest.main(verbosity=2)
