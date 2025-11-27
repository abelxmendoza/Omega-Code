"""
Comprehensive Test Runner for Hardware Optimizations
- Runs all optimization tests
- Generates test reports
- Performance benchmarks
- Coverage analysis
"""

import unittest
import time
import logging
import sys
import os
from io import StringIO
import json

# Add the current directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import all test modules
from test_dma_accelerator import TestDMAAccelerator, TestDMAErrorHandling
from test_cpu_accelerator import TestCPUAccelerator, TestCPUErrorHandling, TestCPUPerformance
from test_error_handler import TestHardwareErrorHandler, TestErrorHandlerDecorator, TestErrorRecoveryStrategies

class TestReporter:
    """Test reporter for generating comprehensive reports"""
    
    def __init__(self):
        self.results = {
            "test_suites": {},
            "total_tests": 0,
            "passed_tests": 0,
            "failed_tests": 0,
            "error_tests": 0,
            "start_time": 0,
            "end_time": 0,
            "duration": 0
        }
    
    def start_test_run(self):
        """Start test run"""
        self.results["start_time"] = time.time()
        print("🚀 Starting Comprehensive Hardware Optimization Tests")
        print("=" * 60)
    
    def end_test_run(self):
        """End test run"""
        self.results["end_time"] = time.time()
        self.results["duration"] = self.results["end_time"] - self.results["start_time"]
        
        print("\n" + "=" * 60)
        print("📊 Test Run Summary")
        print("=" * 60)
        print(f"Total Tests: {self.results['total_tests']}")
        print(f"Passed: {self.results['passed_tests']}")
        print(f"Failed: {self.results['failed_tests']}")
        print(f"Errors: {self.results['error_tests']}")
        print(f"Duration: {self.results['duration']:.2f} seconds")
        
        if self.results['total_tests'] > 0:
            success_rate = (self.results['passed_tests'] / self.results['total_tests']) * 100
            print(f"Success Rate: {success_rate:.1f}%")
        
        print("=" * 60)
    
    def report_suite_result(self, suite_name, result):
        """Report test suite result"""
        self.results["test_suites"][suite_name] = {
            "tests_run": result.testsRun,
            "failures": len(result.failures),
            "errors": len(result.errors),
            "success": result.wasSuccessful()
        }
        
        self.results["total_tests"] += result.testsRun
        self.results["passed_tests"] += result.testsRun - len(result.failures) - len(result.errors)
        self.results["failed_tests"] += len(result.failures)
        self.results["error_tests"] += len(result.errors)
        
        status = "✅ PASSED" if result.wasSuccessful() else "❌ FAILED"
        print(f"\n📋 {suite_name}: {status}")
        print(f"   Tests: {result.testsRun}, Failures: {len(result.failures)}, Errors: {len(result.errors)}")
        
        if result.failures:
            print("   Failures:")
            for test, traceback in result.failures:
                print(f"     - {test}: {traceback.split('AssertionError:')[-1].strip()}")
        
        if result.errors:
            print("   Errors:")
            for test, traceback in result.errors:
                print(f"     - {test}: {traceback.split('Exception:')[-1].strip()}")
    
    def save_report(self, filename="test_report.json"):
        """Save test report to file"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.results, f, indent=2)
            print(f"\n📄 Test report saved to: {filename}")
        except Exception as e:
            print(f"❌ Failed to save test report: {e}")

def run_comprehensive_tests():
    """Run all comprehensive tests"""
    reporter = TestReporter()
    reporter.start_test_run()
    
    # Test suites to run
    test_suites = [
        ("DMA Accelerator Tests", unittest.TestLoader().loadTestsFromTestCase(TestDMAAccelerator)),
        ("DMA Error Handling Tests", unittest.TestLoader().loadTestsFromTestCase(TestDMAErrorHandling)),
        ("CPU Accelerator Tests", unittest.TestLoader().loadTestsFromTestCase(TestCPUAccelerator)),
        ("CPU Error Handling Tests", unittest.TestLoader().loadTestsFromTestCase(TestCPUErrorHandling)),
        ("CPU Performance Tests", unittest.TestLoader().loadTestsFromTestCase(TestCPUPerformance)),
        ("Error Handler Tests", unittest.TestLoader().loadTestsFromTestCase(TestHardwareErrorHandler)),
        ("Error Handler Decorator Tests", unittest.TestLoader().loadTestsFromTestCase(TestErrorHandlerDecorator)),
        ("Error Recovery Strategy Tests", unittest.TestLoader().loadTestsFromTestCase(TestErrorRecoveryStrategies))
    ]
    
    # Run each test suite
    for suite_name, suite in test_suites:
        print(f"\n🧪 Running {suite_name}...")
        result = unittest.TextTestRunner(verbosity=0, stream=StringIO()).run(suite)
        reporter.report_suite_result(suite_name, result)
    
    reporter.end_test_run()
    reporter.save_report()
    
    return reporter.results

def run_performance_benchmarks():
    """Run performance benchmarks"""
    print("\n🏃 Running Performance Benchmarks")
    print("=" * 40)
    
    # Import modules for benchmarking
    from dma_accelerator import DMAAccelerator, DMAOperation, DMAMode
    from cpu_accelerator import CPUAccelerator, CPUComputeTask
    from error_handler import HardwareErrorHandler, ErrorSeverity, ErrorCategory
    
    benchmarks = {}
    
    # DMA Performance Benchmark
    print("📊 DMA Performance Benchmark...")
    dma_accelerator = DMAAccelerator()
    start_time = time.time()
    
    for i in range(100):
        operation = DMAOperation(
            operation_id=f"benchmark_{i}",
            mode=DMAMode.MEMORY_TO_MEMORY,
            source_address=0x1000 + i * 10,
            destination_address=0x2000 + i * 10,
            transfer_size=64,
            priority=0
        )
        dma_accelerator.start_dma_transfer(operation)
    
    dma_time = time.time() - start_time
    benchmarks["dma_100_transfers"] = dma_time
    print(f"   100 DMA transfers: {dma_time:.3f}s")
    
    # CPU Performance Benchmark
    print("📊 CPU Performance Benchmark...")
    cpu_accelerator = CPUAccelerator()
    start_time = time.time()
    
    for i in range(50):
        task = CPUComputeTask(
            task_id=f"benchmark_{i}",
            operation="matrix_multiply",
            input_data={"matrix_a": [[1, 2], [3, 4]], "matrix_b": [[5, 6], [7, 8]]},
            output_format="array"
        )
        cpu_accelerator.submit_compute_task(task)
    
    cpu_time = time.time() - start_time
    benchmarks["cpu_50_tasks"] = cpu_time
    print(f"   50 CPU tasks: {cpu_time:.3f}s")
    
    # Error Handling Performance Benchmark
    print("📊 Error Handling Performance Benchmark...")
    error_handler = HardwareErrorHandler()
    start_time = time.time()
    
    for i in range(200):
        error = Exception(f"Benchmark error {i}")
        error_handler.handle_error("benchmark", error, ErrorSeverity.MEDIUM, ErrorCategory.SOFTWARE)
    
    error_time = time.time() - start_time
    benchmarks["error_200_handles"] = error_time
    print(f"   200 error handles: {error_time:.3f}s")
    
    print("\n📈 Performance Summary:")
    for benchmark, time_taken in benchmarks.items():
        print(f"   {benchmark}: {time_taken:.3f}s")
    
    return benchmarks

def run_coverage_analysis():
    """Run coverage analysis"""
    print("\n📊 Coverage Analysis")
    print("=" * 30)
    
    # This is a simplified coverage analysis
    # In a real implementation, you would use coverage.py
    
    modules = [
        "dma_accelerator",
        "cpu_accelerator", 
        "error_handler",
        "predictive_analytics",
        "pi4b_optimizer",
        "zero_latency_access"
    ]
    
    print("📋 Module Coverage:")
    for module in modules:
        try:
            # Try to import module
            __import__(module)
            print(f"   ✅ {module}: Available")
        except ImportError as e:
            print(f"   ❌ {module}: {e}")
    
    print("\n📋 Test Coverage:")
    test_modules = [
        "test_dma_accelerator",
        "test_cpu_accelerator",
        "test_error_handler"
    ]
    
    for test_module in test_modules:
        try:
            __import__(test_module)
            print(f"   ✅ {test_module}: Available")
        except ImportError as e:
            print(f"   ❌ {test_module}: {e}")

def main():
    """Main test runner"""
    print("🧪 Hardware Optimization Test Suite")
    print("=" * 50)
    
    # Set up logging
    logging.basicConfig(level=logging.WARNING)  # Reduce log noise during tests
    
    try:
        # Run comprehensive tests
        test_results = run_comprehensive_tests()
        
        # Run performance benchmarks
        benchmarks = run_performance_benchmarks()
        
        # Run coverage analysis
        run_coverage_analysis()
        
        # Final summary
        print("\n🎯 Final Summary")
        print("=" * 20)
        
        if test_results["total_tests"] > 0:
            success_rate = (test_results["passed_tests"] / test_results["total_tests"]) * 100
            if success_rate >= 90:
                print("🏆 Excellent! Test success rate >= 90%")
            elif success_rate >= 80:
                print("✅ Good! Test success rate >= 80%")
            elif success_rate >= 70:
                print("⚠️  Fair! Test success rate >= 70%")
            else:
                print("❌ Needs improvement! Test success rate < 70%")
        
        print(f"📊 Total Tests: {test_results['total_tests']}")
        print(f"✅ Passed: {test_results['passed_tests']}")
        print(f"❌ Failed: {test_results['failed_tests']}")
        print(f"⏱️  Duration: {test_results['duration']:.2f}s")
        
        # Performance summary
        print(f"\n🏃 Performance Benchmarks:")
        for benchmark, time_taken in benchmarks.items():
            print(f"   {benchmark}: {time_taken:.3f}s")
        
        print("\n🎉 Test suite completed!")
        
    except Exception as e:
        print(f"❌ Test suite failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
