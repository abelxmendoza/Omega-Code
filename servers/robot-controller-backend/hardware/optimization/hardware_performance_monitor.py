"""
Hardware Performance Monitor for Robot Controller
- Real-time hardware performance tracking
- GPIO timing optimization
- Motor control performance
- Sensor response time monitoring
- Camera performance metrics
- System resource utilization
"""

import time
import psutil
import threading
import logging
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass, field
from collections import deque
from enum import Enum
import asyncio

logger = logging.getLogger(__name__)

class HardwareComponent(Enum):
    """Hardware component types"""
    MOTOR = "motor"
    SERVO = "servo"
    ULTRASONIC = "ultrasonic"
    LINE_TRACKER = "line_tracker"
    CAMERA = "camera"
    BUZZER = "buzzer"
    LED = "led"
    GPIO = "gpio"

@dataclass
class HardwareMetrics:
    """Hardware performance metrics"""
    component: HardwareComponent
    operation: str
    start_time: float
    end_time: float
    duration: float
    success: bool
    error_message: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

@dataclass
class SystemMetrics:
    """System-level hardware metrics"""
    timestamp: float
    cpu_usage: float
    memory_usage: float
    gpu_memory: float
    temperature: float
    voltage: float
    frequency: float
    gpio_pin_states: Dict[int, bool] = field(default_factory=dict)

class HardwarePerformanceMonitor:
    """Real-time hardware performance monitoring"""
    
    def __init__(self, history_size: int = 1000):
        self.history_size = history_size
        self.metrics_history = deque(maxlen=history_size)
        self.system_metrics_history = deque(maxlen=history_size)
        self.component_stats = {}
        self.monitoring = False
        self.monitor_task = None
        self.lock = threading.Lock()
        
        # Performance thresholds
        self.thresholds = {
            HardwareComponent.MOTOR: {"max_duration": 0.1, "max_error_rate": 0.05},
            HardwareComponent.SERVO: {"max_duration": 0.05, "max_error_rate": 0.02},
            HardwareComponent.ULTRASONIC: {"max_duration": 0.2, "max_error_rate": 0.1},
            HardwareComponent.LINE_TRACKER: {"max_duration": 0.01, "max_error_rate": 0.01},
            HardwareComponent.CAMERA: {"max_duration": 0.033, "max_error_rate": 0.05},  # 30 FPS
            HardwareComponent.GPIO: {"max_duration": 0.001, "max_error_rate": 0.001},
        }
    
    async def start_monitoring(self, interval: float = 1.0):
        """Start hardware performance monitoring"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitor_task = asyncio.create_task(self._monitor_loop(interval))
        logger.info("Hardware performance monitoring started")
    
    async def stop_monitoring(self):
        """Stop hardware performance monitoring"""
        if not self.monitoring:
            return
        
        self.monitoring = False
        if self.monitor_task:
            self.monitor_task.cancel()
        logger.info("Hardware performance monitoring stopped")
    
    async def _monitor_loop(self, interval: float):
        """Main monitoring loop"""
        while self.monitoring:
            try:
                metrics = await self._collect_system_metrics()
                self.system_metrics_history.append(metrics)
                await self._check_performance_alerts()
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Hardware monitoring error: {e}")
                await asyncio.sleep(interval)
    
    async def _collect_system_metrics(self) -> SystemMetrics:
        """Collect system-level hardware metrics"""
        try:
            # CPU and memory
            cpu_usage = psutil.cpu_percent()
            memory = psutil.virtual_memory()
            
            # GPU memory (Raspberry Pi specific)
            gpu_memory = 0
            try:
                with open('/sys/kernel/debug/mali0/gpu_memory', 'r') as f:
                    gpu_memory = float(f.read().strip())
            except:
                pass
            
            # Temperature
            temperature = 0
            try:
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                    temperature = float(f.read().strip()) / 1000
            except:
                pass
            
            # Voltage
            voltage = 0
            try:
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                    voltage = float(f.read().strip()) / 1000
            except:
                pass
            
            # CPU frequency
            frequency = 0
            try:
                with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq', 'r') as f:
                    frequency = float(f.read().strip()) / 1000
            except:
                pass
            
            # GPIO pin states (simplified)
            gpio_states = {}
            
            return SystemMetrics(
                timestamp=time.time(),
                cpu_usage=cpu_usage,
                memory_usage=memory.percent,
                gpu_memory=gpu_memory,
                temperature=temperature,
                voltage=voltage,
                frequency=frequency,
                gpio_pin_states=gpio_states
            )
        except Exception as e:
            logger.error(f"Error collecting system metrics: {e}")
            return SystemMetrics(
                timestamp=time.time(),
                cpu_usage=0, memory_usage=0, gpu_memory=0,
                temperature=0, voltage=0, frequency=0
            )
    
    def record_operation(self, component: HardwareComponent, operation: str, 
                        duration: float, success: bool, error_message: str = None,
                        metadata: Dict[str, Any] = None):
        """Record a hardware operation"""
        with self.lock:
            metric = HardwareMetrics(
                component=component,
                operation=operation,
                start_time=time.time() - duration,
                end_time=time.time(),
                duration=duration,
                success=success,
                error_message=error_message,
                metadata=metadata or {}
            )
            
            self.metrics_history.append(metric)
            
            # Update component statistics
            if component not in self.component_stats:
                self.component_stats[component] = {
                    "total_operations": 0,
                    "successful_operations": 0,
                    "failed_operations": 0,
                    "total_duration": 0,
                    "avg_duration": 0,
                    "max_duration": 0,
                    "min_duration": float('inf')
                }
            
            stats = self.component_stats[component]
            stats["total_operations"] += 1
            stats["total_duration"] += duration
            
            if success:
                stats["successful_operations"] += 1
            else:
                stats["failed_operations"] += 1
            
            stats["avg_duration"] = stats["total_duration"] / stats["total_operations"]
            stats["max_duration"] = max(stats["max_duration"], duration)
            stats["min_duration"] = min(stats["min_duration"], duration)
    
    async def _check_performance_alerts(self):
        """Check for performance alerts"""
        for component, stats in self.component_stats.items():
            if component in self.thresholds:
                threshold = self.thresholds[component]
                
                # Check duration threshold
                if stats["avg_duration"] > threshold["max_duration"]:
                    logger.warning(f"Hardware alert: {component.value} average duration "
                                 f"{stats['avg_duration']:.3f}s exceeds threshold "
                                 f"{threshold['max_duration']:.3f}s")
                
                # Check error rate threshold
                if stats["total_operations"] > 0:
                    error_rate = stats["failed_operations"] / stats["total_operations"]
                    if error_rate > threshold["max_error_rate"]:
                        logger.warning(f"Hardware alert: {component.value} error rate "
                                     f"{error_rate:.3f} exceeds threshold "
                                     f"{threshold['max_error_rate']:.3f}")
    
    def get_performance_report(self) -> Dict[str, Any]:
        """Get comprehensive performance report"""
        with self.lock:
            report = {
                "component_stats": dict(self.component_stats),
                "system_metrics": self.system_metrics_history[-1] if self.system_metrics_history else None,
                "total_operations": len(self.metrics_history),
                "monitoring_active": self.monitoring
            }
            
            # Add performance scores
            report["performance_scores"] = {}
            for component, stats in self.component_stats.items():
                if stats["total_operations"] > 0:
                    success_rate = stats["successful_operations"] / stats["total_operations"]
                    avg_duration = stats["avg_duration"]
                    
                    # Calculate performance score (0-100)
                    duration_score = max(0, 100 - (avg_duration * 1000))  # Penalize high duration
                    success_score = success_rate * 100
                    performance_score = (duration_score + success_score) / 2
                    
                    report["performance_scores"][component.value] = {
                        "score": performance_score,
                        "success_rate": success_rate,
                        "avg_duration": avg_duration,
                        "status": "excellent" if performance_score > 90 else
                                 "good" if performance_score > 70 else
                                 "fair" if performance_score > 50 else "poor"
                    }
            
            return report
    
    def get_component_stats(self, component: HardwareComponent) -> Dict[str, Any]:
        """Get statistics for a specific component"""
        return self.component_stats.get(component, {})
    
    def get_recent_metrics(self, component: HardwareComponent = None, 
                          count: int = 100) -> List[HardwareMetrics]:
        """Get recent metrics for a component or all components"""
        with self.lock:
            if component:
                return [m for m in self.metrics_history if m.component == component][-count:]
            return list(self.metrics_history)[-count:]

# Hardware operation decorator
def monitor_hardware_operation(component: HardwareComponent, operation: str = None):
    """Decorator to monitor hardware operations"""
    def decorator(func: Callable) -> Callable:
        async def async_wrapper(*args, **kwargs):
            start_time = time.time()
            success = False
            error_message = None
            
            try:
                result = await func(*args, **kwargs)
                success = True
                return result
            except Exception as e:
                error_message = str(e)
                raise
            finally:
                duration = time.time() - start_time
                hardware_monitor.record_operation(
                    component, operation or func.__name__, duration, success, error_message
                )
        
        def sync_wrapper(*args, **kwargs):
            start_time = time.time()
            success = False
            error_message = None
            
            try:
                result = func(*args, **kwargs)
                success = True
                return result
            except Exception as e:
                error_message = str(e)
                raise
            finally:
                duration = time.time() - start_time
                hardware_monitor.record_operation(
                    component, operation or func.__name__, duration, success, error_message
                )
        
        if asyncio.iscoroutinefunction(func):
            return async_wrapper
        else:
            return sync_wrapper
    
    return decorator

# Global hardware monitor instance
hardware_monitor = HardwarePerformanceMonitor()
