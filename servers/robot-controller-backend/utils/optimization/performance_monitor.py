"""
Performance monitoring system for robot controller.
- Real-time performance metrics
- Resource usage monitoring
- Performance alerts and notifications
- Historical data tracking
"""

import time
import psutil
import asyncio
import logging
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field
from collections import deque
import threading

logger = logging.getLogger(__name__)

@dataclass
class PerformanceMetrics:
    """Performance metrics data structure"""
    timestamp: float
    cpu_usage: float
    memory_usage: float
    memory_percent: float
    disk_usage: float
    network_io: Dict[str, int]
    websocket_connections: int
    response_time: float
    error_rate: float
    throughput: float

class PerformanceMonitor:
    """Real-time performance monitoring"""
    
    def __init__(self, history_size: int = 1000):
        self.history_size = history_size
        self.metrics_history = deque(maxlen=history_size)
        self.alerts = []
        self.thresholds = {
            "cpu_usage": 80.0,
            "memory_percent": 85.0,
            "disk_usage": 90.0,
            "response_time": 1.0,
            "error_rate": 5.0
        }
        self.monitoring = False
        self.monitor_task = None
    
    async def start_monitoring(self, interval: float = 5.0):
        """Start performance monitoring"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitor_task = asyncio.create_task(self._monitor_loop(interval))
        logger.info("Performance monitoring started")
    
    async def stop_monitoring(self):
        """Stop performance monitoring"""
        if not self.monitoring:
            return
        
        self.monitoring = False
        if self.monitor_task:
            self.monitor_task.cancel()
        logger.info("Performance monitoring stopped")
    
    async def _monitor_loop(self, interval: float):
        """Main monitoring loop"""
        while self.monitoring:
            try:
                metrics = await self._collect_metrics()
                self.metrics_history.append(metrics)
                await self._check_alerts(metrics)
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Performance monitoring error: {e}")
                await asyncio.sleep(interval)
    
    async def _collect_metrics(self) -> PerformanceMetrics:
        """Collect current performance metrics"""
        # System metrics
        cpu_usage = psutil.cpu_percent()
        memory = psutil.virtual_memory()
        disk = psutil.disk_usage('/')
        network = psutil.net_io_counters()
        
        # Application metrics (would be populated by actual app)
        websocket_connections = 0  # To be implemented
        response_time = 0.0  # To be implemented
        error_rate = 0.0  # To be implemented
        throughput = 0.0  # To be implemented
        
        return PerformanceMetrics(
            timestamp=time.time(),
            cpu_usage=cpu_usage,
            memory_usage=memory.used / (1024 * 1024),  # MB
            memory_percent=memory.percent,
            disk_usage=disk.percent,
            network_io={
                "bytes_sent": network.bytes_sent,
                "bytes_recv": network.bytes_recv
            },
            websocket_connections=websocket_connections,
            response_time=response_time,
            error_rate=error_rate,
            throughput=throughput
        )
    
    async def _check_alerts(self, metrics: PerformanceMetrics):
        """Check for performance alerts"""
        alerts = []
        
        if metrics.cpu_usage > self.thresholds["cpu_usage"]:
            alerts.append(f"High CPU usage: {metrics.cpu_usage:.1f}%")
        
        if metrics.memory_percent > self.thresholds["memory_percent"]:
            alerts.append(f"High memory usage: {metrics.memory_percent:.1f}%")
        
        if metrics.disk_usage > self.thresholds["disk_usage"]:
            alerts.append(f"High disk usage: {metrics.disk_usage:.1f}%")
        
        if metrics.response_time > self.thresholds["response_time"]:
            alerts.append(f"High response time: {metrics.response_time:.3f}s")
        
        if metrics.error_rate > self.thresholds["error_rate"]:
            alerts.append(f"High error rate: {metrics.error_rate:.1f}%")
        
        if alerts:
            self.alerts.extend(alerts)
            for alert in alerts:
                logger.warning(f"Performance Alert: {alert}")
    
    def get_current_metrics(self) -> Optional[PerformanceMetrics]:
        """Get current performance metrics"""
        return self.metrics_history[-1] if self.metrics_history else None
    
    def get_historical_metrics(self, minutes: int = 60) -> List[PerformanceMetrics]:
        """Get historical metrics for specified minutes"""
        cutoff_time = time.time() - (minutes * 60)
        return [m for m in self.metrics_history if m.timestamp >= cutoff_time]
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get performance summary"""
        if not self.metrics_history:
            return {"status": "No metrics available"}
        
        recent_metrics = self.get_historical_metrics(5)  # Last 5 minutes
        
        if not recent_metrics:
            return {"status": "No recent metrics"}
        
        # Calculate averages
        avg_cpu = sum(m.cpu_usage for m in recent_metrics) / len(recent_metrics)
        avg_memory = sum(m.memory_percent for m in recent_metrics) / len(recent_metrics)
        avg_response_time = sum(m.response_time for m in recent_metrics) / len(recent_metrics)
        
        return {
            "current_cpu": recent_metrics[-1].cpu_usage,
            "current_memory": recent_metrics[-1].memory_percent,
            "current_disk": recent_metrics[-1].disk_usage,
            "avg_cpu_5min": avg_cpu,
            "avg_memory_5min": avg_memory,
            "avg_response_time_5min": avg_response_time,
            "active_alerts": len(self.alerts),
            "total_metrics": len(self.metrics_history)
        }

class ApplicationProfiler:
    """Application-specific performance profiler"""
    
    def __init__(self):
        self.operation_times = {}
        self.request_counts = {}
        self.error_counts = {}
    
    def start_operation(self, operation_name: str) -> str:
        """Start timing an operation"""
        operation_id = f"{operation_name}_{int(time.time() * 1000)}"
        self.operation_times[operation_id] = time.time()
        return operation_id
    
    def end_operation(self, operation_id: str) -> float:
        """End timing an operation"""
        if operation_id in self.operation_times:
            duration = time.time() - self.operation_times[operation_id]
            del self.operation_times[operation_id]
            return duration
        return 0.0
    
    def record_request(self, endpoint: str, duration: float, success: bool):
        """Record request metrics"""
        if endpoint not in self.request_counts:
            self.request_counts[endpoint] = {"count": 0, "total_time": 0.0}
        
        self.request_counts[endpoint]["count"] += 1
        self.request_counts[endpoint]["total_time"] += duration
        
        if not success:
            if endpoint not in self.error_counts:
                self.error_counts[endpoint] = 0
            self.error_counts[endpoint] += 1
    
    def get_profile_report(self) -> Dict[str, Any]:
        """Get profiling report"""
        report = {
            "endpoints": {},
            "total_requests": 0,
            "total_errors": 0
        }
        
        for endpoint, data in self.request_counts.items():
            count = data["count"]
            total_time = data["total_time"]
            avg_time = total_time / count if count > 0 else 0
            errors = self.error_counts.get(endpoint, 0)
            error_rate = (errors / count * 100) if count > 0 else 0
            
            report["endpoints"][endpoint] = {
                "requests": count,
                "avg_response_time": avg_time,
                "total_time": total_time,
                "errors": errors,
                "error_rate": error_rate
            }
            
            report["total_requests"] += count
            report["total_errors"] += errors
        
        return report

# Global instances
performance_monitor = PerformanceMonitor()
app_profiler = ApplicationProfiler()
