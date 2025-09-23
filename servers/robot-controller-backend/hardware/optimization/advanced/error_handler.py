"""
Centralized Error Handling for Hardware Optimizations
- Comprehensive error handling and recovery
- Error monitoring and alerting
- Graceful degradation strategies
- Performance impact tracking
"""

import time
import threading
import logging
import traceback
import json
from typing import Dict, List, Optional, Any, Callable, Union
from dataclasses import dataclass, field
from enum import Enum
import asyncio
from collections import deque
import functools

logger = logging.getLogger(__name__)

class ErrorSeverity(Enum):
    """Error severity levels"""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"

class ErrorCategory(Enum):
    """Error categories"""
    HARDWARE = "hardware"
    SOFTWARE = "software"
    NETWORK = "network"
    MEMORY = "memory"
    PERMISSION = "permission"
    TIMEOUT = "timeout"
    RESOURCE = "resource"

@dataclass
class ErrorInfo:
    """Error information structure"""
    timestamp: float
    component: str
    error_type: str
    severity: ErrorSeverity
    category: ErrorCategory
    message: str
    details: Dict[str, Any] = field(default_factory=dict)
    stack_trace: str = ""
    recovery_action: Optional[str] = None
    performance_impact: float = 0.0

@dataclass
class RecoveryStrategy:
    """Recovery strategy configuration"""
    strategy_id: str
    component: str
    error_patterns: List[str]
    recovery_action: Callable
    max_attempts: int = 3
    backoff_delay: float = 1.0
    success_callback: Optional[Callable] = None
    failure_callback: Optional[Callable] = None

class HardwareErrorHandler:
    """Centralized hardware error handling system"""
    
    def __init__(self):
        self.error_history = deque(maxlen=1000)  # Keep last 1000 errors
        self.recovery_strategies = {}
        self.error_stats = {
            "total_errors": 0,
            "errors_by_severity": {severity.value: 0 for severity in ErrorSeverity},
            "errors_by_category": {category.value: 0 for category in ErrorCategory},
            "errors_by_component": {},
            "recovery_attempts": 0,
            "successful_recoveries": 0,
            "failed_recoveries": 0
        }
        self.running = False
        self.lock = threading.Lock()
        
        # Error monitoring settings
        self.monitoring_enabled = True
        self.alert_thresholds = {
            ErrorSeverity.HIGH: 5,  # Alert after 5 high severity errors
            ErrorSeverity.CRITICAL: 1  # Alert immediately for critical errors
        }
        self.alert_callbacks = []
        
        # Performance impact tracking
        self.performance_impact_threshold = 0.1  # 10% performance impact threshold
        
        # Initialize default recovery strategies
        self._init_default_recovery_strategies()
    
    def _init_default_recovery_strategies(self):
        """Initialize default recovery strategies"""
        try:
            # GPIO recovery strategy
            self.add_recovery_strategy(RecoveryStrategy(
                strategy_id="gpio_recovery",
                component="gpio",
                error_patterns=["GPIO", "pin", "gpiochip"],
                recovery_action=self._gpio_recovery_action,
                max_attempts=3,
                backoff_delay=0.5
            ))
            
            # DMA recovery strategy
            self.add_recovery_strategy(RecoveryStrategy(
                strategy_id="dma_recovery",
                component="dma",
                error_patterns=["DMA", "dma", "transfer"],
                recovery_action=self._dma_recovery_action,
                max_attempts=2,
                backoff_delay=1.0
            ))
            
            # CPU recovery strategy
            self.add_recovery_strategy(RecoveryStrategy(
                strategy_id="cpu_recovery",
                component="cpu",
                error_patterns=["CPU", "cpu", "core", "frequency"],
                recovery_action=self._cpu_recovery_action,
                max_attempts=3,
                backoff_delay=2.0
            ))
            
            # Memory recovery strategy
            self.add_recovery_strategy(RecoveryStrategy(
                strategy_id="memory_recovery",
                component="memory",
                error_patterns=["memory", "Memory", "RAM", "allocation"],
                recovery_action=self._memory_recovery_action,
                max_attempts=2,
                backoff_delay=1.5
            ))
            
            logger.info("Default recovery strategies initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize recovery strategies: {e}")
    
    def add_recovery_strategy(self, strategy: RecoveryStrategy):
        """Add recovery strategy"""
        try:
            self.recovery_strategies[strategy.strategy_id] = strategy
            logger.info(f"Recovery strategy added: {strategy.strategy_id}")
            
        except Exception as e:
            logger.error(f"Failed to add recovery strategy: {e}")
    
    def handle_error(self, component: str, error: Exception, 
                    severity: ErrorSeverity = ErrorSeverity.MEDIUM,
                    category: ErrorCategory = ErrorCategory.SOFTWARE,
                    details: Optional[Dict[str, Any]] = None,
                    recovery_action: Optional[str] = None) -> bool:
        """Handle error with recovery attempt"""
        try:
            # Create error info
            error_info = ErrorInfo(
                timestamp=time.time(),
                component=component,
                error_type=type(error).__name__,
                severity=severity,
                category=category,
                message=str(error),
                details=details or {},
                stack_trace=traceback.format_exc(),
                recovery_action=recovery_action
            )
            
            # Store error
            with self.lock:
                self.error_history.append(error_info)
                self._update_error_stats(error_info)
            
            # Log error
            self._log_error(error_info)
            
            # Attempt recovery
            recovery_success = self._attempt_recovery(error_info)
            
            # Check for alerts
            if self.monitoring_enabled:
                self._check_alerts(error_info)
            
            return recovery_success
            
        except Exception as e:
            logger.error(f"Error handling failed: {e}")
            return False
    
    def _update_error_stats(self, error_info: ErrorInfo):
        """Update error statistics"""
        try:
            self.error_stats["total_errors"] += 1
            self.error_stats["errors_by_severity"][error_info.severity.value] += 1
            self.error_stats["errors_by_category"][error_info.category.value] += 1
            
            if error_info.component not in self.error_stats["errors_by_component"]:
                self.error_stats["errors_by_component"][error_info.component] = 0
            self.error_stats["errors_by_component"][error_info.component] += 1
            
        except Exception as e:
            logger.error(f"Failed to update error stats: {e}")
    
    def _log_error(self, error_info: ErrorInfo):
        """Log error with appropriate level"""
        try:
            log_message = f"[{error_info.component}] {error_info.error_type}: {error_info.message}"
            
            if error_info.severity == ErrorSeverity.CRITICAL:
                logger.critical(log_message)
            elif error_info.severity == ErrorSeverity.HIGH:
                logger.error(log_message)
            elif error_info.severity == ErrorSeverity.MEDIUM:
                logger.warning(log_message)
            else:
                logger.info(log_message)
            
            # Log details if available
            if error_info.details:
                logger.debug(f"Error details: {json.dumps(error_info.details, indent=2)}")
            
            # Log stack trace for high severity errors
            if error_info.severity in [ErrorSeverity.HIGH, ErrorSeverity.CRITICAL]:
                logger.debug(f"Stack trace: {error_info.stack_trace}")
                
        except Exception as e:
            logger.error(f"Failed to log error: {e}")
    
    def _attempt_recovery(self, error_info: ErrorInfo) -> bool:
        """Attempt error recovery"""
        try:
            # Find applicable recovery strategies
            applicable_strategies = []
            for strategy in self.recovery_strategies.values():
                if (strategy.component == error_info.component or
                    any(pattern.lower() in error_info.message.lower() 
                        for pattern in strategy.error_patterns)):
                    applicable_strategies.append(strategy)
            
            if not applicable_strategies:
                logger.debug(f"No recovery strategy found for {error_info.component}")
                return False
            
            # Attempt recovery with each strategy
            for strategy in applicable_strategies:
                recovery_success = self._execute_recovery_strategy(strategy, error_info)
                if recovery_success:
                    return True
            
            return False
            
        except Exception as e:
            logger.error(f"Recovery attempt failed: {e}")
            return False
    
    def _execute_recovery_strategy(self, strategy: RecoveryStrategy, error_info: ErrorInfo) -> bool:
        """Execute recovery strategy"""
        try:
            self.error_stats["recovery_attempts"] += 1
            
            # Execute recovery action
            recovery_result = strategy.recovery_action(error_info)
            
            if recovery_result:
                self.error_stats["successful_recoveries"] += 1
                logger.info(f"Recovery successful: {strategy.strategy_id}")
                
                # Call success callback
                if strategy.success_callback:
                    strategy.success_callback(error_info, strategy)
                
                return True
            else:
                self.error_stats["failed_recoveries"] += 1
                logger.warning(f"Recovery failed: {strategy.strategy_id}")
                
                # Call failure callback
                if strategy.failure_callback:
                    strategy.failure_callback(error_info, strategy)
                
                return False
                
        except Exception as e:
            self.error_stats["failed_recoveries"] += 1
            logger.error(f"Recovery strategy execution failed: {e}")
            return False
    
    def _gpio_recovery_action(self, error_info: ErrorInfo) -> bool:
        """GPIO recovery action"""
        try:
            # Reset GPIO state
            logger.info("Attempting GPIO recovery...")
            
            # Simple recovery: wait and retry
            time.sleep(0.1)
            
            # In a real implementation, this would reset GPIO pins
            # For now, we'll simulate success
            return True
            
        except Exception as e:
            logger.error(f"GPIO recovery failed: {e}")
            return False
    
    def _dma_recovery_action(self, error_info: ErrorInfo) -> bool:
        """DMA recovery action"""
        try:
            # Reset DMA channels
            logger.info("Attempting DMA recovery...")
            
            # Simple recovery: wait and retry
            time.sleep(0.5)
            
            # In a real implementation, this would reset DMA channels
            # For now, we'll simulate success
            return True
            
        except Exception as e:
            logger.error(f"DMA recovery failed: {e}")
            return False
    
    def _cpu_recovery_action(self, error_info: ErrorInfo) -> bool:
        """CPU recovery action"""
        try:
            # Reset CPU frequency
            logger.info("Attempting CPU recovery...")
            
            # Simple recovery: wait and retry
            time.sleep(1.0)
            
            # In a real implementation, this would reset CPU settings
            # For now, we'll simulate success
            return True
            
        except Exception as e:
            logger.error(f"CPU recovery failed: {e}")
            return False
    
    def _memory_recovery_action(self, error_info: ErrorInfo) -> bool:
        """Memory recovery action"""
        try:
            # Clear memory caches
            logger.info("Attempting memory recovery...")
            
            # Simple recovery: wait and retry
            time.sleep(0.5)
            
            # In a real implementation, this would clear caches
            # For now, we'll simulate success
            return True
            
        except Exception as e:
            logger.error(f"Memory recovery failed: {e}")
            return False
    
    def _check_alerts(self, error_info: ErrorInfo):
        """Check for alert conditions"""
        try:
            severity = error_info.severity
            if severity in self.alert_thresholds:
                threshold = self.alert_thresholds[severity]
                current_count = self.error_stats["errors_by_severity"][severity.value]
                
                if current_count >= threshold:
                    self._trigger_alert(error_info, severity, current_count)
                    
        except Exception as e:
            logger.error(f"Alert check failed: {e}")
    
    def _trigger_alert(self, error_info: ErrorInfo, severity: ErrorSeverity, count: int):
        """Trigger alert"""
        try:
            alert_message = f"ALERT: {count} {severity.value} severity errors in {error_info.component}"
            logger.critical(alert_message)
            
            # Call alert callbacks
            for callback in self.alert_callbacks:
                try:
                    callback(error_info, severity, count)
                except Exception as e:
                    logger.error(f"Alert callback failed: {e}")
                    
        except Exception as e:
            logger.error(f"Alert trigger failed: {e}")
    
    def add_alert_callback(self, callback: Callable):
        """Add alert callback"""
        self.alert_callbacks.append(callback)
    
    def get_error_stats(self) -> Dict[str, Any]:
        """Get error statistics"""
        with self.lock:
            stats = dict(self.error_stats)
            stats["recovery_success_rate"] = (
                self.error_stats["successful_recoveries"] / 
                max(self.error_stats["recovery_attempts"], 1)
            )
            stats["total_errors"] = len(self.error_history)
            return stats
    
    def get_recent_errors(self, count: int = 10) -> List[ErrorInfo]:
        """Get recent errors"""
        with self.lock:
            return list(self.error_history)[-count:]
    
    def clear_error_history(self):
        """Clear error history"""
        with self.lock:
            self.error_history.clear()
            logger.info("Error history cleared")
    
    def start_monitoring(self):
        """Start error monitoring"""
        self.running = True
        logger.info("Error monitoring started")
    
    def stop_monitoring(self):
        """Stop error monitoring"""
        self.running = False
        logger.info("Error monitoring stopped")

def error_handler_decorator(component: str, severity: ErrorSeverity = ErrorSeverity.MEDIUM,
                          category: ErrorCategory = ErrorCategory.SOFTWARE):
    """Decorator for automatic error handling"""
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                # Get global error handler instance
                error_handler = getattr(error_handler_decorator, '_instance', None)
                if error_handler:
                    error_handler.handle_error(component, e, severity, category)
                else:
                    logger.error(f"Unhandled error in {component}: {e}")
                raise
        return wrapper
    return decorator

# Global error handler instance
error_handler = HardwareErrorHandler()

# Set instance for decorator
error_handler_decorator._instance = error_handler
