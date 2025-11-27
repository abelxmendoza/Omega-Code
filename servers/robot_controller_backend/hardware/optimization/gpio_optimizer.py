"""
GPIO Optimization Utilities for Robot Controller
- High-performance GPIO operations
- Interrupt-driven sensor reading
- Optimized PWM generation
- GPIO pin state caching
- Real-time GPIO monitoring
"""

import time
import threading
import logging
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass
from enum import Enum
import asyncio

logger = logging.getLogger(__name__)

class GPIOMode(Enum):
    """GPIO operation modes"""
    POLLING = "polling"
    INTERRUPT = "interrupt"
    DMA = "dma"
    HARDWARE_PWM = "hardware_pwm"

@dataclass
class GPIOPin:
    """GPIO pin configuration"""
    pin: int
    mode: str  # 'input', 'output', 'pwm'
    pull: str  # 'up', 'down', 'none'
    edge: str  # 'rising', 'falling', 'both', 'none'
    frequency: Optional[int] = None  # For PWM
    duty_cycle: Optional[float] = None  # For PWM

@dataclass
class GPIOOperation:
    """GPIO operation record"""
    pin: int
    operation: str
    start_time: float
    end_time: float
    duration: float
    success: bool
    value: Any = None

class GPIOOptimizer:
    """High-performance GPIO operations"""
    
    def __init__(self):
        self.pins: Dict[int, GPIOPin] = {}
        self.pin_states: Dict[int, bool] = {}
        self.operation_history: List[GPIOOperation] = []
        self.interrupt_handlers: Dict[int, Callable] = {}
        self.pwm_handles: Dict[int, Any] = {}
        self.lock = threading.Lock()
        self.monitoring = False
        self.monitor_task = None
        
        # Performance settings
        self.polling_interval = 0.001  # 1ms
        self.debounce_time = 0.005  # 5ms
        self.cache_timeout = 0.01  # 10ms
        
        # Initialize GPIO library
        self._init_gpio()
    
    def _init_gpio(self):
        """Initialize GPIO library"""
        try:
            import lgpio
            self.gpio = lgpio
            self.chip = lgpio.gpiochip_open(0)
            logger.info("GPIO initialized with lgpio")
        except ImportError:
            try:
                import RPi.GPIO as GPIO
                self.gpio = GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                logger.info("GPIO initialized with RPi.GPIO")
            except ImportError:
                logger.warning("No GPIO library available, using mock")
                self.gpio = None
                self.chip = None
    
    def configure_pin(self, pin: int, mode: str, pull: str = 'none', 
                     edge: str = 'none', frequency: int = None) -> bool:
        """Configure a GPIO pin for optimal performance"""
        try:
            with self.lock:
                gpio_pin = GPIOPin(
                    pin=pin,
                    mode=mode,
                    pull=pull,
                    edge=edge,
                    frequency=frequency
                )
                
                self.pins[pin] = gpio_pin
                
                if self.gpio and self.chip:
                    if mode == 'input':
                        self.gpio.gpio_claim_input(self.chip, pin)
                        if pull == 'up':
                            self.gpio.gpio_set_pullup_down(self.chip, pin, self.gpio.PULL_UP)
                        elif pull == 'down':
                            self.gpio.gpio_set_pullup_down(self.chip, pin, self.gpio.PULL_DOWN)
                        
                        if edge != 'none':
                            self._setup_interrupt(pin, edge)
                    
                    elif mode == 'output':
                        self.gpio.gpio_claim_output(self.chip, pin)
                        self.pin_states[pin] = False
                    
                    elif mode == 'pwm' and frequency:
                        self._setup_pwm(pin, frequency)
                
                logger.info(f"GPIO pin {pin} configured: {mode}, {pull}, {edge}")
                return True
                
        except Exception as e:
            logger.error(f"Failed to configure GPIO pin {pin}: {e}")
            return False
    
    def _setup_interrupt(self, pin: int, edge: str):
        """Setup interrupt-driven GPIO reading"""
        try:
            if edge == 'rising':
                edge_flag = self.gpio.RISING_EDGE
            elif edge == 'falling':
                edge_flag = self.gpio.FALLING_EDGE
            elif edge == 'both':
                edge_flag = self.gpio.BOTH_EDGES
            else:
                return
            
            # Setup interrupt callback
            def interrupt_callback(chip, gpio, level, timestamp):
                if pin in self.interrupt_handlers:
                    self.interrupt_handlers[pin](level, timestamp)
            
            self.gpio.callback(self.chip, pin, edge_flag, interrupt_callback)
            logger.info(f"Interrupt setup for pin {pin}: {edge}")
            
        except Exception as e:
            logger.error(f"Failed to setup interrupt for pin {pin}: {e}")
    
    def _setup_pwm(self, pin: int, frequency: int):
        """Setup hardware PWM"""
        try:
            if hasattr(self.gpio, 'tx_pwm'):
                # Hardware PWM with lgpio
                self.gpio.tx_pwm(self.chip, pin, frequency, 0)
                self.pwm_handles[pin] = {'frequency': frequency, 'duty_cycle': 0}
                logger.info(f"Hardware PWM setup for pin {pin}: {frequency}Hz")
            else:
                # Software PWM fallback
                logger.warning(f"Hardware PWM not available for pin {pin}, using software PWM")
                
        except Exception as e:
            logger.error(f"Failed to setup PWM for pin {pin}: {e}")
    
    def read_pin(self, pin: int, use_cache: bool = True) -> bool:
        """Read GPIO pin with optimization"""
        start_time = time.time()
        success = False
        value = False
        
        try:
            with self.lock:
                if pin not in self.pins:
                    logger.warning(f"Pin {pin} not configured")
                    return False
                
                pin_config = self.pins[pin]
                
                if pin_config.mode != 'input':
                    logger.warning(f"Pin {pin} not configured as input")
                    return False
                
                # Use cached value if available and recent
                if use_cache and pin in self.pin_states:
                    cache_time = getattr(pin_config, 'last_read', 0)
                    if time.time() - cache_time < self.cache_timeout:
                        value = self.pin_states[pin]
                        success = True
                        return value
                
                # Read actual pin value
                if self.gpio and self.chip:
                    value = self.gpio.gpio_read(self.chip, pin)
                    self.pin_states[pin] = value
                    pin_config.last_read = time.time()
                    success = True
                else:
                    # Mock value for testing
                    value = bool(time.time() % 2)
                    success = True
                
        except Exception as e:
            logger.error(f"Failed to read pin {pin}: {e}")
        finally:
            duration = time.time() - start_time
            self._record_operation(pin, 'read', start_time, time.time(), success, value)
        
        return value
    
    def write_pin(self, pin: int, value: bool) -> bool:
        """Write GPIO pin with optimization"""
        start_time = time.time()
        success = False
        
        try:
            with self.lock:
                if pin not in self.pins:
                    logger.warning(f"Pin {pin} not configured")
                    return False
                
                pin_config = self.pins[pin]
                
                if pin_config.mode != 'output':
                    logger.warning(f"Pin {pin} not configured as output")
                    return False
                
                # Skip write if value hasn't changed
                if pin in self.pin_states and self.pin_states[pin] == value:
                    success = True
                    return True
                
                # Write actual pin value
                if self.gpio and self.chip:
                    self.gpio.gpio_write(self.chip, pin, int(value))
                    self.pin_states[pin] = value
                    success = True
                else:
                    # Mock write for testing
                    self.pin_states[pin] = value
                    success = True
                
        except Exception as e:
            logger.error(f"Failed to write pin {pin}: {e}")
        finally:
            duration = time.time() - start_time
            self._record_operation(pin, 'write', start_time, time.time(), success, value)
        
        return success
    
    def set_pwm(self, pin: int, duty_cycle: float) -> bool:
        """Set PWM duty cycle with optimization"""
        start_time = time.time()
        success = False
        
        try:
            with self.lock:
                if pin not in self.pins:
                    logger.warning(f"Pin {pin} not configured")
                    return False
                
                pin_config = self.pins[pin]
                
                if pin_config.mode != 'pwm':
                    logger.warning(f"Pin {pin} not configured as PWM")
                    return False
                
                # Clamp duty cycle
                duty_cycle = max(0.0, min(1.0, duty_cycle))
                
                # Skip if duty cycle hasn't changed significantly
                if pin in self.pwm_handles:
                    current_duty = self.pwm_handles[pin]['duty_cycle']
                    if abs(current_duty - duty_cycle) < 0.01:  # 1% threshold
                        success = True
                        return True
                
                # Set PWM duty cycle
                if self.gpio and self.chip:
                    if hasattr(self.gpio, 'tx_pwm'):
                        frequency = self.pwm_handles[pin]['frequency']
                        self.gpio.tx_pwm(self.chip, pin, frequency, int(duty_cycle * 1000000))
                        self.pwm_handles[pin]['duty_cycle'] = duty_cycle
                        success = True
                    else:
                        # Software PWM fallback
                        logger.warning(f"Hardware PWM not available for pin {pin}")
                        success = False
                else:
                    # Mock PWM for testing
                    self.pwm_handles[pin] = {'duty_cycle': duty_cycle}
                    success = True
                
        except Exception as e:
            logger.error(f"Failed to set PWM for pin {pin}: {e}")
        finally:
            duration = time.time() - start_time
            self._record_operation(pin, 'pwm', start_time, time.time(), success, duty_cycle)
        
        return success
    
    def set_interrupt_handler(self, pin: int, handler: Callable):
        """Set interrupt handler for a pin"""
        self.interrupt_handlers[pin] = handler
        logger.info(f"Interrupt handler set for pin {pin}")
    
    def _record_operation(self, pin: int, operation: str, start_time: float, 
                         end_time: float, success: bool, value: Any = None):
        """Record GPIO operation for performance monitoring"""
        operation_record = GPIOOperation(
            pin=pin,
            operation=operation,
            start_time=start_time,
            end_time=end_time,
            duration=end_time - start_time,
            success=success,
            value=value
        )
        
        self.operation_history.append(operation_record)
        
        # Keep only recent operations
        if len(self.operation_history) > 1000:
            self.operation_history = self.operation_history[-500:]
    
    async def start_monitoring(self, interval: float = 0.1):
        """Start GPIO monitoring"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitor_task = asyncio.create_task(self._monitor_loop(interval))
        logger.info("GPIO monitoring started")
    
    async def stop_monitoring(self):
        """Stop GPIO monitoring"""
        if self.monitoring:
            self.monitoring = False
            if self.monitor_task:
                self.monitor_task.cancel()
            logger.info("GPIO monitoring stopped")
    
    async def _monitor_loop(self, interval: float):
        """GPIO monitoring loop"""
        while self.monitoring:
            try:
                # Monitor pin states
                for pin, config in self.pins.items():
                    if config.mode == 'input':
                        current_value = self.read_pin(pin, use_cache=False)
                        # Check for state changes
                        if pin in self.pin_states and self.pin_states[pin] != current_value:
                            logger.debug(f"Pin {pin} state changed: {self.pin_states[pin]} -> {current_value}")
                
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"GPIO monitoring error: {e}")
                await asyncio.sleep(interval)
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """Get GPIO performance statistics"""
        if not self.operation_history:
            return {"message": "No operations recorded"}
        
        # Calculate statistics
        total_operations = len(self.operation_history)
        successful_operations = sum(1 for op in self.operation_history if op.success)
        failed_operations = total_operations - successful_operations
        
        durations = [op.duration for op in self.operation_history]
        avg_duration = sum(durations) / len(durations) if durations else 0
        max_duration = max(durations) if durations else 0
        min_duration = min(durations) if durations else 0
        
        # Operations by type
        operations_by_type = {}
        for op in self.operation_history:
            if op.operation not in operations_by_type:
                operations_by_type[op.operation] = 0
            operations_by_type[op.operation] += 1
        
        return {
            "total_operations": total_operations,
            "successful_operations": successful_operations,
            "failed_operations": failed_operations,
            "success_rate": successful_operations / total_operations if total_operations > 0 else 0,
            "avg_duration": avg_duration,
            "max_duration": max_duration,
            "min_duration": min_duration,
            "operations_by_type": operations_by_type,
            "configured_pins": len(self.pins),
            "monitoring_active": self.monitoring
        }
    
    def cleanup(self):
        """Cleanup GPIO resources"""
        try:
            if self.gpio and self.chip:
                # Release all claimed pins
                for pin in self.pins:
                    try:
                        self.gpio.gpio_free(self.chip, pin)
                    except:
                        pass
                
                self.gpio.gpiochip_close(self.chip)
            
            logger.info("GPIO cleanup completed")
        except Exception as e:
            logger.error(f"GPIO cleanup error: {e}")

# Global GPIO optimizer instance
gpio_optimizer = GPIOOptimizer()
