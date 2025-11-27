#!/usr/bin/env python3
# File: /Omega-Code/servers/robot-controller-backend/sensors/ultrasonic_sensor_runner.py

"""
Optimized Ultrasonic Sensor Runner (Pi 5-Compatible – using lgpio)

🎯 Purpose: CLI tool for running ultrasonic sensor with optimizations
   - Different from main_ultrasonic.go (WebSocket server)
   - Use this for: Testing, development, performance analysis
   - Features: Async processing, caching, performance monitoring

This script runs the ultrasonic sensor with performance optimizations:
- Async processing for non-blocking operations
- Caching for sensor data
- Performance monitoring
- Error handling and recovery

Requirements:
- lgpio installed (`pip install lgpio`)
- Script must be run with sufficient permissions (e.g., `newgrp gpio` or via `sudo`)
- Optional: utils.optimization.* for advanced features (falls back to basic mode if unavailable)
"""

import asyncio
import threading
import time
import logging
from collections import deque
from typing import Optional
from ultrasonic_sensor import Ultrasonic

# Import optimization utilities
try:
    import sys
    import os
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from utils.optimization.async_processor import task_processor, TaskPriority
    from utils.optimization.cache_manager import cached, cache_manager
    from utils.optimization.performance_monitor import performance_monitor, app_profiler
    OPTIMIZATION_AVAILABLE = True
except ImportError:
    OPTIMIZATION_AVAILABLE = False
    print("⚠️ Optimization utilities not available, running in basic mode")

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

stop_event = threading.Event()

# Optimized reading with local caching (faster than external cache for frequent reads)
_last_reading_cache = {'value': None, 'timestamp': 0}
CACHE_TTL = 0.5  # 500ms cache

# Create cached function if optimization utilities available
if OPTIMIZATION_AVAILABLE:
    @cached(ttl=CACHE_TTL, key_prefix="ultrasonic:")
    def _get_distance_cached(ultrasonic):
        return ultrasonic.get_distance()

def get_cached_distance(ultrasonic: Ultrasonic) -> float:
    """
    Optimized cached distance reading with dual-layer caching:
    1. Local cache (fastest, in-memory)
    2. External cache (if optimization utilities available)
    """
    current_time = time.time()
    
    # Check local cache first (fastest - O(1))
    if (_last_reading_cache['value'] is not None and 
        current_time - _last_reading_cache['timestamp'] < CACHE_TTL):
        return _last_reading_cache['value']
    
    # Read from sensor (with external cache if available)
    if OPTIMIZATION_AVAILABLE:
        distance = _get_distance_cached(ultrasonic)
    else:
        distance = ultrasonic.get_distance()
    
    # Update local cache atomically
    _last_reading_cache['value'] = distance
    _last_reading_cache['timestamp'] = current_time
    
    return distance

async def run_ultrasonic_optimized():
    """
    Optimized ultrasonic sensor runner with async processing and caching.
    Uses efficient async patterns and reduces overhead.
    """
    ultrasonic: Optional[Ultrasonic] = None
    
    try:
        ultrasonic = Ultrasonic()
    except Exception as e:
        logger.error(f"Failed to initialize ultrasonic sensor: {e}")
        return
    
    if OPTIMIZATION_AVAILABLE:
        # Start performance monitoring
        await performance_monitor.start_monitoring(interval=5.0)
        logger.info("Performance monitoring started for ultrasonic sensor")
    
    # Track last logged distance to reduce logging overhead
    last_logged_distance = -1
    
    try:
        while not stop_event.is_set():
            start_time = time.perf_counter()  # More precise timing
            
            # Get distance with optimized caching
            distance = get_cached_distance(ultrasonic)
            
            # Record performance metrics (only if optimization available)
            if OPTIMIZATION_AVAILABLE:
                processing_time = time.perf_counter() - start_time
                app_profiler.record_request("ultrasonic_reading", processing_time, distance != -1)
            
            # Log only on change or error (reduces I/O overhead)
            if distance == -1:
                logger.warning("❌ Error: Check sensor wiring or power.")
                last_logged_distance = -1
            elif distance != last_logged_distance:
                logger.info(f"📏 Distance: {distance} cm")
                last_logged_distance = distance
            
            # Use async sleep for better performance (non-blocking)
            await asyncio.sleep(1)
            
    except asyncio.CancelledError:
        logger.info("Runner cancelled")
    except Exception as e:
        logger.error(f"⚠️ Runner interrupted: {e}")
    finally:
        if ultrasonic:
            ultrasonic.close()
        if OPTIMIZATION_AVAILABLE:
            await performance_monitor.stop_monitoring()
        logger.info("🧹 GPIO closed and monitoring stopped")

def run_ultrasonic_basic():
    """
    Basic ultrasonic sensor runner (fallback).
    """
    ultrasonic = Ultrasonic()
    try:
        while not stop_event.is_set():
            distance = ultrasonic.get_distance()
            if distance == -1:
                print("❌ Error: Check sensor wiring or power.")
            else:
                print(f"📏 Distance: {distance} cm")
            time.sleep(1)
    except Exception as e:
        print(f"⚠️ Runner interrupted: {e}")
    finally:
        ultrasonic.close()
        print("🧹 GPIO closed")

async def main():
    """Main async function"""
    if OPTIMIZATION_AVAILABLE:
        # Start async task processor
        await task_processor.start()
        logger.info("Async task processor started")
        
        # Run optimized ultrasonic sensor
        await run_ultrasonic_optimized()
        
        # Stop task processor
        await task_processor.stop()
    else:
        # Run basic version
        run_ultrasonic_basic()

if __name__ == "__main__":
    try:
        if OPTIMIZATION_AVAILABLE:
            # Run async version
            asyncio.run(main())
        else:
            # Run basic version
            ultrasonic_thread = threading.Thread(target=run_ultrasonic_basic)
            ultrasonic_thread.start()
            try:
                time.sleep(10)  # Run for 10 seconds or until interrupted
            except KeyboardInterrupt:
                print("🔴 Keyboard interrupt received.")
            print("🛑 Stopping ultrasonic thread")
            stop_event.set()
            ultrasonic_thread.join()
    except KeyboardInterrupt:
        print("🔴 Keyboard interrupt received.")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        stop_event.set()
        print("🛑 Ultrasonic sensor runner stopped")
