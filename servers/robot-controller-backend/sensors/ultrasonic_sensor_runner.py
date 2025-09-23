#!/usr/bin/env python3
# File: /Omega-Code/servers/robot-controller-backend/sensors/ultrasonic_sensor_runner.py

"""
Optimized Ultrasonic Sensor Runner (Pi 5-Compatible – using lgpio)

This script runs the ultrasonic sensor with performance optimizations:
- Async processing for non-blocking operations
- Caching for sensor data
- Performance monitoring
- Error handling and recovery

Requirements:
- lgpio installed (`pip install lgpio`)
- Script must be run with sufficient permissions (e.g., `newgrp gpio` or via `sudo`)
"""

import asyncio
import threading
import time
import logging
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

# Cached sensor reading function
if OPTIMIZATION_AVAILABLE:
    @cached(ttl=0.5, key_prefix="ultrasonic:")  # Cache for 500ms
    def get_cached_distance(ultrasonic):
        return ultrasonic.get_distance()
else:
    def get_cached_distance(ultrasonic):
        return ultrasonic.get_distance()

async def run_ultrasonic_optimized():
    """
    Optimized ultrasonic sensor runner with async processing and caching.
    """
    ultrasonic = Ultrasonic()
    
    if OPTIMIZATION_AVAILABLE:
        # Start performance monitoring
        await performance_monitor.start_monitoring(interval=5.0)
        logger.info("Performance monitoring started for ultrasonic sensor")
    
    try:
        while not stop_event.is_set():
            start_time = time.time()
            
            # Get distance with caching
            distance = get_cached_distance(ultrasonic)
            
            # Record performance metrics
            if OPTIMIZATION_AVAILABLE:
                processing_time = time.time() - start_time
                app_profiler.record_request("ultrasonic_reading", processing_time, distance != -1)
            
            if distance == -1:
                logger.warning("❌ Error: Check sensor wiring or power.")
            else:
                logger.info(f"📏 Distance: {distance} cm")
            
            # Use async sleep for better performance
            await asyncio.sleep(1)
            
    except Exception as e:
        logger.error(f"⚠️ Runner interrupted: {e}")
    finally:
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
