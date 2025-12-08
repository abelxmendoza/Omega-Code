"""
Process Supervisor for OmegaOS

Monitors and manages service processes:
- Launch processes
- Track PIDs
- Restart on crash (based on restart policy)
- Detect crash loops
- Monitor resource usage
"""

import subprocess
import os
import time
import threading
import logging
import psutil
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, asdict
from pathlib import Path

from .utils import get_log_file, get_pid_file, write_pid, remove_pid, setup_logging

log = setup_logging(__name__)


@dataclass
class ServiceState:
    """State information for a managed service"""
    name: str
    pid: Optional[int] = None
    status: str = "stopped"  # stopped, starting, running, stopping, crashed
    restart_policy: str = "never"  # never, on-failure, always
    crash_count: int = 0
    last_restart: Optional[float] = None
    start_time: Optional[float] = None
    cpu_percent: float = 0.0
    memory_mb: float = 0.0
    last_health_check: Optional[float] = None
    health_status: Optional[Dict[str, Any]] = None


class ProcessSupervisor:
    """Supervises service processes with auto-restart and health monitoring"""
    
    def __init__(self, registry_path: str, base_dir: str = None):
        self.registry_path = registry_path
        self.base_dir = base_dir or os.getcwd()
        self.services: Dict[str, ServiceState] = {}
        self.processes: Dict[str, subprocess.Popen] = {}
        self.running = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.crash_cooldown = 30.0  # Seconds to wait before restarting after crash
        self.health_check_interval = 10.0  # Seconds between health checks
        self.lock = threading.Lock()
        
    def load_registry(self) -> List[Dict[str, Any]]:
        """Load service registry from JSON file"""
        import json
        try:
            with open(self.registry_path, 'r') as f:
                registry = json.load(f)
                return registry.get("services", [])
        except Exception as e:
            log.error(f"Failed to load service registry: {e}")
            return []
    
    def start_service(self, service_def: Dict[str, Any]) -> bool:
        """Start a service based on its definition"""
        name = service_def["name"]
        
        with self.lock:
            if name in self.processes:
                log.warning(f"Service {name} is already running")
                return False
            
            # Initialize service state
            if name not in self.services:
                self.services[name] = ServiceState(
                    name=name,
                    restart_policy=service_def.get("restart_policy", "never")
                )
            
            state = self.services[name]
            state.status = "starting"
            
            try:
                # Build command
                cmd = [service_def["cmd"]] + service_def.get("args", [])
                working_dir = service_def.get("working_dir", self.base_dir)
                full_working_dir = os.path.join(self.base_dir, working_dir) if not os.path.isabs(working_dir) else working_dir
                
                # Create log files
                stdout_log = get_log_file(name, "stdout")
                stderr_log = get_log_file(name, "stderr")
                
                # Start process
                with open(stdout_log, "a") as stdout_file, open(stderr_log, "a") as stderr_file:
                    process = subprocess.Popen(
                        cmd,
                        cwd=full_working_dir,
                        stdout=stdout_file,
                        stderr=stderr_file,
                        env=os.environ.copy()
                    )
                
                self.processes[name] = process
                state.pid = process.pid
                state.status = "running"
                state.start_time = time.time()
                state.last_restart = time.time()
                
                # Write PID file
                write_pid(name, process.pid)
                
                log.info(f"Started service {name} (PID: {process.pid})")
                return True
                
            except Exception as e:
                log.error(f"Failed to start service {name}: {e}")
                state.status = "crashed"
                state.crash_count += 1
                return False
    
    def stop_service(self, name: str, force: bool = False) -> bool:
        """Stop a service gracefully or forcefully"""
        with self.lock:
            if name not in self.processes:
                log.warning(f"Service {name} is not running")
                return False
            
            process = self.processes[name]
            state = self.services.get(name)
            
            if state:
                state.status = "stopping"
            
            try:
                # Try graceful termination
                if not force:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        force = True
                
                # Force kill if needed
                if force:
                    process.kill()
                    process.wait()
                
                del self.processes[name]
                
                # Remove PID file
                remove_pid(name)
                
                if state:
                    state.status = "stopped"
                    state.pid = None
                    state.start_time = None
                
                log.info(f"Stopped service {name}")
                return True
                
            except Exception as e:
                log.error(f"Failed to stop service {name}: {e}")
                if state:
                    state.status = "crashed"
                return False
    
    def restart_service(self, service_def: Dict[str, Any]) -> bool:
        """Restart a service"""
        name = service_def["name"]
        log.info(f"Restarting service {name}")
        
        # Stop if running
        if name in self.processes:
            self.stop_service(name, force=True)
            time.sleep(1)  # Brief pause before restart
        
        # Start again
        return self.start_service(service_def)
    
    def get_service_status(self, name: str) -> Optional[Dict[str, Any]]:
        """Get current status of a service"""
        state = self.services.get(name)
        if state is None:
            return None
        
        # Update process info if running
        if name in self.processes:
            process = self.processes[name]
            if process.poll() is not None:
                # Process has terminated
                state.status = "crashed"
                state.crash_count += 1
                del self.processes[name]
                state.pid = None
            else:
                # Process is running - update metrics
                try:
                    proc = psutil.Process(process.pid)
                    state.cpu_percent = proc.cpu_percent()
                    state.memory_mb = proc.memory_info().rss / (1024 * 1024)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
        
        return asdict(state)
    
    def list_services(self) -> List[Dict[str, Any]]:
        """List all services and their status"""
        return [self.get_service_status(name) for name in self.services.keys()]
    
    def _monitor_loop(self):
        """Background monitoring loop"""
        while self.running:
            try:
                time.sleep(2)  # Check every 2 seconds
                
                # Check all processes
                services_to_restart = []
                
                with self.lock:
                    for name, process in list(self.processes.items()):
                        state = self.services.get(name)
                        if state is None:
                            continue
                        
                        # Check if process is still alive
                        if process.poll() is not None:
                            # Process has crashed
                            log.warning(f"Service {name} (PID {process.pid}) has crashed")
                            state.status = "crashed"
                            state.crash_count += 1
                            del self.processes[name]
                            state.pid = None
                            
                            # Check restart policy
                            if state.restart_policy == "always":
                                services_to_restart.append(name)
                            elif state.restart_policy == "on-failure" and state.crash_count < 3:
                                services_to_restart.append(name)
                
                # Restart crashed services (outside lock)
                if services_to_restart:
                    registry = self.load_registry()
                    for name in services_to_restart:
                        service_def = next((s for s in registry if s["name"] == name), None)
                        if service_def:
                            # Check cooldown
                            state = self.services.get(name)
                            if state and state.last_restart:
                                elapsed = time.time() - state.last_restart
                                if elapsed < self.crash_cooldown:
                                    log.info(f"Service {name} in cooldown ({elapsed:.1f}s < {self.crash_cooldown}s)")
                                    continue
                            
                            log.info(f"Auto-restarting service {name}")
                            self.start_service(service_def)
                
            except Exception as e:
                log.error(f"Monitor loop error: {e}")
    
    def start_monitoring(self):
        """Start the background monitoring thread"""
        if self.running:
            return
        
        self.running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        log.info("Process supervisor monitoring started")
    
    def stop_monitoring(self):
        """Stop monitoring and clean up"""
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5)
        
        # Stop all services
        for name in list(self.processes.keys()):
            self.stop_service(name, force=True)
        
        log.info("Process supervisor stopped")
    
    def start_autostart_services(self):
        """Start all services marked with autostart=true"""
        registry = self.load_registry()
        autostart_services = [s for s in registry if s.get("autostart", False)]
        
        log.info(f"Starting {len(autostart_services)} autostart services...")
        for service_def in autostart_services:
            name = service_def["name"]
            log.info(f"Starting autostart service: {name}")
            self.start_service(service_def)
            time.sleep(0.5)  # Small delay between starts

