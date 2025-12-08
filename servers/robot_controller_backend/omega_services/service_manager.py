#!/usr/bin/env python3
"""
OmegaOS Service Manager

Top-level orchestrator for robot services.
- Loads service registry
- Starts autostart services
- Provides management API
- Integrates with process supervisor
"""

import os
import sys
import json
import logging
import signal
import time
from pathlib import Path
from typing import Dict, Any, Optional, List

from .process_supervisor import ProcessSupervisor
from .health_checks import run_health_check
from .utils import (
    setup_logging, get_service_registry_path, get_base_working_dir,
    get_log_file, format_uptime
)

# Import config manager (optional, for loading config at boot)
try:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from omega_config.config_manager import get_config_manager
    CONFIG_AVAILABLE = True
except ImportError:
    CONFIG_AVAILABLE = False
    log.warning("Config manager not available, running without config integration")

# Setup logging
log = setup_logging(__name__)


class ServiceManager:
    """Top-level service orchestrator"""
    
    def __init__(self, registry_path: str = None, base_dir: str = None):
        # Determine paths using utils
        if registry_path is None:
            registry_path = str(get_service_registry_path())
        
        if base_dir is None:
            base_dir = str(get_base_working_dir())
        
        self.registry_path = registry_path
        self.base_dir = base_dir
        self.supervisor = ProcessSupervisor(registry_path, base_dir)
        self.running = False
        
        # Load registry and initialize services
        self._load_registry()
    
    def _load_registry(self):
        """Load service registry and initialize service states"""
        registry = self.supervisor.load_registry()
        for service_def in registry:
            name = service_def["name"]
            if name not in self.supervisor.services:
                from .process_supervisor import ServiceState
                self.supervisor.services[name] = ServiceState(
                    name=name,
                    restart_policy=service_def.get("restart_policy", "never")
                )
    
    def start(self):
        """Start the service manager"""
        if self.running:
            log.warning("Service manager is already running")
            return
        
        log.info("=" * 60)
        log.info("OmegaOS Service Manager Starting")
        log.info("=" * 60)
        
        # Load configuration if available
        if CONFIG_AVAILABLE:
            try:
                config_manager = get_config_manager()
                config = config_manager.get_config()
                robot_name = config.get("robot", {}).get("name", "Omega-1")
                log.info(f"Loaded configuration for: {robot_name}")
                
                # Update boot count in state
                state = config_manager.get_state()
                boot_count = state.get("boot_count", 0) + 1
                config_manager.update_state("boot_count", boot_count)
                config_manager.update_state("last_boot", time.strftime("%Y-%m-%d %H:%M:%S"))
            except Exception as e:
                log.warning(f"Failed to load config: {e}, continuing without config")
        
        # Start process supervisor
        self.supervisor.start_monitoring()
        
        # Start autostart services
        self.supervisor.start_autostart_services()
        
        self.running = True
        log.info("Service manager started successfully")
    
    def stop(self):
        """Stop the service manager"""
        if not self.running:
            return
        
        log.info("Stopping service manager...")
        self.supervisor.stop_monitoring()
        self.running = False
        log.info("Service manager stopped")
    
    def start_service(self, name: str) -> Dict[str, Any]:
        """Start a specific service"""
        registry = self.supervisor.load_registry()
        service_def = next((s for s in registry if s["name"] == name), None)
        
        if service_def is None:
            return {"success": False, "error": f"Service {name} not found in registry"}
        
        success = self.supervisor.start_service(service_def)
        if success:
            return {"success": True, "message": f"Service {name} started"}
        else:
            return {"success": False, "error": f"Failed to start service {name}"}
    
    def stop_service(self, name: str, force: bool = False) -> Dict[str, Any]:
        """Stop a specific service"""
        success = self.supervisor.stop_service(name, force)
        if success:
            return {"success": True, "message": f"Service {name} stopped"}
        else:
            return {"success": False, "error": f"Failed to stop service {name}"}
    
    def restart_service(self, name: str) -> Dict[str, Any]:
        """Restart a specific service"""
        registry = self.supervisor.load_registry()
        service_def = next((s for s in registry if s["name"] == name), None)
        
        if service_def is None:
            return {"success": False, "error": f"Service {name} not found in registry"}
        
        success = self.supervisor.restart_service(service_def)
        if success:
            return {"success": True, "message": f"Service {name} restarted"}
        else:
            return {"success": False, "error": f"Failed to restart service {name}"}
    
    def get_service_status(self, name: str) -> Optional[Dict[str, Any]]:
        """Get status of a specific service"""
        status = self.supervisor.get_service_status(name)
        if status is None:
            return None
        
        # Add health check if available
        registry = self.supervisor.load_registry()
        service_def = next((s for s in registry if s["name"] == name), None)
        if service_def and service_def.get("health_check"):
            health_check_name = service_def["health_check"]
            health_result = run_health_check(health_check_name)
            status["health"] = health_result
        
        return status
    
    def list_services(self) -> List[Dict[str, Any]]:
        """List all services with their status"""
        registry = self.supervisor.load_registry()
        result = []
        
        for service_def in registry:
            name = service_def["name"]
            status = self.get_service_status(name)
            
            if status:
                # Add service definition info
                status["display_name"] = service_def.get("display_name", name)
                status["description"] = service_def.get("description", "")
                status["type"] = service_def.get("type", "unknown")
                status["port"] = service_def.get("port")
                status["autostart"] = service_def.get("autostart", False)
                result.append(status)
            else:
                # Service not initialized
                result.append({
                    "name": name,
                    "display_name": service_def.get("display_name", name),
                    "description": service_def.get("description", ""),
                    "type": service_def.get("type", "unknown"),
                    "port": service_def.get("port"),
                    "autostart": service_def.get("autostart", False),
                    "status": "unknown",
                    "pid": None
                })
        
        return result
    
    def get_logs(self, name: str, lines: int = 50) -> Dict[str, Any]:
        """Get logs for a service"""
        from .utils import get_log_file
        log_file = get_log_file(name, "stdout")
        error_log_file = get_log_file(name, "stderr")
        
        logs = {"stdout": [], "stderr": []}
        
        # Read stdout log
        if log_file.exists():
            try:
                with open(log_file, 'r') as f:
                    all_lines = f.readlines()
                    logs["stdout"] = all_lines[-lines:] if len(all_lines) > lines else all_lines
            except Exception as e:
                logs["stdout"] = [f"Error reading log: {e}"]
        
        # Read stderr log
        if error_log_file.exists():
            try:
                with open(error_log_file, 'r') as f:
                    all_lines = f.readlines()
                    logs["stderr"] = all_lines[-lines:] if len(all_lines) > lines else all_lines
            except Exception as e:
                logs["stderr"] = [f"Error reading log: {e}"]
        
        return logs


# Global service manager instance
_service_manager: Optional[ServiceManager] = None


def get_service_manager(registry_path: str = None, base_dir: str = None) -> ServiceManager:
    """Get or create the global service manager instance"""
    global _service_manager
    if _service_manager is None:
        _service_manager = ServiceManager(registry_path, base_dir)
    return _service_manager


def main():
    """Main entry point for service manager daemon"""
    # Change to base directory
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(base_dir)
    
    manager = ServiceManager()
    
    # Setup signal handlers
    def signal_handler(sig, frame):
        log.info("Received shutdown signal")
        manager.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Start manager
    manager.start()
    
    # Keep running
    try:
        while manager.running:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        manager.stop()


if __name__ == "__main__":
    main()

