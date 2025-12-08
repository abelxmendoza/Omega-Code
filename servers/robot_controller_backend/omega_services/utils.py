"""
Shared Utilities for OmegaOS Service Orchestrator

Provides helper functions for:
- PID file management
- Logging configuration
- Path management
- Time-based utilities
"""

import os
import json
import logging
import time
from pathlib import Path
from typing import Optional, Dict, Any

# Base directories
BASE_DIR = Path(__file__).parent.parent
OMEGA_SERVICES_DIR = Path(__file__).parent
LOG_DIR = OMEGA_SERVICES_DIR / "logs"
PID_DIR = OMEGA_SERVICES_DIR / "pids"

# System log directory (if running as root)
SYSTEM_LOG_DIR = Path("/var/log/omega")

# Ensure directories exist
LOG_DIR.mkdir(exist_ok=True, mode=0o755)
PID_DIR.mkdir(exist_ok=True, mode=0o755)

# Try to create system log dir if we have permissions
try:
    SYSTEM_LOG_DIR.mkdir(exist_ok=True, mode=0o755)
except (PermissionError, OSError):
    pass  # Fall back to local log dir


def get_log_dir() -> Path:
    """Get the appropriate log directory (system if available, local otherwise)"""
    if SYSTEM_LOG_DIR.exists() and os.access(SYSTEM_LOG_DIR, os.W_OK):
        return SYSTEM_LOG_DIR
    return LOG_DIR


def get_pid_file(service_name: str) -> Path:
    """Get PID file path for a service"""
    return PID_DIR / f"{service_name}.pid"


def read_pid(service_name: str) -> Optional[int]:
    """Read PID from file"""
    pid_file = get_pid_file(service_name)
    if not pid_file.exists():
        return None
    
    try:
        with open(pid_file, 'r') as f:
            pid_str = f.read().strip()
            return int(pid_str) if pid_str else None
    except (ValueError, IOError):
        return None


def write_pid(service_name: str, pid: int) -> bool:
    """Write PID to file"""
    pid_file = get_pid_file(service_name)
    try:
        with open(pid_file, 'w') as f:
            f.write(str(pid))
        return True
    except IOError:
        return False


def remove_pid(service_name: str) -> bool:
    """Remove PID file"""
    pid_file = get_pid_file(service_name)
    try:
        if pid_file.exists():
            pid_file.unlink()
        return True
    except IOError:
        return False


def get_log_file(service_name: str, stream: str = "stdout") -> Path:
    """Get log file path for a service"""
    log_dir = get_log_dir()
    return log_dir / f"{service_name}.{stream}.log"


def setup_logging(name: str, level: int = logging.INFO) -> logging.Logger:
    """Setup logging for a module"""
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Avoid duplicate handlers
    if logger.handlers:
        return logger
    
    # File handler
    log_file = get_log_dir() / f"{name}.log"
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(level)
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    
    # Formatter
    formatter = logging.Formatter(
        '[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


def load_json_file(file_path: Path) -> Optional[Dict[str, Any]]:
    """Load JSON file safely"""
    try:
        with open(file_path, 'r') as f:
            return json.load(f)
    except (IOError, json.JSONDecodeError) as e:
        logging.error(f"Failed to load JSON file {file_path}: {e}")
        return None


def save_json_file(file_path: Path, data: Dict[str, Any]) -> bool:
    """Save JSON file safely"""
    try:
        with open(file_path, 'w') as f:
            json.dump(data, f, indent=2)
        return True
    except IOError as e:
        logging.error(f"Failed to save JSON file {file_path}: {e}")
        return False


def format_uptime(seconds: float) -> str:
    """Format uptime in human-readable format"""
    if seconds < 60:
        return f"{int(seconds)}s"
    elif seconds < 3600:
        minutes = int(seconds / 60)
        secs = int(seconds % 60)
        return f"{minutes}m {secs}s"
    else:
        hours = int(seconds / 3600)
        minutes = int((seconds % 3600) / 60)
        return f"{hours}h {minutes}m"


def format_bytes(bytes_count: int) -> str:
    """Format bytes in human-readable format"""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if bytes_count < 1024.0:
            return f"{bytes_count:.1f} {unit}"
        bytes_count /= 1024.0
    return f"{bytes_count:.1f} TB"


def is_process_running(pid: int) -> bool:
    """Check if a process is running"""
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def get_service_registry_path() -> Path:
    """Get path to service registry JSON file"""
    return OMEGA_SERVICES_DIR / "service_registry.json"


def get_base_working_dir() -> Path:
    """Get base working directory for services"""
    return BASE_DIR


def ensure_directories():
    """Ensure all required directories exist"""
    get_log_dir().mkdir(exist_ok=True, mode=0o755)
    PID_DIR.mkdir(exist_ok=True, mode=0o755)


# Initialize directories on import
ensure_directories()

