"""
Service Management API Routes

Provides REST endpoints for OmegaOS service orchestrator.
"""

import logging
from typing import Dict, Any, List, Optional
from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel

# Import service manager
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from omega_services.service_manager import get_service_manager

log = logging.getLogger(__name__)

router = APIRouter(prefix="/api/services", tags=["Services"])

# Initialize service manager
_service_manager = None


def get_manager():
    """Get or initialize service manager"""
    global _service_manager
    if _service_manager is None:
        try:
            _service_manager = get_service_manager()
            if not _service_manager.running:
                _service_manager.start()
        except Exception as e:
            log.error(f"Failed to initialize service manager: {e}")
            raise HTTPException(status_code=500, detail=f"Service manager initialization failed: {e}")
    return _service_manager


@router.get("/list")
async def list_services() -> Dict[str, Any]:
    """
    List all services and their status.
    
    Returns:
        List of services with status, PID, health, etc.
    """
    try:
        manager = get_manager()
        services = manager.list_services()
        return {
            "ok": True,
            "services": services,
            "count": len(services)
        }
    except Exception as e:
        log.error(f"Failed to list services: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/status/{name}")
async def get_service_status(name: str) -> Dict[str, Any]:
    """
    Get status of a specific service.
    
    Args:
        name: Service name
    
    Returns:
        Service status with PID, health, metrics, etc.
    """
    # Validate service name to prevent injection
    from api.input_validators import validate_and_sanitize_input
    try:
        name = validate_and_sanitize_input(name, str, "service name", max_length=100)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    
    try:
        manager = get_manager()
        status = manager.get_service_status(name)
        
        if status is None:
            raise HTTPException(status_code=404, detail=f"Service {name} not found")
        
        return {
            "ok": True,
            "service": status
        }
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to get service status for {name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/start/{name}")
async def start_service(name: str) -> Dict[str, Any]:
    """
    Start a service.
    
    Args:
        name: Service name
    
    Returns:
        Success status and message
    """
    # Validate service name
    from api.input_validators import validate_and_sanitize_input
    try:
        name = validate_and_sanitize_input(name, str, "service name", max_length=100)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    
    try:
        manager = get_manager()
        result = manager.start_service(name)
        
        if result.get("success"):
            return {
                "ok": True,
                "message": result.get("message", f"Service {name} started"),
                "service": manager.get_service_status(name)
            }
        else:
            raise HTTPException(
                status_code=500,
                detail=result.get("error", f"Failed to start service {name}")
            )
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to start service {name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/stop/{name}")
async def stop_service(name: str, force: bool = Query(False, description="Force stop")) -> Dict[str, Any]:
    """
    Stop a service.
    
    Args:
        name: Service name
        force: Force stop (kill instead of terminate)
    
    Returns:
        Success status and message
    """
    try:
        manager = get_manager()
        result = manager.stop_service(name, force=force)
        
        if result.get("success"):
            return {
                "ok": True,
                "message": result.get("message", f"Service {name} stopped"),
                "service": manager.get_service_status(name)
            }
        else:
            raise HTTPException(
                status_code=500,
                detail=result.get("error", f"Failed to stop service {name}")
            )
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to stop service {name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/restart/{name}")
async def restart_service(name: str) -> Dict[str, Any]:
    """Restart a service."""
    # Validate service name
    from api.input_validators import validate_and_sanitize_input
    try:
        name = validate_and_sanitize_input(name, str, "service name", max_length=100)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    """
    Restart a service.
    
    Args:
        name: Service name
    
    Returns:
        Success status and message
    """
    try:
        manager = get_manager()
        result = manager.restart_service(name)
        
        if result.get("success"):
            return {
                "ok": True,
                "message": result.get("message", f"Service {name} restarted"),
                "service": manager.get_service_status(name)
            }
        else:
            raise HTTPException(
                status_code=500,
                detail=result.get("error", f"Failed to restart service {name}")
            )
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to restart service {name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/logs/{name}")
async def get_service_logs(name: str, lines: int = Query(50, ge=1, le=1000, description="Number of lines")) -> Dict[str, Any]:
    """
    Get logs for a service.
    
    Args:
        name: Service name
        lines: Number of lines to return
    
    Returns:
        Service logs (stdout and stderr)
    """
    try:
        manager = get_manager()
        logs = manager.get_logs(name, lines=lines)
        
        return {
            "ok": True,
            "service": name,
            "logs": logs
        }
    except Exception as e:
        log.error(f"Failed to get logs for {name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/health/{name}")
async def check_service_health(name: str) -> Dict[str, Any]:
    """
    Run health check for a service.
    
    Args:
        name: Service name
    
    Returns:
        Health check result
    """
    try:
        manager = get_manager()
        status = manager.get_service_status(name)
        
        if status is None:
            raise HTTPException(status_code=404, detail=f"Service {name} not found")
        
        # Get health check from status
        health = status.get("health")
        if health is None:
            return {
                "ok": True,
                "service": name,
                "health": {"healthy": True, "message": "No health check defined"}
            }
        
        return {
            "ok": True,
            "service": name,
            "health": health
        }
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to check health for {name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

