"""
Configuration Management API Routes

Provides REST endpoints for OmegaOS configuration management.
"""

import logging
from typing import Dict, Any, List, Optional
from fastapi import APIRouter, HTTPException, Body, Query
from pydantic import BaseModel, Field

# Import config manager
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from omega_config.config_manager import get_config_manager

log = logging.getLogger(__name__)

router = APIRouter(prefix="/api/config", tags=["Configuration"])

# Initialize config manager
_config_manager = None


def get_manager():
    """Get or initialize config manager"""
    global _config_manager
    if _config_manager is None:
        try:
            _config_manager = get_config_manager()
        except Exception as e:
            log.error(f"Failed to initialize config manager: {e}")
            raise HTTPException(status_code=500, detail=f"Config manager initialization failed: {e}")
    return _config_manager


class ConfigUpdateRequest(BaseModel):
    """Request model for updating configuration section"""
    section: str = Field(..., description="Configuration section name")
    data: Dict[str, Any] = Field(..., description="Configuration data to update")


class ConfigImportRequest(BaseModel):
    """Request model for importing full configuration"""
    config: Dict[str, Any]
    profile: Optional[Dict[str, Any]] = None
    hardware: Optional[Dict[str, Any]] = None
    state: Optional[Dict[str, Any]] = None


@router.get("/")
async def get_config() -> Dict[str, Any]:
    """
    Get full robot configuration.
    
    Returns:
        Complete configuration including robot, network, services, etc.
    """
    try:
        manager = get_manager()
        config = manager.get_config()
        
        return {
            "ok": True,
            "config": config,
            "version": "1.0"
        }
    except Exception as e:
        log.error(f"Failed to get config: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/{section}")
async def get_config_section(section: str) -> Dict[str, Any]:
    """
    Get a specific configuration section.
    
    Args:
        section: Section name (robot, network, services, camera, movement, lighting, logging, security, telemetry)
    
    Returns:
        Configuration section data
    """
    # Validate section name to prevent path traversal
    from api.input_validators import validate_path_traversal
    try:
        section = validate_path_traversal(section)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    """
    try:
        manager = get_manager()
        section_data = manager.get_section(section)
        
        if section_data is None:
            raise HTTPException(status_code=404, detail=f"Section '{section}' not found")
        
        return {
            "ok": True,
            "section": section,
            "data": section_data
        }
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to get config section {section}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/{section}")
async def update_config_section(
    section: str,
    data: Dict[str, Any] = Body(..., description="Configuration data to update")
) -> Dict[str, Any]:
    """
    Update a configuration section.
    
    Args:
        section: Section name
        data: Configuration data to update
    
    Returns:
        Updated configuration section
    """
    try:
        manager = get_manager()
        success = manager.update_section(section, data)
        
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to update section '{section}'")
        
        # Validate after update
        valid, errors = manager.validate_config()
        if not valid:
            log.warning(f"Config validation errors after update: {errors}")
        
        updated_section = manager.get_section(section)
        
        return {
            "ok": True,
            "message": f"Section '{section}' updated successfully",
            "section": section,
            "data": updated_section,
            "validation_errors": errors if not valid else []
        }
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to update config section {section}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/profile/{profile_name}")
async def get_profile(profile_name: Optional[str] = None) -> Dict[str, Any]:
    """
    Get robot profile configuration.
    
    Args:
        profile_name: Profile name (pi4b, jetson, dev). If None, returns active profile.
    
    Returns:
        Profile configuration
    """
    try:
        manager = get_manager()
        profile = manager.get_profile(profile_name)
        
        if profile is None:
            raise HTTPException(status_code=404, detail=f"Profile '{profile_name}' not found")
        
        return {
            "ok": True,
            "profile": profile_name or "active",
            "data": profile
        }
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to get profile {profile_name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/hardware/map")
async def get_hardware_map() -> Dict[str, Any]:
    """
    Get hardware device mapping.
    
    Returns:
        Hardware mapping including GPIO pins, I2C addresses, etc.
    """
    try:
        manager = get_manager()
        hardware = manager.get_hardware_map()
        
        if hardware is None:
            raise HTTPException(status_code=404, detail="Hardware map not found")
        
        return {
            "ok": True,
            "hardware": hardware
        }
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to get hardware map: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/hardware/map")
async def update_hardware_map(
    hardware: Dict[str, Any] = Body(..., description="Hardware mapping data")
) -> Dict[str, Any]:
    """
    Update hardware device mapping.
    
    Args:
        hardware: Hardware mapping data
    
    Returns:
        Updated hardware mapping
    """
    try:
        manager = get_manager()
        # This would require adding update_hardware_map method to ConfigManager
        # For now, return error
        raise HTTPException(status_code=501, detail="Hardware map update not yet implemented")
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to update hardware map: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/state")
async def get_state() -> Dict[str, Any]:
    """
    Get persistent runtime state.
    
    Returns:
        Persistent state data
    """
    try:
        manager = get_manager()
        state = manager.get_state()
        
        return {
            "ok": True,
            "state": state
        }
    except Exception as e:
        log.error(f"Failed to get state: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/state/{key}")
async def update_state(
    key: str,
    value: Any = Body(..., description="Value to set")
) -> Dict[str, Any]:
    """
    Update persistent state value.
    
    Args:
        key: State key
        value: Value to set
    
    Returns:
        Updated state
    """
    try:
        manager = get_manager()
        success = manager.update_state(key, value)
        
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to update state key '{key}'")
        
        return {
            "ok": True,
            "message": f"State '{key}' updated successfully",
            "key": key,
            "value": value
        }
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to update state {key}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/export")
async def export_config() -> Dict[str, Any]:
    """
    Export full configuration as JSON.
    
    Returns:
        Complete configuration export
    """
    try:
        manager = get_manager()
        export_data = manager.export_config()
        
        return {
            "ok": True,
            **export_data
        }
    except Exception as e:
        log.error(f"Failed to export config: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/import")
async def import_config(request: ConfigImportRequest) -> Dict[str, Any]:
    """
    Import full configuration from JSON.
    
    Args:
        request: Configuration import data
    
    Returns:
        Import result with validation errors if any
    """
    try:
        manager = get_manager()
        
        import_data = {
            "config": request.config,
            "profile": request.profile,
            "hardware": request.hardware,
            "state": request.state
        }
        
        success, errors = manager.import_config(import_data)
        
        if not success:
            return {
                "ok": False,
                "errors": errors,
                "message": "Configuration import failed"
            }
        
        return {
            "ok": True,
            "message": "Configuration imported successfully",
            "errors": []
        }
    except Exception as e:
        log.error(f"Failed to import config: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/validate")
async def validate_config() -> Dict[str, Any]:
    """
    Validate current configuration.
    
    Returns:
        Validation result with any errors
    """
    try:
        manager = get_manager()
        valid, errors = manager.validate_config()
        
        return {
            "ok": valid,
            "valid": valid,
            "errors": errors
        }
    except Exception as e:
        log.error(f"Failed to validate config: {e}")
        raise HTTPException(status_code=500, detail=str(e))

