"""
System Mode Management API Routes

Provides REST endpoints for system mode management.
"""

import logging
from typing import Dict, Any
from fastapi import APIRouter, HTTPException, Body
from pydantic import BaseModel, Field

from video.system_state import get_system_state, SystemMode

# Try to import hybrid system manager (optional)
try:
    from video.hybrid_system import get_hybrid_system_manager
    HYBRID_SYSTEM_AVAILABLE = True
except ImportError:
    get_hybrid_system_manager = None
    HYBRID_SYSTEM_AVAILABLE = False

log = logging.getLogger(__name__)

router = APIRouter(prefix="/system/mode", tags=["System Mode"])


class SetModeRequest(BaseModel):
    """Request model for setting system mode."""
    mode: int = Field(..., ge=0, le=7, description="System mode (0-7)")


@router.get("/list")
async def list_system_modes() -> Dict[str, Any]:
    """
    List all available system modes.
    
    Returns:
        Dict with available modes and descriptions
    """
    try:
        system_state = get_system_state()
        modes = system_state.list_modes()
        
        return {
            "ok": True,
            "modes": modes,
            "current_mode": system_state.get_current_mode(),
        }
    except Exception as e:
        log.error(f"Failed to list system modes: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/status")
async def get_system_mode_status() -> Dict[str, Any]:
    """
    Get current system mode status.
    
    Returns:
        Dict with current mode, description, and status
    """
    try:
        system_state = get_system_state()
        status = system_state.get_status()
        
        # Add hybrid system info if available
        hybrid_info = {}
        if HYBRID_SYSTEM_AVAILABLE and get_hybrid_system_manager is not None:
            try:
                hybrid_manager = get_hybrid_system_manager()
                hybrid_info = {
                    "hybrid_mode": hybrid_manager.get_system_mode().value,
                    "orin_available": hybrid_manager.orin_available,
                    "thermal_temp": hybrid_manager.thermal_monitor.get_temperature(),
                    "cpu_load": hybrid_manager.cpu_monitor.get_load(),
                    "throttling": hybrid_manager.should_throttle_modules(),
                }
            except Exception:
                pass
        
        return {
            "ok": True,
            **status,
            **hybrid_info
        }
    except Exception as e:
        log.error(f"Failed to get system mode status: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/set")
async def set_system_mode(request: SetModeRequest = Body(...)) -> Dict[str, Any]:
    """
    Set system mode.
    
    Args:
        request: SetModeRequest with mode number (0-7)
        
    Returns:
        Dict with success status and new mode info
    """
    try:
        system_state = get_system_state()
        
        # Validate mode
        if request.mode < 0 or request.mode > 7:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid mode: {request.mode}. Must be 0-7"
            )
        
        # Set mode in system state
        success = system_state.set_mode(request.mode, manual=True)
        
        if not success:
            raise HTTPException(
                status_code=500,
                detail="Failed to set system mode"
            )
        
        # Also update hybrid system manager if available
        if HYBRID_SYSTEM_AVAILABLE and get_hybrid_system_manager is not None:
            try:
                hybrid_manager = get_hybrid_system_manager()
                hybrid_manager.set_manual_mode(request.mode)
            except Exception as e:
                log.warning(f"Failed to update hybrid system manager: {e}")
        
        # Get updated status
        status = system_state.get_status()
        
        return {
            "ok": True,
            "message": f"System mode set to {request.mode}",
            **status
        }
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Failed to set system mode: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/history")
async def get_mode_history(limit: int = 10) -> Dict[str, Any]:
    """
    Get mode change history.
    
    Args:
        limit: Maximum number of history entries to return
        
    Returns:
        Dict with mode change history
    """
    try:
        system_state = get_system_state()
        history = system_state.get_mode_history(limit=limit)
        
        return {
            "ok": True,
            "history": history,
            "count": len(history)
        }
    except Exception as e:
        log.error(f"Failed to get mode history: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

