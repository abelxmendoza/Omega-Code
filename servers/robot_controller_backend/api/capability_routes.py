"""
Capability API Routes

Exposes system capabilities to the frontend via REST API.
"""

from fastapi import APIRouter, Query
from fastapi.responses import JSONResponse
from typing import Dict, Any
import logging

from .capability_service import get_capability_service

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/capabilities", tags=["capabilities"])


@router.get("/")
async def get_capabilities(refresh: bool = Query(False, description="Force refresh capabilities")):
    """
    Get current system capability profile.
    
    Returns capability profile including:
    - profile_mode: mac, lenovo, or jetson
    - ml_capable: Whether ML/GPU processing is available
    - slam_capable: Whether SLAM is available
    - max_resolution: Maximum supported camera resolution
    - max_fps: Maximum supported FPS
    - And more...
    """
    try:
        service = get_capability_service()
        profile = service.get_capabilities(refresh=refresh)
        
        return JSONResponse(content={
            "ok": True,
            "capabilities": profile
        })
    except Exception as e:
        logger.error(f"Failed to get capabilities: {e}")
        return JSONResponse(
            content={"ok": False, "error": str(e)},
            status_code=500
        )


@router.get("/check")
async def check_capability(
    feature: str = Query(..., description="Feature to check: ml_capable, slam_capable, etc.")
):
    """
    Check if a specific capability is available.
    
    Example: /api/capabilities/check?feature=ml_capable
    """
    try:
        service = get_capability_service()
        profile = service.get_capabilities()
        
        # Map feature names to methods
        feature_map = {
            "ml_capable": service.is_ml_capable,
            "slam_capable": service.is_slam_capable,
            "tracking": lambda: profile.get("tracking", True),
            "aruco": lambda: profile.get("aruco", True),
            "motion_detection": lambda: profile.get("motion_detection", True),
            "face_recognition": lambda: profile.get("face_recognition", False),
            "yolo": lambda: profile.get("yolo", False),
        }
        
        if feature in feature_map:
            result = feature_map[feature]()
            return JSONResponse(content={
                "ok": True,
                "feature": feature,
                "available": result
            })
        else:
            # Check if it's a direct profile key
            result = profile.get(feature, False)
            return JSONResponse(content={
                "ok": True,
                "feature": feature,
                "available": result
            })
    except Exception as e:
        logger.error(f"Failed to check capability: {e}")
        return JSONResponse(
            content={"ok": False, "error": str(e)},
            status_code=500
        )


@router.get("/resolution")
async def get_max_resolution():
    """Get maximum supported camera resolution."""
    try:
        service = get_capability_service()
        width, height = service.get_max_resolution()
        return JSONResponse(content={
            "ok": True,
            "width": width,
            "height": height,
            "resolution": f"{width}x{height}"
        })
    except Exception as e:
        logger.error(f"Failed to get resolution: {e}")
        return JSONResponse(
            content={"ok": False, "error": str(e)},
            status_code=500
        )


@router.get("/fps")
async def get_max_fps():
    """Get maximum supported FPS."""
    try:
        service = get_capability_service()
        fps = service.get_max_fps()
        return JSONResponse(content={
            "ok": True,
            "fps": fps
        })
    except Exception as e:
        logger.error(f"Failed to get FPS: {e}")
        return JSONResponse(
            content={"ok": False, "error": str(e)},
            status_code=500
        )


@router.get("/profile")
async def get_profile_mode():
    """Get current profile mode (mac, lenovo, or jetson)."""
    try:
        service = get_capability_service()
        mode = service.get_profile_mode()
        return JSONResponse(content={
            "ok": True,
            "profile_mode": mode
        })
    except Exception as e:
        logger.error(f"Failed to get profile: {e}")
        return JSONResponse(
            content={"ok": False, "error": str(e)},
            status_code=500
        )

