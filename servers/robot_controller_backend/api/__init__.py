# api/__init__.py

from fastapi import APIRouter
from .lighting_routes import router as lighting_router
from .autonomy_routes import router as autonomy_router
from .ros_routes import router as ros_router
from .capability_routes import router as capability_router
from .system_mode_routes import router as system_mode_router
from .network_routes import router as network_router

router = APIRouter()
router.include_router(lighting_router, prefix="/lighting", tags=["Lighting"])
router.include_router(autonomy_router)
router.include_router(ros_router)
router.include_router(capability_router)
router.include_router(system_mode_router)
router.include_router(network_router)
