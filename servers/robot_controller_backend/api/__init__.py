# api/__init__.py

from fastapi import APIRouter
from .lighting_routes import router as lighting_router
from .autonomy_routes import router as autonomy_router
from .ros_routes import router as ros_router
from .capability_routes import router as capability_router
from .system_mode_routes import router as system_mode_router
from .performance_routes import router as performance_router
from .service_routes import router as service_router
from .config_routes import router as config_router
from .movement_routes import router as movement_router
from .sensor_ws_routes import router as sensor_ws_router
from .sensor_power_routes import router as sensor_power_router

# Import unified network routes from network module
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from network.api.network_routes import router as network_router

router = APIRouter()
router.include_router(movement_router, tags=["Movement"])
router.include_router(sensor_ws_router, tags=["Sensors"])
router.include_router(sensor_power_router)
router.include_router(lighting_router, prefix="/lighting", tags=["Lighting"])
router.include_router(autonomy_router)
router.include_router(ros_router)
router.include_router(capability_router)
router.include_router(system_mode_router)
router.include_router(performance_router)
router.include_router(network_router)
router.include_router(service_router)
router.include_router(config_router)
