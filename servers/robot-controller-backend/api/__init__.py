# api/__init__.py

from fastapi import APIRouter
from .lighting_routes import router as lighting_router
from .autonomy_routes import router as autonomy_router

router = APIRouter()
router.include_router(lighting_router, prefix="/lighting", tags=["Lighting"])
router.include_router(autonomy_router)
