# api/__init__.py

from fastapi import APIRouter
from .lighting_routes import router as lighting_router

router = APIRouter()
router.include_router(lighting_router, prefix="/lighting", tags=["Lighting"])

