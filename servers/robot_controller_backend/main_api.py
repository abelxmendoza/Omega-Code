# main_api.py

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from api import router as api_router
import uvicorn

app = FastAPI(title="Omega Robot Controller API")

# Configure CORS to allow frontend connections
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include your modular routes
app.include_router(api_router)

if __name__ == "__main__":
    uvicorn.run("main_api:app", host="0.0.0.0", port=8000, reload=True)

