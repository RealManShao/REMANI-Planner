"""
REMANI API - FastAPI service for kinematic model management
"""

from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager
import logging
import sys
import os

# Add the path to import remani_core_py
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

try:
    import remani_core_py
except ImportError as e:
    logging.error(f"Failed to import remani_core_py: {e}")
    logging.error("Make sure the C++ library is built and installed")
    sys.exit(1)

from remani_api.routers import models, instances, fk, analysis, utils, safety
from remani_api.auth.jwt_handler import verify_token
from remani_api.exceptions.handlers import setup_exception_handlers
from remani_api.services.core_bridge import core_bridge

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global managers
model_manager = None
instance_manager = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifecycle manager"""
    global model_manager, instance_manager
    
    # Initialize C++ managers
    try:
        model_manager = remani_core_py.ModelManager()
        instance_manager = remani_core_py.InstanceManager()
        core_bridge.set_managers(model_manager, instance_manager)
        logger.info("Initialized C++ managers and core bridge")
    except Exception as e:
        logger.error(f"Failed to initialize managers: {e}")
        raise
    
    yield
    
    # Cleanup
    logger.info("Shutting down application")

# Create FastAPI app
app = FastAPI(
    title="REMANI Kinematic API",
    description="Industrial Robot Kinematic Model Management API",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Setup exception handlers
setup_exception_handlers(app)

# Include routers
app.include_router(models.router, prefix="/api/v1/kine", tags=["models"])
app.include_router(instances.router, prefix="/api/v1/kine", tags=["instances"])
app.include_router(fk.router, prefix="/api/v1/fk", tags=["forward-kinematics"])
app.include_router(analysis.router, prefix="/api/v1/fk/analysis", tags=["analysis"])
app.include_router(utils.router, prefix="/api/v1/fk/utils", tags=["utilities"])
app.include_router(safety.router, prefix="/api/v1/safety", tags=["safety"])

# Health check endpoint
@app.get("/health", tags=["system"])
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "version": "1.0.0",
        "models_count": model_manager.get_model_count() if model_manager else 0,
        "instances_count": instance_manager.get_instance_count() if instance_manager else 0
    }

@app.get("/", tags=["system"])
async def root():
    """Root endpoint"""
    return {
        "message": "REMANI Kinematic API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
