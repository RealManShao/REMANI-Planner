"""
Model management router
"""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import List, Optional
import logging

from remani_api.models.kinematic import (
    RegisterModelRequest, GetModelDetailsRequest, GetCapabilitiesRequest,
    ModelType
)
from remani_api.models.responses import ModelResponse, CapabilitiesResponse, ErrorResponse
from remani_api.auth.jwt_handler import verify_token
from remani_api.exceptions.handlers import (
    ModelNotFoundError, DuplicateModelError, InvalidParameterError,
    CppException
)
from remani_api.services.core_bridge import core_bridge, CoreBridge

logger = logging.getLogger(__name__)
router = APIRouter()

# Dependency to get core bridge
def get_core_bridge():
    return core_bridge

@router.post("/models", status_code=status.HTTP_201_CREATED, response_model=ModelResponse)
async def register_kinematic_model(
    request: RegisterModelRequest,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Register a new kinematic model"""
    try:
        # Convert Pydantic model to C++ parameters
        cpp_params = bridge.py_to_cpp_kinematic_params(request.params, request.model_type)
        cpp_limits = bridge.py_to_cpp_joint_limits(request.limits) if request.limits else None
        
        # Register model in C++
        model = bridge.model_manager.register_model(
            model_id=request.model_id,
            name=request.name or request.model_id,
            model_type=request.model_type,
            params=cpp_params,
            limits=cpp_limits
        )
        
        # Convert back to response format
        return bridge.cpp_to_py_model_response(model)
        
    except DuplicateModelError as e:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=str(e)
        )
    except InvalidParameterError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except CppException as e:
        if e.code == "DUPLICATE_MODEL":
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=e.message
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=e.message
            )
    except Exception as e:
        logger.error(f"Error registering model: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to register model"
        )

@router.get("/models/{model_id}", response_model=ModelResponse)
async def get_kinematic_model_details(
    model_id: str,
    include_state: Optional[bool] = False,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Get model details by ID"""
    try:
        model = bridge.model_manager.get_model(model_id)
        return bridge.cpp_to_py_model_response(model)
        
    except ModelNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error getting model: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get model"
        )

@router.get("/models/{model_id}/capabilities", response_model=CapabilitiesResponse)
async def get_model_capabilities(
    model_id: str,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Get model capabilities"""
    try:
        capabilities = bridge.model_manager.get_model_capabilities(model_id)
        return bridge.cpp_to_py_capabilities_response(capabilities)
        
    except ModelNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error getting model capabilities: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get model capabilities"
        )

@router.delete("/models/{model_id}")
async def delete_kinematic_model(
    model_id: str,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Delete model by ID"""
    try:
        bridge.model_manager.delete_model(model_id)
        return {"message": f"Model '{model_id}' deleted successfully"}
        
    except ModelNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error deleting model: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to delete model"
        )

@router.get("/models", response_model=List[ModelResponse])
async def list_models(
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """List all registered models"""
    try:
        model_ids = bridge.model_manager.get_model_ids()
        models = []
        
        for model_id in model_ids:
            model = bridge.model_manager.get_model(model_id)
            models.append(bridge.cpp_to_py_model_response(model))
        
        return models
        
    except Exception as e:
        logger.error(f"Error listing models: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to list models"
        )
