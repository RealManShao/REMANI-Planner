"""
Instance management router
"""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import List, Optional
import logging

from remani_api.models.instance import (
    InstanceStatus, ConfigureInstanceRequest, CalibrationRequest, Payload
)
from remani_api.models.responses import ErrorResponse, InstanceResponse
from remani_api.auth.jwt_handler import verify_token
from remani_api.exceptions.handlers import (
    InstanceNotFoundError, DuplicateInstanceError, InvalidParameterError,
    InstanceInMotionError, CppException
)
from remani_api.services.core_bridge import core_bridge, CoreBridge

logger = logging.getLogger(__name__)
router = APIRouter()

def get_core_bridge():
    return core_bridge

@router.put("/instances/{instance_id}", status_code=status.HTTP_201_CREATED, response_model=InstanceResponse)
async def configure_runtime_instance(
    instance_id: str,
    request: ConfigureInstanceRequest,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Configure a new runtime instance"""
    try:
        # Convert request to C++ format
        cpp_instance = bridge.py_to_cpp_instance(instance_id, request)
        
        # Configure instance in C++
        instance = bridge.instance_manager.configure_instance(
            cpp_instance,
            bridge.model_manager
        )
        
        # Convert back to response format
        return bridge.cpp_to_py_instance_response(instance)
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except DuplicateInstanceError as e:
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
        if e.code == "INSTANCE_NOT_FOUND":
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=e.message
            )
        elif e.code == "DUPLICATE_INSTANCE":
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
        logger.error(f"Error configuring instance: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to configure instance"
        )

@router.get("/instances/{instance_id}", response_model=InstanceResponse)
async def get_instance_details(
    instance_id: str,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Get instance details by ID"""
    try:
        instance = bridge.instance_manager.get_instance(instance_id)
        return bridge.cpp_to_py_instance_response(instance)
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error getting instance: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to get instance"
        )

@router.delete("/instances/{instance_id}")
async def delete_instance(
    instance_id: str,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Delete instance by ID"""
    try:
        bridge.instance_manager.delete_instance(instance_id)
        return {"message": f"Instance '{instance_id}' deleted successfully"}
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error deleting instance: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to delete instance"
        )

@router.get("/instances", response_model=List[InstanceResponse])
async def list_instances(
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """List all configured instances"""
    try:
        instance_ids = bridge.instance_manager.get_instance_ids()
        instances = []
        
        for instance_id in instance_ids:
            instance = bridge.instance_manager.get_instance(instance_id)
            instances.append(bridge.cpp_to_py_instance_response(instance))
        
        return instances
        
    except Exception as e:
        logger.error(f"Error listing instances: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to list instances"
        )

@router.patch("/instances/{instance_id}/calibration")
async def calibrate_kinematic_parameters(
    instance_id: str,
    request: CalibrationRequest,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Apply calibration parameters to instance"""
    try:
        # Convert calibration request to C++ format
        cpp_calibration = bridge.py_to_cpp_calibration(request)
        
        # Apply calibration in C++
        bridge.instance_manager.apply_calibration(
            instance_id,
            cpp_calibration,
            bridge.model_manager
        )
        
        return {"message": f"Calibration applied to instance '{instance_id}'"}
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except InstanceInMotionError as e:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=str(e)
        )
    except CppException as e:
        if e.code == "INSTANCE_NOT_FOUND":
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=e.message
            )
        elif e.code == "INSTANCE_IN_MOTION":
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
        logger.error(f"Error applying calibration: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to apply calibration"
        )
