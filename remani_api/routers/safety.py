"""
Safety router - Joint limits, etc.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import List, Optional
import logging

from remani_api.models.responses import JointLimitResponse
from remani_api.auth.jwt_handler import verify_token
from remani_api.exceptions.handlers import (
    InstanceNotFoundError, InvalidParameterError, CppException
)
from remani_api.services.core_bridge import core_bridge, CoreBridge

logger = logging.getLogger(__name__)
router = APIRouter()

def get_core_bridge():
    return core_bridge

@router.post("/joint-limits", response_model=JointLimitResponse)
async def check_joint_limits(
    request: dict,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Check joint limits"""
    try:
        instance_id = request.get('instance_id')
        joints = request.get('joints', [])
        mode = request.get('mode', 'SOFTWARE')
        custom_limits_data = request.get('custom_limits', {})
        
        # Get instance and model
        instance = bridge.instance_manager.get_instance(instance_id)
        model = bridge.model_manager.get_model(instance.model_id)
        solver = bridge.instance_manager.get_kinematic_solver(instance_id, bridge.model_manager)
        
        # Get limits (model limits or custom limits)
        if custom_limits_data:
            # Apply custom limits
            limits = bridge.apply_custom_limits(model.limits, custom_limits_data)
        else:
            limits = model.limits
        
        # Validate joints
        is_valid = solver.validate_joints(joints, limits)
        
        # Get violations
        violations = []
        if not is_valid:
            violations = solver.get_joint_violations(joints, limits)
        
        # Convert violations to response format
        violation_list = []
        for axis, violation_type in violations:
            violation_list.append({
                'axis': axis + 1,  # Convert to 1-based indexing
                'value': joints[axis],
                'limit_min': limits.q_min[axis],
                'limit_max': limits.q_max[axis],
                'violation_type': violation_type
            })
        
        # Current limits for visualization
        current_limits = []
        for i in range(len(limits.q_min)):
            if str(i) in custom_limits_data:
                current_limits.append(custom_limits_data[str(i)])
            else:
                current_limits.append([limits.q_min[i], limits.q_max[i]])
        
        return JointLimitResponse(
            instance_id=instance_id,
            in_range=is_valid,
            violations=violation_list,
            current_limits=current_limits
        )
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error checking joint limits: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to check joint limits"
        )
