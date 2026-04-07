"""
Forward kinematics router
"""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import List, Optional
import logging

from remani_api.models.instance import Pose, Payload
from remani_api.models.responses import (
    PoseResponse, VelocityResponse, AccelerationResponse
)
from remani_api.auth.jwt_handler import verify_token
from remani_api.exceptions.handlers import (
    InstanceNotFoundError, InvalidParameterError, CppException
)
from remani_api.services.core_bridge import core_bridge, CoreBridge
import time

logger = logging.getLogger(__name__)
router = APIRouter()

def get_core_bridge():
    return core_bridge

@router.post("/compute/pose", response_model=PoseResponse)
async def compute_forward_kinematics(
    request: dict,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Compute forward kinematics - end effector pose"""
    try:
        instance_id = request.get('instance_id')
        joints = request.get('joints', [])
        tcp_data = request.get('tcp')
        ref_frame_data = request.get('ref_frame')
        return_format = request.get('return_format', ['MATRIX', 'QUATERNION'])
        
        # Get instance and solver
        instance = bridge.instance_manager.get_instance(instance_id)
        solver = bridge.instance_manager.get_kinematic_solver(instance_id, bridge.model_manager)
        
        # Convert TCP and reference frame if provided
        tcp = None
        if tcp_data:
            tcp = bridge.py_to_cpp_tcp(tcp_data)
        
        # Compute FK
        start_time = time.time()
        pose_matrix = solver.compute_fk_with_tcp(joints, tcp) if tcp else solver.compute_fk(joints)
        compute_time_us = int((time.time() - start_time) * 1_000_000)
        
        # Check for singularity
        is_singular = solver.is_near_singularity(joints)
        
        # Convert to response format
        return bridge.cpp_to_py_pose_response(
            instance_id, pose_matrix, is_singular, compute_time_us, return_format
        )
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
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
        else:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=e.message
            )
    except Exception as e:
        logger.error(f"Error computing FK: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to compute forward kinematics"
        )

@router.post("/compute/velocity", response_model=VelocityResponse)
async def compute_cartesian_velocity(
    request: dict,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Compute Cartesian velocity from joint velocities"""
    try:
        instance_id = request.get('instance_id')
        joints = request.get('joints', [])
        joint_velocities = request.get('joint_velocities', [])
        tcp_data = request.get('tcp')
        ref_frame_data = request.get('ref_frame')
        
        # Get instance and solver
        instance = bridge.instance_manager.get_instance(instance_id)
        solver = bridge.instance_manager.get_kinematic_solver(instance_id, bridge.model_manager)
        
        # Convert TCP if provided
        tcp = None
        if tcp_data:
            tcp = bridge.py_to_cpp_tcp(tcp_data)
        
        # Compute velocity
        start_time = time.time()
        velocity = solver.compute_velocity(joints, joint_velocities)
        compute_time_us = int((time.time() - start_time) * 1_000_000)
        
        # Check T1 limit (250mm/s)
        linear_speed = velocity.linear_magnitude()
        exceeds_t1_limit = linear_speed > 0.25
        
        # Convert to response format
        return bridge.cpp_to_py_velocity_response(
            instance_id, velocity, compute_time_us, exceeds_t1_limit
        )
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error computing velocity: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to compute Cartesian velocity"
        )

@router.post("/compute/acceleration", response_model=AccelerationResponse)
async def compute_cartesian_acceleration(
    request: dict,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Compute Cartesian acceleration from joint accelerations"""
    try:
        instance_id = request.get('instance_id')
        joints = request.get('joints', [])
        joint_velocities = request.get('joint_velocities', [])
        joint_accelerations = request.get('joint_acc', [])
        tcp_data = request.get('tcp')
        ref_frame_data = request.get('ref_frame')
        payload_data = request.get('payload')
        
        # Get instance and solver
        instance = bridge.instance_manager.get_instance(instance_id)
        solver = bridge.instance_manager.get_kinematic_solver(instance_id, bridge.model_manager)
        
        # Convert TCP if provided
        tcp = None
        if tcp_data:
            tcp = bridge.py_to_cpp_tcp(tcp_data)
        
        # Convert payload if provided
        payload_cpp = None
        if payload_data:
            payload_cpp = bridge.py_to_cpp_payload(payload_data)
        
        # Compute acceleration
        start_time = time.time()
        acceleration = solver.compute_acceleration(joints, joint_velocities, joint_accelerations)
        compute_time_us = int((time.time() - start_time) * 1_000_000)
        
        # Convert to response format
        return bridge.cpp_to_py_acceleration_response(
            instance_id, acceleration, compute_time_us, payload_cpp
        )
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error computing acceleration: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to compute Cartesian acceleration"
        )
