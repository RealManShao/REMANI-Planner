"""
Analysis router - Jacobian, singularity, configuration, etc.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import List, Optional
import logging

from remani_api.models.responses import (
    JacobianResponse, SingularityResponse, ConfigFlagsResponse,
    StaticTorqueResponse
)
from remani_api.auth.jwt_handler import verify_token
from remani_api.exceptions.handlers import (
    InstanceNotFoundError, InvalidParameterError, CppException
)
from remani_api.services.core_bridge import core_bridge, CoreBridge

logger = logging.getLogger(__name__)
router = APIRouter()

def get_core_bridge():
    return core_bridge

@router.post("/jacobian", response_model=JacobianResponse)
async def compute_jacobian_matrix(
    request: dict,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Compute geometric Jacobian matrix"""
    try:
        instance_id = request.get('instance_id')
        joints = request.get('joints', [])
        frame = request.get('frame', 'BASE')
        tcp_offset_data = request.get('tcp_offset')
        
        # Get instance and solver
        instance = bridge.instance_manager.get_instance(instance_id)
        solver = bridge.instance_manager.get_kinematic_solver(instance_id, bridge.model_manager)
        
        # Convert TCP offset if provided
        tcp_offset = None
        if tcp_offset_data:
            tcp_offset = bridge.py_to_cpp_tcp(tcp_offset_data)
        
        # Compute Jacobian
        jacobian = solver.compute_jacobian_with_tcp(joints, tcp_offset) if tcp_offset else solver.compute_jacobian(joints)
        
        # Compute analysis metrics
        analysis = {}
        try:
            import numpy as np
            j_np = np.array(jacobian)
            
            # Determinant (for square matrices)
            if j_np.shape[0] == j_np.shape[1]:
                analysis['determinant'] = float(np.linalg.det(j_np))
            
            # Manipulability measure
            jjt = j_np @ j_np.T
            analysis['manipulability_measure'] = float(np.sqrt(np.linalg.det(jjt)))
            
            # Condition number
            if j_np.shape[0] == j_np.shape[1]:
                analysis['condition_number'] = float(np.linalg.cond(j_np))
            
            # Singularity detection
            analysis['is_near_singularity'] = solver.is_near_singularity(joints)
            
        except ImportError:
            # numpy not available, skip analysis
            pass
        except Exception as e:
            logger.warning(f"Error computing Jacobian analysis: {e}")
        
        # Convert to response format
        return bridge.cpp_to_py_jacobian_response(instance_id, jacobian, analysis)
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error computing Jacobian: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to compute Jacobian matrix"
        )

@router.post("/singularity-check", response_model=SingularityResponse)
async def check_singularity(
    request: dict,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Check if configuration is near singularity"""
    try:
        instance_id = request.get('instance_id')
        joints = request.get('joints', [])
        tolerance_data = request.get('tolerance', {})
        method = request.get('method', 'GEOMETRIC')
        
        # Get instance and solver
        instance = bridge.instance_manager.get_instance(instance_id)
        solver = bridge.instance_manager.get_kinematic_solver(instance_id, bridge.model_manager)
        
        # Extract tolerance values
        wrist_tolerance = tolerance_data.get('wrist', 0.087)  # 5 degrees
        elbow_tolerance = tolerance_data.get('elbow', 0.05)
        
        # Check singularity
        is_singular = solver.is_near_singularity(joints, wrist_tolerance)
        min_singular_value = solver.get_min_singular_value(joints)
        
        # Get detailed singularity information
        details = {
            'wrist': {
                'status': False,
                'current_value': 0.0,
                'threshold': wrist_tolerance
            },
            'elbow': {
                'status': False,
                'distance_to_boundary': 0.15
            },
            'shoulder': {
                'status': False,
                'offset': 0.002
            }
        }
        
        # TODO: Implement detailed singularity analysis for specific robot types
        # This would require robot-specific logic
        
        # Identify singularity types
        singularity_types = []
        if is_singular:
            singularity_types.append("WRIST_SINGULARITY")
        
        # Convert to response format
        return bridge.cpp_to_py_singularity_response(
            instance_id, is_singular, singularity_types, details, min_singular_value
        )
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error checking singularity: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to check singularity"
        )

@router.post("/config-flags", response_model=ConfigFlagsResponse)
async def identify_configuration(
    request: dict,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Identify configuration flags (shoulder/elbow/wrist)"""
    try:
        instance_id = request.get('instance_id')
        joints = request.get('joints', [])
        convention = request.get('convention', 'efort')
        
        # Get instance and solver
        instance = bridge.instance_manager.get_instance(instance_id)
        solver = bridge.instance_manager.get_kinematic_solver(instance_id, bridge.model_manager)
        
        # Identify configuration
        config_id, flags = solver.identify_configuration(joints)
        
        # Compute turn numbers (for multi-turn joints)
        turns = [0] * len(joints)  # TODO: Implement turn number calculation
        
        # Convert to response format
        return bridge.cpp_to_py_config_flags_response(instance_id, config_id, flags, turns)
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error identifying configuration: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to identify configuration"
        )

@router.post("/static-torque", response_model=StaticTorqueResponse)
async def compute_static_load_torque(
    request: dict,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Compute static load torque for gravity compensation"""
    try:
        instance_id = request.get('instance_id')
        joints = request.get('joints', [])
        payload_data = request.get('payload')
        gravity_data = request.get('gravity')
        external_wrench_data = request.get('external_wrench')
        
        # Get instance and solver
        instance = bridge.instance_manager.get_instance(instance_id)
        solver = bridge.instance_manager.get_kinematic_solver(instance_id, bridge.model_manager)
        
        # Convert payload if provided
        payload_cpp = None
        if payload_data:
            payload_cpp = bridge.py_to_cpp_payload(payload_data)
        
        # Convert gravity vector
        gravity = [0.0, 0.0, -9.81]
        if gravity_data:
            gravity = gravity_data
        elif hasattr(instance, 'gravity_vector'):
            gravity = instance.gravity_vector
        
        # Compute static torque
        gravity_torques = solver.compute_static_torque(joints, gravity, payload_cpp) if payload_cpp else solver.compute_static_torque(joints, gravity)
        
        # Load analysis
        load_analysis = []
        dof = solver.get_dof()
        for i in range(dof):
            # TODO: Get actual rated torque from model limits if available
            rated = 100.0
            load_analysis.append({
                'axis': i + 1,
                'torque': float(gravity_torques[i]),
                'rated': rated,
                'ratio': abs(float(gravity_torques[i])) / rated
            })
        
        max_load_ratio = max([item['ratio'] for item in load_analysis])
        is_overloaded = max_load_ratio > 1.0
        
        # Convert to response format
        return bridge.cpp_to_py_static_torque_response(
            instance_id, gravity_torques, load_analysis, is_overloaded, max_load_ratio
        )
        
    except InstanceNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error computing static torque: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to compute static load torque"
        )
