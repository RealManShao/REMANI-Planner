"""
Utility router - Transform calculations, DH matrix, etc.
"""

from fastapi import APIRouter, Depends, HTTPException, status
import logging

import numpy as np
import remani_core_py
from remani_api.models.responses import TransformResponse
from remani_api.auth.jwt_handler import verify_token
from remani_api.exceptions.handlers import (
    InvalidParameterError, CppException
)
from remani_api.services.core_bridge import core_bridge, CoreBridge

logger = logging.getLogger(__name__)
router = APIRouter()

def get_core_bridge():
    return core_bridge

@router.post("/transform/calcTransform", response_model=TransformResponse)
async def calc_transform(
    request: dict,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Compute transformation matrix operations"""
    try:
        operation = request.get('operation')
        matrix_a = request.get('matrix_a')
        matrix_b = request.get('matrix_b')
        format_type = request.get('format', 'MATRIX')
        
        if not matrix_a:
            raise InvalidParameterError("matrix_a is required")
        
        # Convert matrices to numpy arrays if needed
        import numpy as np
        
        if isinstance(matrix_a, dict):
            # Convert pose format to matrix
            if 'pos' in matrix_a and 'euler' in matrix_a:
                matrix_a = bridge.pose_to_matrix(matrix_a['pos'], matrix_a['euler'])
            elif 'pos' in matrix_a and 'rot' in matrix_a:
                matrix_a = bridge.quat_to_matrix(matrix_a['pos'], matrix_a['rot'])
            else:
                raise InvalidParameterError("Invalid matrix_a format")
        
        if matrix_b and isinstance(matrix_b, dict):
            if 'pos' in matrix_b and 'euler' in matrix_b:
                matrix_b = bridge.pose_to_matrix(matrix_b['pos'], matrix_b['euler'])
            elif 'pos' in matrix_b and 'rot' in matrix_b:
                matrix_b = bridge.quat_to_matrix(matrix_b['pos'], matrix_b['rot'])
            else:
                raise InvalidParameterError("Invalid matrix_b format")
        
        # Convert to numpy arrays
        if isinstance(matrix_a, list):
            matrix_a = np.array(matrix_a)
        if isinstance(matrix_b, list):
            matrix_b = np.array(matrix_b)
        
        # Perform operation
        if operation == 'MULTIPLY':
            if matrix_b is None:
                raise InvalidParameterError("matrix_b is required for MULTIPLY operation")
            result = matrix_a @ matrix_b
            
        elif operation == 'INVERT':
            result = np.linalg.inv(matrix_a)
            
        elif operation == 'REL_TO':
            if matrix_b is None:
                raise InvalidParameterError("matrix_b is required for REL_TO operation")
            result = np.linalg.inv(matrix_a) @ matrix_b
            
        else:
            raise InvalidParameterError(f"Unknown operation: {operation}")
        
        # Convert result to requested format
        if format_type == 'POSE_ZYX':
            pos = result[:3, 3].tolist()
            euler = bridge.matrix_to_euler_zyx(result).tolist()
            result_data = {'pos': pos, 'euler': euler}
        elif format_type == 'POSE_QUAT':
            pos = result[:3, 3].tolist()
            quat = bridge.matrix_to_quaternion(result).tolist()
            result_data = {'pos': pos, 'quat': quat}
        else:  # MATRIX
            result_data = result.tolist()
        
        return TransformResponse(
            operation=operation,
            result_pose=result_data if format_type != 'MATRIX' else None,
            result_matrix=result_data if format_type == 'MATRIX' else None
        )
        
    except InvalidParameterError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except np.linalg.LinAlgError as e:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=f"Matrix operation failed: {e}"
        )
    except Exception as e:
        logger.error(f"Error computing transform: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to compute transform"
        )

@router.post("/dh-calc/calcLinkMatrix", response_model=TransformResponse)
async def calc_link_matrix(
    request: dict,
    payload: dict = Depends(verify_token),
    bridge: CoreBridge = Depends(get_core_bridge)
):
    """Calculate single link DH transformation matrix"""
    try:
        dh_type = request.get('type', 'MDH')
        params = request.get('params')
        format_type = request.get('format', 'MATRIX')
        
        if not params:
            raise InvalidParameterError("params are required")
        
        theta = params.get('theta', 0.0)
        d = params.get('d', 0.0)
        a = params.get('a', 0.0)
        alpha = params.get('alpha', 0.0)
        
        # Create MDH joint
        joint_type = 'REVOLUTE'  # Default, could be parameterized
        joint = remani_core_py.MDHJoint(
            0,
            remani_core_py.JointType[joint_type],
            a,
            alpha,
            d,
            0.0  # theta_offset
        )
        
        # Compute transformation matrix
        if dh_type == 'MDH':
            result = remani_core_py.MDHModel.compute_mdh_transform(joint, theta)
        else:  # SDH
            # TODO: Implement SDH transformation
            raise InvalidParameterError("SDH not yet implemented")
        
        # Convert result to requested format
        result_array = result.tolist()
        
        if format_type == 'POSE_ZYX':
            pos = result_array[0][3], result_array[1][3], result_array[2][3]
            euler = bridge.matrix_to_euler_zyx(result).tolist()
            result_data = {'pos': pos, 'euler': euler}
        elif format_type == 'POSE_QUAT':
            pos = result_array[0][3], result_array[1][3], result_array[2][3]
            quat = bridge.matrix_to_quaternion(result).tolist()
            result_data = {'pos': pos, 'quat': quat}
        else:  # MATRIX
            result_data = result_array
        
        return TransformResponse(
            operation=dh_type,
            result_pose=result_data if format_type != 'MATRIX' else None,
            result_matrix=result_data if format_type == 'MATRIX' else None
        )
        
    except InvalidParameterError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        logger.error(f"Error calculating DH matrix: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to calculate DH matrix"
        )
