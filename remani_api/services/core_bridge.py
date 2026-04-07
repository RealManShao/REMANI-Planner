"""
Bridge between Python API and C++ core library
"""

import logging
import math
from typing import Dict, Any, List, Optional
import time

try:
    import remani_core_py
except ImportError as e:
    logging.error(f"Failed to import remani_core_py: {e}")
    raise

from remani_api.models.kinematic import (
    ModelType, JointType, MDHJoint, POEParams, URDFParams, DeltaParams,
    JointLimits, KinematicParams, CalibrationData,
    MDHCalibrationCorrection, POECalibrationCorrection
)
from remani_api.models.instance import (
    InstanceStatus, MountingType, BaseFrame, TCPDefinition,
    ConfigureInstanceRequest, CalibrationRequest, Payload
)
from remani_api.models.responses import (
    ModelResponse, CapabilitiesResponse, InstanceResponse
)

logger = logging.getLogger(__name__)

class CoreBridge:
    """Bridge between Python FastAPI and C++ core library"""
    
    def __init__(self):
        # These will be injected from main.py
        self.model_manager = None
        self.instance_manager = None
    
    def set_managers(self, model_manager, instance_manager):
        """Set manager instances from main.py"""
        self.model_manager = model_manager
        self.instance_manager = instance_manager
    
    # ========================================================================
    # Conversion utilities
    # ========================================================================
    
    def py_to_cpp_kinematic_params(self, params: Dict[str, Any], model_type: ModelType):
        """Convert Python kinematic params to C++ format"""
        cpp_params = remani_core_py.KinematicParams()
        cpp_params.model_type = model_type
        cpp_params.dof = params.get('dof', 0)
        
        if model_type in [ModelType.MDH, ModelType.SDH]:
            # Convert MDH chain
            chain_data = params.get('chain', [])
            cpp_mdh_joints = []
            for joint_data in chain_data:
                joint = remani_core_py.MDHJoint(
                    joint_data['joint_index'],
                    remani_core_py.JointType[joint_data['type']],
                    joint_data['a'],
                    joint_data['alpha'],
                    joint_data['d'],
                    joint_data.get('theta_offset', 0.0)
                )
                cpp_mdh_joints.append(joint)
            cpp_params.set_mdh(remani_core_py.MDHParams(cpp_mdh_joints))
            
        elif model_type == ModelType.POE:
            # Convert POE parameters
            screw_axes = params.get('screw_axes', [])
            cpp_screw_axes = []
            for axis_data in screw_axes:
                screw_axis = remani_core_py.POEScrewAxis(axis_data)
                cpp_screw_axes.append(screw_axis)
            
            m_home = params.get('m_home', [])
            cpp_poe = remani_core_py.POEParams(cpp_screw_axes, m_home)
            cpp_params.set_poe(cpp_poe)
            
        elif model_type == ModelType.URDF:
            # Convert URDF parameters
            cpp_urdf = remani_core_py.URDFParams(
                params.get('urdf_content', ''),
                params.get('base_link', ''),
                params.get('tip_link', '')
            )
            cpp_params.set_urdf(cpp_urdf)
            
        elif model_type == ModelType.DELTA:
            # Convert Delta parameters
            coupling_mode = "ROTARY" if params.get('coupling_mode') == "ROTARY" else "LINEAR"
            cpp_delta = remani_core_py.DeltaParams(
                params.get('R_base', 0.0),
                params.get('R_platform', 0.0),
                params.get('L_arm', 0.0),
                params.get('L_rod', 0.0),
                remani_core_py.DeltaCouplingMode[coupling_mode]
            )
            cpp_params.set_delta(cpp_delta)
        
        return cpp_params
    
    def py_to_cpp_joint_limits(self, limits: Optional[JointLimits]) -> Optional[remani_core_py.JointLimits]:
        """Convert Python joint limits to C++ format"""
        if not limits:
            return None
        
        cpp_limits = remani_core_py.JointLimits()
        cpp_limits.q_min = limits.q_min
        cpp_limits.q_max = limits.q_max
        cpp_limits.v_max = limits.v_max or []
        cpp_limits.a_max = limits.a_max or []
        
        return cpp_limits
    
    def cpp_to_py_model_response(self, cpp_model) -> ModelResponse:
        """Convert C++ model to Python response"""
        return ModelResponse(
            model_id=cpp_model.model_id,
            name=cpp_model.name,
            model_type=cpp_model.model_type.value if hasattr(cpp_model.model_type, 'value') else str(cpp_model.model_type),
            dof=cpp_model.dof,
            status=cpp_model.status.value if hasattr(cpp_model.status, 'value') else str(cpp_model.status),
            params=self._cpp_params_to_dict(cpp_model.params),
            limits=self._cpp_limits_to_dict(cpp_model.limits),
            created_at=cpp_model.created_at,
            last_updated=cpp_model.last_updated,
            solver_engine=getattr(cpp_model, 'solver_engine', None),
            total_mass=getattr(cpp_model, 'total_mass', None)
        )
    
    def cpp_to_py_capabilities_response(self, cpp_caps) -> CapabilitiesResponse:
        """Convert C++ capabilities to Python response"""
        return CapabilitiesResponse(
            model_id=cpp_caps.model_id,
            structure_type=getattr(cpp_caps, 'structure_type', 'SERIAL'),
            kinematics=self._cpp_kinematic_caps_to_dict(getattr(cpp_caps, 'kinematics', None)),
            differential=self._cpp_differential_caps_to_dict(getattr(cpp_caps, 'differential', None)),
            dynamics=self._cpp_dynamics_caps_to_dict(getattr(cpp_caps, 'dynamics', None)),
            performance_hints=self._cpp_perf_hints_to_dict(getattr(cpp_caps, 'performance_hints', None))
        )
    
    def _cpp_params_to_dict(self, cpp_params) -> Dict[str, Any]:
        """Convert C++ parameters to dictionary"""
        try:
            model_type = cpp_params.model_type
            if model_type in [remani_core_py.ModelType.MDH, remani_core_py.ModelType.SDH]:
                mdh_params = cpp_params.get_mdh()
                chain = []
                for joint in mdh_params.chain:
                    chain.append({
                        'joint_index': joint.joint_index,
                        'type': joint.type.value if hasattr(joint.type, 'value') else str(joint.type),
                        'a': joint.a,
                        'alpha': joint.alpha,
                        'd': joint.d,
                        'theta_offset': joint.theta_offset
                    })
                return {'chain': chain}
                
            elif model_type == remani_core_py.ModelType.POE:
                poe_params = cpp_params.get_poe()
                screw_axes = []
                for axis in poe_params.screw_axes:
                    screw_axes.append(axis.data.tolist())
                return {
                    'screw_axes': screw_axes,
                    'm_home': poe_params.m_home.tolist()
                }
                
            elif model_type == remani_core_py.ModelType.URDF:
                urdf_params = cpp_params.get_urdf()
                return {
                    'urdf_content': urdf_params.urdf_content,
                    'base_link': urdf_params.base_link,
                    'tip_link': urdf_params.tip_link
                }
                
            elif model_type == remani_core_py.ModelType.DELTA:
                delta_params = cpp_params.get_delta()
                return {
                    'R_base': delta_params.R_base,
                    'R_platform': delta_params.R_platform,
                    'L_arm': delta_params.L_arm,
                    'L_rod': delta_params.L_rod,
                    'coupling_mode': delta_params.coupling_mode.value if hasattr(delta_params.coupling_mode, 'value') else str(delta_params.coupling_mode)
                }
        except Exception as e:
            logger.error(f"Error converting C++ params to dict: {e}")
            return {}
    
    def _cpp_limits_to_dict(self, cpp_limits) -> Optional[Dict[str, List[float]]]:
        """Convert C++ limits to dictionary"""
        if not cpp_limits:
            return None
        
        return {
            'q_min': list(cpp_limits.q_min),
            'q_max': list(cpp_limits.q_max),
            'v_max': list(cpp_limits.v_max) if cpp_limits.v_max else [],
            'a_max': list(cpp_limits.a_max) if cpp_limits.a_max else []
        }
    
    def _cpp_kinematic_caps_to_dict(self, cpp_kinematic) -> Dict[str, Any]:
        """Convert C++ kinematic capabilities to dictionary"""
        if not cpp_kinematic:
            return {}
        
        return {
            'has_closed_form_fk': getattr(cpp_kinematic, 'has_closed_form_fk', False),
            'has_closed_form_ik': getattr(cpp_kinematic, 'has_closed_form_ik', False),
            'requires_numerical_ik': getattr(cpp_kinematic, 'requires_numerical_ik', False),
            'ik_solutions_count': getattr(cpp_kinematic, 'ik_solutions_count', 'UNKNOWN'),
            'supports_spherical_wrist': getattr(cpp_kinematic, 'supports_spherical_wrist', False)
        }
    
    def _cpp_differential_caps_to_dict(self, cpp_differential) -> Dict[str, Any]:
        """Convert C++ differential capabilities to dictionary"""
        if not cpp_differential:
            return {}
        
        return {
            'jacobian_solver': getattr(cpp_differential, 'jacobian_solver', 'UNKNOWN'),
            'is_redundant': getattr(cpp_differential, 'is_redundant', False),
            'singularity_detection': getattr(cpp_differential, 'singularity_detection', False)
        }
    
    def _cpp_dynamics_caps_to_dict(self, cpp_dynamics) -> Dict[str, Any]:
        """Convert C++ dynamics capabilities to dictionary"""
        if not cpp_dynamics:
            return {}
        
        return {
            'supports_inverse_dynamics': getattr(cpp_dynamics, 'supports_inverse_dynamics', False),
            'supports_gravity_comp': getattr(cpp_dynamics, 'supports_gravity_comp', False),
            'supports_inertia_matrix': getattr(cpp_dynamics, 'supports_inertia_matrix', False),
            'payload_sensitive': getattr(cpp_dynamics, 'payload_sensitive', False)
        }
    
    def _cpp_perf_hints_to_dict(self, cpp_perf) -> Dict[str, Any]:
        """Convert C++ performance hints to dictionary"""
        if not cpp_perf:
            return {}
        
        return {
            'fk_compute_cost': getattr(cpp_perf, 'fk_compute_cost', 'UNKNOWN'),
            'ik_compute_cost': getattr(cpp_perf, 'ik_compute_cost', 'UNKNOWN'),
            'recommended_frequency_hz': getattr(cpp_perf, 'recommended_frequency_hz', 1000)
        }

    # ========================================================================
    # Helper methods for math and bridging
    # ========================================================================

    def _to_list(self, value):
        """Convert numpy or Eigen types to Python lists."""
        try:
            return list(value)
        except Exception:
            return value

    def _normalize_quaternion(self, quat):
        """Ensure quaternion is [x,y,z,w] format"""
        if quat is None:
            return [0.0, 0.0, 0.0, 1.0]
        if len(quat) == 4:
            return [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]
        if len(quat) == 7:  # maybe [w,x,y,z,...]
            return [float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])]
        raise ValueError("Quaternion must have length 4")

    def _euler_to_quaternion(self, euler_xyz):
        """Convert Euler ZYX (roll,pitch,yaw) to quaternion [x,y,z,w]"""
        import math
        r, p, y = euler_xyz
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return [qx, qy, qz, qw]

    def matrix_to_euler_zyx(self, matrix):
        """Convert 4x4 matrix to Euler angles (r,p,y) in ZYX order."""
        import numpy as np
        M = np.array(matrix)
        if M.shape == (4, 4):
            R = M[:3, :3]
        elif M.shape == (3, 3):
            R = M
        else:
            raise ValueError("Matrix must be 3x3 or 4x4")

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0.0
        return [x, y, z]

    def matrix_to_quaternion(self, matrix):
        """Convert 4x4 matrix to quaternion [x,y,z,w]."""
        import numpy as np
        m = np.array(matrix)
        t = m[0,0] + m[1,1] + m[2,2]
        if t > 0:
            r = math.sqrt(1.0 + t)
            w = 0.5 * r
            r = 0.5 / r
            x = (m[2,1] - m[1,2]) * r
            y = (m[0,2] - m[2,0]) * r
            z = (m[1,0] - m[0,1]) * r
        elif m[0,0] > m[1,1] and m[0,0] > m[2,2]:
            r = math.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2])
            x = 0.5 * r
            r = 0.5 / r
            y = (m[0,1] + m[1,0]) * r
            z = (m[0,2] + m[2,0]) * r
            w = (m[2,1] - m[1,2]) * r
        elif m[1,1] > m[2,2]:
            r = math.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2])
            y = 0.5 * r
            r = 0.5 / r
            x = (m[0,1] + m[1,0]) * r
            z = (m[1,2] + m[2,1]) * r
            w = (m[0,2] - m[2,0]) * r
        else:
            r = math.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1])
            z = 0.5 * r
            r = 0.5 / r
            x = (m[0,2] + m[2,0]) * r
            y = (m[1,2] + m[2,1]) * r
            w = (m[1,0] - m[0,1]) * r
        return [float(x), float(y), float(z), float(w)]

    def pose_to_matrix(self, pos, rot):
        """Create a 4x4 homogeneous transform from position + quaternion/euler."""
        import numpy as np
        if len(rot) == 4:
            qx, qy, qz, qw = rot
        elif len(rot) == 3:
            qx, qy, qz, qw = self._euler_to_quaternion(rot)
        else:
            raise ValueError("Rotation must be quaternion [x,y,z,w] or euler [r,p,y]")
        # convert to matrix
        xx, yy, zz = qx*qx, qy*qy, qz*qz
        xy, xz, yz = qx*qy, qx*qz, qy*qz
        wx, wy, wz = qw*qx, qw*qy, qw*qz

        R = [[1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
             [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
             [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]]

        T = [[R[0][0], R[0][1], R[0][2], pos[0]],
             [R[1][0], R[1][1], R[1][2], pos[1]],
             [R[2][0], R[2][1], R[2][2], pos[2]],
             [0.0, 0.0, 0.0, 1.0]]
        return T

    def quat_to_matrix(self, pos, rot):
        return self.pose_to_matrix(pos, rot)

    def apply_custom_limits(self, model_limits, custom_limits):
        """Overlay custom limits over model limits."""
        new_limits = remani_core_py.JointLimits()
        new_limits.q_min = list(model_limits.q_min)
        new_limits.q_max = list(model_limits.q_max)
        new_limits.v_max = list(model_limits.v_max) if model_limits.v_max else []
        new_limits.a_max = list(model_limits.a_max) if model_limits.a_max else []

        for axis_str, bounds in custom_limits.items():
            idx = int(axis_str)
            if idx < 0 or idx >= len(new_limits.q_min):
                continue
            if len(bounds) != 2:
                continue
            new_limits.q_min[idx], new_limits.q_max[idx] = bounds[0], bounds[1]

        return new_limits

    def py_to_cpp_base_frame(self, base_frame):
        """Convert base frame parameter in request to C++ BaseFrame."""
        if base_frame is None:
            return remani_core_py.BaseFrame()

        if isinstance(base_frame, remani_core_py.BaseFrame):
            return base_frame

        if isinstance(base_frame, dict):
            pos = base_frame.get('pos') or base_frame.get('position') or [0.0, 0.0, 0.0]
            rot = base_frame.get('rot') or base_frame.get('rotation') or [0.0, 0.0, 0.0, 1.0]
        else:
            pos = getattr(base_frame, 'pos', None) or getattr(base_frame, 'position', [0.0, 0.0, 0.0])
            rot = getattr(base_frame, 'rot', None) or getattr(base_frame, 'rotation', [0.0, 0.0, 0.0, 1.0])

        if len(rot) == 3:
            rot = self._euler_to_quaternion(rot)

        return remani_core_py.BaseFrame.from_pos_rot(pos, rot)

    def py_to_cpp_tcp(self, tcp_data):
        """Convert TCP definition to C++ TCPDefinition."""
        if tcp_data is None:
            return remani_core_py.TCPDefinition()

        if isinstance(tcp_data, remani_core_py.TCPDefinition):
            return tcp_data

        if isinstance(tcp_data, dict):
            pos = tcp_data.get('pos') or tcp_data.get('position') or [0.0, 0.0, 0.0]
            rot = tcp_data.get('rot') or tcp_data.get('rotation') or [0.0, 0.0, 0.0, 1.0]
        else:
            pos = getattr(tcp_data, 'pos', None) or getattr(tcp_data, 'position', [0.0, 0.0, 0.0])
            rot = getattr(tcp_data, 'rot', None) or getattr(tcp_data, 'rotation', [0.0, 0.0, 0.0, 1.0])

        if len(rot) == 3:
            rot = self._euler_to_quaternion(rot)

        return remani_core_py.TCPDefinition.from_pos_quat(pos, rot)

    def py_to_cpp_payload(self, payload_data):
        """Convert payload dictionary to C++ Payload."""
        if payload_data is None:
            return None

        payload = remani_core_py.Payload()
        payload.mass = float(payload_data.get('mass', 0.0))

        cog = payload_data.get('cog', [0.0, 0.0, 0.0])
        payload.cog = list(cog)

        inertia = payload_data.get('inertia')
        if inertia:
            from numpy import array
            payload.inertia = array(inertia)
            payload.has_inertia = True

        return payload

    def py_to_cpp_instance(self, instance_id, request):
        """Convert ConfigureInstanceRequest to the C++ InstanceManager call."""
        if request is None:
            raise ValueError("Instance configuration is required")

        base_frame = self.py_to_cpp_base_frame(getattr(request, 'base_frame', None))
        default_tcp = self.py_to_cpp_tcp(getattr(request, 'default_tcp', None))

        model_id = request.model_id
        mounting_type = getattr(request, 'mounting_type', 'FLOOR')
        if isinstance(mounting_type, str):
            mounting_type = remani_core_py.MountingType[mounting_type]
        else:
            mounting_type = remani_core_py.MountingType[mounting_type]

        return self.instance_manager.configure_instance(
            instance_id,
            model_id,
            base_frame,
            mounting_type,
            float(getattr(request, 'mounting_angle', 0.0)),
            default_tcp,
            self.model_manager
        )

    def py_to_cpp_calibration(self, request):
        """Convert CalibrationRequest to C++ CalibrationData."""
        if request is None:
            raise ValueError("Calibration request is required")

        cpp_calibration = remani_core_py.CalibrationData()
        cpp_calibration.calibration_method = request.calibration_method
        cpp_calibration.joint_offsets = list(getattr(request, 'joint_offsets', []) or [])
        cpp_calibration.force_update = bool(getattr(request, 'force_update', False))

        delta_params = getattr(request, 'delta_params', None)
        if delta_params:
            # Currently not directly applied; conversion can be extended here
            cpp_calibration.corrections = []

        return cpp_calibration

    def cpp_to_py_pose_response(self, instance_id, pose_matrix, is_singular, compute_time_us, return_format):
        """Convert FK output to API response model."""
        pos = [float(pose_matrix[0][3]), float(pose_matrix[1][3]), float(pose_matrix[2][3])]
        quat = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        euler = {'r': 0.0, 'p': 0.0, 'y': 0.0}

        try:
            quat_list = self.matrix_to_quaternion(pose_matrix)
            euler_list = self.matrix_to_euler_zyx(pose_matrix)
            quat = {'x': quat_list[0], 'y': quat_list[1], 'z': quat_list[2], 'w': quat_list[3]}
            euler = {'r': euler_list[0], 'p': euler_list[1], 'y': euler_list[2]}
        except Exception:
            pass

        response = {
            'instance_id': instance_id,
            'pos': pos,
            'quat': quat,
            'euler': euler,
            'pose_matrix': [list(row) for row in pose_matrix],
            'is_singularity': bool(is_singular),
            'compute_time_us': int(compute_time_us)
        }

        # For compatibility with pydantic fields, some values may be omitted if not requested.
        if not return_format or 'MATRIX' not in return_format:
            response['pose_matrix'] = None
        if not return_format or 'QUATERNION' not in return_format:
            response['quat'] = None
            response['euler'] = None

        return response

    def cpp_to_py_velocity_response(self, instance_id, velocity, compute_time_us, exceeds_t1_limit):
        """Convert C++ CartesianVelocity to API response."""
        return {
            'instance_id': instance_id,
            'linear_velocity': {
                'vector': [float(velocity.linear[0]), float(velocity.linear[1]), float(velocity.linear[2])],
                'magnitude': float(velocity.linear_magnitude())
            },
            'angular_velocity': {
                'vector': [float(velocity.angular[0]), float(velocity.angular[1]), float(velocity.angular[2])],
                'magnitude': float(velocity.angular_magnitude())
            },
            'limit_check': {
                't1_exceeded': bool(exceeds_t1_limit),
                't1_limit': 0.25
            },
            'compute_time_us': int(compute_time_us)
        }

    def cpp_to_py_acceleration_response(self, instance_id, acceleration, compute_time_us, payload_cpp=None):
        """Convert C++ CartesianAcceleration to API response."""
        inertial_force = None
        if payload_cpp is not None:
            mass = float(payload_cpp.mass)
            inertial_force = {
                'linear': [float(acceleration.linear[0] * mass), float(acceleration.linear[1] * mass), float(acceleration.linear[2] * mass)],
                'angular': [0.0, 0.0, 0.0]
            }

        return {
            'instance_id': instance_id,
            'linear_acc': {
                'vector': [float(acceleration.linear[0]), float(acceleration.linear[1]), float(acceleration.linear[2])],
                'magnitude': float(acceleration.linear_magnitude())
            },
            'angular_acc': {
                'vector': [float(acceleration.angular[0]), float(acceleration.angular[1]), float(acceleration.angular[2])],
                'magnitude': float(acceleration.angular_magnitude())
            },
            'inertial_force': inertial_force,
            'compute_time_us': int(compute_time_us)
        }

    def cpp_to_py_jacobian_response(self, instance_id, jacobian, analysis):
        """Convert Jacobian matrix to response format."""
        return {
            'instance_id': instance_id,
            'dof': len(jacobian[0]) if jacobian else 0,
            'jacobian_matrix': [list(map(float, row)) for row in jacobian],
            'analysis': analysis
        }

    def cpp_to_py_singularity_response(self, instance_id, is_singular, singularity_types, details, min_singular_value):
        """Convert singularity check output to response format."""
        return {
            'instance_id': instance_id,
            'is_singular': bool(is_singular),
            'singularity_types': singularity_types,
            'details': details,
            'min_singular_value': float(min_singular_value)
        }

    def cpp_to_py_config_flags_response(self, instance_id, config_id, flags, turns):
        """Convert configuration flags result to response format."""
        return {
            'instance_id': instance_id,
            'config_id': int(config_id),
            'flags': {k: str(v) for k, v in flags.items()},
            'turns': [int(t) for t in turns]
        }

    def cpp_to_py_static_torque_response(self, instance_id, gravity_torques, load_analysis, is_overloaded, max_load_ratio):
        """Convert static torque analysis output to response format."""
        return {
            'instance_id': instance_id,
            'gravity_torques': [float(x) for x in gravity_torques],
            'load_analysis': load_analysis,
            'is_overloaded': bool(is_overloaded),
            'max_load_ratio': float(max_load_ratio)
        }


# Global bridge instance with manager injection support
core_bridge = CoreBridge()
