"""
Common response models
"""

from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from datetime import datetime

class ErrorResponse(BaseModel):
    error: Dict[str, Any]
    request_id: Optional[str] = None
    timestamp: datetime = Field(default_factory=datetime.utcnow)

class ModelResponse(BaseModel):
    model_id: str
    name: Optional[str]
    model_type: str
    dof: int
    status: str
    params: Dict[str, Any]
    limits: Optional[Dict[str, Any]]
    created_at: str
    last_updated: str
    solver_engine: Optional[str]
    total_mass: Optional[float]
    build_info: Optional[Dict[str, Any]]

class InstanceResponse(BaseModel):
    instance_id: str
    model_id: str
    base_frame: Dict[str, Any]
    mounting_type: str
    mounting_angle: float
    default_tcp: Dict[str, Any]
    status: str
    current_joints: List[float]
    joint_offsets: List[float]
    gravity_vector: List[float]
    created_at: str
    last_updated: str
    config_hash: str

class CapabilitiesResponse(BaseModel):
    model_id: str
    structure_type: str
    kinematics: Dict[str, Any]
    differential: Dict[str, Any]
    dynamics: Dict[str, Any]
    performance_hints: Dict[str, Any]

class PoseResponse(BaseModel):
    instance_id: str
    pos: List[float]
    quat: Optional[Dict[str, float]]
    euler: Optional[Dict[str, float]]
    pose_matrix: Optional[List[List[float]]]
    is_singularity: bool
    compute_time_us: int

class VelocityResponse(BaseModel):
    instance_id: str
    linear_velocity: Dict[str, float]  # {"vector": [...], "magnitude": ...}
    angular_velocity: Dict[str, float]  # {"vector": [...], "magnitude": ...}
    limit_check: Optional[Dict[str, Any]]
    compute_time_us: int

class AccelerationResponse(BaseModel):
    instance_id: str
    linear_acc: Dict[str, float]  # {"vector": [...], "magnitude": ...}
    angular_acc: Dict[str, float]  # {"vector": [...], "magnitude": ...}
    inertial_force: Optional[Dict[str, Any]]
    compute_time_us: int

class JacobianResponse(BaseModel):
    instance_id: str
    dof: int
    jacobian_matrix: List[List[float]]
    analysis: Optional[Dict[str, Any]]  # determinant, manipulability, condition_number, etc.

class SingularityResponse(BaseModel):
    instance_id: str
    is_singular: bool
    singularity_types: List[str]
    details: Dict[str, Any]  # wrist, elbow, shoulder details
    min_singular_value: float

class JointLimitResponse(BaseModel):
    instance_id: str
    in_range: bool
    violations: List[Dict[str, Any]]  # axis, value, limit_min, limit_max, violation_type
    current_limits: List[List[float]]  # All limits for visualization

class TransformResponse(BaseModel):
    operation: str
    result_pose: Optional[Dict[str, Any]]
    result_matrix: Optional[List[List[float]]]

class ConfigFlagsResponse(BaseModel):
    instance_id: str
    config_id: int
    flags: Dict[str, str]  # arm, elbow, wrist
    turns: List[int]

class StaticTorqueResponse(BaseModel):
    instance_id: str
    gravity_torques: List[float]
    load_analysis: List[Dict[str, Any]]  # axis, torque, rated, ratio
    is_overloaded: bool
    max_load_ratio: float
