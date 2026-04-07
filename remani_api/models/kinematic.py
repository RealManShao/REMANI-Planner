"""
Kinematic model Pydantic models
"""

from pydantic import BaseModel, Field, validator
from typing import Optional, List, Dict, Any, Union, Literal
from enum import Enum

class ModelType(str, Enum):
    MDH = "MDH"
    SDH = "SDH"
    POE = "POE"
    URDF = "URDF"
    DELTA = "DELTA"

class JointType(str, Enum):
    REVOLUTE = "REVOLUTE"
    PRISMATIC = "PRISMATIC"

class MDHJoint(BaseModel):
    joint_index: int
    type: JointType
    a: float = Field(..., description="Link length (m)")
    alpha: float = Field(..., description="Link twist (rad)")
    d: float = Field(..., description="Link offset (m)")
    theta_offset: float = Field(0.0, description="Joint angle offset (rad)")

class POEParams(BaseModel):
    screw_axes: List[List[float]] = Field(..., description="Screw axes [w1,w2,w3,v1,v2,v3]")
    m_home: List[List[float]] = Field(..., description="4x4 home position matrix")

class URDFParams(BaseModel):
    urdf_content: str
    base_link: str
    tip_link: str

class DeltaParams(BaseModel):
    R_base: float = Field(..., description="Base platform radius (m)")
    R_platform: float = Field(..., description="Moving platform radius (m)")
    L_arm: float = Field(..., description="Active arm length (m)")
    L_rod: float = Field(..., description="Passive rod length (m)")
    coupling_mode: Literal["ROTARY", "LINEAR"] = "ROTARY"

class JointLimits(BaseModel):
    q_min: List[float] = Field(..., description="Minimum joint positions")
    q_max: List[float] = Field(..., description="Maximum joint positions")
    v_max: Optional[List[float]] = Field(None, description="Maximum joint velocities")
    a_max: Optional[List[float]] = Field(None, description="Maximum joint accelerations")
    
    @validator('q_max')
    def validate_limits(cls, v, values):
        if 'q_min' in values and len(v) != len(values['q_min']):
            raise ValueError('q_min and q_max must have same length')
        return v

class MDHCalibrationCorrection(BaseModel):
    joint_index: int
    delta_a: float = 0.0
    delta_alpha: float = 0.0
    delta_d: float = 0.0
    delta_theta: float = 0.0

class POECalibrationCorrection(BaseModel):
    index: int
    delta_axis: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    m_home_correction: List[List[float]] = Field(
        default_factory=lambda: [[1.0, 0.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0, 0.0],
                              [0.0, 0.0, 0.0, 1.0]]
    )

class CalibrationData(BaseModel):
    calibration_method: str
    joint_offsets: List[float] = []
    corrections: Union[List[MDHCalibrationCorrection], List[POECalibrationCorrection]] = []
    force_update: bool = False

class KinematicParams(BaseModel):
    model_type: ModelType
    dof: int
    params: Dict[str, Any] = Field(..., description="Polymorphic parameters")
    
    @validator('params')
    def validate_params(cls, v, values):
        model_type = values.get('model_type')
        if model_type == ModelType.MDH or model_type == ModelType.SDH:
            if 'chain' not in v:
                raise ValueError('MDH/SDH requires "chain" parameter')
        elif model_type == ModelType.POE:
            if 'screw_axes' not in v or 'm_home' not in v:
                raise ValueError('POE requires "screw_axes" and "m_home" parameters')
        elif model_type == ModelType.URDF:
            if 'urdf_content' not in v:
                raise ValueError('URDF requires "urdf_content" parameter')
        elif model_type == ModelType.DELTA:
            required = ['R_base', 'R_platform', 'L_arm', 'L_rod']
            if any(k not in v for k in required):
                raise ValueError(f'DELTA requires {required} parameters')
        return v

class RegisterModelRequest(BaseModel):
    model_id: str = Field(..., description="Unique model identifier")
    model_type: ModelType
    dof: int = Field(..., gt=0, description="Degrees of freedom")
    name: Optional[str] = None
    params: Dict[str, Any]
    limits: Optional[JointLimits] = None

class GetModelDetailsRequest(BaseModel):
    include_state: Optional[bool] = False

class GetCapabilitiesRequest(BaseModel):
    pass  # No additional parameters needed
