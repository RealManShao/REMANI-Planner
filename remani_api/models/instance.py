"""
Runtime instance Pydantic models
"""

from pydantic import BaseModel, Field, validator
from typing import Optional, List, Dict, Any
from enum import Enum

class InstanceStatus(str, Enum):
    READY = "READY"
    MOVING = "MOVING"
    ERROR = "ERROR"
    STOPPED = "STOPPED"

class MountingType(str, Enum):
    FLOOR = "FLOOR"
    CEILING = "CEILING"
    WALL = "WALL"
    ANGLED = "ANGLED"

class Pose(BaseModel):
    pos: List[float] = Field(..., description="Position [x, y, z] (m)")
    rot: List[float] = Field(..., description="Rotation [x, y, z, w] quaternion or [r, p, y] euler")

class BaseFrame(BaseModel):
    pos: List[float] = Field(default=[0.0, 0.0, 0.0], description="Position [x, y, z] (m)")
    rot: List[float] = Field(default=[0.0, 0.0, 0.0, 1.0], description="Rotation quaternion [x, y, z, w]")

class TCPDefinition(BaseModel):
    pos: List[float] = Field(default=[0.0, 0.0, 0.0], description="Position [x, y, z] (m)")
    rot: List[float] = Field(default=[0.0, 0.0, 0.0, 1.0], description="Rotation quaternion [x, y, z, w]")

class InitialState(BaseModel):
    joints: Optional[List[float]] = None
    velocities: Optional[List[float]] = None

class ConfigureInstanceRequest(BaseModel):
    model_id: str = Field(..., description="Reference model ID")
    base_frame: Optional[BaseFrame] = None
    mounting_type: MountingType = MountingType.FLOOR
    mounting_angle: float = Field(0.0, description="Angle for ANGLED mounting (rad)")
    default_tcp: Optional[TCPDefinition] = None
    initial_state: Optional[InitialState] = None

class CalibrationRequest(BaseModel):
    calibration_method: str = Field(..., description="Calibration method identifier")
    joint_offsets: Optional[List[float]] = None
    delta_params: Optional[Dict[str, Any]] = None
    force_update: bool = False

class Payload(BaseModel):
    mass: float = Field(..., ge=0, description="Payload mass (kg)")
    cog: List[float] = Field(default=[0.0, 0.0, 0.0], description="Center of gravity [x, y, z] (m)")
    inertia: Optional[List[List[float]]] = None  # 3x3 inertia matrix

class RuntimeInstance(BaseModel):
    instance_id: str
    model_id: str
    base_frame: BaseFrame
    mounting_type: MountingType
    mounting_angle: float
    default_tcp: TCPDefinition
    status: InstanceStatus
    current_joints: List[float]
    joint_offsets: List[float]
    gravity_vector: List[float]  # [gx, gy, gz] (m/s^2)
    created_at: str
    last_updated: str
    config_hash: str
