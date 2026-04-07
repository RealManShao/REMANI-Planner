"""
Pydantic models for API request/response validation
"""

from .kinematic import (
    ModelType, JointType, MDHJoint, POEParams, URDFParams, DeltaParams,
    JointLimits, KinematicParams, CalibrationData
)
from .instance import (
    InstanceStatus, MountingType, BaseFrame, TCPDefinition,
    RuntimeInstance, Payload
)
from .responses import (
    ModelResponse, InstanceResponse, ErrorResponse,
    CapabilitiesResponse, PoseResponse, VelocityResponse,
    AccelerationResponse, JacobianResponse, SingularityResponse,
    JointLimitResponse, TransformResponse, ConfigFlagsResponse,
    StaticTorqueResponse
)

__all__ = [
    # Kinematic models
    "ModelType", "JointType", "MDHJoint", "POEParams", "URDFParams", "DeltaParams",
    "JointLimits", "KinematicParams", "CalibrationData",
    
    # Instance models
    "InstanceStatus", "MountingType", "BaseFrame", "TCPDefinition",
    "RuntimeInstance", "Payload",
    
    # Response models
    "ModelResponse", "InstanceResponse", "ErrorResponse",
    "CapabilitiesResponse", "PoseResponse", "VelocityResponse",
    "AccelerationResponse", "JacobianResponse", "SingularityResponse",
    "JointLimitResponse", "TransformResponse", "ConfigFlagsResponse",
    "StaticTorqueResponse"
]
