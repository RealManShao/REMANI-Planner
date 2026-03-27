# REMANI-Planner API Reconstruction Plan

## 1. Project Overview

### 1.1 Objective
Reconstruct REMANI-Planner to integrate with the DZ Project API, adding:
- Industrial standard RESTful API layer
- Runtime model/instance lifecycle management
- Polymorphic kinematic parameter support (MDH, POE, URDF)
- JWT authentication and standardized error handling

### 1.2 Technology Stack
| Layer | Technology |
|-------|------------|
| **Core Algorithms** | C++ (Eigen, existing REMANI-Planner code) |
| **Python Bindings** | pybind11 |
| **REST API** | FastAPI (Python) |
| **ROS Integration** | ROS Noetic (hybrid mode - optional) |
| **Build System** | CMake + catkin |
| **Serialization** | JSON (nlohmann/json for C++, pydantic for Python) |

### 1.3 Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    Interface Layer (Python)                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │  FastAPI    │  │  JWT Auth   │  │  Request Validation     │  │
│  │  REST API   │  │  Middleware │  │  (Pydantic Models)      │  │
│  └──────┬──────┘  └──────┬──────┘  └───────────┬─────────────┘  │
│         └────────────────┼─────────────────────┘                │
└──────────────────────────┼──────────────────────────────────────┘
                           │ pybind11 bindings
┌──────────────────────────┼──────────────────────────────────────┐
│               Resource Management Layer (C++)                   │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │  ModelManager   │  │ InstanceManager │  │  ConfigStore    │  │
│  │  (CRUD models)  │  │  (CRUD inst.)   │  │  (Persistence)  │  │
│  └────────┬────────┘  └────────┬────────┘  └─────────────────┘  │
│           └───────────────────┼─────────────────────────────────│
└───────────────────────────────┼─────────────────────────────────┘
                                │
┌───────────────────────────────┼─────────────────────────────────┐
│                  Core Algorithm Layer (C++)                     │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │              KinematicModel (Polymorphic Base)              ││
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐        ││
│  │  │MDHModel  │ │ POEModel │ │URDFModel │ │DeltaModel│        ││
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘        ││
│  └─────────────────────────────────────────────────────────────┘│
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────────────────┐ │
│  │ FK/IK Solver │ │ Jacobian     │ │ Dynamics/Safety Analysis │ │
│  └──────────────┘ └──────────────┘ └──────────────────────────┘ │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │           Existing REMANI-Planner Components                ││
│  │  (mm_config, path_searching, traj_opt, plan_manage)         ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Core Data Structures

### 2.1 Kinematic Parameters (C++)

```cpp
// remani_planner/core/include/remani/core/kinematic_params.hpp

namespace remani::core {

// Joint type enumeration
enum class JointType { REVOLUTE, PRISMATIC };

// MDH Parameters (Modified Denavit-Hartenberg / Craig)
struct MDHJoint {
    int joint_index;
    JointType type;
    double a;           // link length (m)
    double alpha;       // link twist (rad)
    double d;           // link offset (m)
    double theta_offset; // joint angle offset (rad)
};

// POE Parameters (Product of Exponentials)
struct POEJoint {
    std::array<double, 6> screw_axis;  // [w1,w2,w3, v1,v2,v3]
};

struct POEParams {
    std::vector<POEJoint> screw_axes;
    Eigen::Matrix4d m_home;  // zero-position end-effector transform
};

// URDF Parameters (delegated to urdf parser)
struct URDFParams {
    std::string urdf_content;
    std::string base_link;
    std::string tip_link;
};

// Delta Parameters (Parallel robot)
struct DeltaParams {
    double R_base;       // base platform radius
    double R_platform;   // moving platform radius
    double L_arm;        // active arm length
    double L_rod;        // passive rod length
    std::string coupling_mode;  // "ROTARY" or "LINEAR"
};

// Polymorphic parameter container
struct KinematicParams {
    std::string model_type;  // "MDH", "SDH", "POE", "URDF", "DELTA"
    int dof;
    
    // Variant storage
    std::variant<
        std::vector<MDHJoint>,
        POEParams,
        URDFParams,
        DeltaParams
    > params;
};

// Joint limits
struct JointLimits {
    std::vector<double> q_min;
    std::vector<double> q_max;
    std::vector<double> v_max;  // velocity limits
    std::vector<double> a_max;  // acceleration limits
};

} // namespace remani::core
```

### 2.2 Model and Instance (C++)

```cpp
// remani_planner/core/include/remani/core/model.hpp

namespace remani::core {

// Model status
enum class ModelStatus { ACTIVE, ERROR, CALIBRATING, READY };

// Kinematic Model (immutable after registration)
struct KinematicModel {
    std::string model_id;
    std::string name;
    std::string model_type;
    int dof;
    ModelStatus status;
    KinematicParams params;
    JointLimits limits;
    
    // Metadata
    std::string created_at;
    std::string solver_engine;  // e.g., "RNE_MDH_OPTIMIZED"
    double total_mass;
};

// Instance status
enum class InstanceStatus { READY, MOVING, ERROR, STOPPED };

// Mounting type
enum class MountingType { FLOOR, CEILING, WALL, ANGLED };

// Runtime Instance (mutable)
struct RuntimeInstance {
    std::string instance_id;
    std::string model_id;  // reference to model
    
    // Configuration
    Eigen::Matrix4d base_frame;  // robot base in world coordinates
    MountingType mounting_type;
    double mounting_angle;
    Eigen::Matrix4d default_tcp;  // default tool center point
    
    // Runtime state
    InstanceStatus status;
    std::vector<double> current_joints;  // cached joint positions
    std::vector<double> joint_offsets;   // calibration offsets
    
    // Gravity vector (computed from mounting type)
    Eigen::Vector3d gravity_vector;
    
    // Metadata
    std::string created_at;
    std::string last_updated;
    std::string config_hash;
};

} // namespace remani::core
```

### 2.3 API Request/Response Models (Python/Pydantic)

```python
# remani_api/models/kinematic.py

from pydantic import BaseModel, Field
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

# MDH Joint
class MDHJoint(BaseModel):
    joint_index: int
    type: JointType
    a: float
    alpha: float
    d: float
    theta_offset: float

# POE Parameters
class POEParams(BaseModel):
    screw_axes: List[List[float]]  # each is [w1,w2,w3,v1,v2,v3]
    m_home: List[List[float]]      # 4x4 matrix

# URDF Parameters
class URDFParams(BaseModel):
    urdf_content: str
    base_link: str
    tip_link: str

# Delta Parameters
class DeltaParams(BaseModel):
    R_base: float
    R_platform: float
    L_arm: float
    L_rod: float
    coupling_mode: Literal["ROTARY", "LINEAR"]

# Polymorphic params
class KinematicParams(BaseModel):
    __root__: Union[List[MDHJoint], POEParams, URDFParams, DeltaParams]

# Joint Limits
class JointLimits(BaseModel):
    q_min: List[float]
    q_max: List[float]
    v_max: Optional[List[float]] = None
    a_max: Optional[List[float]] = None

# Register Model Request
class RegisterModelRequest(BaseModel):
    model_id: str
    model_type: ModelType
    dof: int
    name: Optional[str] = None
    params: Dict[str, Any]  # polymorphic
    limits: Optional[JointLimits] = None

# Pose (position + orientation)
class Pose(BaseModel):
    pos: List[float]  # [x, y, z]
    rot: List[float]  # [qx, qy, qz, qw] quaternion OR [rx, ry, rz] euler

# Configure Instance Request
class ConfigureInstanceRequest(BaseModel):
    model_id: str
    base_frame: Optional[Pose] = None
    mounting_type: Optional[Literal["FLOOR", "CEILING", "WALL", "ANGLED"]] = "FLOOR"
    mounting_angle: Optional[float] = 0.0
    default_tcp: Optional[Pose] = None
    initial_state: Optional[Dict[str, List[float]]] = None

# FK Compute Request
class FKComputeRequest(BaseModel):
    instance_id: str
    joints: List[float]
    tcp: Optional[Pose] = None
    ref_frame: Optional[Pose] = None
    return_format: Optional[List[str]] = ["MATRIX", "QUATERNION"]

# FK Compute Response
class FKComputeResponse(BaseModel):
    instance_id: str
    pos: List[float]
    quat: Optional[Dict[str, float]] = None  # {w, x, y, z}
    euler: Optional[Dict[str, float]] = None  # {mode, r, p, y}
    pose_matrix: Optional[List[List[float]]] = None
    is_singularity: bool
    compute_time_us: int
```

---

## 3. API Endpoint Mapping

| API Name | HTTP Method | Resource Path | C++ Class Method |
|----------|-------------|---------------|------------------|
| **Model Management** |
| registerKinematicModel | POST | `/api/v1/kine/models` | `ModelManager::registerModel()` |
| getKinematicModelDetails | GET | `/api/v1/kine/models/{model_id}` | `ModelManager::getModel()` |
| getModelCapabilities | GET | `/api/v1/kine/models/{model_id}/capabilities` | `ModelManager::getCapabilities()` |
| deleteKinematicModel | DELETE | `/api/v1/kine/models/{model_id}` | `ModelManager::deleteModel()` |
| **Instance Management** |
| configureRuntimeInstance | PUT | `/api/v1/kine/instances/{instance_id}` | `InstanceManager::configureInstance()` |
| getInstanceDetails | GET | `/api/v1/kine/instances/{instance_id}` | `InstanceManager::getInstance()` |
| deleteInstance | DELETE | `/api/v1/kine/instances/{instance_id}` | `InstanceManager::deleteInstance()` |
| calibrateKinematicParameters | PATCH | `/api/v1/kine/instances/{instance_id}/calibration` | `InstanceManager::applyCalibration()` |
| **Forward Kinematics** |
| computeForwardKinematics | POST | `/api/v1/fk/compute/pose` | `KinematicSolver::computeFK()` |
| computeCartesianVelocity | POST | `/api/v1/fk/compute/velocity` | `KinematicSolver::computeVelocity()` |
| computeCartesianAcceleration | POST | `/api/v1/fk/compute/acceleration` | `KinematicSolver::computeAcceleration()` |
| **Analysis Tools** |
| computeJacobianMatrix | POST | `/api/v1/fk/analysis/jacobian` | `KinematicSolver::computeJacobian()` |
| computeStaticLoadTorque | POST | `/api/v1/fk/analysis/static-torque` | `DynamicsSolver::computeStaticTorque()` |
| identifyConfiguration | POST | `/api/v1/fk/analysis/config-flags` | `KinematicSolver::identifyConfig()` |
| checkSingularity | POST | `/api/v1/fk/analysis/singularity-check` | `KinematicSolver::checkSingularity()` |
| **Utility/Math** |
| calcTransform | POST | `/api/v1/fk/utils/transform` | `MathUtils::transform()` |
| calcLinkMatrix | POST | `/api/v1/fk/utils/dh-matrix` | `MathUtils::dhMatrix()` |
| checkJointLimits | POST | `/api/v1/safety/joint-limits` | `SafetyChecker::checkJointLimits()` |

---

## 4. Project Structure

```
REMANI-Planner/
├── remani_planner/
│   ├── core/                          # NEW: Core algorithm layer
│   │   ├── include/remani/core/
│   │   │   ├── kinematic_params.hpp    # Data structures
│   │   │   ├── model.hpp               # Model/Instance definitions
│   │   │   ├── kinematic_model.hpp     # Polymorphic base class
│   │   │   ├── mdh_model.hpp           # MDH implementation
│   │   │   ├── poe_model.hpp           # POE implementation
│   │   │   ├── urdf_model.hpp          # URDF implementation
│   │   │   ├── delta_model.hpp         # Delta parallel robot
│   │   │   ├── model_manager.hpp       # Model lifecycle management
│   │   │   ├── instance_manager.hpp    # Instance lifecycle management
│   │   │   ├── kinematic_solver.hpp    # FK/IK/Jacobian computations
│   │   │   ├── dynamics_solver.hpp     # Dynamics computations
│   │   │   ├── safety_checker.hpp      # Singularity/limit checks
│   │   │   └── math_utils.hpp          # Transform utilities
│   │   ├── src/
│   │   │   ├── kinematic_model.cpp
│   │   │   ├── mdh_model.cpp
│   │   │   ├── poe_model.cpp
│   │   │   ├── urdf_model.cpp
│   │   │   ├── delta_model.cpp
│   │   │   ├── model_manager.cpp
│   │   │   ├── instance_manager.cpp
│   │   │   ├── kinematic_solver.cpp
│   │   │   ├── dynamics_solver.cpp
│   │   │   ├── safety_checker.cpp
│   │   │   └── math_utils.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── bindings/                      # NEW: pybind11 bindings
│   │   ├── include/remani/bindings/
│   │   │   └── pybind.hpp
│   │   ├── src/
│   │   │   ├── py_kinematic_params.cpp
│   │   │   ├── py_model.cpp
│   │   │   ├── py_managers.cpp
│   │   │   └── py_solvers.cpp
│   │   └── CMakeLists.txt
│   │
│   ├── mm_config/                     # EXISTING: Adapt for backward compat
│   │   ├── include/mm_config/
│   │   │   └── mm_config.hpp          # Keep, add adapter to new core
│   │   └── src/
│   │       └── mm_config.cpp          # Refactor to use KinematicModel
│   │
│   ├── path_searching/                # EXISTING: Keep as-is
│   ├── traj_opt/                      # EXISTING: Keep as-is
│   └── plan_manage/                   # EXISTING: Keep as-is
│
├── remani_api/                        # NEW: Python FastAPI service
│   ├── __init__.py
│   ├── main.py                        # FastAPI app entry point
│   ├── config.py                      # Configuration
│   ├── auth/
│   │   ├── __init__.py
│   │   └── jwt_handler.py             # JWT authentication
│   ├── models/
│   │   ├── __init__.py
│   │   ├── kinematic.py               # Pydantic models
│   │   ├── instance.py
│   │   └── responses.py
│   ├── routers/
│   │   ├── __init__.py
│   │   ├── models.py                  # /kine/models endpoints
│   │   ├── instances.py               # /kine/instances endpoints
│   │   ├── fk.py                      # /fk endpoints
│   │   ├── analysis.py                # /fk/analysis endpoints
│   │   ├── utils.py                   # /fk/utils endpoints
│   │   └── safety.py                  # /safety endpoints
│   ├── services/
│   │   ├── __init__.py
│   │   └── core_bridge.py             # Bridge to C++ via pybind11
│   └── exceptions/
│       ├── __init__.py
│       └── handlers.py                # Global exception handlers
│
├── tests/
│   ├── cpp/                           # C++ unit tests
│   │   ├── test_mdh_model.cpp
│   │   ├── test_poe_model.cpp
│   │   └── test_solvers.cpp
│   └── python/                        # Python API tests
│       ├── test_models.py
│       ├── test_instances.py
│       └── test_fk.py
│
├── CMakeLists.txt                     # Main CMake
├── package.xml                        # catkin package
├── requirements.txt                   # Python dependencies
├── setup.py                           # Python package setup
└── README.md
```

---

## 5. Implementation Phases

### Phase 1: Core Data Structures & Model Management (Week 1-2)

**Tasks:**
1. Define C++ data structures (`kinematic_params.hpp`, `model.hpp`)
2. Implement `KinematicModel` polymorphic base class
3. Implement `MDHModel` class with FK computation
4. Implement `ModelManager` for model CRUD operations
5. Create pybind11 bindings for model classes
6. Implement FastAPI endpoints for model registration/retrieval
7. Write unit tests for MDH model

**Deliverables:**
- `POST /api/v1/kine/models` working
- `GET /api/v1/kine/models/{model_id}` working
- `GET /api/v1/kine/models/{model_id}/capabilities` working

### Phase 2: Instance Management (Week 3)

**Tasks:**
1. Implement `InstanceManager` class
2. Add base frame transformation logic
3. Implement mounting type gravity vector computation
4. Implement calibration parameter application
5. Create pybind11 bindings for instance management
6. Implement FastAPI endpoints for instance configuration
7. Write unit tests for instance management

**Deliverables:**
- `PUT /api/v1/kine/instances/{instance_id}` working
- `GET /api/v1/kine/instances/{instance_id}` working
- `PATCH /api/v1/kine/instances/{instance_id}/calibration` working

### Phase 3: Kinematic Computations (Week 4-5)

**Tasks:**
1. Implement `KinematicSolver` class
2. Implement FK computation (pose, velocity, acceleration)
3. Implement Jacobian matrix computation
4. Implement configuration identification
5. Implement singularity detection (geometric + SVD methods)
6. Create pybind11 bindings for solver
7. Implement FastAPI endpoints for FK computations
8. Write unit tests

**Deliverables:**
- `POST /api/v1/fk/compute/pose` working
- `POST /api/v1/fk/compute/velocity` working
- `POST /api/v1/fk/compute/acceleration` working
- `POST /api/v1/fk/analysis/jacobian` working
- `POST /api/v1/fk/analysis/config-flags` working
- `POST /api/v1/fk/analysis/singularity-check` working

### Phase 4: Safety & Utility Tools (Week 6)

**Tasks:**
1. Implement `SafetyChecker` class
2. Implement joint limit checking
3. Implement `MathUtils` for transform operations
4. Implement DH matrix calculation
5. Implement `DynamicsSolver` for static torque computation
6. Create pybind11 bindings
7. Implement FastAPI endpoints
8. Write unit tests

**Deliverables:**
- `POST /api/v1/safety/joint-limits` working
- `POST /api/v1/fk/utils/transform` working
- `POST /api/v1/fk/utils/dh-matrix` working
- `POST /api/v1/fk/analysis/static-torque` working

### Phase 5: Additional Model Types & Integration (Week 7-8)

**Tasks:**
1. Implement `POEModel` class
2. Implement `URDFModel` class (integrate with urdf_parser)
3. Implement `DeltaModel` class for parallel robots
4. Integrate with existing `mm_config` for backward compatibility
5. Add ROS bridge node (optional, for hybrid mode)
6. End-to-end integration testing
7. Documentation

**Deliverables:**
- Full support for MDH, POE, URDF model types
- Backward compatible with existing ROS workflows
- API documentation

---

## 6. Exception Handling Strategy

### 6.1 HTTP Status Codes

| Status Code | Meaning | Usage |
|-------------|---------|-------|
| 200 OK | Success | GET, PATCH success |
| 201 Created | Created | POST model/instance success |
| 400 Bad Request | Parameter error | Invalid JSON, missing fields |
| 401 Unauthorized | Auth failed | Missing/invalid JWT |
| 403 Forbidden | Permission denied | Valid JWT, no permission |
| 404 Not Found | Resource not found | model_id/instance_id doesn't exist |
| 409 Conflict | State conflict | Duplicate ID, instance in MOTION |
| 422 Unprocessable | Semantic error | URDF parse error, NaN in computation |
| 429 Too Many Requests | Rate limited | Exceeded API call limit |
| 500 Server Error | Internal error | C++ exception, memory error |
| 502 Bad Gateway | Connection error | API connection failed |
| 504 Gateway Timeout | Timeout | Component response timeout |

### 6.2 Error Response Format

```json
{
  "error": {
    "code": "MODEL_NOT_FOUND",
    "message": "Model 'fanuc_m20ia_01' not found",
    "details": {
      "model_id": "fanuc_m20ia_01"
    }
  },
  "request_id": "req_abc123",
  "timestamp": "2026-01-20T10:00:00Z"
}
```

### 6.3 C++ Exception Hierarchy

```cpp
namespace remani::core {

class RemaniException : public std::runtime_error {
public:
    explicit RemaniException(const std::string& msg, const std::string& code = "UNKNOWN_ERROR")
        : std::runtime_error(msg), error_code_(code) {}
    
    const std::string& code() const { return error_code_; }
    
private:
    std::string error_code_;
};

class ModelNotFoundException : public RemaniException {
public:
    explicit ModelNotFoundException(const std::string& model_id)
        : RemaniException("Model '" + model_id + "' not found", "MODEL_NOT_FOUND"),
          model_id_(model_id) {}
    
    const std::string& modelId() const { return model_id_; }
    
private:
    std::string model_id_;
};

class InstanceNotFoundException : public RemaniException { /* ... */ };
class InvalidParameterException : public RemaniException { /* ... */ };
class SingularityException : public RemaniException { /* ... */ };
class CalibrationException : public RemaniException { /* ... */ };

} // namespace remani::core
```

### 6.4 Python Exception Handlers

```python
# remani_api/exceptions/handlers.py

from fastapi import Request
from fastapi.responses import JSONResponse
from remani_api.exceptions import (
    ModelNotFoundError,
    InstanceNotFoundError,
    InvalidParameterError,
    SingularityError,
)

@app.exception_handler(ModelNotFoundError)
async def model_not_found_handler(request: Request, exc: ModelNotFoundError):
    return JSONResponse(
        status_code=404,
        content={
            "error": {
                "code": "MODEL_NOT_FOUND",
                "message": str(exc),
                "details": {"model_id": exc.model_id}
            }
        }
    )

@app.exception_handler(SingularityError)
async def singularity_handler(request: Request, exc: SingularityError):
    return JSONResponse(
        status_code=422,
        content={
            "error": {
                "code": "SINGULARITY_DETECTED",
                "message": str(exc),
                "details": exc.details
            }
        }
    )
```

---

## 7. Authentication & Security

### 7.1 JWT Authentication

```python
# remani_api/auth/jwt_handler.py

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import jwt

security = HTTPBearer()

async def verify_token(credentials: HTTPAuthorizationCredentials = Depends(security)):
    try:
        payload = jwt.decode(
            credentials.credentials,
            settings.JWT_SECRET,
            algorithms=["HS256"]
        )
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired"
        )
    except jwt.InvalidTokenError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )
```

### 7.2 Request Signing (Optional)

For additional security, implement request signing:
- `Token`: JWT credential
- `Timestamp`: Request timestamp (ms)
- `Nonce`: Random string (prevent replay)
- `Sign`: HMAC(Token + Timestamp + Nonce)

---

## 8. Testing Strategy

### 8.1 Unit Tests (C++)

```cpp
// tests/cpp/test_mdh_model.cpp

#include <gtest/gtest.h>
#include <remani/core/mdh_model.hpp>

TEST(MDHModelTest, ForwardKinematicsUR5) {
    // UR5 MDH parameters
    std::vector<remani::core::MDHJoint> joints = {
        {0, JointType::REVOLUTE, 0.0, 0.0, 0.089159, 0.0},
        {1, JointType::REVOLUTE, 0.0, 1.5708, 0.0, 0.0},
        // ... remaining joints
    };
    
    remani::core::MDHModel model("ur5", 6, joints);
    
    std::vector<double> q = {0, 0, 0, 0, 0, 0};
    Eigen::Matrix4d T = model.computeFK(q);
    
    // Verify end-effector position
    EXPECT_NEAR(T(0, 3), 0.81725, 0.001);
    EXPECT_NEAR(T(2, 3), 0.19145, 0.001);
}

TEST(MDHModelTest, JacobianComputation) {
    // Test Jacobian at known configuration
}
```

### 8.2 Integration Tests (Python)

```python
# tests/python/test_fk.py

import pytest
from httpx import AsyncClient
from remani_api.main import app

@pytest.mark.asyncio
async def test_compute_fk():
    async with AsyncClient(app=app, base_url="http://test") as client:
        # Register model first
        model_response = await client.post(
            "/api/v1/kine/models",
            json={
                "model_id": "test_ur5",
                "model_type": "MDH",
                "dof": 6,
                "params": {"chain": [...]}
            }
        )
        assert model_response.status_code == 201
        
        # Configure instance
        instance_response = await client.put(
            "/api/v1/kine/instances/test_robot",
            json={"model_id": "test_ur5"}
        )
        assert instance_response.status_code == 201
        
        # Compute FK
        fk_response = await client.post(
            "/api/v1/fk/compute/pose",
            json={
                "instance_id": "test_robot",
                "joints": [0, 0, 0, 0, 0, 0]
            }
        )
        assert fk_response.status_code == 200
        data = fk_response.json()
        assert "pos" in data
        assert len(data["pos"]) == 3
```

---

## 9. Migration from Existing Code

### 9.1 MMConfig Adapter

The existing `MMConfig` class will be adapted to use the new `KinematicModel`:

```cpp
// remani_planner/mm_config/include/mm_config/mm_config.hpp

class MMConfig {
public:
    // Keep existing interface for backward compatibility
    void setParam(ros::NodeHandle &nh);
    void getAJointTran(int joint_num, double theta, 
                       Eigen::Matrix4d &T, Eigen::Matrix4d &T_grad);
    
    // NEW: Bridge to new core
    void setKinematicModel(std::shared_ptr<KinematicModel> model);
    std::shared_ptr<KinematicModel> getKinematicModel() const;
    
private:
    std::shared_ptr<KinematicModel> kinematic_model_;
    // ... existing members kept for transition
};
```

### 9.2 YAML Configuration Migration

Existing `mm_param.yaml` will be supported via adapter:

```yaml
# mm_param.yaml (existing format)
mm:
  mobile_base_dof: 2
  manipulator_dof: 6
  use_fast_armer: true
  manipulator_config: [0.176, 0.417, 0.07, 0.385, 0.0, 0.20]
  manipulator_min_pos: [-3.062, -3.136, -0.008, -2.932, -3.655, -3.14]
  manipulator_max_pos: [3.052, 0.001, 3.044, 2.950, 0.447, 3.14]
```

```cpp
// Adapter to convert legacy YAML to new model format
KinematicModel MMConfigAdapter::fromYAML(const YAML::Node& yaml) {
    // Convert manipulator_config to MDH parameters
    // Convert limits to JointLimits struct
    // Return KinematicModel
}
```

---

## 10. Dependencies

### 10.1 C++ Dependencies

```cmake
# CMakeLists.txt (core)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)  # JSON serialization

# Optional
find_package(urdfdom REQUIRED)        # URDF parsing
find_package(ros REQUIRED)            # ROS integration
```

### 10.2 Python Dependencies

```txt
# requirements.txt

fastapi>=0.100.0
uvicorn>=0.23.0
pydantic>=2.0.0
python-jose[cryptography]>=3.3.0  # JWT
httpx>=0.24.0                      # Testing
pytest>=7.0.0
pytest-asyncio>=0.21.0
```

---

## 13. Open Questions

1. **Inverse Kinematics**: The current API spec focuses on FK. Should IK be added? (Not in DZ spec but may be needed for planning)

2. **Dynamics Parameters**: The spec mentions dynamics (mass, inertia) but doesn't define how they're registered. Should we extend the model registration to include dynamics?

3. **Persistence**: Should models/instances persist across server restarts? If so, what storage backend? (SQLite, file-based, etc.)

4. **gRPC Support**: The spec mentions gRPC for high-frequency interactions. Should this be implemented alongside REST?

5. **ROS Bridge**: For hybrid mode, should we create a ROS node that bridges the REST API to ROS topics/services?

---

*Document Version: 1.0*
*Created: 2026-01-20*
*Author: Cascade AI Assistant*
