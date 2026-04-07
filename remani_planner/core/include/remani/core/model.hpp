/**
 * @file model.hpp
 * @brief Kinematic Model and Runtime Instance definitions
 */

#ifndef REMANI_CORE_MODEL_HPP
#define REMANI_CORE_MODEL_HPP

#include "remani/core/kinematic_params.hpp"
#include <chrono>
#include <memory>
#include <functional>

namespace remani::core {

// ============================================================================
// Model Status
// ============================================================================

/**
 * @brief Model status enumeration
 */
enum class ModelStatus {
    ACTIVE,       ///< Model is active and ready for use
    ERROR,        ///< Model has configuration errors
    CALIBRATING,  ///< Model is being calibrated
    READY         ///< Model is ready (same as ACTIVE)
};

/**
 * @brief Convert ModelStatus to string
 */
inline std::string modelStatusToString(ModelStatus status) {
    switch (status) {
        case ModelStatus::ACTIVE: return "ACTIVE";
        case ModelStatus::ERROR: return "ERROR";
        case ModelStatus::CALIBRATING: return "CALIBRATING";
        case ModelStatus::READY: return "READY";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Convert string to ModelStatus
 */
inline ModelStatus stringToModelStatus(const std::string& str) {
    if (str == "ERROR") return ModelStatus::ERROR;
    if (str == "CALIBRATING") return ModelStatus::CALIBRATING;
    if (str == "READY") return ModelStatus::READY;
    return ModelStatus::ACTIVE;
}

// ============================================================================
// Instance Status
// ============================================================================

/**
 * @brief Instance status enumeration
 */
enum class InstanceStatus {
    READY,    ///< Instance is ready for commands
    MOVING,   ///< Instance is currently in motion
    ERROR,    ///< Instance has encountered an error
    STOPPED   ///< Instance is stopped (e-stop or pause)
};

/**
 * @brief Convert InstanceStatus to string
 */
inline std::string instanceStatusToString(InstanceStatus status) {
    switch (status) {
        case InstanceStatus::READY: return "READY";
        case InstanceStatus::MOVING: return "MOVING";
        case InstanceStatus::ERROR: return "ERROR";
        case InstanceStatus::STOPPED: return "STOPPED";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Convert string to InstanceStatus
 */
inline InstanceStatus stringToInstanceStatus(const std::string& str) {
    if (str == "MOVING") return InstanceStatus::MOVING;
    if (str == "ERROR") return InstanceStatus::ERROR;
    if (str == "STOPPED") return InstanceStatus::STOPPED;
    return InstanceStatus::READY;
}

// ============================================================================
// Mounting Type
// ============================================================================

/**
 * @brief Robot mounting type
 */
enum class MountingType {
    FLOOR,     ///< Floor mounted (default)
    CEILING,   ///< Ceiling mounted (inverted)
    WALL,      ///< Wall mounted
    ANGLED     ///< Angled mounting
};

/**
 * @brief Convert MountingType to string
 */
inline std::string mountingTypeToString(MountingType type) {
    switch (type) {
        case MountingType::FLOOR: return "FLOOR";
        case MountingType::CEILING: return "CEILING";
        case MountingType::WALL: return "WALL";
        case MountingType::ANGLED: return "ANGLED";
        default: return "FLOOR";
    }
}

/**
 * @brief Convert string to MountingType
 */
inline MountingType stringToMountingType(const std::string& str) {
    if (str == "CEILING") return MountingType::CEILING;
    if (str == "WALL") return MountingType::WALL;
    if (str == "ANGLED") return MountingType::ANGLED;
    return MountingType::FLOOR;
}

// ============================================================================
// Kinematic Model
// ============================================================================

/**
 * @brief Kinematic Model (immutable after registration)
 * 
 * Represents a robot's mathematical model with kinematic parameters.
 * This is the "type" definition, not a specific robot instance.
 */
struct KinematicModel {
    std::string model_id;       ///< Unique identifier
    std::string name;           ///< Human-readable name
    ModelType model_type;       ///< MDH, POE, URDF, DELTA
    int dof;                    ///< Degrees of freedom
    ModelStatus status;         ///< Current status
    
    KinematicParams params;     ///< Kinematic parameters
    JointLimits limits;         ///< Joint limits
    
    // Metadata
    std::string created_at;     ///< ISO 8601 timestamp
    std::string last_updated;   ///< ISO 8601 timestamp
    std::string solver_engine;  ///< e.g., "RNE_MDH_OPTIMIZED"
    double total_mass;          ///< Total robot mass (kg), 0 if unknown
    
    KinematicModel() 
        : model_type(ModelType::MDH), dof(0), status(ModelStatus::ACTIVE),
          total_mass(0.0) {}
    
    /**
     * @brief Check if model is valid
     */
    bool isValid() const {
        return !model_id.empty() && dof > 0;
    }
    
    /**
     * @brief Get current timestamp in ISO 8601 format
     */
    static std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        char buf[30];
        std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&time));
        return std::string(buf);
    }
};

// ============================================================================
// Runtime Instance
// ============================================================================

/**
 * @brief Tool Center Point (TCP) definition
 */
struct TCPDefinition {
    Eigen::Vector3d position;      ///< Position relative to flange
    Eigen::Quaterniond rotation;   ///< Orientation relative to flange
    
    TCPDefinition() 
        : position(Eigen::Vector3d::Zero()), 
          rotation(Eigen::Quaterniond::Identity()) {}
    
    TCPDefinition(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot)
        : position(pos), rotation(rot) {}
    
    /**
     * @brief Convert to 4x4 transformation matrix
     */
    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        T.block<3, 1>(0, 3) = position;
        return T;
    }
    
    /**
     * @brief Create from position and quaternion [x, y, z, qw, qx, qy, qz]
     */
    static TCPDefinition fromPosQuat(const std::vector<double>& pos,
                                      const std::vector<double>& quat) {
        TCPDefinition tcp;
        if (pos.size() >= 3) {
            tcp.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
        }
        if (quat.size() >= 4) {
            // Assuming [qw, qx, qy, qz] or [qx, qy, qz, qw] format
            // Common convention: [qx, qy, qz, qw]
            if (quat.size() == 4) {
                tcp.rotation = Eigen::Quaterniond(quat[3], quat[0], quat[1], quat[2]);
            }
        }
        return tcp;
    }
};

/**
 * @brief Base Frame definition
 */
struct BaseFrame {
    Eigen::Vector3d position;      ///< Position in world coordinates
    Eigen::Quaterniond rotation;   ///< Orientation in world coordinates
    
    BaseFrame() 
        : position(Eigen::Vector3d::Zero()),
          rotation(Eigen::Quaterniond::Identity()) {}
    
    /**
     * @brief Convert to 4x4 transformation matrix
     */
    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        T.block<3, 1>(0, 3) = position;
        return T;
    }
    
    /**
     * @brief Create from position and rotation
     */
    static BaseFrame fromPosRot(const std::vector<double>& pos,
                                const std::vector<double>& rot) {
        BaseFrame frame;
        if (pos.size() >= 3) {
            frame.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
        }
        if (rot.size() >= 4) {
            // Quaternion format [qx, qy, qz, qw]
            frame.rotation = Eigen::Quaterniond(rot[3], rot[0], rot[1], rot[2]);
        }
        return frame;
    }
};

/**
 * @brief Runtime Instance (mutable)
 * 
 * Represents a specific robot in the workspace.
 * Binds a KinematicModel to a physical location and configuration.
 */
struct RuntimeInstance {
    std::string instance_id;    ///< Unique identifier (e.g., "robot_cell_01_A")
    std::string model_id;       ///< Reference to kinematic model
    
    // Configuration
    BaseFrame base_frame;       ///< Robot base in world coordinates
    MountingType mounting_type; ///< Mounting configuration
    double mounting_angle;      ///< Angle for ANGLED mounting (rad)
    TCPDefinition default_tcp;  ///< Default tool center point
    
    // Runtime state
    InstanceStatus status;      ///< Current operational status
    std::vector<double> current_joints;   ///< Cached joint positions
    std::vector<double> joint_offsets;    ///< Calibration offsets (rad)
    
    // Computed from mounting type
    Eigen::Vector3d gravity_vector;  ///< Gravity vector in base frame
    
    // Metadata
    std::string created_at;
    std::string last_updated;
    std::string config_hash;    ///< Configuration fingerprint
    
    RuntimeInstance() 
        : mounting_type(MountingType::FLOOR), mounting_angle(0.0),
          status(InstanceStatus::READY),
          gravity_vector(0, 0, -9.81) {}
    
    /**
     * @brief Compute gravity vector based on mounting type
     */
    void computeGravityVector() {
        switch (mounting_type) {
            case MountingType::FLOOR:
                gravity_vector = Eigen::Vector3d(0, 0, -9.81);
                break;
            case MountingType::CEILING:
                gravity_vector = Eigen::Vector3d(0, 0, 9.81);  // Inverted
                break;
            case MountingType::WALL:
                gravity_vector = Eigen::Vector3d(0, -9.81, 0);
                break;
            case MountingType::ANGLED:
                // Compute based on mounting_angle
                gravity_vector = Eigen::Vector3d(
                    0,
                    -9.81 * sin(mounting_angle),
                    -9.81 * cos(mounting_angle)
                );
                break;
        }
    }
    
    /**
     * @brief Check if instance is valid
     */
    bool isValid() const {
        return !instance_id.empty() && !model_id.empty();
    }
    
    /**
     * @brief Check if instance can be modified
     */
    bool canModify() const {
        return status != InstanceStatus::MOVING;
    }
};

// ============================================================================
// Model Capabilities
// ============================================================================

/**
 * @brief Kinematic capabilities descriptor
 */
struct KinematicCapabilities {
    bool has_closed_form_fk;        ///< Has analytical FK solution
    bool has_closed_form_ik;        ///< Has analytical IK solution
    bool requires_numerical_ik;     ///< Requires numerical IK
    std::string ik_solutions_count;  ///< "UNIQUE", "MULTIPLE", "INFINITE"
    bool supports_spherical_wrist;  ///< Has spherical wrist
    
    KinematicCapabilities()
        : has_closed_form_fk(true), has_closed_form_ik(false),
          requires_numerical_ik(true), ik_solutions_count("MULTIPLE"),
          supports_spherical_wrist(false) {}
};

/**
 * @brief Differential kinematics capabilities
 */
struct DifferentialCapabilities {
    std::string jacobian_solver;  ///< "ANALYTIC" or "NUMERICAL"
    bool is_redundant;            ///< DOF > task space dimension
    bool singularity_detection;   ///< Built-in singularity detection
    
    DifferentialCapabilities()
        : jacobian_solver("ANALYTIC"), is_redundant(false),
          singularity_detection(true) {}
};

/**
 * @brief Dynamics capabilities
 */
struct DynamicsCapabilities {
    bool supports_inverse_dynamics;  ///< RNE algorithm
    bool supports_gravity_comp;      ///< Gravity compensation
    bool supports_inertia_matrix;    ///< Mass matrix computation
    bool payload_sensitive;          ///< Sensitive to payload changes
    
    DynamicsCapabilities()
        : supports_inverse_dynamics(true), supports_gravity_comp(true),
          supports_inertia_matrix(false), payload_sensitive(true) {}
};

/**
 * @brief Performance hints
 */
struct PerformanceHints {
    std::string fk_compute_cost;   ///< "LOW", "MEDIUM", "HIGH"
    std::string ik_compute_cost;   ///< "LOW", "MEDIUM", "HIGH"
    int recommended_frequency_hz;  ///< Recommended control loop frequency
    
    PerformanceHints()
        : fk_compute_cost("LOW"), ik_compute_cost("MEDIUM"),
          recommended_frequency_hz(1000) {}
};

/**
 * @brief Complete model capabilities
 */
struct ModelCapabilities {
    std::string model_id;
    std::string structure_type;  ///< "SERIAL", "PARALLEL", "HYBRID"
    
    KinematicCapabilities kinematics;
    DifferentialCapabilities differential;
    DynamicsCapabilities dynamics;
    PerformanceHints performance_hints;
    
    ModelCapabilities() : structure_type("SERIAL") {}
};

// ============================================================================
// Payload Definition
// ============================================================================

/**
 * @brief Payload (end-effector load) definition
 */
struct Payload {
    double mass;                    ///< Payload mass (kg)
    Eigen::Vector3d cog;            ///< Center of gravity relative to flange
    Eigen::Matrix3d inertia;        ///< Inertia tensor (optional)
    bool has_inertia;               ///< Whether inertia is specified
    
    Payload() : mass(0.0), has_inertia(false) {
        cog = Eigen::Vector3d::Zero();
        inertia = Eigen::Matrix3d::Zero();
    }
    
    Payload(double m, const Eigen::Vector3d& c) 
        : mass(m), cog(c), has_inertia(false) {
        inertia = Eigen::Matrix3d::Zero();
    }
    
    Payload(double m, const Eigen::Vector3d& c, const Eigen::Matrix3d& I)
        : mass(m), cog(c), inertia(I), has_inertia(true) {}
};

// ============================================================================
// Pose Structure
// ============================================================================

/**
 * @brief End-effector pose with multiple representations
 */
struct EndEffectorPose {
    Eigen::Vector3d position;        ///< [x, y, z] in meters
    Eigen::Quaterniond quaternion;   ///< [w, x, y, z]
    Eigen::Vector3d euler_zyx;       ///< [roll, pitch, yaw] in radians
    Eigen::Matrix4d matrix;          ///< 4x4 homogeneous transform
    
    EndEffectorPose() 
        : position(Eigen::Vector3d::Zero()),
          quaternion(Eigen::Quaterniond::Identity()),
          euler_zyx(Eigen::Vector3d::Zero()),
          matrix(Eigen::Matrix4d::Identity()) {}
    
    /**
     * @brief Update all representations from matrix
     */
    void fromMatrix(const Eigen::Matrix4d& T) {
        matrix = T;
        position = T.block<3, 1>(0, 3);
        quaternion = Eigen::Quaterniond(T.block<3, 3>(0, 0));
        
        // Extract Euler angles (ZYX convention)
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        euler_zyx = R.eulerAngles(2, 1, 0);  // ZYX -> [yaw, pitch, roll]
        std::swap(euler_zyx.x(), euler_zyx.z());  // Convert to [r, p, y]
    }
};

// ============================================================================
// Velocity and Acceleration
// ============================================================================

/**
 * @brief Cartesian velocity
 */
struct CartesianVelocity {
    Eigen::Vector3d linear;   ///< Linear velocity (m/s)
    Eigen::Vector3d angular;  ///< Angular velocity (rad/s)
    
    double linear_magnitude() const { return linear.norm(); }
    double angular_magnitude() const { return angular.norm(); }
    
    CartesianVelocity() 
        : linear(Eigen::Vector3d::Zero()),
          angular(Eigen::Vector3d::Zero()) {}
};

/**
 * @brief Cartesian acceleration
 */
struct CartesianAcceleration {
    Eigen::Vector3d linear;   ///< Linear acceleration (m/s²)
    Eigen::Vector3d angular;  ///< Angular acceleration (rad/s²)
    
    double linear_magnitude() const { return linear.norm(); }
    double angular_magnitude() const { return angular.norm(); }
    
    CartesianAcceleration()
        : linear(Eigen::Vector3d::Zero()),
          angular(Eigen::Vector3d::Zero()) {}
};

} // namespace remani::core

#endif // REMANI_CORE_MODEL_HPP
