/**
 * @file kinematic_params.hpp
 * @brief Core data structures for kinematic parameters
 * 
 * Supports multiple kinematic modeling conventions:
 * - MDH (Modified Denavit-Hartenberg / Craig)
 * - SDH (Standard Denavit-Hartenberg)
 * - POE (Product of Exponentials)
 * - URDF (Unified Robot Description Format)
 * - DELTA (Parallel Delta robot)
 */

#ifndef REMANI_CORE_KINEMATIC_PARAMS_HPP
#define REMANI_CORE_KINEMATIC_PARAMS_HPP

#include <string>
#include <vector>
#include <array>
#include <variant>
#include <Eigen/Dense>

namespace remani::core {

/**
 * @brief Joint type enumeration
 */
enum class JointType {
    REVOLUTE,   ///< Rotational joint (theta is variable)
    PRISMATIC   ///< Linear joint (d is variable)
};

/**
 * @brief Convert JointType to string
 */
inline std::string jointTypeToString(JointType type) {
    return (type == JointType::REVOLUTE) ? "REVOLUTE" : "PRISMATIC";
}

/**
 * @brief Convert string to JointType
 */
inline JointType stringToJointType(const std::string& str) {
    return (str == "PRISMATIC") ? JointType::PRISMATIC : JointType::REVOLUTE;
}

// ============================================================================
// MDH Parameters (Modified Denavit-Hartenberg / Craig Convention)
// ============================================================================

/**
 * @brief Single joint MDH parameters
 * 
 * Craig's modified DH convention:
 * Transform order: RotX(alpha) -> TransX(a) -> RotZ(theta) -> TransZ(d)
 */
struct MDHJoint {
    int joint_index;       ///< Joint index (0-based)
    JointType type;        ///< Joint type
    double a;              ///< Link length (m) - distance along X
    double alpha;          ///< Link twist (rad) - rotation about X
    double d;              ///< Link offset (m) - distance along Z
    double theta_offset;   ///< Joint angle offset (rad) - rotation about Z

    MDHJoint() 
        : joint_index(0), type(JointType::REVOLUTE), 
          a(0.0), alpha(0.0), d(0.0), theta_offset(0.0) {}
    
    MDHJoint(int idx, JointType t, double a_, double alpha_, double d_, double theta_off = 0.0)
        : joint_index(idx), type(t), a(a_), alpha(alpha_), d(d_), theta_offset(theta_off) {}
};

/**
 * @brief Complete MDH chain parameters
 */
struct MDHParams {
    std::vector<MDHJoint> chain;
    
    MDHParams() = default;
    explicit MDHParams(const std::vector<MDHJoint>& c) : chain(c) {}
};

// ============================================================================
// POE Parameters (Product of Exponentials)
// ============================================================================

/**
 * @brief Single screw axis for POE formulation
 */
struct POEScrewAxis {
    std::array<double, 6> data;  ///< [w1, w2, w3, v1, v2, v3]
    
    POEScrewAxis() : data{0, 0, 0, 0, 0, 0} {}
    POEScrewAxis(const std::array<double, 6>& arr) : data(arr) {}
    
    /// Get angular part (w)
    Eigen::Vector3d angular() const {
        return Eigen::Vector3d(data[0], data[1], data[2]);
    }
    
    /// Get linear part (v)
    Eigen::Vector3d linear() const {
        return Eigen::Vector3d(data[3], data[4], data[5]);
    }
    
    /// Convert to Eigen 6D vector
    Eigen::Matrix<double, 6, 1> toEigen() const {
        Eigen::Matrix<double, 6, 1> vec;
        vec << data[0], data[1], data[2], data[3], data[4], data[5];
        return vec;
    }
};

/**
 * @brief Complete POE parameters
 */
struct POEParams {
    std::vector<POEScrewAxis> screw_axes;  ///< Screw axes S1...Sn
    Eigen::Matrix4d m_home;                 ///< Zero-position end-effector transform M
    
    POEParams() : m_home(Eigen::Matrix4d::Identity()) {}
};

// ============================================================================
// URDF Parameters
// ============================================================================

/**
 * @brief URDF model parameters
 */
struct URDFParams {
    std::string urdf_content;  ///< URDF XML string
    std::string base_link;     ///< Root link name
    std::string tip_link;      ///< End-effector link name
    
    URDFParams() = default;
    URDFParams(const std::string& content, const std::string& base, const std::string& tip)
        : urdf_content(content), base_link(base), tip_link(tip) {}
};

// ============================================================================
// Delta Parameters (Parallel Robot)
// ============================================================================

/**
 * @brief Coupling mode for Delta robot
 */
enum class DeltaCouplingMode {
    ROTARY,   ///< Rotary motor driven
    LINEAR    ///< Linear motor driven
};

/**
 * @brief Delta parallel robot parameters
 */
struct DeltaParams {
    double R_base;       ///< Base platform radius (m)
    double R_platform;   ///< Moving platform radius (m)
    double L_arm;        ///< Active arm length (m)
    double L_rod;        ///< Passive rod length (m)
    DeltaCouplingMode coupling_mode;
    
    DeltaParams() 
        : R_base(0.0), R_platform(0.0), L_arm(0.0), L_rod(0.0),
          coupling_mode(DeltaCouplingMode::ROTARY) {}
    
    DeltaParams(double r_base, double r_platform, double l_arm, double l_rod,
                DeltaCouplingMode mode = DeltaCouplingMode::ROTARY)
        : R_base(r_base), R_platform(r_platform), L_arm(l_arm), L_rod(l_rod),
          coupling_mode(mode) {}
};

// ============================================================================
// Joint Limits
// ============================================================================

/**
 * @brief Joint limits structure
 */
struct JointLimits {
    std::vector<double> q_min;  ///< Minimum joint positions (rad or m)
    std::vector<double> q_max;  ///< Maximum joint positions (rad or m)
    std::vector<double> v_max;  ///< Maximum joint velocities (rad/s or m/s)
    std::vector<double> a_max;  ///< Maximum joint accelerations (rad/s^2 or m/s^2)
    
    JointLimits() = default;
    
    /**
     * @brief Check if limits are valid
     */
    bool isValid() const {
        return !q_min.empty() && !q_max.empty() && 
               q_min.size() == q_max.size();
    }
    
    /**
     * @brief Get number of joints
     */
    size_t size() const { return q_min.size(); }
};

// ============================================================================
// Polymorphic Kinematic Parameters Container
// ============================================================================

/**
 * @brief Model type enumeration
 */
enum class ModelType {
    MDH,
    SDH,
    POE,
    URDF,
    DELTA
};

/**
 * @brief Convert ModelType to string
 */
inline std::string modelTypeToString(ModelType type) {
    switch (type) {
        case ModelType::MDH: return "MDH";
        case ModelType::SDH: return "SDH";
        case ModelType::POE: return "POE";
        case ModelType::URDF: return "URDF";
        case ModelType::DELTA: return "DELTA";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Convert string to ModelType
 */
inline ModelType stringToModelType(const std::string& str) {
    if (str == "SDH") return ModelType::SDH;
    if (str == "POE") return ModelType::POE;
    if (str == "URDF") return ModelType::URDF;
    if (str == "DELTA") return ModelType::DELTA;
    return ModelType::MDH;  // Default
}

/**
 * @brief Polymorphic kinematic parameters container
 * 
 * Uses std::variant to store different parameter types.
 */
struct KinematicParams {
    ModelType model_type;
    int dof;
    
    /// Variant storage for different parameter types
    std::variant<
        MDHParams,
        POEParams,
        URDFParams,
        DeltaParams
    > params;
    
    KinematicParams() : model_type(ModelType::MDH), dof(0) {}
    
    /**
     * @brief Get MDH parameters (throws if wrong type)
     */
    const MDHParams& getMDH() const {
        return std::get<MDHParams>(params);
    }
    
    /**
     * @brief Get POE parameters (throws if wrong type)
     */
    const POEParams& getPOE() const {
        return std::get<POEParams>(params);
    }
    
    /**
     * @brief Get URDF parameters (throws if wrong type)
     */
    const URDFParams& getURDF() const {
        return std::get<URDFParams>(params);
    }
    
    /**
     * @brief Get Delta parameters (throws if wrong type)
     */
    const DeltaParams& getDelta() const {
        return std::get<DeltaParams>(params);
    }
    
    /**
     * @brief Set MDH parameters
     */
    void setMDH(const MDHParams& mdh) {
        model_type = ModelType::MDH;
        params = mdh;
        dof = static_cast<int>(mdh.chain.size());
    }
    
    /**
     * @brief Set POE parameters
     */
    void setPOE(const POEParams& poe) {
        model_type = ModelType::POE;
        params = poe;
        dof = static_cast<int>(poe.screw_axes.size());
    }
    
    /**
     * @brief Set URDF parameters
     */
    void setURDF(const URDFParams& urdf) {
        model_type = ModelType::URDF;
        params = urdf;
        // DOF will be determined by parsing URDF
    }
    
    /**
     * @brief Set Delta parameters
     */
    void setDelta(const DeltaParams& delta) {
        model_type = ModelType::DELTA;
        params = delta;
        dof = 3;  // Delta robots typically have 3 DOF
    }
};

// ============================================================================
// Calibration Parameters
// ============================================================================

/**
 * @brief MDH calibration corrections
 */
struct MDHCalibrationCorrection {
    int joint_index;
    double delta_a;       ///< Correction to link length (m)
    double delta_alpha;   ///< Correction to link twist (rad)
    double delta_d;       ///< Correction to link offset (m)
    double delta_theta;   ///< Correction to joint angle offset (rad)
    
    MDHCalibrationCorrection() 
        : joint_index(0), delta_a(0), delta_alpha(0), delta_d(0), delta_theta(0) {}
};

/**
 * @brief POE calibration corrections
 */
struct POECalibrationCorrection {
    int index;
    std::array<double, 6> delta_axis;  ///< Correction to screw axis
    Eigen::Matrix4d m_home_correction;  ///< Correction to home position
    
    POECalibrationCorrection() 
        : index(0), delta_axis{0,0,0,0,0,0}, 
          m_home_correction(Eigen::Matrix4d::Identity()) {}
};

/**
 * @brief Calibration data for an instance
 */
struct CalibrationData {
    std::string calibration_method;  ///< e.g., "LASER_TRACKER_LEICA_AT960"
    std::vector<double> joint_offsets;  ///< Joint zero position offsets (rad)
    
    /// Variant for different correction types
    std::variant<
        std::vector<MDHCalibrationCorrection>,
        std::vector<POECalibrationCorrection>
    > corrections;
    
    bool force_update;  ///< Force update even if deviation is large
    
    CalibrationData() : force_update(false) {}
};

} // namespace remani::core

#endif // REMANI_CORE_KINEMATIC_PARAMS_HPP
