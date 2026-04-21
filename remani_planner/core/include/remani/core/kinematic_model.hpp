/**
 * @file kinematic_model.hpp
 * @brief Abstract base class for kinematic models
 */

#ifndef REMANI_CORE_KINEMATIC_MODEL_HPP
#define REMANI_CORE_KINEMATIC_MODEL_HPP

#include "remani/core/kinematic_params.hpp"
#include "remani/core/model.hpp"
#include <memory>
#include <vector>

namespace remani::core {

/**
 * @brief Abstract base class for kinematic models
 * 
 * Provides a polymorphic interface for different kinematic representations
 * (MDH, POE, URDF, Delta). Each derived class implements the specific
 * kinematic computations for its representation.
 */
class KinematicModelBase {
public:
    virtual ~KinematicModelBase() = default;
    
    // ========================================================================
    // Model Information
    // ========================================================================
    
    /**
     * @brief Get the model type
     */
    virtual ModelType getModelType() const = 0;
    
    /**
     * @brief Get the model type as string
     */
    virtual std::string getModelTypeString() const = 0;
    
    /**
     * @brief Get degrees of freedom
     */
    virtual int getDOF() const = 0;
    
    /**
     * @brief Get model capabilities
     */
    virtual ModelCapabilities getCapabilities() const = 0;
    
    // ========================================================================
    // Forward Kinematics
    // ========================================================================
    
    /**
     * @brief Compute forward kinematics - end effector pose
     * @param joints Joint angles (rad) or positions (m)
     * @return 4x4 homogeneous transformation matrix
     */
    virtual Eigen::Matrix4d computeFK(const std::vector<double>& joints) const = 0;
    
    /**
     * @brief Compute forward kinematics with TCP offset
     * @param joints Joint angles
     * @param tcp Tool center point offset from flange
     * @return 4x4 transformation matrix in base frame
     */
    virtual Eigen::Matrix4d computeFKWithTCP(
        const std::vector<double>& joints,
        const TCPDefinition& tcp) const;
    
    /**
     * @brief Compute forward kinematics for a specific joint frame
     * @param joints Joint angles
     * @param joint_index Joint index (0-based)
     * @return 4x4 transformation matrix for joint frame
     */
    virtual Eigen::Matrix4d computeJointTransform(
        const std::vector<double>& joints,
        int joint_index) const = 0;
    
    // ========================================================================
    // Jacobian
    // ========================================================================
    
    /**
     * @brief Compute geometric Jacobian matrix
     * @param joints Joint angles
     * @return 6xN Jacobian matrix [J_v; J_w] (linear; angular)
     */
    virtual Eigen::MatrixXd computeJacobian(const std::vector<double>& joints) const = 0;
    
    /**
     * @brief Compute Jacobian with TCP offset
     * @param joints Joint angles
     * @param tcp Tool center point offset
     * @return 6xN Jacobian matrix
     */
    virtual Eigen::MatrixXd computeJacobianWithTCP(
        const std::vector<double>& joints,
        const TCPDefinition& tcp) const;
    
    /**
     * @brief Compute Jacobian time derivative
     * @param joints Joint angles
     * @param joint_velocities Joint velocities
     * @return 6xN Jacobian derivative matrix
     */
    virtual Eigen::MatrixXd computeJacobianDot(
        const std::vector<double>& joints,
        const std::vector<double>& joint_velocities) const;
    
    // ========================================================================
    // Velocity/Acceleration Mapping
    // ========================================================================
    
    /**
     * @brief Compute Cartesian velocity from joint velocity
     * @param joints Joint angles
     * @param joint_velocities Joint velocities
     * @return Cartesian velocity (linear + angular)
     */
    virtual CartesianVelocity computeVelocity(
        const std::vector<double>& joints,
        const std::vector<double>& joint_velocities) const;
    
    /**
     * @brief Compute Cartesian acceleration
     * @param joints Joint angles
     * @param joint_velocities Joint velocities
     * @param joint_accelerations Joint accelerations
     * @return Cartesian acceleration
     */
    virtual CartesianAcceleration computeAcceleration(
        const std::vector<double>& joints,
        const std::vector<double>& joint_velocities,
        const std::vector<double>& joint_accelerations) const;
    
    /**
     * @brief Compute static torque for gravity compensation
     * @param joints Joint positions
     * @param gravity Gravity vector in base frame
     * @param payload Optional payload at TCP
     * @return Static torque vector
     */
    virtual std::vector<double> computeStaticTorque(
        const std::vector<double>& joints,
        const Eigen::Vector3d& gravity,
        const Payload& payload = Payload()) const;
    
    // ========================================================================
    // Singularity Detection
    // ========================================================================
    
    /**
     * @brief Check if configuration is near singularity
     * @param joints Joint angles
     * @param threshold Singularity threshold (default 0.01)
     * @return true if near singularity
     */
    virtual bool isNearSingularity(const std::vector<double>& joints,
                                   double threshold = 0.01) const;
    
    /**
     * @brief Get minimum singular value of Jacobian
     * @param joints Joint angles
     * @return Minimum singular value
     */
    virtual double getMinSingularValue(const std::vector<double>& joints) const;
    
    /**
     * @brief Compute manipulability measure (Yoshikawa)
     * @param joints Joint angles
     * @return Manipulability measure sqrt(det(J*J^T))
     */
    virtual double computeManipulability(const std::vector<double>& joints) const;
    
    // ========================================================================
    // Configuration Identification
    // ========================================================================
    
    /**
     * @brief Identify configuration flags (shoulder/elbow/wrist)
     * @param joints Joint angles
     * @return Configuration ID and flags
     */
    virtual std::pair<int, std::map<std::string, std::string>> identifyConfiguration(
        const std::vector<double>& joints) const;
    
    // ========================================================================
    // Joint Validation
    // ========================================================================
    
    /**
     * @brief Validate joint angles against limits
     * @param joints Joint angles
     * @param limits Joint limits
     * @return true if all joints within limits
     */
    virtual bool validateJoints(const std::vector<double>& joints,
                                const JointLimits& limits) const;
    
    /**
     * @brief Get joint limit violations
     * @param joints Joint angles
     * @param limits Joint limits
     * @return List of violations
     */
    virtual std::vector<std::pair<int, std::string>> getJointViolations(
        const std::vector<double>& joints,
        const JointLimits& limits) const;

protected:
    /**
     * @brief Validate input joint vector size
     * @throws std::invalid_argument if size mismatch
     */
    void validateJointSize(const std::vector<double>& joints, int expected_size) const;
};

// ============================================================================
// MDH Model Implementation
// ============================================================================

/**
 * @brief MDH (Modified Denavit-Hartenberg) Kinematic Model
 * 
 * Implements kinematics using Craig's modified DH convention.
 * Transform for each joint: T = RotX(alpha) * TransX(a) * RotZ(theta) * TransZ(d)
 */
class MDHModel : public KinematicModelBase {
public:
    /**
     * @brief Construct from MDH parameters
     */
    explicit MDHModel(const MDHParams& params);
    
    /**
     * @brief Construct from vector of MDH joints
     */
    explicit MDHModel(const std::vector<MDHJoint>& chain);
    
    // Implement base class methods
    ModelType getModelType() const override { return ModelType::MDH; }
    std::string getModelTypeString() const override { return "MDH"; }
    int getDOF() const override { return static_cast<int>(chain_.size()); }
    ModelCapabilities getCapabilities() const override;
    
    Eigen::Matrix4d computeFK(const std::vector<double>& joints) const override;
    Eigen::Matrix4d computeJointTransform(
        const std::vector<double>& joints, int joint_index) const override;
    Eigen::MatrixXd computeJacobian(const std::vector<double>& joints) const override;
    
    std::pair<int, std::map<std::string, std::string>> identifyConfiguration(
        const std::vector<double>& joints) const override;
    
    // MDH-specific methods
    
    /**
     * @brief Get the MDH parameter chain
     */
    const std::vector<MDHJoint>& getChain() const { return chain_; }
    
    /**
     * @brief Compute single joint transformation matrix
     * @param joint MDH joint parameters
     * @param theta Joint angle (variable for revolute)
     * @return 4x4 transformation matrix
     */
    static Eigen::Matrix4d computeMDHTransform(const MDHJoint& joint, double theta);
    
    /**
     * @brief Compute transformation matrix derivative w.r.t. theta
     */
    static Eigen::Matrix4d computeMDHTransformDerivative(const MDHJoint& joint, double theta);
    
    /**
     * @brief Apply calibration corrections
     */
    void applyCalibration(const std::vector<MDHCalibrationCorrection>& corrections);
    
private:
    std::vector<MDHJoint> chain_;
    
    /**
     * @brief Compute axis of rotation for a joint in base frame
     */
    Eigen::Vector3d computeJointAxis(const std::vector<double>& joints, int joint_idx) const;
    
    /**
     * @brief Compute position of joint origin in base frame
     */
    Eigen::Vector3d computeJointOrigin(const std::vector<double>& joints, int joint_idx) const;
};

// ============================================================================
// POE Model Implementation
// ============================================================================

/**
 * @brief POE (Product of Exponentials) Kinematic Model
 * 
 * Implements kinematics using the Product of Exponentials formulation
 * based on screw theory. T = e^[S1]q1 * e^[S2]q2 * ... * e^[Sn]qn * M
 */
class POEModel : public KinematicModelBase {
public:
    /**
     * @brief Construct from POE parameters
     */
    explicit POEModel(const POEParams& params);
    
    // Implement base class methods
    ModelType getModelType() const override { return ModelType::POE; }
    std::string getModelTypeString() const override { return "POE"; }
    int getDOF() const override { return static_cast<int>(screw_axes_.size()); }
    ModelCapabilities getCapabilities() const override;
    
    Eigen::Matrix4d computeFK(const std::vector<double>& joints) const override;
    Eigen::Matrix4d computeJointTransform(
        const std::vector<double>& joints, int joint_index) const override;
    Eigen::MatrixXd computeJacobian(const std::vector<double>& joints) const override;
    
    // POE-specific methods
    
    /**
     * @brief Get screw axes
     */
    const std::vector<POEScrewAxis>& getScrewAxes() const { return screw_axes_; }
    
    /**
     * @brief Get home position transformation
     */
    const Eigen::Matrix4d& getMHome() const { return m_home_; }
    
    /**
     * @brief Compute matrix exponential of a screw
     * @param screw Screw axis [w; v]
     * @param theta Displacement
     * @return 4x4 transformation matrix
     */
    static Eigen::Matrix4d matrixExponential(const POEScrewAxis& screw, double theta);
    
    /**
     * @brief Compute screw adjoint matrix
     */
    static Eigen::Matrix<double, 6, 6> adjoint(const Eigen::Matrix4d& T);
    
private:
    std::vector<POEScrewAxis> screw_axes_;
    Eigen::Matrix4d m_home_;
};

// ============================================================================
// URDF Model Implementation
// ============================================================================

/**
 * @brief URDF Kinematic Model
 */
class URDFModel : public KinematicModelBase {
public:
    explicit URDFModel(const URDFParams& params);
    
    ModelType getModelType() const override { return ModelType::URDF; }
    std::string getModelTypeString() const override { return "URDF"; }
    int getDOF() const override { return dof_; }
    ModelCapabilities getCapabilities() const override;
    
    Eigen::Matrix4d computeFK(const std::vector<double>& joints) const override;
    Eigen::Matrix4d computeJointTransform(
        const std::vector<double>& joints, int joint_index) const override;
    Eigen::MatrixXd computeJacobian(const std::vector<double>& joints) const override;
    
private:
    std::string urdf_content_;
    std::string base_link_;
    std::string tip_link_;
    int dof_;
    // URDF model internal representation would go here
};

// ============================================================================
// Delta Model Implementation
// ============================================================================

/**
 * @brief Delta Parallel Robot Kinematic Model
 */
class DeltaModel : public KinematicModelBase {
public:
    explicit DeltaModel(const DeltaParams& params);
    
    ModelType getModelType() const override { return ModelType::DELTA; }
    std::string getModelTypeString() const override { return "DELTA"; }
    int getDOF() const override { return 3; }
    ModelCapabilities getCapabilities() const override;
    
    Eigen::Matrix4d computeFK(const std::vector<double>& joints) const override;
    Eigen::Matrix4d computeJointTransform(
        const std::vector<double>& joints, int joint_index) const override;
    Eigen::MatrixXd computeJacobian(const std::vector<double>& joints) const override;
    
private:
    DeltaParams params_;
};

// ============================================================================
// Factory Function
// ============================================================================

/**
 * @brief Factory function to create kinematic model from parameters
 * @param params Kinematic parameters (polymorphic)
 * @return Shared pointer to created model
 */
std::shared_ptr<KinematicModelBase> createKinematicModel(const KinematicParams& params);

/**
 * @brief Factory function to create kinematic model from model struct
 * @param model Kinematic model definition
 * @return Shared pointer to created model
 */
std::shared_ptr<KinematicModelBase> createKinematicModel(const KinematicModel& model);

} // namespace remani::core

#endif // REMANI_CORE_KINEMATIC_MODEL_HPP
