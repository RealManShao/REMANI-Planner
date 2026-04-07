/**
 * @file kinematic_model.cpp
 * @brief Implementation of kinematic model base class methods
 */

#include "remani/core/kinematic_model.hpp"
#include <stdexcept>
#include <cmath>
#include <Eigen/SVD>

namespace remani::core {

// ============================================================================
// KinematicModelBase Implementation
// ============================================================================

void KinematicModelBase::validateJointSize(const std::vector<double>& joints, 
                                            int expected_size) const {
    if (static_cast<int>(joints.size()) != expected_size) {
        throw std::invalid_argument(
            "Joint vector size mismatch: expected " + std::to_string(expected_size) +
            ", got " + std::to_string(joints.size()));
    }
}

Eigen::Matrix4d KinematicModelBase::computeFKWithTCP(
    const std::vector<double>& joints,
    const TCPDefinition& tcp) const {
    
    Eigen::Matrix4d T_flange = computeFK(joints);
    return T_flange * tcp.toMatrix();
}

Eigen::Matrix4d KinematicModelBase::computeJointTransform(
    const std::vector<double>& joints,
    int joint_index) const {
    // Default implementation: compute FK up to joint_index
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i <= joint_index && i < getDOF(); ++i) {
        // This should be overridden by derived classes for efficiency
    }
    return T;
}

Eigen::MatrixXd KinematicModelBase::computeJacobianWithTCP(
    const std::vector<double>& joints,
    const TCPDefinition& tcp) const {
    
    // Get base Jacobian
    Eigen::MatrixXd J = computeJacobian(joints);
    
    // Transform to TCP frame
    Eigen::Matrix4d T_flange = computeFK(joints);
    Eigen::Matrix4d T_tcp = T_flange * tcp.toMatrix();
    Eigen::Vector3d p_tcp = T_tcp.block<3, 1>(0, 3);
    Eigen::Vector3d p_flange = T_flange.block<3, 1>(0, 3);
    Eigen::Vector3d offset = p_tcp - p_flange;
    
    // Adjust linear velocity part: v_tcp = v_flange + omega × offset
    for (int i = 0; i < J.cols(); ++i) {
        Eigen::Vector3d omega(J(3, i), J(4, i), J(5, i));
        Eigen::Vector3d v_flange(J(0, i), J(1, i), J(2, i));
        Eigen::Vector3d v_tcp = v_flange + omega.cross(offset);
        J(0, i) = v_tcp.x();
        J(1, i) = v_tcp.y();
        J(2, i) = v_tcp.z();
    }
    
    return J;
}

Eigen::MatrixXd KinematicModelBase::computeJacobianDot(
    const std::vector<double>& joints,
    const std::vector<double>& joint_velocities) const {
    
    validateJointSize(joints, getDOF());
    validateJointSize(joint_velocities, getDOF());
    
    // Numerical differentiation of Jacobian
    const double eps = 1e-7;
    int n = getDOF();
    Eigen::MatrixXd J_dot = Eigen::MatrixXd::Zero(6, n);
    
    for (int i = 0; i < n; ++i) {
        std::vector<double> joints_plus = joints;
        joints_plus[i] += eps;
        
        Eigen::MatrixXd J_plus = computeJacobian(joints_plus);
        Eigen::MatrixXd J_current = computeJacobian(joints);
        
        // dJ/dq_i * dq_i/dt
        J_dot += (J_plus - J_current) / eps * joint_velocities[i];
    }
    
    return J_dot;
}

CartesianVelocity KinematicModelBase::computeVelocity(
    const std::vector<double>& joints,
    const std::vector<double>& joint_velocities) const {
    
    validateJointSize(joints, getDOF());
    validateJointSize(joint_velocities, getDOF());
    
    Eigen::MatrixXd J = computeJacobian(joints);
    Eigen::Matrix<double, 6, 1> q_dot;
    for (int i = 0; i < getDOF(); ++i) {
        q_dot(i) = joint_velocities[i];
    }
    
    Eigen::Matrix<double, 6, 1> v = J * q_dot;
    
    CartesianVelocity vel;
    vel.linear = v.head<3>();
    vel.angular = v.tail<3>();
    
    return vel;
}

CartesianAcceleration KinematicModelBase::computeAcceleration(
    const std::vector<double>& joints,
    const std::vector<double>& joint_velocities,
    const std::vector<double>& joint_accelerations) const {
    
    validateJointSize(joints, getDOF());
    validateJointSize(joint_velocities, getDOF());
    validateJointSize(joint_accelerations, getDOF());
    
    int n = getDOF();
    Eigen::MatrixXd J = computeJacobian(joints);
    Eigen::MatrixXd J_dot = computeJacobianDot(joints, joint_velocities);
    
    Eigen::Matrix<double, 6, 1> q_dot, q_ddot;
    for (int i = 0; i < n; ++i) {
        q_dot(i) = joint_velocities[i];
        q_ddot(i) = joint_accelerations[i];
    }
    
    // a = J * q_ddot + J_dot * q_dot
    Eigen::Matrix<double, 6, 1> a = J * q_ddot + J_dot * q_dot;
    
    CartesianAcceleration acc;
    acc.linear = a.head<3>();
    acc.angular = a.tail<3>();
    
    return acc;
}

std::vector<double> KinematicModelBase::computeStaticTorque(
    const std::vector<double>& joints,
    const Eigen::Vector3d& gravity,
    const Payload& payload) const {
    
    validateJointSize(joints, getDOF());
    
    // Simple implementation: tau = J^T * F_gravity
    // Total force at TCP = payload_mass * gravity
    double total_mass = payload.mass; // In a full implementation, add link masses
    Eigen::Vector3d F_g = total_mass * gravity;
    Eigen::Vector3d M_g = Eigen::Vector3d::Zero(); // Ignore moments for simple version
    
    Eigen::VectorXd wrench(6);
    wrench << F_g, M_g;
    
    Eigen::MatrixXd J = computeJacobian(joints);
    Eigen::VectorXd tau = J.transpose() * wrench;
    
    std::vector<double> result(getDOF());
    for (int i = 0; i < getDOF(); ++i) {
        result[i] = tau(i);
    }
    
    return result;
}

bool KinematicModelBase::isNearSingularity(const std::vector<double>& joints,
                                           double threshold) const {
    return getMinSingularValue(joints) < threshold;
}

double KinematicModelBase::getMinSingularValue(const std::vector<double>& joints) const {
    Eigen::MatrixXd J = computeJacobian(joints);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    return svd.singularValues()(svd.singularValues().size() - 1);
}

double KinematicModelBase::computeManipulability(const std::vector<double>& joints) const {
    Eigen::MatrixXd J = computeJacobian(joints);
    Eigen::MatrixXd JJt = J * J.transpose();
    return std::sqrt(JJt.determinant());
}

std::pair<int, std::map<std::string, std::string>> KinematicModelBase::identifyConfiguration(
    const std::vector<double>& joints) const {
    
    // Default implementation - should be overridden for specific robot types
    std::map<std::string, std::string> flags;
    flags["arm"] = "UNKNOWN";
    flags["elbow"] = "UNKNOWN";
    flags["wrist"] = "UNKNOWN";
    
    return {0, flags};
}

bool KinematicModelBase::validateJoints(const std::vector<double>& joints,
                                        const JointLimits& limits) const {
    if (!limits.isValid()) return true;  // No limits defined
    
    if (static_cast<int>(joints.size()) != static_cast<int>(limits.size())) {
        return false;
    }
    
    for (size_t i = 0; i < joints.size(); ++i) {
        if (joints[i] < limits.q_min[i] || joints[i] > limits.q_max[i]) {
            return false;
        }
    }
    
    return true;
}

std::vector<std::pair<int, std::string>> KinematicModelBase::getJointViolations(
    const std::vector<double>& joints,
    const JointLimits& limits) const {
    
    std::vector<std::pair<int, std::string>> violations;
    
    if (!limits.isValid()) return violations;
    
    for (size_t i = 0; i < joints.size() && i < limits.size(); ++i) {
        if (joints[i] < limits.q_min[i]) {
            violations.emplace_back(static_cast<int>(i), "LOWER_BOUND");
        } else if (joints[i] > limits.q_max[i]) {
            violations.emplace_back(static_cast<int>(i), "UPPER_BOUND");
        }
    }
    
    return violations;
}

// ============================================================================
// MDHModel Implementation
// ============================================================================

MDHModel::MDHModel(const MDHParams& params)
    : chain_(params.chain) {
}

MDHModel::MDHModel(const std::vector<MDHJoint>& chain)
    : chain_(chain) {
}

ModelCapabilities MDHModel::getCapabilities() const {
    ModelCapabilities caps;
    caps.model_id = "";
    caps.structure_type = "SERIAL";
    
    // MDH models typically have analytical FK
    caps.kinematics.has_closed_form_fk = true;
    caps.kinematics.has_closed_form_ik = (getDOF() == 6);  // 6-DOF may have analytical IK
    caps.kinematics.requires_numerical_ik = (getDOF() != 6);
    caps.kinematics.ik_solutions_count = "MULTIPLE";
    caps.kinematics.supports_spherical_wrist = false;
    
    caps.differential.jacobian_solver = "ANALYTIC";
    caps.differential.is_redundant = (getDOF() > 6);
    caps.differential.singularity_detection = true;
    
    caps.dynamics.supports_inverse_dynamics = true;
    caps.dynamics.supports_gravity_comp = true;
    caps.dynamics.supports_inertia_matrix = false;
    caps.dynamics.payload_sensitive = true;
    
    caps.performance_hints.fk_compute_cost = "LOW";
    caps.performance_hints.ik_compute_cost = "MEDIUM";
    caps.performance_hints.recommended_frequency_hz = 1000;
    
    return caps;
}

Eigen::Matrix4d MDHModel::computeMDHTransform(const MDHJoint& joint, double theta) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(joint.alpha);
    double sa = std::sin(joint.alpha);
    
    // Craig's MDH: RotX(alpha) * TransX(a) * RotZ(theta) * TransZ(d)
    Eigen::Matrix4d T;
    T << ca*ct,          -ca*st,         sa,     joint.a,
         sa*ct + ca*st,  -sa*st + ca*ct, -sa*ct, -joint.d * sa * ct,
         -sa*st,         sa*ct + ca*st,  ca,     joint.d * ca,
         0,              0,              0,      1;
    
    // Simplified standard form
    T << ct,      -st,     0,      joint.a,
         st*ca,   ct*ca,   -sa,    -joint.d * sa,
         st*sa,   ct*sa,   ca,     joint.d * ca,
         0,       0,       0,      1;
    
    return T;
}

Eigen::Matrix4d MDHModel::computeMDHTransformDerivative(const MDHJoint& joint, double theta) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(joint.alpha);
    double sa = std::sin(joint.alpha);
    
    // dT/d(theta)
    Eigen::Matrix4d dT;
    dT << -st,     -ct,     0,      0,
          ct*ca,   -st*ca,  0,      0,
          ct*sa,   -st*sa,  0,      0,
          0,       0,       0,      0;
    
    return dT;
}

Eigen::Matrix4d MDHModel::computeFK(const std::vector<double>& joints) const {
    validateJointSize(joints, getDOF());
    
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    for (size_t i = 0; i < chain_.size(); ++i) {
        double theta = joints[i] + chain_[i].theta_offset;
        if (chain_[i].type == JointType::REVOLUTE) {
            T = T * computeMDHTransform(chain_[i], theta);
        } else {
            // Prismatic: d is variable, theta is fixed
            MDHJoint temp = chain_[i];
            temp.d = joints[i] + chain_[i].d;
            T = T * computeMDHTransform(temp, chain_[i].theta_offset);
        }
    }
    
    return T;
}

Eigen::Matrix4d MDHModel::computeJointTransform(
    const std::vector<double>& joints, int joint_index) const {
    
    validateJointSize(joints, getDOF());
    
    if (joint_index < 0 || joint_index >= getDOF()) {
        throw std::out_of_range("Joint index out of range");
    }
    
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    for (int i = 0; i <= joint_index; ++i) {
        double theta = joints[i] + chain_[i].theta_offset;
        T = T * computeMDHTransform(chain_[i], theta);
    }
    
    return T;
}

Eigen::Vector3d MDHModel::computeJointAxis(const std::vector<double>& joints, 
                                            int joint_idx) const {
    // Z-axis of joint frame in base coordinates
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    for (int i = 0; i < joint_idx; ++i) {
        double theta = joints[i] + chain_[i].theta_offset;
        T = T * computeMDHTransform(chain_[i], theta);
    }
    
    // Z-axis is the third column of rotation matrix
    return T.block<3, 1>(0, 2);
}

Eigen::Vector3d MDHModel::computeJointOrigin(const std::vector<double>& joints,
                                              int joint_idx) const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    for (int i = 0; i < joint_idx; ++i) {
        double theta = joints[i] + chain_[i].theta_offset;
        T = T * computeMDHTransform(chain_[i], theta);
    }
    
    return T.block<3, 1>(0, 3);
}

Eigen::MatrixXd MDHModel::computeJacobian(const std::vector<double>& joints) const {
    validateJointSize(joints, getDOF());
    
    int n = getDOF();
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, n);
    
    // Compute end-effector position
    Eigen::Matrix4d T_ee = computeFK(joints);
    Eigen::Vector3d p_ee = T_ee.block<3, 1>(0, 3);
    
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d z_i = computeJointAxis(joints, i);
        Eigen::Vector3d p_i = computeJointOrigin(joints, i);
        
        if (chain_[i].type == JointType::REVOLUTE) {
            // Revolute joint
            J.block<3, 1>(0, i) = z_i.cross(p_ee - p_i);  // Linear velocity
            J.block<3, 1>(3, i) = z_i;                      // Angular velocity
        } else {
            // Prismatic joint
            J.block<3, 1>(0, i) = z_i;                      // Linear velocity
            J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();  // No angular velocity
        }
    }
    
    return J;
}

std::pair<int, std::map<std::string, std::string>> MDHModel::identifyConfiguration(
    const std::vector<double>& joints) const {
    
    validateJointSize(joints, getDOF());
    
    std::map<std::string, std::string> flags;
    int config_id = 0;
    
    // Basic implementation for a typical 6-DOF robot
    if (getDOF() >= 3) {
        // Shoulder: front/back
        if (joints[0] >= 0) {
            flags["shoulder"] = "FRONT";
        } else {
            flags["shoulder"] = "BACK";
            config_id |= 1;
        }
        
        // Elbow: up/down
        if (joints[2] >= 0) {
            flags["elbow"] = "UP";
        } else {
            flags["elbow"] = "DOWN";
            config_id |= 2;
        }
    }
    
    if (getDOF() >= 5) {
        // Wrist: flip/no-flip
        if (joints[4] >= 0) {
            flags["wrist"] = "NONFLIP";
        } else {
            flags["wrist"] = "FLIP";
            config_id |= 4;
        }
    }
    
    return {config_id, flags};
}

void MDHModel::applyCalibration(const std::vector<MDHCalibrationCorrection>& corrections) {
    for (const auto& corr : corrections) {
        if (corr.joint_index >= 0 && corr.joint_index < static_cast<int>(chain_.size())) {
            chain_[corr.joint_index].a += corr.delta_a;
            chain_[corr.joint_index].alpha += corr.delta_alpha;
            chain_[corr.joint_index].d += corr.delta_d;
            chain_[corr.joint_index].theta_offset += corr.delta_theta;
        }
    }
}

// ============================================================================
// POEModel Implementation
// ============================================================================

POEModel::POEModel(const POEParams& params)
    : screw_axes_(params.screw_axes),
      m_home_(params.m_home) {
}

ModelCapabilities POEModel::getCapabilities() const {
    ModelCapabilities caps;
    caps.model_id = "";
    caps.structure_type = "SERIAL";
    
    caps.kinematics.has_closed_form_fk = true;
    caps.kinematics.has_closed_form_ik = false;
    caps.kinematics.requires_numerical_ik = true;
    caps.kinematics.ik_solutions_count = "MULTIPLE";
    caps.kinematics.supports_spherical_wrist = false;
    
    caps.differential.jacobian_solver = "ANALYTIC";
    caps.differential.is_redundant = (getDOF() > 6);
    caps.differential.singularity_detection = true;
    
    caps.dynamics.supports_inverse_dynamics = true;
    caps.dynamics.supports_gravity_comp = true;
    
    caps.performance_hints.fk_compute_cost = "LOW";
    caps.performance_hints.ik_compute_cost = "HIGH";
    
    return caps;
}

Eigen::Matrix4d POEModel::matrixExponential(const POEScrewAxis& screw, double theta) {
    Eigen::Vector3d w = screw.angular();
    Eigen::Vector3d v = screw.linear();
    
    double norm_w = w.norm();
    
    if (norm_w < 1e-10) {
        // Pure translation
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 1>(0, 3) = v * theta;
        return T;
    }
    
    // Rotation axis (normalized)
    Eigen::Vector3d w_hat = w / norm_w;
    double theta_norm = theta * norm_w;
    
    // Rodrigues' formula for rotation
    Eigen::Matrix3d R;
    double ct = std::cos(theta_norm);
    double st = std::sin(theta_norm);
    Eigen::Matrix3d w_skew;
    w_skew << 0, -w_hat.z(), w_hat.y(),
              w_hat.z(), 0, -w_hat.x(),
              -w_hat.y(), w_hat.x(), 0;
    
    R = Eigen::Matrix3d::Identity() * ct + 
        (1 - ct) * w_hat * w_hat.transpose() + 
        st * w_skew;
    
    // Translation part
    Eigen::Vector3d p = (Eigen::Matrix3d::Identity() - R) * 
                        (w_hat.cross(v) / (norm_w * norm_w)) +
                        w_hat * (w_hat.dot(v)) * theta_norm / norm_w;
    
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;
    
    return T;
}

Eigen::Matrix<double, 6, 6> POEModel::adjoint(const Eigen::Matrix4d& T) {
    Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
    
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d p = T.block<3, 1>(0, 3);
    
    // Skew-symmetric of p
    Eigen::Matrix3d p_skew;
    p_skew << 0, -p.z(), p.y(),
              p.z(), 0, -p.x(),
              -p.y(), p.x(), 0;
    
    Ad.block<3, 3>(0, 0) = R;
    Ad.block<3, 3>(0, 3) = p_skew * R;
    Ad.block<3, 3>(3, 3) = R;
    
    return Ad;
}

Eigen::Matrix4d POEModel::computeFK(const std::vector<double>& joints) const {
    validateJointSize(joints, getDOF());
    
    Eigen::Matrix4d T = m_home_;
    
    // Product of exponentials: T = e^[S1]q1 * ... * e^[Sn]qn * M
    for (size_t i = 0; i < screw_axes_.size(); ++i) {
        T = matrixExponential(screw_axes_[i], joints[i]) * T;
    }
    
    return T;
}

Eigen::Matrix4d POEModel::computeJointTransform(
    const std::vector<double>& joints, int joint_index) const {
    
    validateJointSize(joints, getDOF());
    
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    for (int i = 0; i <= joint_index; ++i) {
        T = T * matrixExponential(screw_axes_[i], joints[i]);
    }
    
    return T;
}

Eigen::MatrixXd POEModel::computeJacobian(const std::vector<double>& joints) const {
    validateJointSize(joints, getDOF());
    
    int n = getDOF();
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, n);
    
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    for (int i = 0; i < n; ++i) {
        // Adjoint of transform up to joint i
        Eigen::Matrix<double, 6, 6> Ad = adjoint(T);
        
        // Transform screw axis to base frame
        Eigen::Matrix<double, 6, 1> S = screw_axes_[i].toEigen();
        J.col(i) = Ad * S;
        
        // Update transform
        T = T * matrixExponential(screw_axes_[i], joints[i]);
    }
    
    return J;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::shared_ptr<KinematicModelBase> createKinematicModel(const KinematicParams& params) {
    switch (params.model_type) {
        case ModelType::MDH:
        case ModelType::SDH:
            return std::make_shared<MDHModel>(params.getMDH());
        case ModelType::POE:
            return std::make_shared<POEModel>(params.getPOE());
        case ModelType::URDF:
            return std::make_shared<URDFModel>(params.getURDF());
        case ModelType::DELTA:
            return std::make_shared<DeltaModel>(params.getDelta());
        default:
            throw std::runtime_error("Unknown model type");
    }
}

std::shared_ptr<KinematicModelBase> createKinematicModel(const KinematicModel& model) {
    return createKinematicModel(model.params);
}

} // namespace remani::core
