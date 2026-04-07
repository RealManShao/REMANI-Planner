/**
 * @file math_utils.cpp
 * @brief Mathematical utilities for kinematics
 */

#include "remani/core/kinematic_model.hpp"
#include <Eigen/Geometry>

namespace remani::core {

/**
 * @brief Compute transformation matrix from position and Euler angles
 */
Eigen::Matrix4d poseToMatrix(const Eigen::Vector3d& pos, 
                              const Eigen::Vector3d& euler_zyx) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) = pos;
    
    // ZYX Euler angles to rotation matrix
    Eigen::AngleAxisd yawAngle(euler_zyx.z(), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(euler_zyx.y(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(euler_zyx.x(), Eigen::Vector3d::UnitX());
    
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    
    return T;
}

/**
 * @brief Compute transformation matrix from position and quaternion
 */
Eigen::Matrix4d poseQuatToMatrix(const Eigen::Vector3d& pos,
                                  const Eigen::Vector4d& quat) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) = pos;
    
    // Quaternion [qx, qy, qz, qw] to rotation matrix
    Eigen::Quaterniond q(quat.w(), quat.x(), quat.y(), quat.z());
    q.normalize();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    
    return T;
}

/**
 * @brief Multiply two transformation matrices
 */
Eigen::Matrix4d multiplyTransforms(const Eigen::Matrix4d& A, const Eigen::Matrix4d& B) {
    return A * B;
}

/**
 * @brief Invert transformation matrix
 */
Eigen::Matrix4d invertTransform(const Eigen::Matrix4d& T) {
    Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
    T_inv.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
    T_inv.block<3, 1>(0, 3) = -T_inv.block<3, 3>(0, 0) * T.block<3, 1>(0, 3);
    return T_inv;
}

/**
 * @brief Compute relative transform from A to B
 */
Eigen::Matrix4d relativeTransform(const Eigen::Matrix4d& A, const Eigen::Matrix4d& B) {
    return invertTransform(A) * B;
}

/**
 * @brief Extract position from transformation matrix
 */
Eigen::Vector3d getPosition(const Eigen::Matrix4d& T) {
    return T.block<3, 1>(0, 3);
}

/**
 * @brief Extract rotation as quaternion from transformation matrix
 */
Eigen::Vector4d getQuaternion(const Eigen::Matrix4d& T) {
    Eigen::Quaterniond q(T.block<3, 3>(0, 0));
    q.normalize();
    return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
}

/**
 * @brief Extract rotation as Euler angles (ZYX) from transformation matrix
 */
Eigen::Vector3d getEulerAnglesZYX(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);  // [yaw, pitch, roll]
    return Eigen::Vector3d(euler.z(), euler.y(), euler.x());  // [roll, pitch, yaw]
}

} // namespace remani::core
