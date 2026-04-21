/**
 * @file py_model.cpp
 * @brief pybind11 bindings for model and instance structures
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "remani/core/model.hpp"

namespace py = pybind11;
namespace remani::core {

void bindModel(py::module& m) {
    // ModelStatus enum
    py::enum_<ModelStatus>(m, "ModelStatus")
        .value("ACTIVE", ModelStatus::ACTIVE)
        .value("ERROR", ModelStatus::ERROR)
        .value("CALIBRATING", ModelStatus::CALIBRATING)
        .value("READY", ModelStatus::READY)
        .def("to_string", &modelStatusToString);
    
    // InstanceStatus enum
    py::enum_<InstanceStatus>(m, "InstanceStatus")
        .value("READY", InstanceStatus::READY)
        .value("MOVING", InstanceStatus::MOVING)
        .value("ERROR", InstanceStatus::ERROR)
        .value("STOPPED", InstanceStatus::STOPPED)
        .def("to_string", &instanceStatusToString);
    
    // MountingType enum
    py::enum_<MountingType>(m, "MountingType")
        .value("FLOOR", MountingType::FLOOR)
        .value("CEILING", MountingType::CEILING)
        .value("WALL", MountingType::WALL)
        .value("ANGLED", MountingType::ANGLED)
        .def("to_string", &mountingTypeToString);
    
    // TCPDefinition
    py::class_<TCPDefinition>(m, "TCPDefinition")
        .def(py::init<>())
        .def(py::init<const Eigen::Vector3d&, const Eigen::Quaterniond&>(),
             py::arg("position"), py::arg("rotation"))
        .def_readwrite("position", &TCPDefinition::position)
        .def_readwrite("rotation", &TCPDefinition::rotation)
        .def("to_matrix", &TCPDefinition::toMatrix)
        .def_static("from_pos_quat", &TCPDefinition::fromPosQuat);
    
    // BaseFrame
    py::class_<BaseFrame>(m, "BaseFrame")
        .def(py::init<>())
        .def_readwrite("position", &BaseFrame::position)
        .def_readwrite("rotation", &BaseFrame::rotation)
        .def("to_matrix", &BaseFrame::toMatrix)
        .def_static("from_pos_rot", &BaseFrame::fromPosRot);
    
    // KinematicModel
    py::class_<KinematicModel>(m, "KinematicModel")
        .def(py::init<>())
        .def_readwrite("model_id", &KinematicModel::model_id)
        .def_readwrite("name", &KinematicModel::name)
        .def_readwrite("model_type", &KinematicModel::model_type)
        .def_readwrite("dof", &KinematicModel::dof)
        .def_readwrite("status", &KinematicModel::status)
        .def_readwrite("params", &KinematicModel::params)
        .def_readwrite("limits", &KinematicModel::limits)
        .def_readwrite("created_at", &KinematicModel::created_at)
        .def_readwrite("last_updated", &KinematicModel::last_updated)
        .def_readwrite("solver_engine", &KinematicModel::solver_engine)
        .def_readwrite("total_mass", &KinematicModel::total_mass)
        .def("is_valid", &KinematicModel::isValid)
        .def_static("get_current_timestamp", &KinematicModel::getCurrentTimestamp);
    
    // RuntimeInstance
    py::class_<RuntimeInstance>(m, "RuntimeInstance")
        .def(py::init<>())
        .def_readwrite("instance_id", &RuntimeInstance::instance_id)
        .def_readwrite("model_id", &RuntimeInstance::model_id)
        .def_readwrite("base_frame", &RuntimeInstance::base_frame)
        .def_readwrite("mounting_type", &RuntimeInstance::mounting_type)
        .def_readwrite("mounting_angle", &RuntimeInstance::mounting_angle)
        .def_readwrite("default_tcp", &RuntimeInstance::default_tcp)
        .def_readwrite("status", &RuntimeInstance::status)
        .def_readwrite("current_joints", &RuntimeInstance::current_joints)
        .def_readwrite("joint_offsets", &RuntimeInstance::joint_offsets)
        .def_readwrite("gravity_vector", &RuntimeInstance::gravity_vector)
        .def_readwrite("created_at", &RuntimeInstance::created_at)
        .def_readwrite("last_updated", &RuntimeInstance::last_updated)
        .def_readwrite("config_hash", &RuntimeInstance::config_hash)
        .def("compute_gravity_vector", &RuntimeInstance::computeGravityVector)
        .def("is_valid", &RuntimeInstance::isValid)
        .def("can_modify", &RuntimeInstance::canModify);
    
    // KinematicCapabilities
    py::class_<KinematicCapabilities>(m, "KinematicCapabilities")
        .def(py::init<>())
        .def_readwrite("has_closed_form_fk", &KinematicCapabilities::has_closed_form_fk)
        .def_readwrite("has_closed_form_ik", &KinematicCapabilities::has_closed_form_ik)
        .def_readwrite("requires_numerical_ik", &KinematicCapabilities::requires_numerical_ik)
        .def_readwrite("ik_solutions_count", &KinematicCapabilities::ik_solutions_count)
        .def_readwrite("supports_spherical_wrist", &KinematicCapabilities::supports_spherical_wrist);
    
    // DifferentialCapabilities
    py::class_<DifferentialCapabilities>(m, "DifferentialCapabilities")
        .def(py::init<>())
        .def_readwrite("jacobian_solver", &DifferentialCapabilities::jacobian_solver)
        .def_readwrite("is_redundant", &DifferentialCapabilities::is_redundant)
        .def_readwrite("singularity_detection", &DifferentialCapabilities::singularity_detection);
    
    // DynamicsCapabilities
    py::class_<DynamicsCapabilities>(m, "DynamicsCapabilities")
        .def(py::init<>())
        .def_readwrite("supports_inverse_dynamics", &DynamicsCapabilities::supports_inverse_dynamics)
        .def_readwrite("supports_gravity_comp", &DynamicsCapabilities::supports_gravity_comp)
        .def_readwrite("supports_inertia_matrix", &DynamicsCapabilities::supports_inertia_matrix)
        .def_readwrite("payload_sensitive", &DynamicsCapabilities::payload_sensitive);
    
    // PerformanceHints
    py::class_<PerformanceHints>(m, "PerformanceHints")
        .def(py::init<>())
        .def_readwrite("fk_compute_cost", &PerformanceHints::fk_compute_cost)
        .def_readwrite("ik_compute_cost", &PerformanceHints::ik_compute_cost)
        .def_readwrite("recommended_frequency_hz", &PerformanceHints::recommended_frequency_hz);
    
    // ModelCapabilities
    py::class_<ModelCapabilities>(m, "ModelCapabilities")
        .def(py::init<>())
        .def_readwrite("model_id", &ModelCapabilities::model_id)
        .def_readwrite("structure_type", &ModelCapabilities::structure_type)
        .def_readwrite("kinematics", &ModelCapabilities::kinematics)
        .def_readwrite("differential", &ModelCapabilities::differential)
        .def_readwrite("dynamics", &ModelCapabilities::dynamics)
        .def_readwrite("performance_hints", &ModelCapabilities::performance_hints);
    
    // Payload
    py::class_<Payload>(m, "Payload")
        .def(py::init<>())
        .def(py::init<double, const Eigen::Vector3d&>(),
             py::arg("mass"), py::arg("cog"))
        .def(py::init<double, const Eigen::Vector3d&, const Eigen::Matrix3d&>(),
             py::arg("mass"), py::arg("cog"), py::arg("inertia"))
        .def_readwrite("mass", &Payload::mass)
        .def_readwrite("cog", &Payload::cog)
        .def_readwrite("inertia", &Payload::inertia)
        .def_readwrite("has_inertia", &Payload::has_inertia);
    
    // EndEffectorPose
    py::class_<EndEffectorPose>(m, "EndEffectorPose")
        .def(py::init<>())
        .def_readwrite("position", &EndEffectorPose::position)
        .def_readwrite("quaternion", &EndEffectorPose::quaternion)
        .def_readwrite("euler_zyx", &EndEffectorPose::euler_zyx)
        .def_readwrite("matrix", &EndEffectorPose::matrix)
        .def("from_matrix", &EndEffectorPose::fromMatrix);
    
    // CartesianVelocity
    py::class_<CartesianVelocity>(m, "CartesianVelocity")
        .def(py::init<>())
        .def_readwrite("linear", &CartesianVelocity::linear)
        .def_readwrite("angular", &CartesianVelocity::angular)
        .def("linear_magnitude", &CartesianVelocity::linear_magnitude)
        .def("angular_magnitude", &CartesianVelocity::angular_magnitude);
    
    // CartesianAcceleration
    py::class_<CartesianAcceleration>(m, "CartesianAcceleration")
        .def(py::init<>())
        .def_readwrite("linear", &CartesianAcceleration::linear)
        .def_readwrite("angular", &CartesianAcceleration::angular)
        .def("linear_magnitude", &CartesianAcceleration::linear_magnitude)
        .def("angular_magnitude", &CartesianAcceleration::angular_magnitude);
}

} // namespace remani::core
