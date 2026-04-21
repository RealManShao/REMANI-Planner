/**
 * @file py_solvers.cpp
 * @brief pybind11 bindings for kinematic solvers and main module
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "remani/core/kinematic_model.hpp"
#include "remani/core/model_manager.hpp"

namespace py = pybind11;
namespace remani::core {

void bindSolvers(py::module& m) {
    // KinematicModelBase (abstract base class)
    py::class_<KinematicModelBase, std::shared_ptr<KinematicModelBase>>(m, "KinematicModelBase")
        .def("get_model_type", &KinematicModelBase::getModelType)
        .def("get_model_type_string", &KinematicModelBase::getModelTypeString)
        .def("get_dof", &KinematicModelBase::getDOF)
        .def("get_capabilities", &KinematicModelBase::getCapabilities)
        .def("compute_fk", &KinematicModelBase::computeFK, py::arg("joints"))
        .def("compute_fk_with_tcp", &KinematicModelBase::computeFKWithTCP,
             py::arg("joints"), py::arg("tcp"))
        .def("compute_joint_transform", &KinematicModelBase::computeJointTransform,
             py::arg("joints"), py::arg("joint_index"))
        .def("compute_jacobian", &KinematicModelBase::computeJacobian, py::arg("joints"))
        .def("compute_jacobian_with_tcp", &KinematicModelBase::computeJacobianWithTCP,
             py::arg("joints"), py::arg("tcp"))
        .def("compute_jacobian_dot", &KinematicModelBase::computeJacobianDot,
             py::arg("joints"), py::arg("joint_velocities"))
        .def("compute_velocity", &KinematicModelBase::computeVelocity,
             py::arg("joints"), py::arg("joint_velocities"))
        .def("compute_acceleration", &KinematicModelBase::computeAcceleration,
             py::arg("joints"), py::arg("joint_velocities"), py::arg("joint_accelerations"))
        .def("compute_static_torque", &KinematicModelBase::computeStaticTorque,
             py::arg("joints"), py::arg("gravity"), py::arg("payload") = Payload())
        .def("is_near_singularity", &KinematicModelBase::isNearSingularity,
             py::arg("joints"), py::arg("threshold") = 0.01)
        .def("get_min_singular_value", &KinematicModelBase::getMinSingularValue,
             py::arg("joints"))
        .def("compute_manipulability", &KinematicModelBase::computeManipulability,
             py::arg("joints"))
        .def("identify_configuration", &KinematicModelBase::identifyConfiguration,
             py::arg("joints"))
        .def("validate_joints", &KinematicModelBase::validateJoints,
             py::arg("joints"), py::arg("limits"))
        .def("get_joint_violations", &KinematicModelBase::getJointViolations,
             py::arg("joints"), py::arg("limits"));
    
    // MDHModel
    py::class_<MDHModel, KinematicModelBase, std::shared_ptr<MDHModel>>(m, "MDHModel")
        .def(py::init<const MDHParams&>(), py::arg("params"))
        .def(py::init<const std::vector<MDHJoint>&>(), py::arg("chain"))
        .def("get_chain", &MDHModel::getChain, py::return_value_policy::reference_internal)
        .def_static("compute_mdh_transform", &MDHModel::computeMDHTransform,
                    py::arg("joint"), py::arg("theta"))
        .def_static("compute_mdh_transform_derivative", 
                    &MDHModel::computeMDHTransformDerivative,
                    py::arg("joint"), py::arg("theta"))
        .def("apply_calibration", &MDHModel::applyCalibration, py::arg("corrections"));
    
    // POEModel
    py::class_<POEModel, KinematicModelBase, std::shared_ptr<POEModel>>(m, "POEModel")
        .def(py::init<const POEParams&>(), py::arg("params"))
        .def("get_screw_axes", &POEModel::getScrewAxes, py::return_value_policy::reference_internal)
        .def("get_m_home", &POEModel::getMHome)
        .def_static("matrix_exponential", &POEModel::matrixExponential,
                    py::arg("screw"), py::arg("theta"))
        .def_static("adjoint", &POEModel::adjoint, py::arg("T"));
    
    // Factory functions
    m.def("create_kinematic_model",
          py::overload_cast<const KinematicParams&>(&createKinematicModel),
          py::arg("params"));
    
    m.def("create_kinematic_model",
          py::overload_cast<const KinematicModel&>(&createKinematicModel),
          py::arg("model"));
}

// Main module definition
PYBIND11_MODULE(remani_core_py, m) {
    m.doc() = "REMANI Core - Kinematic model library with Python bindings";
    
    // Bind all components
    bindKinematicParams(m);
    bindModel(m);
    bindManagers(m);
    bindSolvers(m);
    
    // Version info
    m.attr("__version__") = "0.1.0";
}

} // namespace remani::core
