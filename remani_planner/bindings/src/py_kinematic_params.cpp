/**
 * @file py_kinematic_params.cpp
 * @brief pybind11 bindings for kinematic parameters
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "remani/core/kinematic_params.hpp"

namespace py = pybind11;
namespace remani::core {

void bindKinematicParams(py::module& m) {
    // JointType enum
    py::enum_<JointType>(m, "JointType")
        .value("REVOLUTE", JointType::REVOLUTE)
        .value("PRISMATIC", JointType::PRISMATIC)
        .def("to_string", &jointTypeToString);
    
    // ModelType enum
    py::enum_<ModelType>(m, "ModelType")
        .value("MDH", ModelType::MDH)
        .value("SDH", ModelType::SDH)
        .value("POE", ModelType::POE)
        .value("URDF", ModelType::URDF)
        .value("DELTA", ModelType::DELTA)
        .def("to_string", &modelTypeToString);
    
    // MDHJoint
    py::class_<MDHJoint>(m, "MDHJoint")
        .def(py::init<>())
        .def(py::init<int, JointType, double, double, double, double>(),
             py::arg("joint_index"), py::arg("type"), py::arg("a"),
             py::arg("alpha"), py::arg("d"), py::arg("theta_offset") = 0.0)
        .def_readwrite("joint_index", &MDHJoint::joint_index)
        .def_readwrite("type", &MDHJoint::type)
        .def_readwrite("a", &MDHJoint::a)
        .def_readwrite("alpha", &MDHJoint::alpha)
        .def_readwrite("d", &MDHJoint::d)
        .def_readwrite("theta_offset", &MDHJoint::theta_offset)
        .def("__repr__", [](const MDHJoint& j) {
            return "<MDHJoint idx=" + std::to_string(j.joint_index) +
                   " a=" + std::to_string(j.a) + " alpha=" + std::to_string(j.alpha) +
                   " d=" + std::to_string(j.d) + ">";
        });
    
    // MDHParams
    py::class_<MDHParams>(m, "MDHParams")
        .def(py::init<>())
        .def(py::init<const std::vector<MDHJoint>&>(), py::arg("chain"))
        .def_readwrite("chain", &MDHParams::chain)
        .def("size", [](const MDHParams& p) { return p.chain.size(); });
    
    // POEScrewAxis
    py::class_<POEScrewAxis>(m, "POEScrewAxis")
        .def(py::init<>())
        .def(py::init<const std::array<double, 6>&>(), py::arg("data"))
        .def_readwrite("data", &POEScrewAxis::data)
        .def("angular", &POEScrewAxis::angular)
        .def("linear", &POEScrewAxis::linear)
        .def("to_eigen", &POEScrewAxis::toEigen);
    
    // POEParams
    py::class_<POEParams>(m, "POEParams")
        .def(py::init<>())
        .def_readwrite("screw_axes", &POEParams::screw_axes)
        .def_readwrite("m_home", &POEParams::m_home);
    
    // URDFParams
    py::class_<URDFParams>(m, "URDFParams")
        .def(py::init<>())
        .def(py::init<const std::string&, const std::string&, const std::string&>(),
             py::arg("urdf_content"), py::arg("base_link"), py::arg("tip_link"))
        .def_readwrite("urdf_content", &URDFParams::urdf_content)
        .def_readwrite("base_link", &URDFParams::base_link)
        .def_readwrite("tip_link", &URDFParams::tip_link);
    
    // DeltaCouplingMode
    py::enum_<DeltaCouplingMode>(m, "DeltaCouplingMode")
        .value("ROTARY", DeltaCouplingMode::ROTARY)
        .value("LINEAR", DeltaCouplingMode::LINEAR);
    
    // DeltaParams
    py::class_<DeltaParams>(m, "DeltaParams")
        .def(py::init<>())
        .def(py::init<double, double, double, double, DeltaCouplingMode>(),
             py::arg("R_base"), py::arg("R_platform"), py::arg("L_arm"),
             py::arg("L_rod"), py::arg("coupling_mode") = DeltaCouplingMode::ROTARY)
        .def_readwrite("R_base", &DeltaParams::R_base)
        .def_readwrite("R_platform", &DeltaParams::R_platform)
        .def_readwrite("L_arm", &DeltaParams::L_arm)
        .def_readwrite("L_rod", &DeltaParams::L_rod)
        .def_readwrite("coupling_mode", &DeltaParams::coupling_mode);
    
    // JointLimits
    py::class_<JointLimits>(m, "JointLimits")
        .def(py::init<>())
        .def_readwrite("q_min", &JointLimits::q_min)
        .def_readwrite("q_max", &JointLimits::q_max)
        .def_readwrite("v_max", &JointLimits::v_max)
        .def_readwrite("a_max", &JointLimits::a_max)
        .def("is_valid", &JointLimits::isValid)
        .def("size", &JointLimits::size);
    
    // KinematicParams
    py::class_<KinematicParams>(m, "KinematicParams")
        .def(py::init<>())
        .def_readwrite("model_type", &KinematicParams::model_type)
        .def_readwrite("dof", &KinematicParams::dof)
        .def_readwrite("params", &KinematicParams::params)
        .def("get_mdh", &KinematicParams::getMDH)
        .def("get_poe", &KinematicParams::getPOE)
        .def("get_urdf", &KinematicParams::getURDF)
        .def("get_delta", &KinematicParams::getDelta)
        .def("set_mdh", &KinematicParams::setMDH)
        .def("set_poe", &KinematicParams::setPOE)
        .def("set_urdf", &KinematicParams::setURDF)
        .def("set_delta", &KinematicParams::setDelta);
    
    // MDHCalibrationCorrection
    py::class_<MDHCalibrationCorrection>(m, "MDHCalibrationCorrection")
        .def(py::init<>())
        .def_readwrite("joint_index", &MDHCalibrationCorrection::joint_index)
        .def_readwrite("delta_a", &MDHCalibrationCorrection::delta_a)
        .def_readwrite("delta_alpha", &MDHCalibrationCorrection::delta_alpha)
        .def_readwrite("delta_d", &MDHCalibrationCorrection::delta_d)
        .def_readwrite("delta_theta", &MDHCalibrationCorrection::delta_theta);
    
    // CalibrationData
    py::class_<CalibrationData>(m, "CalibrationData")
        .def(py::init<>())
        .def_readwrite("calibration_method", &CalibrationData::calibration_method)
        .def_readwrite("joint_offsets", &CalibrationData::joint_offsets)
        .def_readwrite("corrections", &CalibrationData::corrections)
        .def_readwrite("force_update", &CalibrationData::force_update);
}

} // namespace remani::core
