/**
 * @file py_managers.cpp
 * @brief pybind11 bindings for ModelManager and InstanceManager
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "remani/core/model_manager.hpp"

namespace py = pybind11;
namespace remani::core {

// Exception translator
void translateException(const RemaniException& e) {
    std::string error_type = e.code();
    if (error_type == "MODEL_NOT_FOUND") {
        PyErr_SetString(PyExc_KeyError, e.what());
    } else if (error_type == "INSTANCE_NOT_FOUND") {
        PyErr_SetString(PyExc_KeyError, e.what());
    } else if (error_type == "DUPLICATE_MODEL" || error_type == "DUPLICATE_INSTANCE") {
        PyErr_SetString(PyExc_ValueError, e.what());
    } else if (error_type == "INVALID_PARAMETER") {
        PyErr_SetString(PyExc_ValueError, e.what());
    } else if (error_type == "INSTANCE_IN_MOTION") {
        PyErr_SetString(PyExc_RuntimeError, e.what());
    } else {
        PyErr_SetString(PyExc_RuntimeError, e.what());
    }
}

void bindManagers(py::module& m) {
    // Register exception translator
    py::register_exception_translator([](std::exception_ptr p) {
        try {
            if (p) std::rethrow_exception(p);
        } catch (const RemaniException& e) {
            translateException(e);
        }
    });
    
    // ModelManager
    py::class_<ModelManager>(m, "ModelManager")
        .def(py::init<>())
        .def("register_model", 
             py::overload_cast<const KinematicModel&>(&ModelManager::registerModel),
             py::arg("model"),
             py::return_value_policy::reference_internal)
        .def("register_model",
             py::overload_cast<const std::string&, const std::string&, ModelType,
                              const KinematicParams&, const JointLimits&>(
                                  &ModelManager::registerModel),
             py::arg("model_id"), py::arg("name"), py::arg("model_type"),
             py::arg("params"), py::arg("limits") = JointLimits(),
             py::return_value_policy::reference_internal)
        .def("get_model", &ModelManager::getModel, py::arg("model_id"),
             py::return_value_policy::reference_internal)
        .def("has_model", &ModelManager::hasModel, py::arg("model_id"))
        .def("get_model_ids", &ModelManager::getModelIds)
        .def("get_model_count", &ModelManager::getModelCount)
        .def("update_model_status", &ModelManager::updateModelStatus,
             py::arg("model_id"), py::arg("status"))
        .def("delete_model", &ModelManager::deleteModel, py::arg("model_id"))
        .def("clear", &ModelManager::clear)
        .def("get_model_capabilities", &ModelManager::getModelCapabilities,
             py::arg("model_id"))
        .def("get_kinematic_solver", &ModelManager::getKinematicSolver,
             py::arg("model_id"))
        .def("__len__", &ModelManager::getModelCount)
        .def("__contains__", &ModelManager::hasModel)
        .def("__getitem__", &ModelManager::getModel,
             py::return_value_policy::reference_internal);
    
    // InstanceManager
    py::class_<InstanceManager>(m, "InstanceManager")
        .def(py::init<>())
        .def("configure_instance",
             py::overload_cast<const RuntimeInstance&, const ModelManager&>(
                 &InstanceManager::configureInstance),
             py::arg("instance"), py::arg("model_manager"),
             py::return_value_policy::reference_internal)
        .def("configure_instance",
             py::overload_cast<const std::string&, const std::string&, const BaseFrame&,
                              MountingType, double, const TCPDefinition&,
                              const ModelManager&>(&InstanceManager::configureInstance),
             py::arg("instance_id"), py::arg("model_id"),
             py::arg("base_frame") = BaseFrame(),
             py::arg("mounting_type") = MountingType::FLOOR,
             py::arg("mounting_angle") = 0.0,
             py::arg("default_tcp") = TCPDefinition(),
             py::arg("model_manager") = ModelManager(),
             py::return_value_policy::reference_internal)
        .def("get_instance", &InstanceManager::getInstance, py::arg("instance_id"),
             py::return_value_policy::reference_internal)
        .def("has_instance", &InstanceManager::hasInstance, py::arg("instance_id"))
        .def("get_instance_ids", &InstanceManager::getInstanceIds)
        .def("get_instance_count", &InstanceManager::getInstanceCount)
        .def("update_instance_status", &InstanceManager::updateInstanceStatus,
             py::arg("instance_id"), py::arg("status"))
        .def("update_instance_joints", &InstanceManager::updateInstanceJoints,
             py::arg("instance_id"), py::arg("joints"))
        .def("delete_instance", &InstanceManager::deleteInstance, py::arg("instance_id"))
        .def("clear", &InstanceManager::clear)
        .def("apply_calibration", &InstanceManager::applyCalibration,
             py::arg("instance_id"), py::arg("calibration"), py::arg("model_manager"))
        .def("get_calibration_data", &InstanceManager::getCalibrationData,
             py::arg("instance_id"))
        .def("get_instances_by_model", &InstanceManager::getInstancesByModel,
             py::arg("model_id"))
        .def("get_kinematic_solver", &InstanceManager::getKinematicSolver,
             py::arg("instance_id"), py::arg("model_manager"))
        .def("__len__", &InstanceManager::getInstanceCount)
        .def("__contains__", &InstanceManager::hasInstance)
        .def("__getitem__", &InstanceManager::getInstance,
             py::return_value_policy::reference_internal);
}

} // namespace remani::core
