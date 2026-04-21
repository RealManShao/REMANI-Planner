#include "remani/core/mm_config_adapter.hpp"
#include <Eigen/Geometry>

namespace remani::core {

KinematicModel MMConfigAdapter::fromYAML(const YAML::Node& yaml, const std::string& model_id) {
    KinematicModel model;
    model.model_id = model_id;
    
    if (!yaml["mm"]) {
        throw std::runtime_error("YAML does not contain 'mm' node");
    }
    
    auto mm = yaml["mm"];
    model.dof = mm["manipulator_dof"].as<int>();
    model.name = mm["name"] ? mm["name"].as<std::string>() : model_id;
    
    bool use_fast_armer = mm["use_fast_armer"] ? mm["use_fast_armer"].as<bool>() : true;
    std::vector<double> config;
    if (mm["manipulator_config"]) {
        config = mm["manipulator_config"].as<std::vector<double>>();
    }
    
    model.params = convertToMDH(config, use_fast_armer);
    model.limits = convertLimits(mm, model.dof);
    
    return model;
}

MDHParams MMConfigAdapter::convertToMDH(const std::vector<double>& config, bool use_fast_armer) {
    std::vector<MDHJoint> chain;
    
    if (use_fast_armer) {
        // FastArmer MDH Parameters mapping (approximate based on MMConfig::getAJointTran)
        // linkLength = manipulator_config_(joint_num)
        
        // joint 0
        chain.emplace_back(0, JointType::REVOLUTE, 0.0, 0.0, config[0], 0.0);
        // joint 1
        chain.emplace_back(1, JointType::REVOLUTE, 0.0, -M_PI_2, 0.0, 0.0);
        // joint 2
        chain.emplace_back(2, JointType::REVOLUTE, config[1], 0.0, 0.0, -M_PI_2);
        // joint 3
        chain.emplace_back(3, JointType::REVOLUTE, 0.0, M_PI_2, config[3], -M_PI);
        // joint 4
        chain.emplace_back(4, JointType::REVOLUTE, 0.0, M_PI_2, 0.0, M_PI_2);
        // joint 5
        chain.emplace_back(5, JointType::REVOLUTE, 0.0, -M_PI_2, config[5], 0.0);
        
    } else {
        // UR5 MDH Parameters
        chain.emplace_back(0, JointType::REVOLUTE, 0.0, 0.0, config[0], 0.0);
        chain.emplace_back(1, JointType::REVOLUTE, 0.0, M_PI_2, 0.0, 0.0);
        chain.emplace_back(2, JointType::REVOLUTE, config[1], 0.0, 0.0, 0.0);
        chain.emplace_back(3, JointType::REVOLUTE, config[2], 0.0, config[3], 0.0);
        chain.emplace_back(4, JointType::REVOLUTE, 0.0, M_PI_2, 0.0, 0.0);
        chain.emplace_back(5, JointType::REVOLUTE, 0.0, -M_PI_2, config[4], 0.0);
    }
    
    return MDHParams(chain);
}

JointLimits MMConfigAdapter::convertLimits(const YAML::Node& mm, int dof) {
    JointLimits limits;
    
    if (mm["manipulator_min_pos"]) {
        limits.q_min = mm["manipulator_min_pos"].as<std::vector<double>>();
    }
    
    if (mm["manipulator_max_pos"]) {
        limits.q_max = mm["manipulator_max_pos"].as<std::vector<double>>();
    }
    
    // Resize to match DOF if necessary
    if (limits.q_min.size() > (size_t)dof) limits.q_min.resize(dof);
    if (limits.q_max.size() > (size_t)dof) limits.q_max.resize(dof);
    
    return limits;
}

} // namespace remani::core
