/**
 * @file mm_config_adapter.hpp
 * @brief Adapter to convert legacy YAML configuration to new KinematicModel
 */

#ifndef REMANI_CORE_MM_CONFIG_ADAPTER_HPP
#define REMANI_CORE_MM_CONFIG_ADAPTER_HPP

#include "remani/core/kinematic_params.hpp"
#include "remani/core/model.hpp"
#include "remani/core/kinematic_model.hpp"
#include <yaml-cpp/yaml.h>

namespace remani::core {

class MMConfigAdapter {
public:
    /**
     * @brief Create a KinematicModel from legacy YAML configuration
     * @param yaml YAML node containing "mm" configuration
     * @param model_id Identifier for the model
     * @return Shared pointer to created model
     */
    static KinematicModel fromYAML(const YAML::Node& yaml, const std::string& model_id);
    
    /**
     * @brief Convert manipulator_config and use_fast_armer to MDH parameters
     * @param config Vector of link parameters
     * @param use_fast_armer Whether to use FastArmer kinematics
     * @return MDH parameters
     */
    static MDHParams convertToMDH(const std::vector<double>& config, bool use_fast_armer);
    
    /**
     * @brief Convert joint limits from YAML to JointLimits struct
     * @param yaml YAML node
     * @param dof Degrees of freedom
     * @return JointLimits struct
     */
    static JointLimits convertLimits(const YAML::Node& yaml, int dof);
};

} // namespace remani::core

#endif // REMANI_CORE_MM_CONFIG_ADAPTER_HPP
