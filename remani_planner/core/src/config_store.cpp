/**
 * @file config_store.cpp
 * @brief Configuration persistence (placeholder)
 */

#include "remani/core/model_manager.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

namespace remani::core {

using json = nlohmann::json;

/**
 * @brief Save models to JSON file
 */
void saveModels(const ModelManager& manager, const std::string& filepath) {
    json j;
    
    for (const auto& id : manager.getModelIds()) {
        const auto& model = manager.getModel(id);
        // TODO: Serialize model to JSON
    }
    
    std::ofstream file(filepath);
    file << j.dump(2);
}

/**
 * @brief Load models from JSON file
 */
void loadModels(ModelManager& manager, const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return;
    
    json j;
    file >> j;
    
    // TODO: Deserialize models from JSON
}

} // namespace remani::core
