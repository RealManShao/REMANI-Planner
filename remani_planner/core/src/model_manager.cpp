/**
 * @file model_manager.cpp
 * @brief Implementation of ModelManager and InstanceManager
 */

#include "remani/core/model_manager.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <openssl/sha.h>

namespace remani::core {

// ============================================================================
// ModelManager Implementation
// ============================================================================

void ModelManager::validateModel(const KinematicModel& model) const {
    if (model.model_id.empty()) {
        throw InvalidParameterException("Model ID cannot be empty");
    }
    
    if (model.dof <= 0) {
        throw InvalidParameterException("Model DOF must be positive");
    }
    
    // Validate DOF matches parameters
    if (model.params.dof > 0 && model.params.dof != model.dof) {
        throw InvalidParameterException("Model DOF does not match parameters");
    }
    
    // Validate joint limits
    if (model.limits.isValid() && 
        static_cast<int>(model.limits.size()) != model.dof) {
        throw InvalidParameterException("Joint limits size does not match DOF");
    }
}

const KinematicModel& ModelManager::registerModel(const KinematicModel& model) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    validateModel(model);
    
    if (models_.find(model.model_id) != models_.end()) {
        throw DuplicateModelException(model.model_id);
    }
    
    KinematicModel registered = model;
    registered.created_at = KinematicModel::getCurrentTimestamp();
    registered.last_updated = registered.created_at;
    registered.status = ModelStatus::ACTIVE;
    
    models_[model.model_id] = registered;
    
    // Create kinematic solver
    try {
        solvers_[model.model_id] = createKinematicModel(registered);
    } catch (const std::exception& e) {
        models_.erase(model.model_id);
        throw InvalidParameterException(std::string("Failed to create kinematic solver: ") + e.what());
    }
    
    return models_[model.model_id];
}

const KinematicModel& ModelManager::registerModel(
    const std::string& model_id,
    const std::string& name,
    ModelType model_type,
    const KinematicParams& params,
    const JointLimits& limits) {
    
    KinematicModel model;
    model.model_id = model_id;
    model.name = name;
    model.model_type = model_type;
    model.params = params;
    model.dof = params.dof;
    model.limits = limits;
    
    return registerModel(model);
}

const KinematicModel& ModelManager::getModel(const std::string& model_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = models_.find(model_id);
    if (it == models_.end()) {
        throw ModelNotFoundException(model_id);
    }
    
    return it->second;
}

bool ModelManager::hasModel(const std::string& model_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return models_.find(model_id) != models_.end();
}

std::vector<std::string> ModelManager::getModelIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<std::string> ids;
    ids.reserve(models_.size());
    
    for (const auto& pair : models_) {
        ids.push_back(pair.first);
    }
    
    return ids;
}

size_t ModelManager::getModelCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return models_.size();
}

void ModelManager::updateModelStatus(const std::string& model_id, ModelStatus status) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = models_.find(model_id);
    if (it == models_.end()) {
        throw ModelNotFoundException(model_id);
    }
    
    it->second.status = status;
    it->second.last_updated = KinematicModel::getCurrentTimestamp();
}

void ModelManager::deleteModel(const std::string& model_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = models_.find(model_id);
    if (it == models_.end()) {
        throw ModelNotFoundException(model_id);
    }
    
    models_.erase(it);
    solvers_.erase(model_id);
}

void ModelManager::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    models_.clear();
    solvers_.clear();
}

ModelCapabilities ModelManager::getModelCapabilities(const std::string& model_id) const {
    auto solver = getKinematicSolver(model_id);
    auto caps = solver->getCapabilities();
    caps.model_id = model_id;
    return caps;
}

std::shared_ptr<KinematicModelBase> ModelManager::getKinematicSolver(
    const std::string& model_id) const {
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = solvers_.find(model_id);
    if (it == solvers_.end()) {
        throw ModelNotFoundException(model_id);
    }
    
    return it->second;
}

// ============================================================================
// InstanceManager Implementation
// ============================================================================

void InstanceManager::validateInstance(const RuntimeInstance& instance,
                                        const ModelManager& model_manager) const {
    if (instance.instance_id.empty()) {
        throw InvalidParameterException("Instance ID cannot be empty");
    }
    
    if (instance.model_id.empty()) {
        throw InvalidParameterException("Instance must reference a model");
    }
    
    // Verify model exists
    if (!model_manager.hasModel(instance.model_id)) {
        throw ModelNotFoundException(instance.model_id);
    }
}

std::string InstanceManager::computeConfigHash(const RuntimeInstance& instance) const {
    // Create a hash from configuration parameters
    std::stringstream ss;
    ss << instance.model_id << "|";
    ss << instance.base_frame.position.x() << "|";
    ss << instance.base_frame.position.y() << "|";
    ss << instance.base_frame.position.z() << "|";
    ss << static_cast<int>(instance.mounting_type) << "|";
    ss << instance.mounting_angle;
    
    std::string data = ss.str();
    
    // SHA-256 hash
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(data.c_str()), data.size(), hash);
    
    std::stringstream hash_ss;
    for (int i = 0; i < 8; ++i) {  // Use first 8 bytes for shorter hash
        hash_ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hash[i]);
    }
    
    return hash_ss.str();
}

const RuntimeInstance& InstanceManager::configureInstance(
    const RuntimeInstance& instance,
    const ModelManager& model_manager) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    validateInstance(instance, model_manager);
    
    if (instances_.find(instance.instance_id) != instances_.end()) {
        throw DuplicateInstanceException(instance.instance_id);
    }
    
    // Get model DOF for initialization
    const auto& model = model_manager.getModel(instance.model_id);
    
    RuntimeInstance configured = instance;
    configured.created_at = KinematicModel::getCurrentTimestamp();
    configured.last_updated = configured.created_at;
    configured.status = InstanceStatus::READY;
    configured.config_hash = computeConfigHash(configured);
    
    // Initialize joint vectors if empty
    if (configured.current_joints.empty()) {
        configured.current_joints.resize(model.dof, 0.0);
    }
    if (configured.joint_offsets.empty()) {
        configured.joint_offsets.resize(model.dof, 0.0);
    }
    
    // Compute gravity vector
    configured.computeGravityVector();
    
    instances_[instance.instance_id] = configured;
    
    return instances_[instance.instance_id];
}

const RuntimeInstance& InstanceManager::configureInstance(
    const std::string& instance_id,
    const std::string& model_id,
    const BaseFrame& base_frame,
    MountingType mounting_type,
    double mounting_angle,
    const TCPDefinition& default_tcp,
    const ModelManager& model_manager) {
    
    RuntimeInstance instance;
    instance.instance_id = instance_id;
    instance.model_id = model_id;
    instance.base_frame = base_frame;
    instance.mounting_type = mounting_type;
    instance.mounting_angle = mounting_angle;
    instance.default_tcp = default_tcp;
    
    return configureInstance(instance, model_manager);
}

const RuntimeInstance& InstanceManager::getInstance(const std::string& instance_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(instance_id);
    if (it == instances_.end()) {
        throw InstanceNotFoundException(instance_id);
    }
    
    return it->second;
}

bool InstanceManager::hasInstance(const std::string& instance_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return instances_.find(instance_id) != instances_.end();
}

std::vector<std::string> InstanceManager::getInstanceIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<std::string> ids;
    ids.reserve(instances_.size());
    
    for (const auto& pair : instances_) {
        ids.push_back(pair.first);
    }
    
    return ids;
}

size_t InstanceManager::getInstanceCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return instances_.size();
}

void InstanceManager::updateInstanceStatus(const std::string& instance_id, 
                                            InstanceStatus status) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(instance_id);
    if (it == instances_.end()) {
        throw InstanceNotFoundException(instance_id);
    }
    
    it->second.status = status;
    it->second.last_updated = KinematicModel::getCurrentTimestamp();
}

void InstanceManager::updateInstanceJoints(const std::string& instance_id,
                                            const std::vector<double>& joints) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(instance_id);
    if (it == instances_.end()) {
        throw InstanceNotFoundException(instance_id);
    }
    
    it->second.current_joints = joints;
    it->second.last_updated = KinematicModel::getCurrentTimestamp();
}

void InstanceManager::deleteInstance(const std::string& instance_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(instance_id);
    if (it == instances_.end()) {
        throw InstanceNotFoundException(instance_id);
    }
    
    instances_.erase(it);
    calibrations_.erase(instance_id);
}

void InstanceManager::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    instances_.clear();
    calibrations_.clear();
}

void InstanceManager::applyCalibration(
    const std::string& instance_id,
    const CalibrationData& calibration,
    ModelManager& model_manager) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(instance_id);
    if (it == instances_.end()) {
        throw InstanceNotFoundException(instance_id);
    }
    
    if (it->second.status == InstanceStatus::MOVING) {
        throw InstanceInMotionException(instance_id);
    }
    
    // Validate calibration data
    const auto& model = model_manager.getModel(it->second.model_id);
    
    if (!calibration.joint_offsets.empty() &&
        static_cast<int>(calibration.joint_offsets.size()) != model.dof) {
        throw InvalidParameterException("Joint offsets size does not match model DOF");
    }
    
    // Check for excessive deviations (unless force_update)
    const double MAX_DEVIATION = 0.01;  // 10mm or 0.01 rad
    if (!calibration.force_update) {
        // TODO: Implement deviation checking
    }
    
    // Apply joint offsets
    if (!calibration.joint_offsets.empty()) {
        it->second.joint_offsets = calibration.joint_offsets;
    }
    
    // Apply parameter corrections
    if (std::holds_alternative<std::vector<MDHCalibrationCorrection>>(calibration.corrections)) {
        const auto& corrections = 
            std::get<std::vector<MDHCalibrationCorrection>>(calibration.corrections);
        
        // Update model parameters
        auto solver = model_manager.getKinematicSolver(it->second.model_id);
        if (auto mdh_solver = std::dynamic_pointer_cast<MDHModel>(solver)) {
            mdh_solver->applyCalibration(corrections);
        }
    }
    
    // Store calibration data
    calibrations_[instance_id] = calibration;
    
    // Update timestamp
    it->second.last_updated = KinematicModel::getCurrentTimestamp();
}

CalibrationData InstanceManager::getCalibrationData(const std::string& instance_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = calibrations_.find(instance_id);
    if (it == calibrations_.end()) {
        return CalibrationData();  // Return empty calibration
    }
    
    return it->second;
}

std::vector<std::string> InstanceManager::getInstancesByModel(const std::string& model_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<std::string> result;
    for (const auto& pair : instances_) {
        if (pair.second.model_id == model_id) {
            result.push_back(pair.first);
        }
    }
    
    return result;
}

std::shared_ptr<KinematicModelBase> InstanceManager::getKinematicSolver(
    const std::string& instance_id,
    const ModelManager& model_manager) const {
    
    const auto& instance = getInstance(instance_id);
    return model_manager.getKinematicSolver(instance.model_id);
}

} // namespace remani::core
