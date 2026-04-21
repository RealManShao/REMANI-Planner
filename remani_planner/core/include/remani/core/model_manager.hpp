/**
 * @file model_manager.hpp
 * @brief Model and Instance lifecycle management
 */

#ifndef REMANI_CORE_MODEL_MANAGER_HPP
#define REMANI_CORE_MODEL_MANAGER_HPP

#include "remani/core/kinematic_params.hpp"
#include "remani/core/model.hpp"
#include "remani/core/kinematic_model.hpp"
#include <unordered_map>
#include <memory>
#include <mutex>
#include <functional>

namespace remani::core {

// ============================================================================
// Exceptions
// ============================================================================

/**
 * @brief Base exception for REMANI core
 */
class RemaniException : public std::runtime_error {
public:
    explicit RemaniException(const std::string& msg, const std::string& code = "UNKNOWN_ERROR")
        : std::runtime_error(msg), error_code_(code) {}
    
    const std::string& code() const { return error_code_; }
    
private:
    std::string error_code_;
};

/**
 * @brief Model not found exception
 */
class ModelNotFoundException : public RemaniException {
public:
    explicit ModelNotFoundException(const std::string& model_id)
        : RemaniException("Model '" + model_id + "' not found", "MODEL_NOT_FOUND"),
          model_id_(model_id) {}
    
    const std::string& modelId() const { return model_id_; }
    
private:
    std::string model_id_;
};

/**
 * @brief Instance not found exception
 */
class InstanceNotFoundException : public RemaniException {
public:
    explicit InstanceNotFoundException(const std::string& instance_id)
        : RemaniException("Instance '" + instance_id + "' not found", "INSTANCE_NOT_FOUND"),
          instance_id_(instance_id) {}
    
    const std::string& instanceId() const { return instance_id_; }
    
private:
    std::string instance_id_;
};

/**
 * @brief Duplicate model exception
 */
class DuplicateModelException : public RemaniException {
public:
    explicit DuplicateModelException(const std::string& model_id)
        : RemaniException("Model '" + model_id + "' already exists", "DUPLICATE_MODEL"),
          model_id_(model_id) {}
    
private:
    std::string model_id_;
};

/**
 * @brief Duplicate instance exception
 */
class DuplicateInstanceException : public RemaniException {
public:
    explicit DuplicateInstanceException(const std::string& instance_id)
        : RemaniException("Instance '" + instance_id + "' already exists", "DUPLICATE_INSTANCE"),
          instance_id_(instance_id) {}
    
private:
    std::string instance_id_;
};

/**
 * @brief Invalid parameter exception
 */
class InvalidParameterException : public RemaniException {
public:
    explicit InvalidParameterException(const std::string& msg)
        : RemaniException(msg, "INVALID_PARAMETER") {}
};

/**
 * @brief Instance in motion exception
 */
class InstanceInMotionException : public RemaniException {
public:
    explicit InstanceInMotionException(const std::string& instance_id)
        : RemaniException("Instance '" + instance_id + "' is in motion and cannot be modified", 
                          "INSTANCE_IN_MOTION"),
          instance_id_(instance_id) {}
    
private:
    std::string instance_id_;
};

// ============================================================================
// Model Manager
// ============================================================================

/**
 * @brief Manages kinematic models (CRUD operations)
 * 
 * Thread-safe model registry with validation and persistence support.
 */
class ModelManager {
public:
    ModelManager() = default;
    ~ModelManager() = default;
    
    // Non-copyable, movable
    ModelManager(const ModelManager&) = delete;
    ModelManager& operator=(const ModelManager&) = delete;
    ModelManager(ModelManager&&) = default;
    ModelManager& operator=(ModelManager&&) = default;
    
    // ========================================================================
    // Model CRUD Operations
    // ========================================================================
    
    /**
     * @brief Register a new kinematic model
     * @param model Model definition
     * @return Reference to registered model
     * @throws DuplicateModelException if model_id already exists
     * @throws InvalidParameterException if model is invalid
     */
    const KinematicModel& registerModel(const KinematicModel& model);
    
    /**
     * @brief Register model from parameters
     */
    const KinematicModel& registerModel(
        const std::string& model_id,
        const std::string& name,
        ModelType model_type,
        const KinematicParams& params,
        const JointLimits& limits = JointLimits());
    
    /**
     * @brief Get model by ID
     * @param model_id Model identifier
     * @return Reference to model
     * @throws ModelNotFoundException if not found
     */
    const KinematicModel& getModel(const std::string& model_id) const;
    
    /**
     * @brief Check if model exists
     */
    bool hasModel(const std::string& model_id) const;
    
    /**
     * @brief Get all model IDs
     */
    std::vector<std::string> getModelIds() const;
    
    /**
     * @brief Get model count
     */
    size_t getModelCount() const;
    
    /**
     * @brief Update model status
     */
    void updateModelStatus(const std::string& model_id, ModelStatus status);
    
    /**
     * @brief Delete model by ID
     * @throws ModelNotFoundException if not found
     */
    void deleteModel(const std::string& model_id);
    
    /**
     * @brief Clear all models
     */
    void clear();
    
    // ========================================================================
    // Model Capabilities
    // ========================================================================
    
    /**
     * @brief Get model capabilities
     * @throws ModelNotFoundException if not found
     */
    ModelCapabilities getModelCapabilities(const std::string& model_id) const;
    
    /**
     * @brief Get kinematic solver for model
     * @throws ModelNotFoundException if not found
     */
    std::shared_ptr<KinematicModelBase> getKinematicSolver(const std::string& model_id) const;
    
private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, KinematicModel> models_;
    std::unordered_map<std::string, std::shared_ptr<KinematicModelBase>> solvers_;
    
    void validateModel(const KinematicModel& model) const;
};

// ============================================================================
// Instance Manager
// ============================================================================

/**
 * @brief Manages runtime instances (CRUD operations)
 * 
 * Thread-safe instance registry with configuration and calibration support.
 */
class InstanceManager {
public:
    InstanceManager() = default;
    ~InstanceManager() = default;
    
    // Non-copyable, movable
    InstanceManager(const InstanceManager&) = delete;
    InstanceManager& operator=(const InstanceManager&) = delete;
    InstanceManager(InstanceManager&&) = default;
    InstanceManager& operator=(InstanceManager&&) = default;
    
    // ========================================================================
    // Instance CRUD Operations
    // ========================================================================
    
    /**
     * @brief Configure a new runtime instance
     * @param instance Instance definition
     * @param model_manager Reference to model manager (for validation)
     * @return Reference to configured instance
     * @throws DuplicateInstanceException if instance_id already exists
     * @throws ModelNotFoundException if referenced model doesn't exist
     * @throws InvalidParameterException if instance is invalid
     */
    const RuntimeInstance& configureInstance(
        const RuntimeInstance& instance,
        const ModelManager& model_manager);
    
    /**
     * @brief Configure instance with parameters
     */
    const RuntimeInstance& configureInstance(
        const std::string& instance_id,
        const std::string& model_id,
        const BaseFrame& base_frame = BaseFrame(),
        MountingType mounting_type = MountingType::FLOOR,
        double mounting_angle = 0.0,
        const TCPDefinition& default_tcp = TCPDefinition(),
        const ModelManager& model_manager = ModelManager());
    
    /**
     * @brief Get instance by ID
     * @throws InstanceNotFoundException if not found
     */
    const RuntimeInstance& getInstance(const std::string& instance_id) const;
    
    /**
     * @brief Check if instance exists
     */
    bool hasInstance(const std::string& instance_id) const;
    
    /**
     * @brief Get all instance IDs
     */
    std::vector<std::string> getInstanceIds() const;
    
    /**
     * @brief Get instance count
     */
    size_t getInstanceCount() const;
    
    /**
     * @brief Update instance status
     */
    void updateInstanceStatus(const std::string& instance_id, InstanceStatus status);
    
    /**
     * @brief Update instance joint positions
     */
    void updateInstanceJoints(const std::string& instance_id, 
                               const std::vector<double>& joints);
    
    /**
     * @brief Delete instance by ID
     * @throws InstanceNotFoundException if not found
     */
    void deleteInstance(const std::string& instance_id);
    
    /**
     * @brief Clear all instances
     */
    void clear();
    
    // ========================================================================
    // Calibration
    // ========================================================================
    
    /**
     * @brief Apply calibration to instance
     * @param instance_id Instance identifier
     * @param calibration Calibration data
     * @param model_manager Model manager (to update model parameters)
     * @throws InstanceNotFoundException if not found
     * @throws InstanceInMotionException if instance is moving
     * @throws InvalidParameterException if calibration data is invalid
     */
    void applyCalibration(
        const std::string& instance_id,
        const CalibrationData& calibration,
        ModelManager& model_manager);
    
    /**
     * @brief Get calibration data for instance
     */
    CalibrationData getCalibrationData(const std::string& instance_id) const;
    
    // ========================================================================
    // Utility Methods
    // ========================================================================
    
    /**
     * @brief Get instances using a specific model
     */
    std::vector<std::string> getInstancesByModel(const std::string& model_id) const;
    
    /**
     * @brief Get kinematic solver for instance
     */
    std::shared_ptr<KinematicModelBase> getKinematicSolver(
        const std::string& instance_id,
        const ModelManager& model_manager) const;
    
private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, RuntimeInstance> instances_;
    std::unordered_map<std::string, CalibrationData> calibrations_;
    
    void validateInstance(const RuntimeInstance& instance, 
                          const ModelManager& model_manager) const;
    
    std::string computeConfigHash(const RuntimeInstance& instance) const;
};

} // namespace remani::core

#endif // REMANI_CORE_MODEL_MANAGER_HPP
