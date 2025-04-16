#pragma once

#include <Eigen/Core>
#include <vector>
#include <map>
#include <chrono>

namespace asv_planning {

/**
 * @class TemporalDecay
 * @brief Manages temporal decay of environmental information
 * 
 * This class handles the temporal aspects of environmental data, applying
 * decay functions to measurements based on their age. It supports different
 * decay models (linear, exponential, etc.) and can be used to manage the
 * reliability of sensed data over time.
 */
class TemporalDecay {
public:
    /**
     * @brief Decay model types for environmental information
     */
    enum class DecayModel {
        NONE,       // No decay
        LINEAR,     // Linear decay with time
        EXPONENTIAL,// Exponential decay with time
        CUSTOM      // Custom decay function
    };

    /**
     * @brief Constructor
     * @param model Decay model to use
     * @param half_life Half-life for decay in seconds (for exponential model)
     */
    TemporalDecay(DecayModel model = DecayModel::EXPONENTIAL, double half_life = 300.0);
    
    /**
     * @brief Destructor
     */
    ~TemporalDecay() = default;
    
    /**
     * @brief Add a new measurement with timestamp
     * @param position Position where measurement was taken
     * @param value Measured value
     * @param timestamp Time when measurement was taken (in seconds since epoch)
     */
    void addMeasurement(const Eigen::Vector2d& position, double value, double timestamp);
    
    /**
     * @brief Get current value at position, accounting for decay
     * @param position Position to query
     * @param current_time Current time (in seconds since epoch)
     * @return Decayed value at the position
     */
    double getValueWithDecay(const Eigen::Vector2d& position, double current_time) const;
    
    /**
     * @brief Get confidence in a value based on its age
     * @param position Position to query
     * @param current_time Current time (in seconds since epoch)
     * @return Confidence value between 0.0 (not confident) and 1.0 (fully confident)
     */
    double getConfidence(const Eigen::Vector2d& position, double current_time) const;
    
    /**
     * @brief Set decay model parameters
     * @param model Decay model type
     * @param half_life Half-life for decay in seconds (for exponential model)
     */
    void setDecayParameters(DecayModel model, double half_life);
    
    /**
     * @brief Clear all measurements older than a specified age
     * @param max_age Maximum age of measurements to keep (in seconds)
     */
    void clearOldMeasurements(double max_age);

private:
    // Decay model being used
    DecayModel decay_model_;
    
    // Parameters for decay models
    double half_life_;  // Half-life in seconds (for exponential decay)
    
    // Data structure to store measurements with timestamps
    struct MeasurementData {
        double value;
        double timestamp;
    };
    
    // Map of positions to measurement data
    std::map<std::pair<int, int>, MeasurementData> measurements_;
    
    // Helper to compute decay factor
    double computeDecayFactor(double age) const;
    
    // Convert continuous position to discrete grid index
    std::pair<int, int> positionToIndex(const Eigen::Vector2d& position) const;
};

} // namespace asv_planning 