#include "../include/temporal_decay.hpp"
#include <cmath>
#include <algorithm>

namespace asv_planning {

TemporalDecay::TemporalDecay(DecayModel model, double half_life)
    : decay_model_(model), half_life_(half_life) {
    // Initialize the temporal decay model
}

void TemporalDecay::addMeasurement(const Eigen::Vector2d& position, double value, double timestamp) {
    // Convert position to grid index
    auto index = positionToIndex(position);
    
    // Create or update measurement at this position
    MeasurementData data;
    data.value = value;
    data.timestamp = timestamp;
    
    measurements_[index] = data;
}

double TemporalDecay::getValueWithDecay(const Eigen::Vector2d& position, double current_time) const {
    // Convert position to grid index
    auto index = positionToIndex(position);
    
    // Check if we have a measurement at this position
    auto it = measurements_.find(index);
    if (it == measurements_.end()) {
        return 0.0;  // No measurement available
    }
    
    // Calculate the age of the measurement
    double age = current_time - it->second.timestamp;
    
    // Apply decay model to the measurement value
    double decay_factor = computeDecayFactor(age);
    return it->second.value * decay_factor;
}

double TemporalDecay::getConfidence(const Eigen::Vector2d& position, double current_time) const {
    // Convert position to grid index
    auto index = positionToIndex(position);
    
    // Check if we have a measurement at this position
    auto it = measurements_.find(index);
    if (it == measurements_.end()) {
        return 0.0;  // No confidence if no measurement
    }
    
    // Calculate the age of the measurement
    double age = current_time - it->second.timestamp;
    
    // Confidence is directly related to the decay factor
    return computeDecayFactor(age);
}

void TemporalDecay::setDecayParameters(DecayModel model, double half_life) {
    decay_model_ = model;
    half_life_ = half_life;
}

void TemporalDecay::clearOldMeasurements(double max_age) {
    double current_time = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // Remove measurements older than max_age
    auto it = measurements_.begin();
    while (it != measurements_.end()) {
        double age = current_time - it->second.timestamp;
        if (age > max_age) {
            it = measurements_.erase(it);
        } else {
            ++it;
        }
    }
}

double TemporalDecay::computeDecayFactor(double age) const {
    // If age is negative (future measurement), return 1.0
    if (age < 0.0) return 1.0;
    
    // Apply different decay models
    switch (decay_model_) {
        case DecayModel::NONE:
            return 1.0;  // No decay
        
        case DecayModel::LINEAR:
            // Linear decay: 1.0 at age=0, 0.0 at age=2*half_life
            return std::max(0.0, 1.0 - (age / (2.0 * half_life_)));
        
        case DecayModel::EXPONENTIAL:
            // Exponential decay: exp(-ln(2) * age / half_life)
            return std::exp(-0.693147180559945 * age / half_life_);
        
        case DecayModel::CUSTOM:
            // Custom decay model can be implemented here
            // Placeholder implementation - similar to exponential but with different curve
            return 1.0 / (1.0 + age / half_life_);
        
        default:
            return 1.0;
    }
}

std::pair<int, int> TemporalDecay::positionToIndex(const Eigen::Vector2d& position) const {
    // Simple grid discretization - adjust resolution as needed
    const double resolution = 1.0;  // 1.0 meter grid resolution
    
    // Convert continuous position to discrete grid indices
    int x_index = static_cast<int>(std::round(position.x() / resolution));
    int y_index = static_cast<int>(std::round(position.y() / resolution));
    
    return std::make_pair(x_index, y_index);
}

} // namespace asv_planning 