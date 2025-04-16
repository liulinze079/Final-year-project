#include "../../include/environment_representation/energy_cost_map.hpp"
#include <cmath>
#include <algorithm>

namespace asv_planning {

EnergyCostMap::EnergyCostMap()
    : width_(0),
      height_(0),
      resolution_(1.0),
      vessel_speed_(2.0),
      drag_coefficient_(0.5),
      current_weight_(0.5),
      wave_weight_(0.3),
      shipping_lane_preference_(0.2),
      default_energy_cost_(1.0) {
}

void EnergyCostMap::updateFromEnvironment(const MultiResolutionGrid& environment_map) {
    // Store reference to environment map
    environment_map_ = environment_map;
    
    // Get dimensions from environment map
    auto dimensions = environment_map.getDimensions();
    width_ = dimensions.first;
    height_ = dimensions.second;
    resolution_ = environment_map.getBaseResolution();
    
    // Resize base energy cost array
    base_energy_costs_.resize(width_ * height_, default_energy_cost_);
    
    // Calculate base energy costs for each cell
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            // Convert to world coordinates
            double world_x, world_y;
            world_x = x * resolution_;
            world_y = y * resolution_;
            
            Eigen::Vector2d position(world_x, world_y);
            
            // Calculate base energy cost (averaged over all headings)
            double total_cost = 0.0;
            const int heading_samples = 8;
            
            for (int i = 0; i < heading_samples; ++i) {
                double heading = (2.0 * M_PI * i) / heading_samples;
                double current_cost = calculateCurrentEnergyCost(position, heading);
                double wave_cost = calculateWaveEnergyCost(position, heading);
                double lane_adjustment = calculateShippingLaneAdjustment(position);
                
                total_cost += (current_cost * current_weight_ + 
                              wave_cost * wave_weight_) * 
                              lane_adjustment;
            }
            
            // Store average cost
            int idx = y * width_ + x;
            base_energy_costs_[idx] = total_cost / heading_samples;
        }
    }
}

double EnergyCostMap::getEnergyCost(const Eigen::Vector2d& position, double heading) const {
    // Calculate grid coordinates
    int grid_x = static_cast<int>(position.x() / resolution_);
    int grid_y = static_cast<int>(position.y() / resolution_);
    
    // Check if position is within map bounds
    if (grid_x < 0 || grid_x >= width_ || grid_y < 0 || grid_y >= height_) {
        return default_energy_cost_;
    }
    
    // Get base cost from precomputed map
    int idx = grid_y * width_ + grid_x;
    double base_cost = base_energy_costs_[idx];
    
    // Apply heading-specific adjustments
    double current_cost = calculateCurrentEnergyCost(position, heading);
    double wave_cost = calculateWaveEnergyCost(position, heading);
    double lane_adjustment = calculateShippingLaneAdjustment(position);
    
    // Combine costs with weights
    double adjusted_cost = (current_cost * current_weight_ + 
                         wave_cost * wave_weight_) * 
                         lane_adjustment;
    
    // Blend base cost with heading-specific cost (75% heading-specific, 25% base)
    return 0.25 * base_cost + 0.75 * adjusted_cost;
}

double EnergyCostMap::getTrajectoryEnergyCost(
    const Eigen::Vector2d& from,
    const Eigen::Vector2d& to,
    double heading) const {
    
    // Calculate trajectory length
    double dx = to.x() - from.x();
    double dy = to.y() - from.y();
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Sample points along trajectory
    const int num_samples = std::max(3, static_cast<int>(distance / resolution_));
    double total_cost = 0.0;
    
    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        Eigen::Vector2d point = from + t * (to - from);
        total_cost += getEnergyCost(point, heading);
    }
    
    // Average cost along trajectory, scaled by distance
    double avg_cost = total_cost / num_samples;
    return avg_cost * distance;
}

void EnergyCostMap::setSpeed(double speed) {
    vessel_speed_ = std::max(0.1, speed);
}

void EnergyCostMap::setDragCoefficient(double drag_coefficient) {
    drag_coefficient_ = std::max(0.0, drag_coefficient);
}

void EnergyCostMap::setCurrentWeight(double weight) {
    current_weight_ = std::max(0.0, std::min(1.0, weight));
}

void EnergyCostMap::setWaveWeight(double weight) {
    wave_weight_ = std::max(0.0, std::min(1.0, weight));
}

void EnergyCostMap::setShippingLanePreference(double preference) {
    shipping_lane_preference_ = std::max(0.0, std::min(1.0, preference));
}

std::pair<int, int> EnergyCostMap::getDimensions() const {
    return {width_, height_};
}

std::shared_ptr<EnergyCostMap> EnergyCostMap::getSharedPtr() {
    return std::shared_ptr<EnergyCostMap>(this);
}

double EnergyCostMap::calculateCurrentEnergyCost(const Eigen::Vector2d& position, double heading) const {
    // Get current data from environment map
    double current_magnitude = environment_map_.getForceFieldMagnitude(position);
    double current_direction = environment_map_.getForceFieldDirection(position);
    
    if (current_magnitude < 0.01) {
        return 1.0;  // No significant current
    }
    
    // Calculate angle between heading and current
    double angle_diff = std::abs(heading - current_direction);
    while (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;
    
    // Normalize to [0, 1] where 0 is with current, 1 is against
    double normalized_diff = angle_diff / M_PI;
    
    // Calculate energy cost:
    // - Moving against current: higher cost (increases with current strength)
    // - Moving with current: lower cost (decreases with current strength)
    // - Moving perpendicular: neutral
    
    // Power required to overcome current = 0.5 * rho * Cd * A * v^2
    // where v = relative velocity between vessel and current
    
    // Simplified model: 
    // cost = 1 + current_effect * direction_factor
    double current_effect = current_magnitude / vessel_speed_;
    double direction_factor = 2.0 * normalized_diff - 1.0;  // [-1, 1]
    
    return 1.0 + current_effect * direction_factor * drag_coefficient_;
}

double EnergyCostMap::calculateWaveEnergyCost(const Eigen::Vector2d& position, double heading) const {
    // Get wave data from environment map
    double wave_height = environment_map_.getWaveHeight(position);
    double wave_direction = environment_map_.getWaveDirection(position);
    
    if (wave_height < 0.01) {
        return 1.0;  // No significant waves
    }
    
    // Calculate angle between heading and wave direction
    double angle_diff = std::abs(heading - wave_direction);
    while (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;
    
    // Energy cost depends on wave encounter angle:
    // - Waves from the side (90 deg): highest impact
    // - Waves from bow/stern: lower impact
    double direction_impact = std::sin(angle_diff);
    
    // Energy cost scales with wave height squared
    // (based on added resistance in waves theory)
    double height_factor = 1.0 + wave_height * wave_height * 0.2;
    
    return 1.0 + direction_impact * (height_factor - 1.0);
}

double EnergyCostMap::calculateShippingLaneAdjustment(const Eigen::Vector2d& position) const {
    // Check if in shipping lane
    bool in_lane = environment_map_.isInShippingLane(position);
    
    if (in_lane) {
        // Reduce energy cost in shipping lanes (better efficiency due to 
        // maintained channels, optimized routes, etc.)
        return 1.0 - shipping_lane_preference_;
    }
    
    return 1.0;
}

} // namespace asv_planning 