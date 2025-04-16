#include "planning/maritime_heuristics.hpp"
#include "environment_representation/multi_resolution_grid.hpp"
#include <cmath>

namespace asv_planning {

double MaritimeHeuristics::getEnvironmentalFactor(const MultiResolutionGrid& map, 
                                                const Eigen::Vector2d& position) {
    // Combine all environmental factors with appropriate weights
    double current_factor = getCurrentTraversalCost(map, position, position) * current_weight_;
    double wave_factor = getWaveImpactFactor(map, position, position, 0.0) * wave_weight_;
    double lane_factor = getShippingLanePreference(map, position) * lane_weight_;
    double safety_factor = getSafetyFactor(map, position) * safety_weight_;
    
    return current_factor + wave_factor + lane_factor + safety_factor;
}

double MaritimeHeuristics::getCurrentTraversalCost(const MultiResolutionGrid& map,
                                                 const Eigen::Vector2d& from,
                                                 const Eigen::Vector2d& to) {
    // Calculate movement direction
    Eigen::Vector2d direction = (to - from).normalized();
    
    // Get current vector at position
    Eigen::Vector2d current = map.getCurrentVector(from);
    
    // Calculate the dot product to determine if currents help or hinder movement
    double dot_product = direction.dot(current);
    
    // Normalize to a cost factor (lower when currents help, higher when against)
    double current_magnitude = current.norm();
    if (current_magnitude < 0.01) {
        return 1.0; // Negligible current
    }
    
    // Scale dot product to get cost factor (1.0 is neutral, <1.0 is beneficial, >1.0 is costly)
    return 1.0 - dot_product * 0.5;
}

double MaritimeHeuristics::getWaveImpactFactor(const MultiResolutionGrid& map,
                                             const Eigen::Vector2d& from,
                                             const Eigen::Vector2d& to,
                                             double heading) {
    // Get wave height and direction at position
    double wave_height = map.getWaveHeight(from);
    double wave_direction = map.getWaveDirection(from);
    
    // Calculate relative angle between vessel heading and wave direction
    double relative_angle = std::abs(heading - wave_direction);
    while (relative_angle > M_PI) {
        relative_angle -= 2.0 * M_PI;
    }
    relative_angle = std::abs(relative_angle);
    
    // Maximum impact when waves are from the side (90 degrees)
    // Minimum impact when waves are from front or back (0 or 180 degrees)
    double angle_factor = std::sin(relative_angle);
    
    // Scale with wave height (higher waves create more impact)
    return 1.0 + (wave_height * angle_factor * 0.5);
}

double MaritimeHeuristics::getShippingLanePreference(const MultiResolutionGrid& map,
                                                   const Eigen::Vector2d& position) {
    // Check if position is in a shipping lane
    bool in_shipping_lane = map.isInShippingLane(position);
    
    // Prefer to stay in shipping lanes where safe
    return in_shipping_lane ? 0.8 : 1.0;
}

double MaritimeHeuristics::getSafetyFactor(const MultiResolutionGrid& map,
                                         const Eigen::Vector2d& position) {
    // Get distance to the nearest obstacle
    double obstacle_distance = map.getDistanceToNearestObstacle(position);
    
    // Define safety threshold (adjust based on vessel size)
    double safety_threshold = 20.0; // meters
    
    // Exponential safety factor that increases rapidly as we get close to obstacles
    if (obstacle_distance < safety_threshold) {
        double normalized_distance = obstacle_distance / safety_threshold;
        // Exponential increase as we get closer to obstacles
        return 1.0 + 5.0 * std::exp(3.0 * (1.0 - normalized_distance));
    }
    
    return 1.0; // Normal safety cost in open areas
}

} // namespace asv_planning 