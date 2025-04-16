#include "maritime_heuristics.hpp"
#include "../../../environment_representation/include/multi_resolution_grid.hpp"
#include <cmath>

namespace asv_planning {

double MaritimeHeuristics::getEnvironmentalFactor(const MultiResolutionGrid& map, const Eigen::Vector2d& position) {
    // Combine different environmental factors
    double current_factor = 1.0;
    double wave_factor = 1.0;
    double lane_factor = 1.0;
    double safety_factor = 1.0;
    
    // Get current-related cost (without specific direction)
    current_factor = map.getForceFieldMagnitude(position) * 0.2 + 1.0;
    
    // Get wave-related cost
    // Simplified: higher wave height increases cost
    wave_factor = map.getWaveHeight(position) * 0.1 + 1.0;
    
    // Get shipping lane preference (reduces cost in shipping lanes)
    lane_factor = getShippingLanePreference(map, position);
    
    // Get safety factor (increases cost near obstacles)
    safety_factor = getSafetyFactor(map, position);
    
    // Combine all factors
    // Current weight: 30%, Wave weight: 30%, Lane weight: 20%, Safety weight: 20%
    return 0.3 * current_factor + 0.3 * wave_factor + 0.2 * lane_factor + 0.2 * safety_factor;
}

double MaritimeHeuristics::getCurrentTraversalCost(const MultiResolutionGrid& map, 
                                                const Eigen::Vector2d& from, 
                                                const Eigen::Vector2d& to) {
    // Calculate movement direction
    Eigen::Vector2d direction = to - from;
    double movement_angle = std::atan2(direction.y(), direction.x());
    
    // Get current direction and magnitude at the midpoint
    Eigen::Vector2d midpoint = (from + to) / 2.0;
    double current_magnitude = map.getForceFieldMagnitude(midpoint);
    double current_direction = map.getForceFieldDirection(midpoint);
    
    // Calculate angle difference between movement and current
    double angle_diff = std::abs(movement_angle - current_direction);
    while (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;
    
    // Calculate cost factor:
    // - Moving against current (180 deg): highest cost
    // - Moving with current (0 deg): lowest cost
    // - Moving perpendicular to current (90 deg): neutral
    double cost_factor = 1.0 + current_magnitude * std::cos(angle_diff) * (-0.3);
    
    // Ensure cost doesn't go below a minimum threshold (e.g., 0.7)
    return std::max(0.7, cost_factor);
}

double MaritimeHeuristics::getWaveImpactFactor(const MultiResolutionGrid& map,
                                            const Eigen::Vector2d& from,
                                            const Eigen::Vector2d& to,
                                            double heading) {
    // Get wave properties at the midpoint
    Eigen::Vector2d midpoint = (from + to) / 2.0;
    double wave_height = map.getWaveHeight(midpoint);
    double wave_direction = map.getWaveDirection(midpoint);
    
    // Calculate angle difference between ASV heading and wave direction
    double angle_diff = std::abs(heading - wave_direction);
    while (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;
    
    // Calculate impact factor based on:
    // - Waves from the side (90 deg): highest impact
    // - Waves from front/back (0 or 180 deg): lower impact
    double direction_impact = std::sin(angle_diff);
    
    // Scale impact by wave height
    double impact_factor = 1.0 + wave_height * direction_impact * 0.2;
    
    return impact_factor;
}

double MaritimeHeuristics::getShippingLanePreference(const MultiResolutionGrid& map,
                                                  const Eigen::Vector2d& position) {
    // Check if position is in a shipping lane
    bool in_shipping_lane = map.isInShippingLane(position);
    
    // Prefer paths within shipping lanes by reducing cost
    if (in_shipping_lane) {
        return 0.8;  // 20% cost reduction
    }
    
    return 1.0;  // Neutral cost outside shipping lanes
}

double MaritimeHeuristics::getSafetyFactor(const MultiResolutionGrid& map,
                                        const Eigen::Vector2d& position) {
    // Get distance to nearest obstacle
    double obstacle_distance = map.getDistanceToNearestObstacle(position);
    
    // Define safety thresholds
    const double safe_distance = 10.0;     // Distance at which safety factor becomes 1.0
    const double critical_distance = 2.0;  // Distance at which safety factor is maximum
    
    // If far enough from obstacles, return neutral factor
    if (obstacle_distance >= safe_distance) {
        return 1.0;
    }
    
    // If closer than critical distance, return maximum safety factor
    if (obstacle_distance <= critical_distance) {
        return 2.0;  // Double the cost
    }
    
    // Linear interpolation between critical and safe distances
    double t = (obstacle_distance - critical_distance) / (safe_distance - critical_distance);
    return 2.0 - t * 1.0;  // Scale from 2.0 at critical to 1.0 at safe
}

} // namespace asv_planning 