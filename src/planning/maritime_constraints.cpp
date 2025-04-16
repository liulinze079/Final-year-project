#include "../../include/planning/maritime_constraints.hpp"
#include <cmath>
#include <algorithm>

namespace asv_planning {

MaritimeConstraints::MaritimeConstraints()
    : max_speed_(5.0),
      max_turning_rate_(0.5),
      safety_weight_(0.6),
      efficiency_weight_(0.4) {
}

void MaritimeConstraints::setMaxSpeed(double speed) {
    max_speed_ = std::max(0.1, speed);
}

void MaritimeConstraints::setMaxTurningRate(double rate) {
    max_turning_rate_ = std::max(0.01, rate);
}

void MaritimeConstraints::setSafetyWeight(double weight) {
    safety_weight_ = std::min(1.0, std::max(0.0, weight));
    // Ensure weights sum to 1.0
    efficiency_weight_ = 1.0 - safety_weight_;
}

void MaritimeConstraints::setEfficiencyWeight(double weight) {
    efficiency_weight_ = std::min(1.0, std::max(0.0, weight));
    // Ensure weights sum to 1.0
    safety_weight_ = 1.0 - efficiency_weight_;
}

bool MaritimeConstraints::checkMotionConstraints(
    const Point2D& current,
    const Point2D& next,
    double current_heading,
    double next_heading,
    double velocity) const {
    
    // Check if velocity is within limits
    if (velocity > max_speed_) {
        return false;
    }
    
    // Check turning rate constraints
    double heading_diff = std::abs(next_heading - current_heading);
    while (heading_diff > M_PI) heading_diff = 2.0 * M_PI - heading_diff;
    
    double distance = std::sqrt(
        std::pow(next.x - current.x, 2) + 
        std::pow(next.y - current.y, 2));
    
    // If distance is too small, avoid division by zero
    if (distance < 0.01) {
        return true;
    }
    
    // Calculate the turning rate (radians per meter)
    double turning_rate_per_meter = heading_diff / distance;
    
    // Calculate the turning rate in radians per second based on velocity
    double turning_rate_per_second = turning_rate_per_meter * velocity;
    
    // Check if turning rate exceeds maximum
    return turning_rate_per_second <= max_turning_rate_;
}

double MaritimeConstraints::evaluatePathConstraints(
    const std::vector<Point2D>& path,
    const std::vector<double>& headings,
    const std::vector<double>& velocities) const {
    
    if (path.size() < 2 || headings.size() < path.size() || velocities.size() < path.size()) {
        return 1.0;  // Default score for invalid paths
    }
    
    double total_constraint_violation = 0.0;
    double total_segments = static_cast<double>(path.size() - 1);
    
    for (size_t i = 1; i < path.size(); ++i) {
        // Calculate heading change
        double current_heading = headings[i-1];
        double next_heading = headings[i];
        double velocity = velocities[i-1];
        
        // Check constraints for this segment
        if (!checkMotionConstraints(
                path[i-1], path[i],
                current_heading, next_heading,
                velocity)) {
            
            // Calculate how much the constraint is violated
            double heading_diff = std::abs(next_heading - current_heading);
            while (heading_diff > M_PI) heading_diff = 2.0 * M_PI - heading_diff;
            
            double distance = std::sqrt(
                std::pow(path[i].x - path[i-1].x, 2) + 
                std::pow(path[i].y - path[i-1].y, 2));
            
            if (distance < 0.01) {
                distance = 0.01;  // Avoid division by zero
            }
            
            double turning_rate = heading_diff / distance * velocity;
            double violation = (turning_rate / max_turning_rate_) - 1.0;
            
            // Accumulate constraint violation
            total_constraint_violation += std::min(1.0, violation);
        }
        
        // Check speed constraint
        if (velocity > max_speed_) {
            double speed_violation = (velocity / max_speed_) - 1.0;
            total_constraint_violation += std::min(1.0, speed_violation);
        }
    }
    
    // Calculate constraint satisfaction score (0-1, higher is better)
    double total_possible_violations = 2.0 * total_segments;  // 2 constraints per segment
    double constraint_score = 1.0 - (total_constraint_violation / total_possible_violations);
    
    return std::max(0.0, constraint_score);
}

} // namespace asv_planning 