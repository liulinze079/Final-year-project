#pragma once

#include <vector>
#include <cmath>
#include "../local_planner/local_planner.hpp"

namespace asv_planning {

/**
 * @brief Constraints for maritime vessel motion planning
 * 
 * This class defines constraints specific to maritime vessel motion,
 * including speed, turning rates, and safety considerations.
 */
class MaritimeConstraints {
public:
    /**
     * @brief Constructor
     */
    MaritimeConstraints();
    
    /**
     * @brief Set the maximum vessel speed
     * @param speed Maximum speed in m/s
     */
    void setMaxSpeed(double speed);
    
    /**
     * @brief Set the maximum turning rate
     * @param rate Maximum turning rate in rad/s
     */
    void setMaxTurningRate(double rate);
    
    /**
     * @brief Set the weight for safety in planning
     * @param weight Safety weight (0-1)
     */
    void setSafetyWeight(double weight);
    
    /**
     * @brief Set the weight for efficiency in planning
     * @param weight Efficiency weight (0-1)
     */
    void setEfficiencyWeight(double weight);
    
    /**
     * @brief Get the maximum vessel speed
     * @return Maximum speed in m/s
     */
    double getMaxSpeed() const { return max_speed_; }
    
    /**
     * @brief Get the maximum turning rate
     * @return Maximum turning rate in rad/s
     */
    double getMaxTurningRate() const { return max_turning_rate_; }
    
    /**
     * @brief Get the safety weight
     * @return Safety weight (0-1)
     */
    double getSafetyWeight() const { return safety_weight_; }
    
    /**
     * @brief Get the efficiency weight
     * @return Efficiency weight (0-1)
     */
    double getEfficiencyWeight() const { return efficiency_weight_; }
    
    /**
     * @brief Check if a motion satisfies the constraints
     * @param current Current position
     * @param next Next position
     * @param current_heading Current heading (radians)
     * @param next_heading Next heading (radians)
     * @param velocity Velocity (m/s)
     * @return True if the motion is valid
     */
    bool checkMotionConstraints(
        const Point2D& current,
        const Point2D& next,
        double current_heading,
        double next_heading,
        double velocity) const;
    
    /**
     * @brief Evaluate a path against the constraints
     * @param path Path as a vector of points
     * @param headings Headings along the path
     * @param velocities Velocities along the path
     * @return Constraint satisfaction score (0-1, higher is better)
     */
    double evaluatePathConstraints(
        const std::vector<Point2D>& path,
        const std::vector<double>& headings,
        const std::vector<double>& velocities) const;
    
private:
    double max_speed_;        // Maximum vessel speed (m/s)
    double max_turning_rate_; // Maximum turning rate (rad/s)
    double safety_weight_;    // Weight for safety considerations (0-1)
    double efficiency_weight_; // Weight for efficiency considerations (0-1)
};

} // namespace asv_planning 