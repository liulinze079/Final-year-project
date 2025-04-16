#ifndef ASV_PLANNING_MARITIME_HEURISTICS_HPP
#define ASV_PLANNING_MARITIME_HEURISTICS_HPP

#include <Eigen/Dense>

namespace asv_planning {

// Forward declaration
class MultiResolutionGrid;

/**
 * @class MaritimeHeuristics
 * @brief Provides specialized heuristics for maritime path planning
 * 
 * This class implements various heuristics and cost functions specific to maritime
 * environments, considering factors like currents, waves, shipping lanes, and
 * safety constraints for autonomous surface vehicle navigation.
 */
class MaritimeHeuristics {
public:
    /**
     * @brief Constructor for MaritimeHeuristics
     */
    MaritimeHeuristics() = default;
    
    /**
     * @brief Gets a combined environmental factor for a position
     * @param map The environment map
     * @param position The position to evaluate
     * @return Environmental factor (higher values indicate higher costs)
     */
    double getEnvironmentalFactor(const MultiResolutionGrid& map, const Eigen::Vector2d& position);
    
    /**
     * @brief Calculates the cost of traveling between two points considering currents
     * @param map The environment map
     * @param from Start position
     * @param to End position
     * @return Current traversal cost factor
     */
    double getCurrentTraversalCost(const MultiResolutionGrid& map, 
                                  const Eigen::Vector2d& from, 
                                  const Eigen::Vector2d& to);
    
    /**
     * @brief Calculates the impact of waves on vessel motion
     * @param map The environment map
     * @param from Start position
     * @param to End position
     * @param heading Vessel heading in radians
     * @return Wave impact factor
     */
    double getWaveImpactFactor(const MultiResolutionGrid& map,
                             const Eigen::Vector2d& from,
                             const Eigen::Vector2d& to,
                             double heading);
    
    /**
     * @brief Gets a cost adjustment factor for shipping lanes
     * @param map The environment map
     * @param position The position to evaluate
     * @return Shipping lane preference factor (lower values indicate preference)
     */
    double getShippingLanePreference(const MultiResolutionGrid& map,
                                   const Eigen::Vector2d& position);
    
    /**
     * @brief Calculates a safety factor based on proximity to obstacles
     * @param map The environment map
     * @param position The position to evaluate
     * @return Safety factor (higher values indicate higher costs near obstacles)
     */
    double getSafetyFactor(const MultiResolutionGrid& map,
                         const Eigen::Vector2d& position);
    
    /**
     * @brief Sets the current weight for combined heuristics
     * @param weight Current weight (0-1)
     */
    void setCurrentWeight(double weight) { current_weight_ = weight; }
    
    /**
     * @brief Sets the wave weight for combined heuristics
     * @param weight Wave weight (0-1)
     */
    void setWaveWeight(double weight) { wave_weight_ = weight; }
    
    /**
     * @brief Sets the shipping lane weight for combined heuristics
     * @param weight Shipping lane weight (0-1)
     */
    void setShippingLaneWeight(double weight) { lane_weight_ = weight; }
    
    /**
     * @brief Sets the safety weight for combined heuristics
     * @param weight Safety weight (0-1)
     */
    void setSafetyWeight(double weight) { safety_weight_ = weight; }
    
private:
    // Weights for different environmental factors
    double current_weight_ = 0.3;
    double wave_weight_ = 0.3;
    double lane_weight_ = 0.2;
    double safety_weight_ = 0.2;
};

} // namespace asv_planning

#endif // ASV_PLANNING_MARITIME_HEURISTICS_HPP 