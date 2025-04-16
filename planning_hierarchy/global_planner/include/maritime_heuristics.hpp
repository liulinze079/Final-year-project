#ifndef MARITIME_HEURISTICS_HPP
#define MARITIME_HEURISTICS_HPP

#include <Eigen/Dense>

namespace asv_planning {

// Forward declarations
class MultiResolutionGrid;

/**
 * @brief Maritime-specific heuristics for path planning
 * 
 * This class contains specialized heuristic functions for ASV path planning
 * that account for maritime conditions such as currents, waves, and shipping lanes.
 */
class MaritimeHeuristics {
public:
    /**
     * @brief Get an environmental factor to adjust path cost based on position
     * 
     * @param map The environmental grid map
     * @param position Current position to evaluate
     * @return Environmental factor (1.0 is neutral, >1.0 increases cost, <1.0 decreases cost)
     */
    static double getEnvironmentalFactor(const MultiResolutionGrid& map, const Eigen::Vector2d& position);
    
    /**
     * @brief Calculate the cost of traversing between two points accounting for currents
     * 
     * @param map The environmental grid map
     * @param from Start position
     * @param to End position
     * @return Cost factor (1.0 is neutral, >1.0 increases cost, <1.0 decreases cost)
     */
    static double getCurrentTraversalCost(const MultiResolutionGrid& map, 
                                        const Eigen::Vector2d& from, 
                                        const Eigen::Vector2d& to);
    
    /**
     * @brief Calculate the wave impact on the planned path
     * 
     * @param map The environmental grid map
     * @param from Start position
     * @param to End position
     * @param heading ASV heading in radians
     * @return Wave impact factor (1.0 is neutral, >1.0 increases cost)
     */
    static double getWaveImpactFactor(const MultiResolutionGrid& map,
                                    const Eigen::Vector2d& from,
                                    const Eigen::Vector2d& to,
                                    double heading);
    
    /**
     * @brief Calculate the shipping lane preference factor
     * 
     * @param map The environmental grid map
     * @param position Current position to evaluate
     * @return Shipping lane factor (1.0 is neutral, <1.0 means lane is preferred)
     */
    static double getShippingLanePreference(const MultiResolutionGrid& map,
                                         const Eigen::Vector2d& position);
    
    /**
     * @brief Calculate the safety cost factor based on proximity to obstacles
     * 
     * @param map The environmental grid map
     * @param position Current position to evaluate
     * @return Safety factor (1.0 is neutral, >1.0 increases cost near obstacles)
     */
    static double getSafetyFactor(const MultiResolutionGrid& map,
                               const Eigen::Vector2d& position);
};

} // namespace asv_planning

#endif // MARITIME_HEURISTICS_HPP 