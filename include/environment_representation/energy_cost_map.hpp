#ifndef ASV_PLANNING_ENERGY_COST_MAP_HPP
#define ASV_PLANNING_ENERGY_COST_MAP_HPP

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include "multi_resolution_grid.hpp"

namespace asv_planning {

/**
 * @class EnergyCostMap
 * @brief Represents the energy consumption costs for ASV navigation in a maritime environment
 * 
 * This class manages a grid-based representation of energy costs, incorporating
 * factors such as currents, waves, water depth, and shipping lanes to provide
 * energy cost estimates for path planning.
 */
class EnergyCostMap {
public:
    /**
     * @brief Constructor for EnergyCostMap
     */
    EnergyCostMap();
    
    /**
     * @brief Default destructor
     */
    ~EnergyCostMap() = default;
    
    /**
     * @brief Updates the energy cost map based on environmental data
     * @param environment_map The multi-resolution grid containing environmental data
     */
    void updateFromEnvironment(const MultiResolutionGrid& environment_map);
    
    /**
     * @brief Gets the energy cost at a specific position and heading
     * @param position The position to check
     * @param heading The vessel heading in radians
     * @return Energy cost (higher values indicate higher energy consumption)
     */
    double getEnergyCost(const Eigen::Vector2d& position, double heading) const;
    
    /**
     * @brief Gets the energy cost for traveling between two points
     * @param from Start position
     * @param to End position
     * @param heading Vessel heading in radians
     * @return Energy cost for the trajectory
     */
    double getTrajectoryEnergyCost(
        const Eigen::Vector2d& from,
        const Eigen::Vector2d& to,
        double heading) const;
    
    /**
     * @brief Sets the ASV speed parameter for energy calculations
     * @param speed Vessel speed in m/s
     */
    void setSpeed(double speed);
    
    /**
     * @brief Sets the ASV drag coefficient
     * @param drag_coefficient Drag coefficient value
     */
    void setDragCoefficient(double drag_coefficient);
    
    /**
     * @brief Sets the current influence weight
     * @param weight Weight value (0-1)
     */
    void setCurrentWeight(double weight);
    
    /**
     * @brief Sets the wave influence weight
     * @param weight Weight value (0-1)
     */
    void setWaveWeight(double weight);
    
    /**
     * @brief Sets the shipping lane preference
     * @param preference Preference value (0-1)
     */
    void setShippingLanePreference(double preference);
    
    /**
     * @brief Gets the dimensions of the map
     * @return A pair of width and height
     */
    std::pair<int, int> getDimensions() const;
    
    /**
     * @brief Gets a shared pointer to this energy cost map
     * @return Shared pointer to this instance
     */
    std::shared_ptr<EnergyCostMap> getSharedPtr();
    
private:
    /**
     * @brief Calculates energy cost due to moving against currents
     * @param position Position to check
     * @param heading Vessel heading in radians
     * @return Energy cost factor due to currents
     */
    double calculateCurrentEnergyCost(const Eigen::Vector2d& position, double heading) const;
    
    /**
     * @brief Calculates energy cost due to wave effects
     * @param position Position to check
     * @param heading Vessel heading in radians
     * @return Energy cost factor due to waves
     */
    double calculateWaveEnergyCost(const Eigen::Vector2d& position, double heading) const;
    
    /**
     * @brief Calculates energy cost adjustment for shipping lanes
     * @param position Position to check
     * @return Energy cost adjustment factor for shipping lanes
     */
    double calculateShippingLaneAdjustment(const Eigen::Vector2d& position) const;
    
    // Map properties
    int width_;
    int height_;
    double resolution_;
    
    // Energy cost map data (base costs without heading dependency)
    std::vector<double> base_energy_costs_;
    
    // Reference to environmental data
    MultiResolutionGrid environment_map_;
    
    // Energy model parameters
    double vessel_speed_;
    double drag_coefficient_;
    double current_weight_;
    double wave_weight_;
    double shipping_lane_preference_;
    
    // Default value
    double default_energy_cost_;
};

} // namespace asv_planning

#endif // ASV_PLANNING_ENERGY_COST_MAP_HPP 