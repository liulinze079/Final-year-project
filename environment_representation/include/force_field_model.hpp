#pragma once

#include <Eigen/Core>
#include <vector>
#include <memory>

namespace asv_planning {

/**
 * @class ForceFieldModel
 * @brief Models environmental forces as vector fields (currents, winds)
 * 
 * This class implements a force field model for representing currents and other
 * environmental forces in the maritime environment. It allows querying force
 * vectors at arbitrary positions and supports time-varying force fields.
 */
class ForceFieldModel {
public:
    /**
     * @brief Constructor
     * @param resolution Grid resolution for discretized force field
     */
    ForceFieldModel(double resolution = 1.0);
    
    /**
     * @brief Destructor
     */
    ~ForceFieldModel() = default;
    
    /**
     * @brief Get force vector at a specific position
     * @param position 2D position to query
     * @return Force vector (magnitude and direction)
     */
    Eigen::Vector2d getForceAt(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Get force magnitude at a specific position
     * @param position 2D position to query
     * @return Force magnitude
     */
    double getForceMagnitude(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Get force direction at a specific position
     * @param position 2D position to query
     * @return Force direction in radians
     */
    double getForceDirection(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Update the force field model based on new data
     * @param positions List of positions where force is measured
     * @param forces List of force vectors corresponding to positions
     */
    void updateForceField(const std::vector<Eigen::Vector2d>& positions,
                         const std::vector<Eigen::Vector2d>& forces);
    
    /**
     * @brief Define an analytic force field model (e.g., for simulation)
     * @param model_type Type of analytic model ("uniform", "radial", "vortex")
     * @param parameters Model-specific parameters
     */
    void setAnalyticModel(const std::string& model_type,
                         const std::vector<double>& parameters);
    
    /**
     * @brief Update the force field for the current timestamp
     * @param time_stamp Current time stamp for time-varying fields
     */
    void updateTimeStamp(double time_stamp);

private:
    // Resolution of the discretized force field grid
    double resolution_;
    
    // Current time stamp
    double current_time_;
    
    // Internal data structure for storing force field
    // (implementation details will depend on specific approach)
    
    // Interpolation methods to get force between grid points
    Eigen::Vector2d interpolateForce(const Eigen::Vector2d& position) const;
};

} // namespace asv_planning 