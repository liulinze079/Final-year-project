#ifndef MULTI_RESOLUTION_GRID_HPP
#define MULTI_RESOLUTION_GRID_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>

namespace asv_planning {

/**
 * @brief Grid cell in the multi-resolution grid
 */
struct GridCell {
    bool occupied = false;              ///< Cell occupancy state
    double obstacle_distance = -1.0;    ///< Distance to nearest obstacle (-1 if not computed)
    
    // Environmental factors
    Eigen::Vector2d force_field;        ///< Current vector (direction and magnitude)
    double wave_height = 0.0;           ///< Wave height (meters)
    double wave_direction = 0.0;        ///< Wave direction (radians)
    bool in_shipping_lane = false;      ///< Whether cell is in a shipping lane
    
    // Temporal factors
    double last_update_time = 0.0;      ///< Time of last update
    double decay_factor = 0.0;          ///< Temporal decay factor for occupancy
};

/**
 * @brief Multi-resolution grid for environment representation
 * 
 * This class implements a grid-based environment representation with:
 * 1. Multiple resolution levels (finer near the ASV and coarser further away)
 * 2. Force field representation for currents and waves
 * 3. Temporal decay for dynamic obstacles
 */
class MultiResolutionGrid {
public:
    /**
     * @brief Constructor
     * 
     * @param min_x Minimum x-coordinate of the map (meters)
     * @param max_x Maximum x-coordinate of the map (meters)
     * @param min_y Minimum y-coordinate of the map (meters)
     * @param max_y Maximum y-coordinate of the map (meters)
     * @param base_resolution Base resolution (finest level) in meters
     * @param max_levels Maximum number of resolution levels
     */
    MultiResolutionGrid(double min_x, double max_x, double min_y, double max_y, 
                    double base_resolution = 1.0, int max_levels = 3);
    
    /**
     * @brief Destructor
     */
    virtual ~MultiResolutionGrid() = default;
    
    /**
     * @brief Check if a point is within the map bounds
     * 
     * @param x X-coordinate
     * @param y Y-coordinate
     * @return True if position is within bounds
     */
    bool isWithinBounds(double x, double y) const;
    
    /**
     * @brief Convert world coordinates to grid indices at the base resolution
     * 
     * @param x X-coordinate in world frame
     * @param y Y-coordinate in world frame
     * @param grid_x Output grid x-index
     * @param grid_y Output grid y-index
     * @return True if conversion succeeded (point is within bounds)
     */
    bool worldToGrid(double x, double y, int& grid_x, int& grid_y) const;
    
    /**
     * @brief Convert grid indices to world coordinates (cell center)
     * 
     * @param grid_x Grid x-index
     * @param grid_y Grid y-index
     * @param x Output x-coordinate in world frame
     * @param y Output y-coordinate in world frame
     */
    void gridToWorld(int grid_x, int grid_y, double& x, double& y) const;
    
    /**
     * @brief Check if a cell is occupied
     * 
     * @param x X-coordinate in world frame
     * @param y Y-coordinate in world frame
     * @return True if the cell is occupied
     */
    bool isOccupied(double x, double y) const;
    
    /**
     * @brief Add an obstacle at the specified position
     * 
     * @param position Obstacle position
     * @param radius Obstacle radius (meters)
     */
    void addObstacle(const Eigen::Vector2d& position, double radius);
    
    /**
     * @brief Add a force field vector at the specified position
     * 
     * @param position Position in world coordinates
     * @param force Force vector (direction and magnitude)
     */
    void addForceField(const Eigen::Vector2d& position, const Eigen::Vector2d& force);
    
    /**
     * @brief Set wave properties at the specified position
     * 
     * @param position Position in world coordinates
     * @param height Wave height (meters)
     * @param direction Wave direction (radians)
     */
    void setWaveProperties(const Eigen::Vector2d& position, double height, double direction);
    
    /**
     * @brief Define a shipping lane in the grid
     * 
     * @param start_point Start point of the lane
     * @param end_point End point of the lane
     * @param width Width of the lane (meters)
     */
    void defineShippingLane(const Eigen::Vector2d& start_point, 
                          const Eigen::Vector2d& end_point,
                          double width);
    
    /**
     * @brief Update temporal decay for dynamic obstacles
     * 
     * @param current_time Current simulation time
     */
    void updateTemporalDecay(double current_time);
    
    /**
     * @brief Get force field magnitude at a position
     * 
     * @param position Position in world coordinates
     * @return Force magnitude
     */
    double getForceFieldMagnitude(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Get force field direction at a position
     * 
     * @param position Position in world coordinates
     * @return Force direction (radians)
     */
    double getForceFieldDirection(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Get wave height at a position
     * 
     * @param position Position in world coordinates
     * @return Wave height (meters)
     */
    double getWaveHeight(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Get wave direction at a position
     * 
     * @param position Position in world coordinates
     * @return Wave direction (radians)
     */
    double getWaveDirection(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Check if a position is in a shipping lane
     * 
     * @param position Position in world coordinates
     * @return True if in a shipping lane
     */
    bool isInShippingLane(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Get distance to nearest obstacle
     * 
     * @param position Position in world coordinates
     * @return Distance to nearest obstacle (meters)
     */
    double getDistanceToNearestObstacle(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Generate a ROS visualization marker for the grid
     * 
     * @return Visualization marker message
     */
    // visualization_msgs::msg::MarkerArray getVisualizationMarkers() const;
    
private:
    double min_x_;                      ///< Minimum x-coordinate (m)
    double max_x_;                      ///< Maximum x-coordinate (m)
    double min_y_;                      ///< Minimum y-coordinate (m)
    double max_y_;                      ///< Maximum y-coordinate (m)
    double base_resolution_;            ///< Base resolution (m)
    int max_levels_;                    ///< Maximum number of resolution levels
    
    int width_;                         ///< Grid width in cells (base resolution)
    int height_;                        ///< Grid height in cells (base resolution)
    
    std::vector<std::vector<GridCell>> grid_;  ///< The actual grid cells
    
    /**
     * @brief Get grid cell at the specified position
     * 
     * @param x X-coordinate in world frame
     * @param y Y-coordinate in world frame
     * @return Pointer to grid cell, or nullptr if out of bounds
     */
    const GridCell* getCell(double x, double y) const;
    
    /**
     * @brief Get mutable grid cell at the specified position
     * 
     * @param x X-coordinate in world frame
     * @param y Y-coordinate in world frame
     * @return Pointer to grid cell, or nullptr if out of bounds
     */
    GridCell* getMutableCell(double x, double y);
    
    /**
     * @brief Compute distance field for obstacles
     * 
     * Updates obstacle_distance in all cells
     */
    void computeDistanceField();
};

} // namespace asv_planning

#endif // MULTI_RESOLUTION_GRID_HPP 