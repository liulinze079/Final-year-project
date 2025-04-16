#pragma once

#include "map_representation.hpp"
#include <vector>
#include <limits>

namespace asv_planning {

/**
 * @brief Grid-based implementation of the MapRepresentation interface
 * 
 * This class provides a concrete implementation of the MapRepresentation 
 * interface using a 2D grid. It includes an occupancy grid for obstacle 
 * representation and force field grids for environmental effects.
 */
class GridMapRepresentation : public MapRepresentation {
public:
    /**
     * @brief Constructor
     * 
     * @param resolution Cell size (meters)
     * @param width Map width (meters)
     * @param height Map height (meters)
     * @param origin_x X-coordinate of map origin (meters)
     * @param origin_y Y-coordinate of map origin (meters)
     */
    GridMapRepresentation(double resolution = 1.0,
                         double width = 1000.0,
                         double height = 1000.0,
                         double origin_x = -500.0,
                         double origin_y = -500.0);
    
    /**
     * @brief Virtual destructor
     */
    virtual ~GridMapRepresentation() = default;
    
    // MapRepresentation interface implementation
    bool isWithinBounds(double x, double y) const override;
    bool isOccupied(double x, double y) const override;
    bool isInCollision(double x, double y, double safety_distance = 0.0) const override;
    bool isLineInCollision(double start_x, double start_y,
                         double end_x, double end_y,
                         double resolution = 0.1,
                         double safety_distance = 0.0) const override;
    double getForceFieldMagnitude(const Eigen::Vector2d& position) const override;
    double getForceFieldDirection(const Eigen::Vector2d& position) const override;
    Eigen::Vector2d getForceVector(const Eigen::Vector2d& position) const override;
    double getDistanceToClosestObstacle(double x, double y, double max_range = 100.0) const override;
    std::vector<Eigen::Vector2d> getNearbyObstacles(double center_x, double center_y, double radius) const override;
    bool updateFromSensorData(const void* sensor_data) override;
    bool loadFromFile(const std::string& filename) override;
    bool saveToFile(const std::string& filename) const override;
    
    /**
     * @brief Set a cell as occupied or free
     * 
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param occupied True to set as occupied, false to set as free
     */
    void setOccupied(double x, double y, bool occupied);
    
    /**
     * @brief Set force field at a specific position
     * 
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param magnitude Force magnitude
     * @param direction Force direction (radians)
     */
    void setForceField(double x, double y, double magnitude, double direction);
    
    /**
     * @brief Compute the distance grid (distance transform)
     * 
     * This precomputes the distance from each cell to the nearest obstacle
     * to speed up distance queries.
     */
    void computeDistanceGrid();
    
private:
    /**
     * @brief Convert world coordinates to grid coordinates
     * 
     * @param x World X-coordinate
     * @param y World Y-coordinate
     * @param grid_x Output grid X-coordinate
     * @param grid_y Output grid Y-coordinate
     */
    void worldToGrid(double x, double y, int& grid_x, int& grid_y) const;
    
    /**
     * @brief Convert grid coordinates to world coordinates
     * 
     * @param grid_x Grid X-coordinate
     * @param grid_y Grid Y-coordinate
     * @param x Output world X-coordinate
     * @param y Output world Y-coordinate
     */
    void gridToWorld(int grid_x, int grid_y, double& x, double& y) const;
    
    // Grid parameters
    double resolution_;       ///< Cell size (meters)
    double width_;            ///< Map width (meters)
    double height_;           ///< Map height (meters)
    double origin_x_;         ///< X-coordinate of map origin (meters)
    double origin_y_;         ///< Y-coordinate of map origin (meters)
    
    // Grid data
    std::vector<std::vector<bool>> occupancy_grid_;           ///< Occupancy grid (true = occupied)
    std::vector<std::vector<double>> distance_grid_;          ///< Distance grid (cells to nearest obstacle)
    std::vector<std::vector<double>> force_magnitude_grid_;   ///< Force field magnitude
    std::vector<std::vector<double>> force_direction_grid_;   ///< Force field direction (radians)
};

} // namespace asv_planning 