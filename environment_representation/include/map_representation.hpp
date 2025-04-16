#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <string>

namespace asv_planning {

/**
 * @brief Abstract base class for map representations
 * 
 * This class defines the interface for different map representations that can be used
 * by the planning hierarchy. Derived classes implement specific map types (grid maps,
 * multi-resolution grids, etc.).
 */
class MapRepresentation {
public:
    /**
     * @brief Constructor
     */
    MapRepresentation() = default;
    
    /**
     * @brief Virtual destructor
     */
    virtual ~MapRepresentation() = default;
    
    /**
     * @brief Check if a position is within map bounds
     * 
     * @param x X-coordinate
     * @param y Y-coordinate
     * @return true if position is within bounds, false otherwise
     */
    virtual bool isWithinBounds(double x, double y) const = 0;
    
    /**
     * @brief Check if a position is occupied by an obstacle
     * 
     * @param x X-coordinate
     * @param y Y-coordinate
     * @return true if position is occupied, false otherwise
     */
    virtual bool isOccupied(double x, double y) const = 0;
    
    /**
     * @brief Check if a position is in collision with obstacles
     * 
     * This method can include additional factors like safety margins
     * 
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param safety_distance Safety distance from obstacles
     * @return true if position is in collision, false otherwise
     */
    virtual bool isInCollision(double x, double y, double safety_distance = 0.0) const = 0;
    
    /**
     * @brief Check if a line segment is in collision with obstacles
     * 
     * @param start_x Start X-coordinate
     * @param start_y Start Y-coordinate
     * @param end_x End X-coordinate
     * @param end_y End Y-coordinate
     * @param resolution Resolution for collision checking
     * @param safety_distance Safety distance from obstacles
     * @return true if line is in collision, false otherwise
     */
    virtual bool isLineInCollision(
        double start_x, double start_y,
        double end_x, double end_y,
        double resolution = 0.1,
        double safety_distance = 0.0) const = 0;
    
    /**
     * @brief Get the magnitude of environmental force field at a position
     * 
     * @param position Position vector (x,y)
     * @return Force magnitude
     */
    virtual double getForceFieldMagnitude(const Eigen::Vector2d& position) const = 0;
    
    /**
     * @brief Get the direction of environmental force field at a position
     * 
     * @param position Position vector (x,y)
     * @return Force direction in radians
     */
    virtual double getForceFieldDirection(const Eigen::Vector2d& position) const = 0;
    
    /**
     * @brief Get the environmental force vector at a position
     * 
     * @param position Position vector (x,y)
     * @return Force vector
     */
    virtual Eigen::Vector2d getForceVector(const Eigen::Vector2d& position) const = 0;
    
    /**
     * @brief Get the closest obstacle distance from a position
     * 
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param max_range Maximum search range
     * @return Distance to closest obstacle, returns max_range if no obstacle found
     */
    virtual double getDistanceToClosestObstacle(double x, double y, double max_range = 100.0) const = 0;
    
    /**
     * @brief Get nearby obstacle positions around a center point
     * 
     * @param center_x Center X-coordinate
     * @param center_y Center Y-coordinate
     * @param radius Search radius
     * @return Vector of obstacle positions
     */
    virtual std::vector<Eigen::Vector2d> getNearbyObstacles(
        double center_x, double center_y, double radius) const = 0;
    
    /**
     * @brief Update the map from sensor data
     * 
     * @param sensor_data Sensor data for map update
     * @return true if update succeeded, false otherwise
     */
    virtual bool updateFromSensorData(const void* sensor_data) = 0;
    
    /**
     * @brief Load map from a file
     * 
     * @param filename File path
     * @return true if loading succeeded, false otherwise
     */
    virtual bool loadFromFile(const std::string& filename) = 0;
    
    /**
     * @brief Save map to a file
     * 
     * @param filename File path
     * @return true if saving succeeded, false otherwise
     */
    virtual bool saveToFile(const std::string& filename) const = 0;
};

} // namespace asv_planning 