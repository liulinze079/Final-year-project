#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <asv_energy_aware_planning/msg/environment_map.hpp>
#include <asv_energy_aware_planning/msg/environmental_force.hpp>
#include <asv_energy_aware_planning/srv/environment_query.hpp>
#include <asv_energy_aware_planning/srv/map_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Dense>

namespace asv_planning {

/**
 * @class EnvironmentMap
 * @brief Manages the environmental map for ASV planning
 * 
 * This class manages the representation of the maritime environment,
 * including static obstacles, dynamic obstacles, environmental forces,
 * and depth information.
 */
class EnvironmentMap : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     * @param node_name Name of the ROS node
     */
    explicit EnvironmentMap(const std::string& node_name);
    
    /**
     * @brief Destructor
     */
    virtual ~EnvironmentMap() = default;
    
    /**
     * @brief Initialize the environment map
     * @return True if initialization was successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Check if a point is within an obstacle
     * @param point Point to check
     * @return True if point is within an obstacle, false otherwise
     */
    bool isPointInObstacle(const Eigen::Vector2d& point) const;
    
    /**
     * @brief Get distance to nearest obstacle
     * @param point Point to check from
     * @return Distance to nearest obstacle
     */
    double getDistanceToNearestObstacle(const Eigen::Vector2d& point) const;
    
    /**
     * @brief Check if a point is within a shipping lane
     * @param point Point to check
     * @return True if point is within a shipping lane, false otherwise
     */
    bool isPointInShippingLane(const Eigen::Vector2d& point) const;
    
    /**
     * @brief Get environmental force at a position
     * @param position Position to get force at
     * @return Environmental force at the position
     */
    asv_energy_aware_planning::msg::EnvironmentalForce getForceAt(
        const geometry_msgs::msg::Point& position) const;
    
    /**
     * @brief Get water depth at a position
     * @param position Position to get depth at
     * @return Water depth in meters
     */
    double getDepthAt(const geometry_msgs::msg::Point& position) const;
    
    /**
     * @brief Get energy cost multiplier at a position
     * @param position Position to get energy cost at
     * @return Energy cost multiplier (1.0 = normal)
     */
    double getEnergyCostAt(const geometry_msgs::msg::Point& position) const;
    
    /**
     * @brief Update the map with new information
     * @param update_type Type of update
     * @param data Update data
     * @param region Region to update
     * @return True if update was successful, false otherwise
     */
    bool updateMap(
        uint8_t update_type,
        const std::vector<uint8_t>& data,
        const std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>& region);
    
    /**
     * @brief Get the current environment map message
     * @return Environment map message
     */
    asv_energy_aware_planning::msg::EnvironmentMap::SharedPtr getMapMessage() const;

    /**
     * @brief Update from an occupancy grid
     * @param grid Occupancy grid
     */
    void updateFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid);
    
    /**
     * @brief Get the distance to the nearest obstacle
     * @param point Query point
     * @return Distance to the nearest obstacle (meters)
     */
    double getDistanceToNearestObstacle(const geometry_msgs::msg::Point& point) const;
    
    /**
     * @brief Get the distance to the nearest obstacle
     * @param point Query point
     * @return Distance to the nearest obstacle (meters)
     */
    double getDistanceToNearestObstacle(const Eigen::Vector2d& point) const;
    
    /**
     * @brief Get the current factor at a point and heading
     * @param point Query point
     * @param heading Travel heading (radians)
     * @return Current factor (> 1.0 if against current, < 1.0 if with current)
     */
    double getCurrentFactor(const geometry_msgs::msg::Point& point, double heading) const;
    
    /**
     * @brief Get the wave factor at a point
     * @param point Query point
     * @return Wave factor (larger with higher waves)
     */
    double getWaveFactor(const geometry_msgs::msg::Point& point) const;
    
    /**
     * @brief Check if a point is in a shipping lane
     * @param point Query point
     * @return True if in a shipping lane
     */
    bool isInShippingLane(const geometry_msgs::msg::Point& point) const;
    
    /**
     * @brief Get the current direction at a point
     * @param point Query point
     * @return Current direction (radians)
     */
    double getCurrentDirection(const geometry_msgs::msg::Point& point) const;
    
    /**
     * @brief Get the current magnitude at a point
     * @param point Query point
     * @return Current magnitude (m/s)
     */
    double getCurrentMagnitude(const geometry_msgs::msg::Point& point) const;
    
    /**
     * @brief Get the wave height at a point
     * @param point Query point
     * @return Wave height (meters)
     */
    double getWaveHeight(const geometry_msgs::msg::Point& point) const;
    
    /**
     * @brief Get the wave direction at a point
     * @param point Query point
     * @return Wave direction (radians)
     */
    double getWaveDirection(const geometry_msgs::msg::Point& point) const;
    
    /**
     * @brief Check if a point is occupied
     * @param point Query point
     * @return True if occupied
     */
    bool isOccupied(const geometry_msgs::msg::Point& point) const;
    
    /**
     * @brief Get the bounds of the map
     * @return [min_x, max_x, min_y, max_y]
     */
    std::array<double, 4> getBounds() const;

private:
    /**
     * @brief Callback for environment query service
     * @param request Request containing query parameters
     * @param response Response containing query results
     */
    void environmentQueryCallback(
        const std::shared_ptr<asv_energy_aware_planning::srv::EnvironmentQuery::Request> request,
        std::shared_ptr<asv_energy_aware_planning::srv::EnvironmentQuery::Response> response);
    
    /**
     * @brief Callback for map update service
     * @param request Request containing update parameters
     * @param response Response containing update results
     */
    void mapUpdateCallback(
        const std::shared_ptr<asv_energy_aware_planning::srv::MapUpdate::Request> request,
        std::shared_ptr<asv_energy_aware_planning::srv::MapUpdate::Response> response);
    
    /**
     * @brief Convert from global coordinates to grid coordinates
     * @param point Point in global coordinates
     * @return Grid coordinates (x, y)
     */
    std::pair<int, int> globalToGrid(const geometry_msgs::msg::Point& point) const;
    
    /**
     * @brief Convert from grid coordinates to global coordinates
     * @param x Grid x-coordinate
     * @param y Grid y-coordinate
     * @return Point in global coordinates
     */
    geometry_msgs::msg::Point gridToGlobal(int x, int y) const;
    
    // ROS services
    rclcpp::Service<asv_energy_aware_planning::srv::EnvironmentQuery>::SharedPtr query_service_;
    rclcpp::Service<asv_energy_aware_planning::srv::MapUpdate>::SharedPtr update_service_;
    
    // ROS publishers
    rclcpp::Publisher<asv_energy_aware_planning::msg::EnvironmentMap>::SharedPtr map_pub_;
    
    // Environment map
    asv_energy_aware_planning::msg::EnvironmentMap::SharedPtr map_;
    
    // Ship lanes
    std::vector<geometry_msgs::msg::Polygon> shipping_lanes_;
    
    // Mutex for thread safety
    mutable std::mutex mutex_;
    
    // Map representation
    std::vector<int8_t> occupancy_data_;
    int width_;
    int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    
    // Environmental data
    std::vector<double> current_directions_;
    std::vector<double> current_magnitudes_;
    std::vector<double> wave_heights_;
    std::vector<double> wave_directions_;
    std::vector<bool> shipping_lanes_;
    
    // Default values for regions without data
    double default_obstacle_distance_;
    double default_current_direction_;
    double default_current_magnitude_;
    double default_wave_height_;
    double default_wave_direction_;
};

} // namespace asv_planning 