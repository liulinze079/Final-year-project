#pragma once

#include <vector>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <map>

namespace asv_planning {

/**
 * @class ObstacleAvoidance
 * @brief Evaluates obstacle avoidance performance of ASV planning algorithms
 * 
 * This class provides functionality to evaluate how well planning algorithms
 * avoid obstacles, including static obstacles, dynamic obstacles, and
 * restricted areas in maritime environments.
 */
class ObstacleAvoidance {
public:
    /**
     * @brief Constructor
     * @param config_file Path to configuration file
     */
    ObstacleAvoidance(const std::string& config_file = "");
    
    /**
     * @brief Destructor
     */
    ~ObstacleAvoidance() = default;
    
    /**
     * @brief Calculate minimum safety distance to obstacles along a path
     * @param path Vector of waypoints in the path
     * @param obstacle_map Obstacle map or representation
     * @return Minimum distance to any obstacle along the path
     */
    double calculateMinSafetyDistance(
        const std::vector<Eigen::Vector2d>& path,
        const std::shared_ptr<void>& obstacle_map);
    
    /**
     * @brief Evaluate path for collisions with static obstacles
     * @param path Vector of waypoints in the path
     * @param obstacle_map Obstacle map
     * @param vessel_radius ASV radius or safety envelope
     * @return True if path is collision-free, false otherwise
     */
    bool isPathCollisionFree(
        const std::vector<Eigen::Vector2d>& path,
        const std::shared_ptr<void>& obstacle_map,
        double vessel_radius);
    
    /**
     * @brief Evaluate path for collisions with dynamic obstacles
     * @param path Vector of waypoints in the path
     * @param waypoint_times Times at which ASV reaches each waypoint
     * @param dynamic_obstacles Map of dynamic obstacles with trajectories
     * @param vessel_radius ASV radius or safety envelope
     * @return True if path is collision-free with dynamic obstacles, false otherwise
     */
    bool isPathDynamicCollisionFree(
        const std::vector<Eigen::Vector2d>& path,
        const std::vector<double>& waypoint_times,
        const std::vector<std::pair<std::vector<Eigen::Vector2d>, std::vector<double>>>& dynamic_obstacles,
        double vessel_radius);
    
    /**
     * @brief Calculate risk level along a path
     * @param path Vector of waypoints in the path
     * @param obstacle_map Obstacle map
     * @param traffic_map Maritime traffic density map
     * @return Risk scores along the path (0-1 scale where 1 is highest risk)
     */
    std::vector<double> calculateRiskLevels(
        const std::vector<Eigen::Vector2d>& path,
        const std::shared_ptr<void>& obstacle_map,
        const std::shared_ptr<void>& traffic_map);
    
    /**
     * @brief Check if path violates restricted navigation areas
     * @param path Vector of waypoints in the path
     * @param restricted_areas Vector of restricted area polygons
     * @return Percentage of path inside restricted areas
     */
    double calculateRestrictedAreaViolation(
        const std::vector<Eigen::Vector2d>& path,
        const std::vector<std::vector<Eigen::Vector2d>>& restricted_areas);
    
    /**
     * @brief Generate a comprehensive obstacle avoidance report
     * @param path Vector of waypoints in the path
     * @param waypoint_times Times at which ASV reaches each waypoint
     * @param obstacle_map Static obstacle map
     * @param dynamic_obstacles Dynamic obstacles with trajectories
     * @param restricted_areas Restricted navigation areas
     * @param vessel_radius ASV radius or safety envelope
     * @param output_file Path to output file (empty for console output)
     */
    void generateObstacleReport(
        const std::vector<Eigen::Vector2d>& path,
        const std::vector<double>& waypoint_times,
        const std::shared_ptr<void>& obstacle_map,
        const std::vector<std::pair<std::vector<Eigen::Vector2d>, std::vector<double>>>& dynamic_obstacles,
        const std::vector<std::vector<Eigen::Vector2d>>& restricted_areas,
        double vessel_radius,
        const std::string& output_file = "");

private:
    // Configuration parameters
    double safety_distance_threshold_;
    double collision_risk_threshold_;
    
    // Helper methods
    bool detectStaticCollision(
        const Eigen::Vector2d& point,
        const std::shared_ptr<void>& obstacle_map,
        double vessel_radius);
    
    bool detectDynamicCollision(
        const Eigen::Vector2d& asv_position, double asv_time,
        const std::vector<Eigen::Vector2d>& obstacle_trajectory,
        const std::vector<double>& obstacle_times,
        double vessel_radius);
    
    double interpolateObstaclePosition(
        const std::vector<Eigen::Vector2d>& obstacle_trajectory,
        const std::vector<double>& obstacle_times,
        double query_time);
    
    bool isPointInPolygon(
        const Eigen::Vector2d& point,
        const std::vector<Eigen::Vector2d>& polygon);
};

} // namespace asv_planning 