#pragma once

#include <vector>
#include <Eigen/Core>
#include <memory>
#include <string>

namespace asv_planning {

/**
 * @class PathQuality
 * @brief Evaluates the quality of planned paths for ASVs
 * 
 * This class provides functionality to measure and analyze the quality
 * of planned paths for ASVs. It includes metrics for smoothness, safety,
 * feasibility, and adherence to constraints.
 */
class PathQuality {
public:
    /**
     * @brief Constructor
     * @param config_file Path to configuration file
     */
    PathQuality(const std::string& config_file = "");
    
    /**
     * @brief Destructor
     */
    ~PathQuality() = default;
    
    /**
     * @brief Calculate smoothness of a path
     * @param path Vector of waypoints in the path
     * @return Smoothness score (higher is smoother)
     */
    double calculateSmoothness(const std::vector<Eigen::Vector2d>& path);
    
    /**
     * @brief Calculate safety score of a path
     * @param path Vector of waypoints in the path
     * @param obstacle_map Obstacle map or representation
     * @return Safety score (higher is safer)
     */
    double calculateSafetyScore(
        const std::vector<Eigen::Vector2d>& path,
        const std::shared_ptr<void>& obstacle_map); // Generic pointer to obstacle representation
    
    /**
     * @brief Check if a path is feasible for the ASV
     * @param path Vector of waypoints in the path
     * @param velocities Velocities at each waypoint
     * @param turning_radius Minimum turning radius of the ASV
     * @return True if the path is feasible, false otherwise
     */
    bool isPathFeasible(
        const std::vector<Eigen::Vector2d>& path,
        const std::vector<double>& velocities,
        double turning_radius);
    
    /**
     * @brief Calculate curvature at each point in the path
     * @param path Vector of waypoints in the path
     * @return Vector of curvature values
     */
    std::vector<double> calculateCurvature(const std::vector<Eigen::Vector2d>& path);
    
    /**
     * @brief Calculate heading changes in the path
     * @param path Vector of waypoints in the path
     * @return Vector of heading changes in radians
     */
    std::vector<double> calculateHeadingChanges(const std::vector<Eigen::Vector2d>& path);
    
    /**
     * @brief Check if a path adheres to shipping lane constraints
     * @param path Vector of waypoints in the path
     * @param shipping_lanes Vector of shipping lane polygons
     * @return Percentage of path within shipping lanes
     */
    double calculateShippingLaneAdherence(
        const std::vector<Eigen::Vector2d>& path,
        const std::vector<std::vector<Eigen::Vector2d>>& shipping_lanes);
    
    /**
     * @brief Generate a comprehensive quality report for a path
     * @param path Vector of waypoints in the path
     * @param velocities Velocities at each waypoint
     * @param obstacle_map Obstacle map
     * @param shipping_lanes Shipping lane polygons
     * @param output_file Path to output file (empty for console output)
     */
    void generateQualityReport(
        const std::vector<Eigen::Vector2d>& path,
        const std::vector<double>& velocities,
        const std::shared_ptr<void>& obstacle_map,
        const std::vector<std::vector<Eigen::Vector2d>>& shipping_lanes,
        const std::string& output_file = "");
    
    /**
     * @brief Compare two paths and determine which is better
     * @param path1 First path
     * @param path2 Second path
     * @param obstacle_map Obstacle map
     * @param weights Weights for different quality factors
     * @return Positive if path1 is better, negative if path2 is better, 0 if equal
     */
    double comparePaths(
        const std::vector<Eigen::Vector2d>& path1,
        const std::vector<Eigen::Vector2d>& path2,
        const std::shared_ptr<void>& obstacle_map,
        const std::vector<double>& weights = {1.0, 1.0, 1.0});

private:
    // Configuration parameters
    double smoothness_weight_;
    double safety_weight_;
    double shipping_lane_weight_;
    
    // Helper methods
    double calculateMinDistanceToObstacles(
        const Eigen::Vector2d& point,
        const std::shared_ptr<void>& obstacle_map);
    
    bool isPointInPolygon(
        const Eigen::Vector2d& point,
        const std::vector<Eigen::Vector2d>& polygon);
};

} // namespace asv_planning 