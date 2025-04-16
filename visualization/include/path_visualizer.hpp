#pragma once

#include <vector>
#include <Eigen/Core>
#include <string>
#include <memory>
#include <map>

namespace asv_planning {

/**
 * @class PathVisualizer
 * @brief Visualizes planned paths and related data for ASVs
 * 
 * This class provides functionality to visualize planned paths, environmental
 * data, and other relevant information for maritime path planning.
 */
class PathVisualizer {
public:
    /**
     * @brief Constructor
     * @param visualization_server ROS visualization server (e.g., rviz)
     */
    PathVisualizer(const std::string& visualization_server = "");
    
    /**
     * @brief Destructor
     */
    ~PathVisualizer() = default;
    
    /**
     * @brief Visualize a planned path
     * @param path Vector of waypoints in the path
     * @param path_id Identifier for the path
     * @param color RGB color for the path (r,g,b values between 0-1)
     * @param thickness Line thickness
     */
    void visualizePath(
        const std::vector<Eigen::Vector2d>& path,
        const std::string& path_id,
        const std::vector<double>& color = {0.0, 0.7, 0.0},
        double thickness = 0.5);
    
    /**
     * @brief Visualize multiple paths for comparison
     * @param paths Vector of paths (each a vector of waypoints)
     * @param path_ids Vector of path identifiers
     * @param colors Vector of RGB colors for paths
     * @param thicknesses Vector of line thicknesses
     */
    void compareMultiplePaths(
        const std::vector<std::vector<Eigen::Vector2d>>& paths,
        const std::vector<std::string>& path_ids,
        const std::vector<std::vector<double>>& colors,
        const std::vector<double>& thicknesses);
    
    /**
     * @brief Visualize obstacles in the environment
     * @param obstacle_map Obstacle map or representation
     * @param color RGB color for obstacles
     * @param opacity Opacity for visualization (0-1)
     */
    void visualizeObstacles(
        const std::shared_ptr<void>& obstacle_map,
        const std::vector<double>& color = {0.7, 0.0, 0.0},
        double opacity = 0.7);
    
    /**
     * @brief Visualize current force field
     * @param start_pos Start position for the force field grid
     * @param end_pos End position for the force field grid
     * @param resolution Grid resolution
     * @param current_field Function to evaluate current vector at each position
     * @param scale Scale factor for the arrow size
     * @param color RGB color for the force field
     */
    void visualizeCurrentField(
        const Eigen::Vector2d& start_pos,
        const Eigen::Vector2d& end_pos,
        double resolution,
        std::function<Eigen::Vector2d(const Eigen::Vector2d&)> current_field,
        double scale = 1.0,
        const std::vector<double>& color = {0.0, 0.0, 0.7});
    
    /**
     * @brief Visualize energy consumption along a path
     * @param path Vector of waypoints in the path
     * @param energy_values Energy consumption at each path segment
     * @param path_id Identifier for the path
     * @param color_low RGB color for low energy consumption
     * @param color_high RGB color for high energy consumption
     */
    void visualizeEnergyConsumption(
        const std::vector<Eigen::Vector2d>& path,
        const std::vector<double>& energy_values,
        const std::string& path_id,
        const std::vector<double>& color_low = {0.0, 1.0, 0.0},
        const std::vector<double>& color_high = {1.0, 0.0, 0.0});
    
    /**
     * @brief Visualize shipping lanes
     * @param shipping_lanes Vector of shipping lane polygons
     * @param color RGB color for shipping lanes
     * @param opacity Opacity for visualization (0-1)
     */
    void visualizeShippingLanes(
        const std::vector<std::vector<Eigen::Vector2d>>& shipping_lanes,
        const std::vector<double>& color = {0.0, 0.5, 0.5},
        double opacity = 0.3);
    
    /**
     * @brief Visualize restricted navigation areas
     * @param restricted_areas Vector of restricted area polygons
     * @param color RGB color for restricted areas
     * @param opacity Opacity for visualization (0-1)
     */
    void visualizeRestrictedAreas(
        const std::vector<std::vector<Eigen::Vector2d>>& restricted_areas,
        const std::vector<double>& color = {0.8, 0.0, 0.0},
        double opacity = 0.5);
    
    /**
     * @brief Save visualization to file
     * @param filename Output filename
     * @param width Image width
     * @param height Image height
     * @param dpi DPI for the image
     * @return True if save was successful, false otherwise
     */
    bool saveVisualization(
        const std::string& filename,
        int width = 1200,
        int height = 800,
        int dpi = 100);
    
    /**
     * @brief Clear all visualizations
     */
    void clearVisualizations();

private:
    // Internal state
    std::string visualization_server_;
    std::map<std::string, int> visualization_markers_;
    int marker_id_counter_;
    
    // Helper methods
    std::vector<double> interpolateColors(
        const std::vector<double>& color1,
        const std::vector<double>& color2,
        double t);
    
    void publishMarkers();
};

} // namespace asv_planning 