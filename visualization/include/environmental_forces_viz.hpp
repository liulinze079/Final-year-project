#pragma once

#include <vector>
#include <Eigen/Core>
#include <string>
#include <memory>
#include <functional>

namespace asv_planning {

/**
 * @class EnvironmentalForcesViz
 * @brief Visualizes environmental forces for maritime environments
 * 
 * This class provides functionality to visualize environmental forces
 * like currents, waves, and wind that affect ASV motion and energy consumption.
 */
class EnvironmentalForcesViz {
public:
    /**
     * @brief Constructor
     * @param visualization_server ROS visualization server (e.g., rviz)
     */
    EnvironmentalForcesViz(const std::string& visualization_server = "");
    
    /**
     * @brief Destructor
     */
    ~EnvironmentalForcesViz() = default;
    
    /**
     * @brief Visualize current force field
     * @param bounds Bounds for visualization [min_x, min_y, max_x, max_y]
     * @param resolution Grid resolution
     * @param current_field Function to evaluate current vector at each position
     * @param scale Scale factor for the arrow size
     * @param color_map Function mapping magnitude to color (optional)
     */
    void visualizeCurrentField(
        const std::vector<double>& bounds,
        double resolution,
        std::function<Eigen::Vector2d(const Eigen::Vector2d&)> current_field,
        double scale = 1.0,
        std::function<std::vector<double>(double)> color_map = nullptr);
    
    /**
     * @brief Visualize wave field
     * @param bounds Bounds for visualization [min_x, min_y, max_x, max_y]
     * @param resolution Grid resolution
     * @param wave_height_field Function to evaluate wave height at each position
     * @param wave_direction_field Function to evaluate wave direction at each position
     * @param height_scale Scale factor for height visualization
     * @param color_map Function mapping wave height to color (optional)
     */
    void visualizeWaveField(
        const std::vector<double>& bounds,
        double resolution,
        std::function<double(const Eigen::Vector2d&)> wave_height_field,
        std::function<double(const Eigen::Vector2d&)> wave_direction_field,
        double height_scale = 1.0,
        std::function<std::vector<double>(double)> color_map = nullptr);
    
    /**
     * @brief Visualize wind field
     * @param bounds Bounds for visualization [min_x, min_y, max_x, max_y]
     * @param resolution Grid resolution
     * @param wind_field Function to evaluate wind vector at each position
     * @param scale Scale factor for the arrow size
     * @param color_map Function mapping magnitude to color (optional)
     */
    void visualizeWindField(
        const std::vector<double>& bounds,
        double resolution,
        std::function<Eigen::Vector2d(const Eigen::Vector2d&)> wind_field,
        double scale = 1.0,
        std::function<std::vector<double>(double)> color_map = nullptr);
    
    /**
     * @brief Visualize energy cost field (how much energy would be consumed to traverse)
     * @param bounds Bounds for visualization [min_x, min_y, max_x, max_y]
     * @param resolution Grid resolution
     * @param energy_cost_field Function to evaluate energy cost at each position
     * @param heading Direction of travel for energy calculation
     * @param color_low RGB color for low energy cost
     * @param color_high RGB color for high energy cost
     */
    void visualizeEnergyCostField(
        const std::vector<double>& bounds,
        double resolution,
        std::function<double(const Eigen::Vector2d&, double)> energy_cost_field,
        double heading,
        const std::vector<double>& color_low = {0.0, 1.0, 0.0},
        const std::vector<double>& color_high = {1.0, 0.0, 0.0});
    
    /**
     * @brief Visualize force effect on an ASV
     * @param asv_position ASV position
     * @param asv_heading ASV heading
     * @param asv_velocity ASV velocity
     * @param force_vector Total force vector
     * @param scale Scale factor for visualization
     */
    void visualizeForceOnASV(
        const Eigen::Vector2d& asv_position,
        double asv_heading,
        double asv_velocity,
        const Eigen::Vector3d& force_vector,
        double scale = 1.0);
    
    /**
     * @brief Create an animated visualization of environmental forces
     * @param bounds Bounds for visualization [min_x, min_y, max_x, max_y]
     * @param resolution Grid resolution
     * @param force_field_time Function to evaluate force field at each position and time
     * @param duration Duration of animation in seconds
     * @param frame_rate Frame rate for animation
     * @param output_path Path to save animation frames
     * @param show_animation Whether to display animation during creation
     */
    void createForceFieldAnimation(
        const std::vector<double>& bounds,
        double resolution,
        std::function<Eigen::Vector2d(const Eigen::Vector2d&, double)> force_field_time,
        double duration,
        double frame_rate,
        const std::string& output_path,
        bool show_animation = false);
    
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
    int marker_id_counter_;
    
    // Default color maps
    std::vector<double> defaultMagnitudeToColor(double magnitude, double max_magnitude = 2.0);
    std::vector<double> defaultWaveHeightToColor(double height, double max_height = 3.0);
    
    // Helper methods
    std::vector<double> interpolateColors(
        const std::vector<double>& color1,
        const std::vector<double>& color2,
        double t);
    
    void publishMarkers();
};

} // namespace asv_planning 