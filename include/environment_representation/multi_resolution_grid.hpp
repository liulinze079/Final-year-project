#ifndef ASV_PLANNING_MULTI_RESOLUTION_GRID_HPP
#define ASV_PLANNING_MULTI_RESOLUTION_GRID_HPP

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace asv_planning {

/**
 * @class MultiResolutionGrid
 * @brief A multi-resolution grid representation of the environment using a quadtree structure.
 * 
 * This class provides a more efficient representation of the environment by using varying
 * resolution depending on the complexity of the area. Areas near obstacles or with complex
 * features use higher resolution, while open areas use lower resolution to save memory
 * and computation time.
 */
class MultiResolutionGrid {
public:
    /**
     * @brief Constructor for MultiResolutionGrid
     */
    MultiResolutionGrid();
    
    /**
     * @brief Default destructor
     */
    ~MultiResolutionGrid() = default;
    
    /**
     * @brief Updates the grid from an occupancy grid message
     * @param grid The occupancy grid message
     */
    void updateFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid);
    
    /**
     * @brief Gets the distance to the nearest obstacle from a given position
     * @param position The position to check from
     * @return The distance to the nearest obstacle
     */
    double getDistanceToNearestObstacle(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Gets the force field magnitude at a given position
     * @param position The position to check
     * @return The force field magnitude
     */
    double getForceFieldMagnitude(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Gets the force field direction at a given position
     * @param position The position to check
     * @return The force field direction in radians
     */
    double getForceFieldDirection(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Gets the current vector at a given position
     * @param position The position to check
     * @return The current vector (force field as a 2D vector)
     */
    Eigen::Vector2d getCurrentVector(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Gets the wave height at a given position
     * @param position The position to check
     * @return The wave height in meters
     */
    double getWaveHeight(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Gets the wave direction at a given position
     * @param position The position to check
     * @return The wave direction in radians
     */
    double getWaveDirection(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Checks if a position is in a shipping lane
     * @param position The position to check
     * @return True if in shipping lane, false otherwise
     */
    bool isInShippingLane(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Checks if a position is occupied by an obstacle
     * @param position The position to check
     * @return True if position is occupied, false otherwise
     */
    bool isOccupied(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Gets the cell size at a given position
     * @param position The position to check
     * @return The cell size in meters
     */
    double getCellSize(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Sets force field data (currents)
     * @param magnitudes Force field magnitudes
     * @param directions Force field directions in radians
     */
    void setForceField(
        const std::vector<double>& magnitudes,
        const std::vector<double>& directions);
    
    /**
     * @brief Sets wave data
     * @param heights Wave heights in meters
     * @param directions Wave directions in radians
     */
    void setWaveData(
        const std::vector<double>& heights,
        const std::vector<double>& directions);
    
    /**
     * @brief Sets shipping lane data
     * @param lanes Boolean vector indicating shipping lanes
     */
    void setShippingLanes(const std::vector<bool>& lanes);
    
    /**
     * @brief Gets the dimensions of the grid
     * @return A pair of width and height
     */
    std::pair<int, int> getDimensions() const;
    
    /**
     * @brief Gets the origin of the grid
     * @return A pair of x and y coordinates
     */
    std::pair<double, double> getOrigin() const;
    
    /**
     * @brief Gets the base resolution of the grid
     * @return The base resolution in meters
     */
    double getBaseResolution() const;
    
    /**
     * @brief Gets the resolution of the grid
     * @return The resolution in meters (same as base resolution)
     */
    double getResolution() const;
    
private:
    // Quadtree node structure
    struct QuadNode {
        bool is_leaf;
        int level;
        double min_x, min_y, max_x, max_y;
        uint8_t occupancy;  // 0 = free, 1 = mixed, 2 = occupied
        std::vector<std::shared_ptr<QuadNode>> children;
    };
    
    /**
     * @brief Converts world coordinates to grid coordinates
     * @param world_x X coordinate in world frame
     * @param world_y Y coordinate in world frame
     * @param grid_x Output grid X coordinate
     * @param grid_y Output grid Y coordinate
     * @return True if coordinates are within grid bounds, false otherwise
     */
    bool worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const;
    
    /**
     * @brief Converts grid coordinates to world coordinates
     * @param grid_x Grid X coordinate
     * @param grid_y Grid Y coordinate
     * @param world_x Output X coordinate in world frame
     * @param world_y Output Y coordinate in world frame
     */
    void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const;
    
    /**
     * @brief Gets the cell index from grid coordinates
     * @param grid_x Grid X coordinate
     * @param grid_y Grid Y coordinate
     * @return The 1D index in the data array, or -1 if invalid
     */
    int getCellIndex(int grid_x, int grid_y) const;
    
    /**
     * @brief Builds the quadtree representation
     */
    void buildQuadtree();
    
    /**
     * @brief Computes the distance field
     */
    void computeDistanceField();
    
    // Grid properties
    int width_;
    int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    int max_depth_;
    
    // Occupancy and distance data
    std::vector<int8_t> occupancy_data_;
    std::vector<double> distance_field_;
    
    // Environmental data
    std::vector<double> force_magnitudes_;
    std::vector<double> force_directions_;
    std::vector<double> wave_heights_;
    std::vector<double> wave_directions_;
    std::vector<bool> shipping_lanes_;
    
    // Default values
    double default_distance_;
    double default_force_magnitude_;
    double default_force_direction_;
    double default_wave_height_;
    double default_wave_direction_;
    
    // Quadtree root
    std::shared_ptr<QuadNode> quadtree_root_;
};

} // namespace asv_planning

#endif // ASV_PLANNING_MULTI_RESOLUTION_GRID_HPP 