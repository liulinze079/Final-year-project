#include "../../include/environment_representation/environment_map.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace asv_planning {

EnvironmentMap::EnvironmentMap()
    : width_(0),
      height_(0),
      resolution_(1.0),
      origin_x_(0.0),
      origin_y_(0.0),
      default_obstacle_distance_(100.0),
      default_current_direction_(0.0),
      default_current_magnitude_(0.0),
      default_wave_height_(0.0),
      default_wave_direction_(0.0) {
}

void EnvironmentMap::updateFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid) {
    // Update grid properties
    width_ = grid.info.width;
    height_ = grid.info.height;
    resolution_ = grid.info.resolution;
    origin_x_ = grid.info.origin.position.x;
    origin_y_ = grid.info.origin.position.y;
    
    // Copy occupancy data
    occupancy_data_ = grid.data;
    
    // Initialize environmental data if needed
    if (current_directions_.size() != occupancy_data_.size()) {
        current_directions_.resize(occupancy_data_.size(), default_current_direction_);
        current_magnitudes_.resize(occupancy_data_.size(), default_current_magnitude_);
        wave_heights_.resize(occupancy_data_.size(), default_wave_height_);
        wave_directions_.resize(occupancy_data_.size(), default_wave_direction_);
        shipping_lanes_.resize(occupancy_data_.size(), false);
    }
}

double EnvironmentMap::getDistanceToNearestObstacle(const geometry_msgs::msg::Point& point) const {
    // Convert world coordinates to grid coordinates
    int grid_x, grid_y;
    if (!worldToGrid(point.x, point.y, grid_x, grid_y)) {
        return default_obstacle_distance_;  // Outside grid bounds
    }
    
    // If the point itself is an obstacle, return 0
    if (isOccupied({point})) {
        return 0.0;
    }
    
    // Simple BFS to find nearest obstacle
    const int max_search_distance = 50;  // Limit search radius
    double min_distance = default_obstacle_distance_;
    
    for (int y = std::max(0, grid_y - max_search_distance); 
         y < std::min(height_, grid_y + max_search_distance); ++y) {
        for (int x = std::max(0, grid_x - max_search_distance); 
             x < std::min(width_, grid_x + max_search_distance); ++x) {
            
            // Check if this cell is an obstacle
            if (occupancy_data_[y * width_ + x] > 50) {  // Occupied threshold
                double world_x, world_y;
                gridToWorld(x, y, world_x, world_y);
                
                // Calculate Euclidean distance
                double dx = world_x - point.x;
                double dy = world_y - point.y;
                double distance = std::sqrt(dx * dx + dy * dy);
                
                min_distance = std::min(min_distance, distance);
            }
        }
    }
    
    return min_distance;
}

double EnvironmentMap::getDistanceToNearestObstacle(const Eigen::Vector2d& point) const {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    return getDistanceToNearestObstacle(p);
}

double EnvironmentMap::getCurrentFactor(const geometry_msgs::msg::Point& point, double heading) const {
    // Convert world coordinates to grid coordinates
    int grid_x, grid_y;
    if (!worldToGrid(point.x, point.y, grid_x, grid_y)) {
        return 1.0;  // Default value outside grid
    }
    
    // Get current direction and magnitude
    double current_direction = current_directions_[grid_y * width_ + grid_x];
    double current_magnitude = current_magnitudes_[grid_y * width_ + grid_x];
    
    // Calculate angle difference between heading and current
    double angle_diff = std::abs(heading - current_direction);
    while (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;
    
    // Calculate current factor:
    // - If moving against current (angle_diff near 180 deg): higher factor (e.g., 1.5)
    // - If moving with current (angle_diff near 0 deg): lower factor (e.g., 0.8)
    // - Neutral if perpendicular or no current
    if (current_magnitude < 0.01) {
        return 1.0;  // No significant current
    }
    
    // Normalize angle_diff to [0, 1] where 0 is with current, 1 is against
    double normalized_diff = angle_diff / M_PI;
    
    // Scale factor based on magnitude and direction difference
    // Formula: 1.0 + magnitude_effect * direction_effect
    // Where magnitude_effect scales with current strength
    // And direction_effect is 1 against current, -0.5 with current
    double magnitude_effect = current_magnitude * 0.5;  // Scale factor
    double direction_effect = 2.0 * normalized_diff - 0.5;  // Range: -0.5 to 1.5
    
    return 1.0 + magnitude_effect * direction_effect;
}

double EnvironmentMap::getWaveFactor(const geometry_msgs::msg::Point& point) const {
    // Convert world coordinates to grid coordinates
    int grid_x, grid_y;
    if (!worldToGrid(point.x, point.y, grid_x, grid_y)) {
        return 1.0;  // Default value outside grid
    }
    
    // Get wave height
    double wave_height = wave_heights_[grid_y * width_ + grid_x];
    
    // Calculate wave factor (higher waves = higher factor)
    return 1.0 + wave_height * 0.1;  // 10% increase per meter of wave height
}

bool EnvironmentMap::isInShippingLane(const geometry_msgs::msg::Point& point) const {
    // Convert world coordinates to grid coordinates
    int grid_x, grid_y;
    if (!worldToGrid(point.x, point.y, grid_x, grid_y)) {
        return false;  // Outside grid is not in shipping lane
    }
    
    return shipping_lanes_[grid_y * width_ + grid_x];
}

double EnvironmentMap::getCurrentDirection(const geometry_msgs::msg::Point& point) const {
    // Convert world coordinates to grid coordinates
    int grid_x, grid_y;
    if (!worldToGrid(point.x, point.y, grid_x, grid_y)) {
        return default_current_direction_;  // Default value outside grid
    }
    
    return current_directions_[grid_y * width_ + grid_x];
}

double EnvironmentMap::getCurrentMagnitude(const geometry_msgs::msg::Point& point) const {
    // Convert world coordinates to grid coordinates
    int grid_x, grid_y;
    if (!worldToGrid(point.x, point.y, grid_x, grid_y)) {
        return default_current_magnitude_;  // Default value outside grid
    }
    
    return current_magnitudes_[grid_y * width_ + grid_x];
}

double EnvironmentMap::getWaveHeight(const geometry_msgs::msg::Point& point) const {
    // Convert world coordinates to grid coordinates
    int grid_x, grid_y;
    if (!worldToGrid(point.x, point.y, grid_x, grid_y)) {
        return default_wave_height_;  // Default value outside grid
    }
    
    return wave_heights_[grid_y * width_ + grid_x];
}

double EnvironmentMap::getWaveDirection(const geometry_msgs::msg::Point& point) const {
    // Convert world coordinates to grid coordinates
    int grid_x, grid_y;
    if (!worldToGrid(point.x, point.y, grid_x, grid_y)) {
        return default_wave_direction_;  // Default value outside grid
    }
    
    return wave_directions_[grid_y * width_ + grid_x];
}

bool EnvironmentMap::isOccupied(const geometry_msgs::msg::Point& point) const {
    // Convert world coordinates to grid coordinates
    int grid_x, grid_y;
    if (!worldToGrid(point.x, point.y, grid_x, grid_y)) {
        return false;  // Outside grid is considered unoccupied
    }
    
    // Check occupancy (threshold of 50 for occupied)
    return occupancy_data_[grid_y * width_ + grid_x] > 50;
}

std::array<double, 4> EnvironmentMap::getBounds() const {
    double min_x = origin_x_;
    double min_y = origin_y_;
    double max_x = origin_x_ + width_ * resolution_;
    double max_y = origin_y_ + height_ * resolution_;
    
    return {min_x, max_x, min_y, max_y};
}

bool EnvironmentMap::worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const {
    // Convert world coordinates to grid coordinates
    grid_x = static_cast<int>((world_x - origin_x_) / resolution_);
    grid_y = static_cast<int>((world_y - origin_y_) / resolution_);
    
    // Check if the coordinates are within grid bounds
    return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

void EnvironmentMap::gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const {
    // Convert grid coordinates to world coordinates (cell center)
    world_x = origin_x_ + (grid_x + 0.5) * resolution_;
    world_y = origin_y_ + (grid_y + 0.5) * resolution_;
}

} // namespace asv_planning 