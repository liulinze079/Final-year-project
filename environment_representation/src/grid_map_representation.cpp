#include "../include/map_representation.hpp"
#include "../include/grid_map_representation.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

namespace asv_planning {

GridMapRepresentation::GridMapRepresentation(double resolution, 
                                           double width, 
                                           double height, 
                                           double origin_x, 
                                           double origin_y)
    : resolution_(resolution),
      width_(width),
      height_(height),
      origin_x_(origin_x),
      origin_y_(origin_y) {
    
    // Calculate grid dimensions
    int grid_width = static_cast<int>(std::ceil(width / resolution));
    int grid_height = static_cast<int>(std::ceil(height / resolution));
    
    // Initialize occupancy grid
    occupancy_grid_.resize(grid_width, std::vector<bool>(grid_height, false));
    
    // Initialize distance grid (for distance transform)
    distance_grid_.resize(grid_width, std::vector<double>(grid_height, std::numeric_limits<double>::max()));
    
    // Initialize force field grids
    force_magnitude_grid_.resize(grid_width, std::vector<double>(grid_height, 0.0));
    force_direction_grid_.resize(grid_width, std::vector<double>(grid_height, 0.0));
}

bool GridMapRepresentation::isWithinBounds(double x, double y) const {
    int grid_x, grid_y;
    worldToGrid(x, y, grid_x, grid_y);
    
    return grid_x >= 0 && grid_x < static_cast<int>(occupancy_grid_.size()) &&
           grid_y >= 0 && grid_y < static_cast<int>(occupancy_grid_[0].size());
}

bool GridMapRepresentation::isOccupied(double x, double y) const {
    int grid_x, grid_y;
    worldToGrid(x, y, grid_x, grid_y);
    
    if (!isWithinBounds(x, y)) {
        return true;  // Treat out-of-bounds as occupied
    }
    
    return occupancy_grid_[grid_x][grid_y];
}

bool GridMapRepresentation::isInCollision(double x, double y, double safety_distance) const {
    if (!isWithinBounds(x, y)) {
        return true;
    }
    
    int grid_x, grid_y;
    worldToGrid(x, y, grid_x, grid_y);
    
    // If the cell itself is occupied
    if (occupancy_grid_[grid_x][grid_y]) {
        return true;
    }
    
    // If safety distance is specified, check if within that distance to obstacles
    if (safety_distance > 0.0) {
        double distance = getDistanceToClosestObstacle(x, y);
        return distance < safety_distance;
    }
    
    return false;
}

bool GridMapRepresentation::isLineInCollision(
    double start_x, double start_y,
    double end_x, double end_y,
    double step_size,
    double safety_distance) const {
    
    // Calculate line length
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double length = std::sqrt(dx * dx + dy * dy);
    
    // Normalize direction
    double nx = dx / length;
    double ny = dy / length;
    
    // Check points along the line
    for (double d = 0.0; d <= length; d += step_size) {
        double x = start_x + nx * d;
        double y = start_y + ny * d;
        
        if (isInCollision(x, y, safety_distance)) {
            return true;
        }
    }
    
    // Check the endpoint
    return isInCollision(end_x, end_y, safety_distance);
}

double GridMapRepresentation::getForceFieldMagnitude(const Eigen::Vector2d& position) const {
    int grid_x, grid_y;
    worldToGrid(position.x(), position.y(), grid_x, grid_y);
    
    if (!isWithinBounds(position.x(), position.y())) {
        return 0.0;
    }
    
    return force_magnitude_grid_[grid_x][grid_y];
}

double GridMapRepresentation::getForceFieldDirection(const Eigen::Vector2d& position) const {
    int grid_x, grid_y;
    worldToGrid(position.x(), position.y(), grid_x, grid_y);
    
    if (!isWithinBounds(position.x(), position.y())) {
        return 0.0;
    }
    
    return force_direction_grid_[grid_x][grid_y];
}

Eigen::Vector2d GridMapRepresentation::getForceVector(const Eigen::Vector2d& position) const {
    double magnitude = getForceFieldMagnitude(position);
    double direction = getForceFieldDirection(position);
    
    return Eigen::Vector2d(magnitude * std::cos(direction), 
                          magnitude * std::sin(direction));
}

double GridMapRepresentation::getDistanceToClosestObstacle(double x, double y, double max_range) const {
    int grid_x, grid_y;
    worldToGrid(x, y, grid_x, grid_y);
    
    if (!isWithinBounds(x, y)) {
        return 0.0;  // Out of bounds is treated as collision
    }
    
    // If the distance grid is computed, use it
    if (!distance_grid_.empty()) {
        return distance_grid_[grid_x][grid_y] * resolution_;
    }
    
    // Otherwise, do a simple search up to max range
    double min_distance = max_range;
    int search_radius = static_cast<int>(std::ceil(max_range / resolution_));
    
    for (int i = std::max(0, grid_x - search_radius); 
         i < std::min(static_cast<int>(occupancy_grid_.size()), grid_x + search_radius); ++i) {
        for (int j = std::max(0, grid_y - search_radius); 
             j < std::min(static_cast<int>(occupancy_grid_[0].size()), grid_y + search_radius); ++j) {
            
            if (occupancy_grid_[i][j]) {
                double world_x, world_y;
                gridToWorld(i, j, world_x, world_y);
                
                double dist = std::sqrt(std::pow(world_x - x, 2) + std::pow(world_y - y, 2));
                min_distance = std::min(min_distance, dist);
            }
        }
    }
    
    return min_distance;
}

std::vector<Eigen::Vector2d> GridMapRepresentation::getNearbyObstacles(
    double center_x, double center_y, double radius) const {
    
    std::vector<Eigen::Vector2d> obstacles;
    
    int grid_x, grid_y;
    worldToGrid(center_x, center_y, grid_x, grid_y);
    
    int search_radius = static_cast<int>(std::ceil(radius / resolution_));
    
    for (int i = std::max(0, grid_x - search_radius); 
         i < std::min(static_cast<int>(occupancy_grid_.size()), grid_x + search_radius); ++i) {
        for (int j = std::max(0, grid_y - search_radius); 
             j < std::min(static_cast<int>(occupancy_grid_[0].size()), grid_y + search_radius); ++j) {
            
            if (occupancy_grid_[i][j]) {
                double world_x, world_y;
                gridToWorld(i, j, world_x, world_y);
                
                double dist = std::sqrt(std::pow(world_x - center_x, 2) + std::pow(world_y - center_y, 2));
                
                if (dist <= radius) {
                    obstacles.emplace_back(world_x, world_y);
                }
            }
        }
    }
    
    return obstacles;
}

bool GridMapRepresentation::updateFromSensorData(const void* sensor_data) {
    // This would depend on the specific sensor data format
    // Placeholder implementation
    std::cout << "Warning: updateFromSensorData not implemented" << std::endl;
    return false;
}

bool GridMapRepresentation::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open map file: " << filename << std::endl;
        return false;
    }
    
    // Read map metadata
    file >> resolution_ >> width_ >> height_ >> origin_x_ >> origin_y_;
    
    // Calculate grid dimensions
    int grid_width = static_cast<int>(std::ceil(width_ / resolution_));
    int grid_height = static_cast<int>(std::ceil(height_ / resolution_));
    
    // Resize grids
    occupancy_grid_.resize(grid_width, std::vector<bool>(grid_height, false));
    force_magnitude_grid_.resize(grid_width, std::vector<double>(grid_height, 0.0));
    force_direction_grid_.resize(grid_width, std::vector<double>(grid_height, 0.0));
    
    // Read occupancy grid
    for (int j = 0; j < grid_height; ++j) {
        for (int i = 0; i < grid_width; ++i) {
            int occupied;
            file >> occupied;
            occupancy_grid_[i][j] = (occupied != 0);
        }
    }
    
    // Read force field (if available)
    if (!file.eof()) {
        for (int j = 0; j < grid_height; ++j) {
            for (int i = 0; i < grid_width; ++i) {
                file >> force_magnitude_grid_[i][j] >> force_direction_grid_[i][j];
            }
        }
    }
    
    // Compute distance grid
    computeDistanceGrid();
    
    return true;
}

bool GridMapRepresentation::saveToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }
    
    // Write map metadata
    file << resolution_ << " " << width_ << " " << height_ << " " 
         << origin_x_ << " " << origin_y_ << std::endl;
    
    // Write occupancy grid
    int grid_width = static_cast<int>(occupancy_grid_.size());
    int grid_height = static_cast<int>(occupancy_grid_[0].size());
    
    for (int j = 0; j < grid_height; ++j) {
        for (int i = 0; i < grid_width; ++i) {
            file << (occupancy_grid_[i][j] ? "1 " : "0 ");
        }
        file << std::endl;
    }
    
    // Write force field
    for (int j = 0; j < grid_height; ++j) {
        for (int i = 0; i < grid_width; ++i) {
            file << force_magnitude_grid_[i][j] << " " << force_direction_grid_[i][j] << " ";
        }
        file << std::endl;
    }
    
    return true;
}

void GridMapRepresentation::setOccupied(double x, double y, bool occupied) {
    int grid_x, grid_y;
    worldToGrid(x, y, grid_x, grid_y);
    
    if (isWithinBounds(x, y)) {
        occupancy_grid_[grid_x][grid_y] = occupied;
        
        // Invalidate distance grid to force recomputation
        distance_grid_.clear();
    }
}

void GridMapRepresentation::setForceField(double x, double y, double magnitude, double direction) {
    int grid_x, grid_y;
    worldToGrid(x, y, grid_x, grid_y);
    
    if (isWithinBounds(x, y)) {
        force_magnitude_grid_[grid_x][grid_y] = magnitude;
        force_direction_grid_[grid_x][grid_y] = direction;
    }
}

void GridMapRepresentation::computeDistanceGrid() {
    int grid_width = static_cast<int>(occupancy_grid_.size());
    int grid_height = static_cast<int>(occupancy_grid_[0].size());
    
    // Initialize distance grid
    distance_grid_.resize(grid_width, std::vector<double>(grid_height, std::numeric_limits<double>::max()));
    
    // First pass: Set occupied cells to 0 and initialize neighbors
    for (int i = 0; i < grid_width; ++i) {
        for (int j = 0; j < grid_height; ++j) {
            if (occupancy_grid_[i][j]) {
                distance_grid_[i][j] = 0.0;
            } else {
                // Initialize with distance to nearest occupied neighbor (if any)
                for (int di = -1; di <= 1; ++di) {
                    for (int dj = -1; dj <= 1; ++dj) {
                        if (di == 0 && dj == 0) continue;
                        
                        int ni = i + di;
                        int nj = j + dj;
                        
                        if (ni >= 0 && ni < grid_width && nj >= 0 && nj < grid_height && 
                            occupancy_grid_[ni][nj]) {
                            double dist = (di != 0 && dj != 0) ? 1.414 : 1.0; // Diagonal vs cardinal
                            distance_grid_[i][j] = std::min(distance_grid_[i][j], dist);
                        }
                    }
                }
            }
        }
    }
    
    // Multiple passes to propagate distances
    bool changed = true;
    while (changed) {
        changed = false;
        
        for (int i = 0; i < grid_width; ++i) {
            for (int j = 0; j < grid_height; ++j) {
                double current_dist = distance_grid_[i][j];
                
                // Skip occupied cells
                if (current_dist == 0.0) continue;
                
                // Check neighbors
                for (int di = -1; di <= 1; ++di) {
                    for (int dj = -1; dj <= 1; ++dj) {
                        if (di == 0 && dj == 0) continue;
                        
                        int ni = i + di;
                        int nj = j + dj;
                        
                        if (ni >= 0 && ni < grid_width && nj >= 0 && nj < grid_height) {
                            double neighbor_dist = distance_grid_[ni][nj];
                            double dist_inc = (di != 0 && dj != 0) ? 1.414 : 1.0; // Diagonal vs cardinal
                            
                            if (neighbor_dist + dist_inc < current_dist) {
                                distance_grid_[i][j] = neighbor_dist + dist_inc;
                                changed = true;
                                current_dist = distance_grid_[i][j];
                            }
                        }
                    }
                }
            }
        }
    }
}

void GridMapRepresentation::worldToGrid(double x, double y, int& grid_x, int& grid_y) const {
    grid_x = static_cast<int>((x - origin_x_) / resolution_);
    grid_y = static_cast<int>((y - origin_y_) / resolution_);
}

void GridMapRepresentation::gridToWorld(int grid_x, int grid_y, double& x, double& y) const {
    x = grid_x * resolution_ + origin_x_;
    y = grid_y * resolution_ + origin_y_;
}

} // namespace asv_planning 