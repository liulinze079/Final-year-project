#include "multi_resolution_grid.hpp"
#include <cmath>
#include <algorithm>
#include <queue>
#include <limits>

namespace asv_planning {

MultiResolutionGrid::MultiResolutionGrid(double min_x, double max_x, double min_y, double max_y,
                                     double base_resolution, int max_levels)
    : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y),
      base_resolution_(base_resolution), max_levels_(max_levels) {
    
    // Calculate grid dimensions
    width_ = static_cast<int>(std::ceil((max_x_ - min_x_) / base_resolution_));
    height_ = static_cast<int>(std::ceil((max_y_ - min_y_) / base_resolution_));
    
    // Initialize grid with empty cells
    grid_.resize(height_, std::vector<GridCell>(width_));
    
    // Initialize force field vectors
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            grid_[y][x].force_field = Eigen::Vector2d::Zero();
        }
    }
}

bool MultiResolutionGrid::isWithinBounds(double x, double y) const {
    return x >= min_x_ && x < max_x_ && y >= min_y_ && y < max_y_;
}

bool MultiResolutionGrid::worldToGrid(double x, double y, int& grid_x, int& grid_y) const {
    if (!isWithinBounds(x, y)) {
        return false;
    }
    
    grid_x = static_cast<int>((x - min_x_) / base_resolution_);
    grid_y = static_cast<int>((y - min_y_) / base_resolution_);
    
    // Clamp to valid indices
    grid_x = std::max(0, std::min(grid_x, width_ - 1));
    grid_y = std::max(0, std::min(grid_y, height_ - 1));
    
    return true;
}

void MultiResolutionGrid::gridToWorld(int grid_x, int grid_y, double& x, double& y) const {
    // Convert to center of cell
    x = min_x_ + (grid_x + 0.5) * base_resolution_;
    y = min_y_ + (grid_y + 0.5) * base_resolution_;
}

bool MultiResolutionGrid::isOccupied(double x, double y) const {
    const GridCell* cell = getCell(x, y);
    return cell ? cell->occupied : true;  // Consider out-of-bounds as occupied
}

void MultiResolutionGrid::addObstacle(const Eigen::Vector2d& position, double radius) {
    int grid_radius = static_cast<int>(std::ceil(radius / base_resolution_));
    int center_x, center_y;
    
    if (!worldToGrid(position.x(), position.y(), center_x, center_y)) {
        return;  // Position is out of bounds
    }
    
    // Mark cells within radius as occupied
    for (int y = center_y - grid_radius; y <= center_y + grid_radius; ++y) {
        for (int x = center_x - grid_radius; x <= center_x + grid_radius; ++x) {
            if (y >= 0 && y < height_ && x >= 0 && x < width_) {
                double world_x, world_y;
                gridToWorld(x, y, world_x, world_y);
                
                // Check if cell center is within obstacle radius
                double dx = world_x - position.x();
                double dy = world_y - position.y();
                double distance = std::sqrt(dx * dx + dy * dy);
                
                if (distance <= radius) {
                    grid_[y][x].occupied = true;
                }
            }
        }
    }
    
    // Recompute distance field after adding obstacle
    computeDistanceField();
}

void MultiResolutionGrid::addForceField(const Eigen::Vector2d& position, const Eigen::Vector2d& force) {
    GridCell* cell = getMutableCell(position.x(), position.y());
    if (cell) {
        cell->force_field = force;
    }
}

void MultiResolutionGrid::setWaveProperties(const Eigen::Vector2d& position, double height, double direction) {
    GridCell* cell = getMutableCell(position.x(), position.y());
    if (cell) {
        cell->wave_height = height;
        cell->wave_direction = direction;
    }
}

void MultiResolutionGrid::defineShippingLane(const Eigen::Vector2d& start_point, 
                                         const Eigen::Vector2d& end_point,
                                         double width) {
    // Direction vector of the lane
    Eigen::Vector2d lane_dir = end_point - start_point;
    lane_dir.normalize();
    
    // Perpendicular vector (for width)
    Eigen::Vector2d perp_dir(-lane_dir.y(), lane_dir.x());
    
    // Lane length
    double lane_length = (end_point - start_point).norm();
    
    // Half width
    double half_width = width / 2.0;
    
    // Mark cells within lane as shipping lane
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            double world_x, world_y;
            gridToWorld(x, y, world_x, world_y);
            
            // Convert to position relative to lane start
            Eigen::Vector2d rel_pos(world_x - start_point.x(), world_y - start_point.y());
            
            // Project onto lane direction
            double along_dist = rel_pos.dot(lane_dir);
            
            // Check if within lane length
            if (along_dist >= 0 && along_dist <= lane_length) {
                // Project onto perpendicular direction
                double cross_dist = std::abs(rel_pos.dot(perp_dir));
                
                // Check if within lane width
                if (cross_dist <= half_width) {
                    grid_[y][x].in_shipping_lane = true;
                }
            }
        }
    }
}

void MultiResolutionGrid::updateTemporalDecay(double current_time) {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            GridCell& cell = grid_[y][x];
            
            // Skip cells with no decay factor
            if (cell.decay_factor <= 0.0) {
                continue;
            }
            
            // Calculate time since last update
            double time_diff = current_time - cell.last_update_time;
            
            // Apply exponential decay to occupancy probability
            if (time_diff > 0 && cell.occupied) {
                // Simple decay model: P(occupied) decreases over time
                // If decay factor is high enough, mark as free
                if (std::exp(-cell.decay_factor * time_diff) < 0.5) {
                    cell.occupied = false;
                }
            }
            
            // Update timestamp
            cell.last_update_time = current_time;
        }
    }
    
    // Recompute distance field after decay
    computeDistanceField();
}

double MultiResolutionGrid::getForceFieldMagnitude(const Eigen::Vector2d& position) const {
    const GridCell* cell = getCell(position.x(), position.y());
    return cell ? cell->force_field.norm() : 0.0;
}

double MultiResolutionGrid::getForceFieldDirection(const Eigen::Vector2d& position) const {
    const GridCell* cell = getCell(position.x(), position.y());
    if (!cell || cell->force_field.norm() < 1e-6) {
        return 0.0;  // Default direction if no force or out of bounds
    }
    return std::atan2(cell->force_field.y(), cell->force_field.x());
}

double MultiResolutionGrid::getWaveHeight(const Eigen::Vector2d& position) const {
    const GridCell* cell = getCell(position.x(), position.y());
    return cell ? cell->wave_height : 0.0;
}

double MultiResolutionGrid::getWaveDirection(const Eigen::Vector2d& position) const {
    const GridCell* cell = getCell(position.x(), position.y());
    return cell ? cell->wave_direction : 0.0;
}

bool MultiResolutionGrid::isInShippingLane(const Eigen::Vector2d& position) const {
    const GridCell* cell = getCell(position.x(), position.y());
    return cell ? cell->in_shipping_lane : false;
}

double MultiResolutionGrid::getDistanceToNearestObstacle(const Eigen::Vector2d& position) const {
    const GridCell* cell = getCell(position.x(), position.y());
    if (!cell) {
        return 0.0;  // Out of bounds is considered as colliding
    }
    
    // If distance field hasn't been computed, return a default value
    if (cell->obstacle_distance < 0) {
        return 10.0;  // Arbitrary default value
    }
    
    return cell->obstacle_distance;
}

const GridCell* MultiResolutionGrid::getCell(double x, double y) const {
    int grid_x, grid_y;
    if (!worldToGrid(x, y, grid_x, grid_y)) {
        return nullptr;  // Out of bounds
    }
    return &grid_[grid_y][grid_x];
}

GridCell* MultiResolutionGrid::getMutableCell(double x, double y) {
    int grid_x, grid_y;
    if (!worldToGrid(x, y, grid_x, grid_y)) {
        return nullptr;  // Out of bounds
    }
    return &grid_[grid_y][grid_x];
}

void MultiResolutionGrid::computeDistanceField() {
    // Initialize distances
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (grid_[y][x].occupied) {
                grid_[y][x].obstacle_distance = 0.0;  // Obstacle cells have zero distance
            } else {
                grid_[y][x].obstacle_distance = -1.0;  // Unprocessed cells
            }
        }
    }
    
    // Queue for breadth-first search
    std::queue<std::pair<int, int>> queue;
    
    // Add all obstacle cells to the queue
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (grid_[y][x].occupied) {
                queue.push({x, y});
            }
        }
    }
    
    // Process the queue (breadth-first wavefront)
    static const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    
    while (!queue.empty()) {
        auto [curr_x, curr_y] = queue.front();
        queue.pop();
        
        double curr_dist = grid_[curr_y][curr_x].obstacle_distance;
        
        // Check neighbors
        for (int i = 0; i < 8; ++i) {
            int nx = curr_x + dx[i];
            int ny = curr_y + dy[i];
            
            // Skip if out of bounds
            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
                continue;
            }
            
            // Skip if obstacle or already processed with better distance
            if (grid_[ny][nx].occupied || 
                (grid_[ny][nx].obstacle_distance >= 0 && 
                 grid_[ny][nx].obstacle_distance <= curr_dist + base_resolution_)) {
                continue;
            }
            
            // Calculate distance increment (diagonal moves cost more)
            double increment = (dx[i] != 0 && dy[i] != 0) ? 
                               base_resolution_ * 1.414 : base_resolution_;
            
            // Update distance
            grid_[ny][nx].obstacle_distance = curr_dist + increment;
            
            // Add to queue for further processing
            queue.push({nx, ny});
        }
    }
}

} // namespace asv_planning 