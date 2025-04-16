#include "../../include/environment_representation/multi_resolution_grid.hpp"
#include <cmath>
#include <queue>
#include <limits>
#include <algorithm>

namespace asv_planning {

MultiResolutionGrid::MultiResolutionGrid()
    : width_(0),
      height_(0),
      resolution_(1.0),
      origin_x_(0.0),
      origin_y_(0.0),
      max_depth_(4),
      default_distance_(std::numeric_limits<double>::max()),
      default_force_magnitude_(0.0),
      default_force_direction_(0.0),
      default_wave_height_(0.0),
      default_wave_direction_(0.0),
      quadtree_root_(nullptr) {
}

void MultiResolutionGrid::updateFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& grid) {
    // Update grid properties
    width_ = grid.info.width;
    height_ = grid.info.height;
    resolution_ = grid.info.resolution;
    origin_x_ = grid.info.origin.position.x;
    origin_y_ = grid.info.origin.position.y;
    
    // Resize data arrays
    occupancy_data_.resize(width_ * height_);
    distance_field_.resize(width_ * height_, default_distance_);
    force_magnitudes_.resize(width_ * height_, default_force_magnitude_);
    force_directions_.resize(width_ * height_, default_force_direction_);
    wave_heights_.resize(width_ * height_, default_wave_height_);
    wave_directions_.resize(width_ * height_, default_wave_direction_);
    shipping_lanes_.resize(width_ * height_, false);
    
    // Convert occupancy grid data to internal format
    for (int i = 0; i < width_ * height_; ++i) {
        // Convert from ROS occupancy grid format (0-100 + -1 for unknown)
        // to our internal format (0 = free, 1 = unknown, 2 = occupied)
        int8_t value = grid.data[i];
        if (value == -1) {
            occupancy_data_[i] = 1; // Unknown
        } else if (value >= 50) {
            occupancy_data_[i] = 2; // Occupied
        } else {
            occupancy_data_[i] = 0; // Free
        }
    }
    
    // Build quadtree and compute distance field
    buildQuadtree();
    computeDistanceField();
}

double MultiResolutionGrid::getDistanceToNearestObstacle(const Eigen::Vector2d& position) const {
    int grid_x, grid_y;
    if (!worldToGrid(position.x(), position.y(), grid_x, grid_y)) {
        return default_distance_;
    }
    
    int idx = getCellIndex(grid_x, grid_y);
    if (idx == -1) {
        return default_distance_;
    }
    
    return distance_field_[idx];
}

double MultiResolutionGrid::getForceFieldMagnitude(const Eigen::Vector2d& position) const {
    int grid_x, grid_y;
    if (!worldToGrid(position.x(), position.y(), grid_x, grid_y)) {
        return default_force_magnitude_;
    }
    
    int idx = getCellIndex(grid_x, grid_y);
    if (idx == -1) {
        return default_force_magnitude_;
    }
    
    return force_magnitudes_[idx];
}

double MultiResolutionGrid::getForceFieldDirection(const Eigen::Vector2d& position) const {
    int grid_x, grid_y;
    if (!worldToGrid(position.x(), position.y(), grid_x, grid_y)) {
        return default_force_direction_;
    }
    
    int idx = getCellIndex(grid_x, grid_y);
    if (idx == -1) {
        return default_force_direction_;
    }
    
    return force_directions_[idx];
}

Eigen::Vector2d MultiResolutionGrid::getCurrentVector(const Eigen::Vector2d& position) const {
    double magnitude = getForceFieldMagnitude(position);
    double direction = getForceFieldDirection(position);
    
    // Convert polar to Cartesian coordinates
    return Eigen::Vector2d(
        magnitude * std::cos(direction),
        magnitude * std::sin(direction)
    );
}

double MultiResolutionGrid::getWaveHeight(const Eigen::Vector2d& position) const {
    int grid_x, grid_y;
    if (!worldToGrid(position.x(), position.y(), grid_x, grid_y)) {
        return default_wave_height_;
    }
    
    int idx = getCellIndex(grid_x, grid_y);
    if (idx == -1) {
        return default_wave_height_;
    }
    
    return wave_heights_[idx];
}

double MultiResolutionGrid::getWaveDirection(const Eigen::Vector2d& position) const {
    int grid_x, grid_y;
    if (!worldToGrid(position.x(), position.y(), grid_x, grid_y)) {
        return default_wave_direction_;
    }
    
    int idx = getCellIndex(grid_x, grid_y);
    if (idx == -1) {
        return default_wave_direction_;
    }
    
    return wave_directions_[idx];
}

bool MultiResolutionGrid::isInShippingLane(const Eigen::Vector2d& position) const {
    int grid_x, grid_y;
    if (!worldToGrid(position.x(), position.y(), grid_x, grid_y)) {
        return false;
    }
    
    int idx = getCellIndex(grid_x, grid_y);
    if (idx == -1) {
        return false;
    }
    
    return shipping_lanes_[idx];
}

bool MultiResolutionGrid::isOccupied(const Eigen::Vector2d& position) const {
    int grid_x, grid_y;
    if (!worldToGrid(position.x(), position.y(), grid_x, grid_y)) {
        return true;  // Outside map is considered occupied for safety
    }
    
    int idx = getCellIndex(grid_x, grid_y);
    if (idx == -1) {
        return true;  // Invalid index is considered occupied for safety
    }
    
    // Check if the occupancy value is "occupied" (value 2)
    return occupancy_data_[idx] == 2;
}

double MultiResolutionGrid::getCellSize(const Eigen::Vector2d& position) const {
    // Find the leaf node containing this position
    if (!quadtree_root_) {
        return resolution_;
    }
    
    // Starting from the root, find the leaf node containing this position
    std::shared_ptr<QuadNode> node = quadtree_root_;
    while (!node->is_leaf) {
        bool found_child = false;
        for (const auto& child : node->children) {
            if (position.x() >= child->min_x && position.x() < child->max_x &&
                position.y() >= child->min_y && position.y() < child->max_y) {
                node = child;
                found_child = true;
                break;
            }
        }
        
        if (!found_child) {
            // Should not happen if quadtree is properly constructed
            break;
        }
    }
    
    // Return the cell size based on the node level
    return resolution_ * std::pow(2.0, max_depth_ - node->level);
}

void MultiResolutionGrid::setForceField(
    const std::vector<double>& magnitudes,
    const std::vector<double>& directions) {
    
    if (magnitudes.size() == width_ * height_ && directions.size() == width_ * height_) {
        force_magnitudes_ = magnitudes;
        force_directions_ = directions;
    }
}

void MultiResolutionGrid::setWaveData(
    const std::vector<double>& heights,
    const std::vector<double>& directions) {
    
    if (heights.size() == width_ * height_ && directions.size() == width_ * height_) {
        wave_heights_ = heights;
        wave_directions_ = directions;
    }
}

void MultiResolutionGrid::setShippingLanes(const std::vector<bool>& lanes) {
    if (lanes.size() == width_ * height_) {
        shipping_lanes_ = lanes;
    }
}

std::pair<int, int> MultiResolutionGrid::getDimensions() const {
    return {width_, height_};
}

std::pair<double, double> MultiResolutionGrid::getOrigin() const {
    return {origin_x_, origin_y_};
}

double MultiResolutionGrid::getBaseResolution() const {
    return resolution_;
}

double MultiResolutionGrid::getResolution() const {
    return resolution_;
}

bool MultiResolutionGrid::worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const {
    grid_x = static_cast<int>((world_x - origin_x_) / resolution_);
    grid_y = static_cast<int>((world_y - origin_y_) / resolution_);
    
    return (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_);
}

void MultiResolutionGrid::gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const {
    world_x = origin_x_ + (grid_x + 0.5) * resolution_;
    world_y = origin_y_ + (grid_y + 0.5) * resolution_;
}

int MultiResolutionGrid::getCellIndex(int grid_x, int grid_y) const {
    if (grid_x < 0 || grid_x >= width_ || grid_y < 0 || grid_y >= height_) {
        return -1;
    }
    
    return grid_y * width_ + grid_x;
}

void MultiResolutionGrid::buildQuadtree() {
    if (width_ <= 0 || height_ <= 0) {
        return;
    }
    
    // Create root node covering the entire grid
    quadtree_root_ = std::make_shared<QuadNode>();
    quadtree_root_->is_leaf = false;
    quadtree_root_->level = 0;
    quadtree_root_->min_x = origin_x_;
    quadtree_root_->min_y = origin_y_;
    quadtree_root_->max_x = origin_x_ + width_ * resolution_;
    quadtree_root_->max_y = origin_y_ + height_ * resolution_;
    quadtree_root_->occupancy = 1; // Mixed until proven otherwise
    
    // Build the quadtree recursively
    std::queue<std::shared_ptr<QuadNode>> node_queue;
    node_queue.push(quadtree_root_);
    
    while (!node_queue.empty()) {
        std::shared_ptr<QuadNode> current = node_queue.front();
        node_queue.pop();
        
        // Check if we should subdivide this node
        if (current->level >= max_depth_) {
            current->is_leaf = true;
            continue;
        }
        
        // Check if the node is homogeneous (all free or all occupied)
        int min_grid_x, min_grid_y, max_grid_x, max_grid_y;
        worldToGrid(current->min_x, current->min_y, min_grid_x, min_grid_y);
        worldToGrid(current->max_x, current->max_y, max_grid_x, max_grid_y);
        
        // Clamp to grid boundaries
        min_grid_x = std::max(0, min_grid_x);
        min_grid_y = std::max(0, min_grid_y);
        max_grid_x = std::min(width_ - 1, max_grid_x);
        max_grid_y = std::min(height_ - 1, max_grid_y);
        
        bool is_homogeneous = true;
        uint8_t first_value = 0;
        bool has_valid_value = false;
        
        for (int y = min_grid_y; y <= max_grid_y; ++y) {
            for (int x = min_grid_x; x <= max_grid_x; ++x) {
                int idx = getCellIndex(x, y);
                if (idx != -1) {
                    if (!has_valid_value) {
                        first_value = occupancy_data_[idx];
                        has_valid_value = true;
                    } else if (occupancy_data_[idx] != first_value) {
                        is_homogeneous = false;
                        break;
                    }
                }
            }
            if (!is_homogeneous) {
                break;
            }
        }
        
        if (is_homogeneous && has_valid_value) {
            current->is_leaf = true;
            current->occupancy = first_value;
            continue;
        }
        
        // Subdivide the node
        current->is_leaf = false;
        current->children.reserve(4);
        double mid_x = (current->min_x + current->max_x) / 2.0;
        double mid_y = (current->min_y + current->max_y) / 2.0;
        
        // Create four child nodes (NW, NE, SW, SE)
        std::shared_ptr<QuadNode> nw = std::make_shared<QuadNode>();
        nw->is_leaf = true;
        nw->level = current->level + 1;
        nw->min_x = current->min_x;
        nw->min_y = mid_y;
        nw->max_x = mid_x;
        nw->max_y = current->max_y;
        current->children.push_back(nw);
        
        std::shared_ptr<QuadNode> ne = std::make_shared<QuadNode>();
        ne->is_leaf = true;
        ne->level = current->level + 1;
        ne->min_x = mid_x;
        ne->min_y = mid_y;
        ne->max_x = current->max_x;
        ne->max_y = current->max_y;
        current->children.push_back(ne);
        
        std::shared_ptr<QuadNode> sw = std::make_shared<QuadNode>();
        sw->is_leaf = true;
        sw->level = current->level + 1;
        sw->min_x = current->min_x;
        sw->min_y = current->min_y;
        sw->max_x = mid_x;
        sw->max_y = mid_y;
        current->children.push_back(sw);
        
        std::shared_ptr<QuadNode> se = std::make_shared<QuadNode>();
        se->is_leaf = true;
        se->level = current->level + 1;
        se->min_x = mid_x;
        se->min_y = current->min_y;
        se->max_x = current->max_x;
        se->max_y = mid_y;
        current->children.push_back(se);
        
        // Add child nodes to the queue for further processing
        for (const auto& child : current->children) {
            node_queue.push(child);
        }
    }
}

void MultiResolutionGrid::computeDistanceField() {
    // Initialize distance field
    for (int i = 0; i < width_ * height_; ++i) {
        if (occupancy_data_[i] == 2) { // Occupied
            distance_field_[i] = 0.0;
        } else {
            distance_field_[i] = std::numeric_limits<double>::max();
        }
    }
    
    // Use a breadth-first approach to compute distances
    std::queue<std::pair<int, int>> cell_queue;
    
    // First pass: add occupied cells to the queue
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int idx = getCellIndex(x, y);
            if (idx != -1 && occupancy_data_[idx] == 2) {
                cell_queue.push({x, y});
            }
        }
    }
    
    // Define neighbor offsets (8-connectivity)
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    
    // Process the queue
    while (!cell_queue.empty()) {
        auto [x, y] = cell_queue.front();
        cell_queue.pop();
        
        int idx = getCellIndex(x, y);
        double current_dist = distance_field_[idx];
        
        // Check all neighbors
        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            int nidx = getCellIndex(nx, ny);
            
            if (nidx == -1) {
                continue; // Out of bounds
            }
            
            // Compute Euclidean distance
            double dist = current_dist + resolution_ * (i < 4 ? 1.0 : std::sqrt(2.0));
            
            // Update if this path gives a shorter distance
            if (dist < distance_field_[nidx]) {
                distance_field_[nidx] = dist;
                cell_queue.push({nx, ny});
            }
        }
    }
}

} // namespace asv_planning 