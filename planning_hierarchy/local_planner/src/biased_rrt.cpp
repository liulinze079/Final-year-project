#include "biased_rrt.hpp"
#include "sampling_strategies.hpp"
#include "../../asv_core/include/asv_dynamics.hpp"
#include "../../../environment_representation/include/multi_resolution_grid.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>
#include <stdexcept>

namespace asv_planning {

BiasedRRT::BiasedRRT(double lookahead_distance, int max_iterations, double step_size)
    : lookahead_distance_(lookahead_distance),
      max_iterations_(max_iterations),
      step_size_(step_size),
      goal_bias_(0.1),
      global_path_bias_(0.4),
      current_heading_(0.0),
      rng_(std::random_device()()),
      uniform_dist_(0.0, 1.0) {
}

void BiasedRRT::setMap(std::shared_ptr<MultiResolutionGrid> map) {
    map_ = map;
    if (sampling_strategy_) {
        sampling_strategy_->setMap(map);
    }
}

void BiasedRRT::setGlobalPath(const std::vector<Eigen::Vector2d>& global_path) {
    global_path_ = global_path;
}

void BiasedRRT::setCurrentPose(const Eigen::Vector2d& position, double heading) {
    current_position_ = position;
    current_heading_ = heading;
}

void BiasedRRT::setDynamicsModel(std::shared_ptr<ASVDynamics> dynamics) {
    dynamics_ = dynamics;
}

void BiasedRRT::setSamplingStrategy(std::shared_ptr<SamplingStrategy> strategy) {
    sampling_strategy_ = strategy;
    if (map_ && sampling_strategy_) {
        sampling_strategy_->setMap(map_);
    }
}

void BiasedRRT::setGoalBias(double bias) {
    if (bias < 0.0 || bias > 1.0) {
        throw std::invalid_argument("Goal bias must be between 0.0 and 1.0");
    }
    goal_bias_ = bias;
}

void BiasedRRT::setGlobalPathBias(double bias) {
    if (bias < 0.0 || bias > 1.0) {
        throw std::invalid_argument("Global path bias must be between 0.0 and 1.0");
    }
    global_path_bias_ = bias;
}

std::pair<Eigen::Vector2d, int> BiasedRRT::getClosestPointOnGlobalPath(
    const Eigen::Vector2d& position) const {
    
    if (global_path_.empty()) {
        return {Eigen::Vector2d(0.0, 0.0), -1};
    }
    
    double min_distance = std::numeric_limits<double>::max();
    int closest_idx = 0;
    Eigen::Vector2d closest_point;
    
    for (size_t i = 0; i < global_path_.size(); ++i) {
        double distance = (position - global_path_[i]).norm();
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = static_cast<int>(i);
            closest_point = global_path_[i];
        }
    }
    
    return {closest_point, closest_idx};
}

std::vector<Eigen::Vector2d> BiasedRRT::planPath() {
    if (!map_) {
        throw std::runtime_error("No map set for planning");
    }
    
    if (global_path_.empty()) {
        throw std::runtime_error("Global path is empty");
    }
    
    if (!sampling_strategy_) {
        throw std::runtime_error("No sampling strategy set");
    }
    
    // Clear previous local path
    local_path_.clear();
    
    // Get target point on global path
    Eigen::Vector2d target = getGlobalPathTarget();
    
    // Initialize RRT
    std::vector<Node> nodes;
    nodes.emplace_back(current_position_);  // Root node is current position
    
    // Calculate sampling bounds (centered on current position)
    Eigen::Vector4d bounds;
    bounds << current_position_.x() - lookahead_distance_,
              current_position_.y() - lookahead_distance_,
              current_position_.x() + lookahead_distance_,
              current_position_.y() + lookahead_distance_;
    
    // Main RRT loop
    int target_idx = -1;
    for (int i = 0; i < max_iterations_; ++i) {
        // Sample a random position
        Eigen::Vector2d sample_pos;
        double rand_val = uniform_dist_(rng_);
        
        if (rand_val < goal_bias_) {
            // Sample toward target
            sample_pos = target;
        } else if (rand_val < goal_bias_ + global_path_bias_) {
            // Sample from global path
            int random_idx = std::uniform_int_distribution<int>(
                0, static_cast<int>(global_path_.size()) - 1)(rng_);
            sample_pos = global_path_[random_idx];
        } else {
            // Sample using strategy
            sample_pos = sampling_strategy_->samplePosition(
                current_position_, current_heading_, bounds, target, goal_bias_);
        }
        
        // Find nearest node
        int nearest_idx = findNearestNode(sample_pos, nodes);
        
        // Extend tree
        if (extend(nearest_idx, sample_pos, nodes)) {
            // Check if we've reached the target (within step size)
            if ((nodes.back().position - target).norm() < step_size_) {
                target_idx = static_cast<int>(nodes.size()) - 1;
                break;
            }
        }
    }
    
    // If we didn't reach the exact target, find the closest node to it
    if (target_idx < 0) {
        target_idx = findNearestNode(target, nodes);
    }
    
    // Extract path
    local_path_ = extractPath(nodes, target_idx);
    
    return local_path_;
}

Eigen::Vector2d BiasedRRT::sampleRandomPosition() {
    // Use the sampling strategy if available, otherwise default to uniform
    if (sampling_strategy_) {
        Eigen::Vector4d bounds;
        bounds << current_position_.x() - lookahead_distance_,
                  current_position_.y() - lookahead_distance_,
                  current_position_.x() + lookahead_distance_,
                  current_position_.y() + lookahead_distance_;
                  
        Eigen::Vector2d target = getGlobalPathTarget();
        
        return sampling_strategy_->samplePosition(
            current_position_, current_heading_, bounds, target, goal_bias_);
    }
    
    // Default uniform sampling
    double x = std::uniform_real_distribution<double>(
        current_position_.x() - lookahead_distance_,
        current_position_.x() + lookahead_distance_)(rng_);
        
    double y = std::uniform_real_distribution<double>(
        current_position_.y() - lookahead_distance_,
        current_position_.y() + lookahead_distance_)(rng_);
        
    return Eigen::Vector2d(x, y);
}

int BiasedRRT::findNearestNode(const Eigen::Vector2d& position, 
                           const std::vector<Node>& nodes) const {
    if (nodes.empty()) {
        return -1;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    int nearest_idx = 0;
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        double distance = (position - nodes[i].position).norm();
        if (distance < min_distance) {
            min_distance = distance;
            nearest_idx = static_cast<int>(i);
        }
    }
    
    return nearest_idx;
}

bool BiasedRRT::extend(int nearest_idx, const Eigen::Vector2d& sample_pos, 
                   std::vector<Node>& nodes) {
    if (nearest_idx < 0 || nearest_idx >= static_cast<int>(nodes.size())) {
        return false;
    }
    
    // Calculate direction vector
    Eigen::Vector2d direction = sample_pos - nodes[nearest_idx].position;
    double distance = direction.norm();
    
    // Normalize and scale by step size
    if (distance > step_size_) {
        direction = direction / distance * step_size_;
    }
    
    // Calculate new position
    Eigen::Vector2d new_pos = nodes[nearest_idx].position + direction;
    
    // Check if collision-free
    if (!isCollisionFree(nodes[nearest_idx].position, new_pos)) {
        return false;
    }
    
    // Add new node to tree
    nodes.emplace_back(new_pos, nearest_idx);
    
    return true;
}

bool BiasedRRT::isCollisionFree(const Eigen::Vector2d& from, 
                           const Eigen::Vector2d& to) const {
    if (!map_) {
        return true;  // No map, assume free
    }
    
    // Check if either endpoint is in collision
    if (map_->isOccupied(from.x(), from.y()) || map_->isOccupied(to.x(), to.y())) {
        return false;
    }
    
    // Simple line collision check (could be improved with ray casting)
    double distance = (to - from).norm();
    int steps = static_cast<int>(std::ceil(distance / (step_size_ * 0.5)));
    steps = std::max(steps, 3);  // At least 3 checks
    
    Eigen::Vector2d step = (to - from) / static_cast<double>(steps - 1);
    
    for (int i = 1; i < steps - 1; ++i) {
        Eigen::Vector2d point = from + step * static_cast<double>(i);
        if (map_->isOccupied(point.x(), point.y())) {
            return false;
        }
    }
    
    return true;
}

std::vector<Eigen::Vector2d> BiasedRRT::extractPath(const std::vector<Node>& nodes, 
                                               int goal_idx) const {
    std::vector<Eigen::Vector2d> path;
    
    if (nodes.empty() || goal_idx < 0 || goal_idx >= static_cast<int>(nodes.size())) {
        return path;
    }
    
    // Follow parent pointers from goal to start
    int current_idx = goal_idx;
    while (current_idx >= 0) {
        path.push_back(nodes[current_idx].position);
        current_idx = nodes[current_idx].parent_idx;
    }
    
    // Reverse to get start-to-goal ordering
    std::reverse(path.begin(), path.end());
    
    return path;
}

Eigen::Vector2d BiasedRRT::getGlobalPathTarget() const {
    if (global_path_.empty()) {
        return current_position_;
    }
    
    // Find closest point on the global path
    auto [closest_point, closest_idx] = getClosestPointOnGlobalPath(current_position_);
    
    // Find a point ahead on the global path
    int target_idx = closest_idx;
    double accumulated_distance = 0.0;
    
    // Look for a point that's about lookahead_distance_ away
    while (target_idx + 1 < static_cast<int>(global_path_.size()) && 
           accumulated_distance < lookahead_distance_) {
        double segment_length = (global_path_[target_idx + 1] - global_path_[target_idx]).norm();
        accumulated_distance += segment_length;
        target_idx++;
    }
    
    return global_path_[target_idx];
}

} // namespace asv_planning 