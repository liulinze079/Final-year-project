#include "planning/a_star_planner.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace asv_planning {

AStarPlanner::AStarPlanner()
    : heuristic_type_(1),  // Default to Euclidean
      heuristic_weight_(1.0),
      allow_diagonal_(true),
      grid_resolution_(0.5),
      iterations_(0),
      path_cost_(0.0),
      map_(nullptr),
      maritime_heuristics_(nullptr) {
}

bool AStarPlanner::initialize(MultiResolutionGrid* map) {
    if (!map) {
        return false;
    }
    
    map_ = map;
    grid_resolution_ = map_->getResolution();
    
    return true;
}

bool AStarPlanner::planPath(const geometry_msgs::msg::PoseStamped& start,
                           const geometry_msgs::msg::PoseStamped& goal,
                           std::vector<geometry_msgs::msg::PoseStamped>& path) {
    // Clear the result
    path.clear();
    iterations_ = 0;
    path_cost_ = 0.0;
    
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Convert start and goal to grid coordinates
    Eigen::Vector2d start_pos(start.pose.position.x, start.pose.position.y);
    Eigen::Vector2d goal_pos(goal.pose.position.x, goal.pose.position.y);
    
    // Check if start or goal is in collision
    if (!map_ || map_->isOccupied(start_pos) || map_->isOccupied(goal_pos)) {
        return false;
    }
    
    // Initialize open and closed sets
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeComparison> open_set;
    std::unordered_map<int, std::shared_ptr<Node>> open_map;
    std::unordered_set<int> closed_set;
    
    // Hash function for grid positions
    auto hash_position = [this](const Eigen::Vector2d& pos) -> int {
        // Simple hash function for 2D positions
        int x = static_cast<int>(std::round(pos.x() / grid_resolution_));
        int y = static_cast<int>(std::round(pos.y() / grid_resolution_));
        return x * 1000000 + y;  // Assuming y < 1000000
    };
    
    // Create start node
    auto start_node = std::make_shared<Node>(start_pos);
    start_node->g_cost = 0.0;
    start_node->h_cost = calculateHeuristic(start_pos, goal_pos);
    start_node->f_cost = start_node->g_cost + heuristic_weight_ * start_node->h_cost;
    
    // Add start node to open set
    open_set.push(start_node);
    open_map[hash_position(start_pos)] = start_node;
    
    // Search loop
    while (!open_set.empty()) {
        // Get node with lowest f_cost
        auto current = open_set.top();
        open_set.pop();
        
        iterations_++;
        
        // Check if we've reached the goal
        if ((current->position - goal_pos).norm() < grid_resolution_) {
            // Reconstruct path
            reconstructPath(current, start, goal, path);
            
            // Calculate path cost
            path_cost_ = current->g_cost;
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            
            return true;
        }
        
        // Add to closed set
        int current_hash = hash_position(current->position);
        if (closed_set.find(current_hash) != closed_set.end()) {
            continue;  // Already processed
        }
        closed_set.insert(current_hash);
        
        // Remove from open map
        open_map.erase(current_hash);
        
        // Get neighbors
        auto neighbors = getNeighbors(current->position);
        
        // Process each neighbor
        for (const auto& neighbor_pos : neighbors) {
            // Skip if in closed set
            int neighbor_hash = hash_position(neighbor_pos);
            if (closed_set.find(neighbor_hash) != closed_set.end()) {
                continue;
            }
            
            // Skip if occupied
            if (map_->isOccupied(neighbor_pos)) {
                continue;
            }
            
            // Calculate new cost
            double movement_cost = calculateMovementCost(current->position, neighbor_pos);
            double new_g_cost = current->g_cost + movement_cost;
            
            // Check if neighbor is in open set
            auto it = open_map.find(neighbor_hash);
            if (it != open_map.end()) {
                // If new path is better, update
                if (new_g_cost < it->second->g_cost) {
                    it->second->g_cost = new_g_cost;
                    it->second->f_cost = new_g_cost + heuristic_weight_ * it->second->h_cost;
                    it->second->parent = current;
                }
            } else {
                // Add new node to open set
                auto neighbor_node = std::make_shared<Node>(neighbor_pos);
                neighbor_node->g_cost = new_g_cost;
                neighbor_node->h_cost = calculateHeuristic(neighbor_pos, goal_pos);
                neighbor_node->f_cost = neighbor_node->g_cost + heuristic_weight_ * neighbor_node->h_cost;
                neighbor_node->parent = current;
                
                open_set.push(neighbor_node);
                open_map[neighbor_hash] = neighbor_node;
            }
        }
    }
    
    // No path found
    return false;
}

double AStarPlanner::calculateHeuristic(const Eigen::Vector2d& from, const Eigen::Vector2d& to) {
    double dx = std::abs(to.x() - from.x());
    double dy = std::abs(to.y() - from.y());
    
    switch (heuristic_type_) {
        case 0:  // Manhattan
            return dx + dy;
        case 1:  // Euclidean
            return std::sqrt(dx * dx + dy * dy);
        case 2:  // Diagonal
            return std::max(dx, dy) + 0.414 * std::min(dx, dy);  // 0.414 = sqrt(2) - 1
        case 3:  // Maritime
            if (maritime_heuristics_) {
                // Get base heuristic with Euclidean distance
                double base_cost = std::sqrt(dx * dx + dy * dy);
                
                // Add maritime-specific factors
                double heading = std::atan2(to.y() - from.y(), to.x() - from.x());
                double env_factor = maritime_heuristics_->getEnvironmentalFactor(from, heading);
                
                return base_cost * (1.0 + env_factor);
            } else {
                // Fallback to Euclidean
                return std::sqrt(dx * dx + dy * dy);
            }
        default:
            return std::sqrt(dx * dx + dy * dy);  // Default to Euclidean
    }
}

double AStarPlanner::calculateMovementCost(const Eigen::Vector2d& from, const Eigen::Vector2d& to) {
    // Calculate base cost (Euclidean distance)
    double dx = to.x() - from.x();
    double dy = to.y() - from.y();
    double base_cost = std::sqrt(dx * dx + dy * dy);
    
    // Apply terrain cost if available
    double terrain_cost = 1.0;
    if (map_) {
        // Get distance to nearest obstacle as a factor of safety
        double obstacle_distance = map_->getDistanceToNearestObstacle(to);
        double safety_factor = 1.0;
        
        if (obstacle_distance < 5.0) {  // 5 meters threshold
            safety_factor = 1.0 + (5.0 - obstacle_distance) / 2.5;  // Increase cost near obstacles
        }
        
        terrain_cost *= safety_factor;
    }
    
    // Apply maritime specific costs if available
    if (maritime_heuristics_) {
        double heading = std::atan2(dy, dx);
        double env_factor = maritime_heuristics_->getEnvironmentalFactor(from, heading);
        terrain_cost *= (1.0 + env_factor);
    }
    
    return base_cost * terrain_cost;
}

std::vector<Eigen::Vector2d> AStarPlanner::getNeighbors(const Eigen::Vector2d& position) {
    std::vector<Eigen::Vector2d> neighbors;
    
    // 8-connected grid
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            
            // Skip diagonals if not allowed
            if (!allow_diagonal_ && dx != 0 && dy != 0) continue;
            
            Eigen::Vector2d neighbor(
                position.x() + dx * grid_resolution_,
                position.y() + dy * grid_resolution_
            );
            
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}

void AStarPlanner::reconstructPath(const std::shared_ptr<Node>& end_node,
                                  const geometry_msgs::msg::PoseStamped& start,
                                  const geometry_msgs::msg::PoseStamped& goal,
                                  std::vector<geometry_msgs::msg::PoseStamped>& path) {
    // Collect nodes from end to start
    std::vector<std::shared_ptr<Node>> reverse_path;
    std::shared_ptr<Node> current = end_node;
    
    while (current) {
        reverse_path.push_back(current);
        current = current->parent;
    }
    
    // Reverse to get path from start to goal
    std::reverse(reverse_path.begin(), reverse_path.end());
    
    // Convert to PoseStamped
    path.clear();
    
    if (reverse_path.empty()) {
        return;
    }
    
    // Add start pose
    path.push_back(start);
    
    // Add intermediate poses
    for (size_t i = 1; i < reverse_path.size(); ++i) {
        // Calculate heading to next point
        double heading;
        if (i < reverse_path.size() - 1) {
            // Use direction to next point
            Eigen::Vector2d dir = reverse_path[i+1]->position - reverse_path[i]->position;
            heading = std::atan2(dir.y(), dir.x());
        } else {
            // For last point, use goal heading
            tf2::Quaternion q;
            tf2::fromMsg(goal.pose.orientation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            heading = yaw;
        }
        
        path.push_back(poseToPoseStamped(reverse_path[i]->position, heading));
    }
    
    // Add goal pose
    path.push_back(goal);
}

geometry_msgs::msg::PoseStamped AStarPlanner::poseToPoseStamped(const Eigen::Vector2d& position, double heading) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = rclcpp::Clock().now();
    
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading);
    pose.pose.orientation = tf2::toMsg(q);
    
    return pose;
}

void AStarPlanner::setHeuristicType(int type) {
    if (type >= 0 && type <= 3) {
        heuristic_type_ = type;
    }
}

void AStarPlanner::setHeuristicWeight(double weight) {
    if (weight > 0.0) {
        heuristic_weight_ = weight;
    }
}

void AStarPlanner::setAllowDiagonal(bool allow) {
    allow_diagonal_ = allow;
}

int AStarPlanner::getIterations() const {
    return iterations_;
}

double AStarPlanner::getPathCost() const {
    return path_cost_;
}

void AStarPlanner::setMaritimeHeuristics(std::shared_ptr<MaritimeHeuristics> maritime_heuristics) {
    maritime_heuristics_ = maritime_heuristics;
}

} // namespace asv_planning 