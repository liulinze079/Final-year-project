#include "../../include/global_planner/a_star_planner.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_set>

namespace asv_planning {

AStarPlanner::AStarPlanner()
    : grid_resolution_(1.0),
      obstacle_clearance_(5.0),
      min_safety_distance_(2.0) {
}

void AStarPlanner::setEnvironmentMap(const MultiResolutionGrid& map) {
    environment_map_ = map;
    grid_resolution_ = map.getBaseResolution();
}

void AStarPlanner::setEnergyCostMap(const EnergyCostMap& energy_map) {
    energy_cost_map_ = energy_map;
}

void AStarPlanner::setMaritimeHeuristics(const MaritimeHeuristics& heuristics) {
    maritime_heuristics_ = heuristics;
}

AStarPlanner::PlanningResult AStarPlanner::planPath(const PlanningParams& params) {
    // Initialize result
    PlanningResult result;
    result.success = false;
    
    // Start timer
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Check if start and goal are valid (collision-free)
    if (!isPathCollisionFree(params.start, params.start)) {
        result.error_message = "Start position is in collision with obstacles";
        return result;
    }
    
    if (!isPathCollisionFree(params.goal, params.goal)) {
        result.error_message = "Goal position is in collision with obstacles";
        return result;
    }
    
    // Initialize A* search structures
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_map<size_t, Node> came_from;
    std::unordered_set<size_t> closed_set;
    
    // Create start node
    Node start_node;
    start_node.position = params.start;
    start_node.g_cost = 0.0;
    start_node.h_cost = calculateHeuristic(params.start, params.goal, params.heuristic_type) * params.heuristic_weight;
    start_node.f_cost = start_node.g_cost + start_node.h_cost;
    start_node.heading = params.start_heading;
    start_node.safety_score = calculateSafetyScore(params.start);
    
    // Add start node to open set
    open_set.push(start_node);
    came_from[hashPosition(params.start)] = start_node;
    
    // Main A* loop
    int iterations = 0;
    while (!open_set.empty()) {
        // Check timeout
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - start_time;
        if (elapsed.count() > params.planning_timeout) {
            result.error_message = "Planning timeout exceeded";
            result.iterations = iterations;
            result.planning_time = elapsed.count();
            return result;
        }
        
        // Get node with lowest f-cost
        Node current = open_set.top();
        open_set.pop();
        iterations++;
        
        // Check if reached goal (within tolerance)
        double distance_to_goal = (current.position - params.goal).norm();
        if (distance_to_goal < grid_resolution_) {
            // Path found, reconstruct and return
            auto [path, headings] = reconstructPath(current, came_from);
            
            result.success = true;
            result.path = path;
            result.headings = headings;
            result.iterations = iterations;
            
            // Calculate path length and energy cost
            result.path_length = 0.0;
            result.energy_cost = 0.0;
            result.safety_score = 1.0;
            
            for (size_t i = 1; i < path.size(); ++i) {
                // Calculate segment length
                double segment_length = (path[i] - path[i-1]).norm();
                result.path_length += segment_length;
                
                // Calculate energy cost if enabled
                if (params.consider_energy) {
                    double segment_energy = energy_cost_map_.getTrajectoryEnergyCost(
                        path[i-1], path[i], headings[i-1]);
                    result.energy_cost += segment_energy;
                }
                
                // Accumulate minimum safety score
                double segment_safety = calculateSafetyScore(path[i]);
                result.safety_score = std::min(result.safety_score, segment_safety);
            }
            
            // Calculate planning time
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> planning_time = end_time - start_time;
            result.planning_time = planning_time.count();
            
            return result;
        }
        
        // Add current node to closed set
        size_t current_hash = hashPosition(current.position);
        closed_set.insert(current_hash);
        
        // Get neighbors
        auto neighbors = getNeighbors(current, params.allow_diagonal);
        
        // Process each neighbor
        for (const auto& neighbor_pos : neighbors) {
            // Check if neighbor is already in closed set
            size_t neighbor_hash = hashPosition(neighbor_pos);
            if (closed_set.find(neighbor_hash) != closed_set.end()) {
                continue;
            }
            
            // Check if path to neighbor is collision-free
            if (!isPathCollisionFree(current.position, neighbor_pos)) {
                continue;
            }
            
            // Calculate heading to neighbor
            double neighbor_heading = calculateHeading(current.position, neighbor_pos);
            
            // Calculate movement cost
            double movement_cost = calculateMovementCost(
                current.position, neighbor_pos, current.heading, params);
            
            // Calculate g-cost (cost from start to neighbor through current)
            double g_cost = current.g_cost + movement_cost;
            
            // Calculate safety score
            double safety_score = calculateSafetyScore(neighbor_pos);
            
            // Check if neighbor is already in came_from and has a better g-cost
            auto it = came_from.find(neighbor_hash);
            if (it != came_from.end() && g_cost >= it->second.g_cost) {
                continue;
            }
            
            // Create neighbor node
            Node neighbor_node;
            neighbor_node.position = neighbor_pos;
            neighbor_node.g_cost = g_cost;
            neighbor_node.h_cost = calculateHeuristic(neighbor_pos, params.goal, params.heuristic_type) * params.heuristic_weight;
            neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost;
            neighbor_node.heading = neighbor_heading;
            neighbor_node.parent = current.position;
            neighbor_node.safety_score = safety_score;
            
            // Add energy cost if enabled
            if (params.consider_energy) {
                double energy_cost = energy_cost_map_.getTrajectoryEnergyCost(
                    current.position, neighbor_pos, current.heading);
                neighbor_node.energy_cost = current.energy_cost + energy_cost;
            }
            
            // Add or update neighbor in came_from
            came_from[neighbor_hash] = neighbor_node;
            
            // Add neighbor to open set
            open_set.push(neighbor_node);
        }
    }
    
    // No path found
    result.error_message = "No path found";
    result.iterations = iterations;
    
    // Calculate planning time
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time = end_time - start_time;
    result.planning_time = planning_time.count();
    
    return result;
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<double>> 
AStarPlanner::smoothPath(const std::vector<Eigen::Vector2d>& path,
                         const std::vector<double>& headings) {
    // Check if path is valid
    if (path.size() <= 2) {
        return {path, headings};
    }
    
    // Create smoothed path and headings
    std::vector<Eigen::Vector2d> smoothed_path;
    std::vector<double> smoothed_headings;
    
    // Add start point
    smoothed_path.push_back(path.front());
    smoothed_headings.push_back(headings.front());
    
    // Iteratively check if we can skip nodes
    size_t current_idx = 0;
    while (current_idx < path.size() - 1) {
        // Find the furthest node that can be reached directly
        size_t target_idx = current_idx + 1;
        for (size_t i = current_idx + 2; i < path.size(); ++i) {
            if (isPathCollisionFree(path[current_idx], path[i])) {
                target_idx = i;
            } else {
                break;
            }
        }
        
        // Add the target node to the smoothed path
        smoothed_path.push_back(path[target_idx]);
        
        // Calculate new heading
        double heading = calculateHeading(path[current_idx], path[target_idx]);
        smoothed_headings.push_back(heading);
        
        // Update current index
        current_idx = target_idx;
    }
    
    // Ensure goal heading is preserved
    if (!smoothed_headings.empty() && !headings.empty()) {
        smoothed_headings.back() = headings.back();
    }
    
    return {smoothed_path, smoothed_headings};
}

double AStarPlanner::calculateHeuristic(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& goal,
    const std::string& heuristic_type) {
    
    if (heuristic_type == "euclidean") {
        // Euclidean distance
        return (position - goal).norm();
    } else if (heuristic_type == "manhattan") {
        // Manhattan distance
        return std::abs(position.x() - goal.x()) + 
               std::abs(position.y() - goal.y());
    } else if (heuristic_type == "maritime") {
        // Custom maritime heuristic
        return calculateMaritimeHeuristic(position, goal);
    } else {
        // Default to Euclidean distance
        return (position - goal).norm();
    }
}

double AStarPlanner::calculateMaritimeHeuristic(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& goal) {
    
    // Base distance
    double base_distance = (position - goal).norm();
    
    // Apply environmental factor from maritime heuristics
    double env_factor = maritime_heuristics_.getEnvironmentalFactor(
        environment_map_, position);
    
    // Calculate direct movement cost considering currents
    double direct_heading = calculateHeading(position, goal);
    double current_cost = maritime_heuristics_.getCurrentTraversalCost(
        environment_map_, position, goal);
    
    // Combine factors
    return base_distance * env_factor * current_cost;
}

std::vector<Eigen::Vector2d> AStarPlanner::getNeighbors(
    const Node& current,
    bool allow_diagonal) {
    
    std::vector<Eigen::Vector2d> neighbors;
    neighbors.reserve(allow_diagonal ? 8 : 4);
    
    // Define neighbor offsets
    const int dx[8] = {1, 0, -1, 0, 1, -1, -1, 1};
    const int dy[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    const int num_neighbors = allow_diagonal ? 8 : 4;
    
    // Add each neighbor
    for (int i = 0; i < num_neighbors; ++i) {
        Eigen::Vector2d neighbor_pos(
            current.position.x() + dx[i] * grid_resolution_,
            current.position.y() + dy[i] * grid_resolution_);
        
        neighbors.push_back(neighbor_pos);
    }
    
    return neighbors;
}

double AStarPlanner::calculateMovementCost(
    const Eigen::Vector2d& from,
    const Eigen::Vector2d& to,
    double from_heading,
    const PlanningParams& params) {
    
    // Calculate base movement cost (Euclidean distance)
    double distance = (to - from).norm();
    
    // Calculate heading change
    double to_heading = calculateHeading(from, to);
    double heading_diff = std::abs(to_heading - from_heading);
    while (heading_diff > M_PI) heading_diff = 2.0 * M_PI - heading_diff;
    
    // Penalize sharp turns (0.2 is a weight for the turning penalty)
    double turning_cost = heading_diff * 0.2 * distance;
    
    // Get safety score
    double safety_score = calculateSafetyScore(to);
    double safety_cost = (1.0 - safety_score) * params.safety_weight * distance;
    
    // Get energy cost if enabled
    double energy_cost = 0.0;
    if (params.consider_energy) {
        energy_cost = energy_cost_map_.getTrajectoryEnergyCost(from, to, from_heading) * params.energy_weight;
    }
    
    // Combine costs
    return distance + turning_cost + safety_cost + energy_cost;
}

double AStarPlanner::calculateHeading(
    const Eigen::Vector2d& from,
    const Eigen::Vector2d& to) {
    
    return std::atan2(to.y() - from.y(), to.x() - from.x());
}

bool AStarPlanner::isPathCollisionFree(
    const Eigen::Vector2d& from,
    const Eigen::Vector2d& to) {
    
    // Calculate distance and direction
    double dx = to.x() - from.x();
    double dy = to.y() - from.y();
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Check collision along the path
    const int num_checks = std::max(1, static_cast<int>(distance / (grid_resolution_ / 2.0)));
    
    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        Eigen::Vector2d pos = from + t * (to - from);
        
        // Check if position is too close to obstacles
        double obstacle_distance = environment_map_.getDistanceToNearestObstacle(pos);
        if (obstacle_distance < obstacle_clearance_) {
            return false;
        }
    }
    
    return true;
}

double AStarPlanner::calculateSafetyScore(const Eigen::Vector2d& position) {
    // Get distance to nearest obstacle
    double obstacle_distance = environment_map_.getDistanceToNearestObstacle(position);
    
    // Calculate safety score (0 to 1)
    // 0 = unsafe (at or below min_safety_distance)
    // 1 = safe (at or above obstacle_clearance)
    if (obstacle_distance <= min_safety_distance_) {
        return 0.0;
    } else if (obstacle_distance >= obstacle_clearance_) {
        return 1.0;
    } else {
        // Linear interpolation between min_safety_distance and obstacle_clearance
        return (obstacle_distance - min_safety_distance_) / 
               (obstacle_clearance_ - min_safety_distance_);
    }
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<double>> 
AStarPlanner::reconstructPath(
    const Node& goal_node,
    const std::unordered_map<size_t, Node>& came_from) {
    
    // Initialize path and headings
    std::vector<Eigen::Vector2d> path;
    std::vector<double> headings;
    
    // Add goal node
    path.push_back(goal_node.position);
    headings.push_back(goal_node.heading);
    
    // Trace back from goal to start
    Eigen::Vector2d current = goal_node.parent;
    
    while (came_from.find(hashPosition(current)) != came_from.end()) {
        const Node& node = came_from.at(hashPosition(current));
        path.push_back(node.position);
        headings.push_back(node.heading);
        
        // Check if we've reached the start node (no parent)
        if (node.parent == node.position) {
            break;
        }
        
        current = node.parent;
    }
    
    // Reverse path and headings (start to goal)
    std::reverse(path.begin(), path.end());
    std::reverse(headings.begin(), headings.end());
    
    return {path, headings};
}

size_t AStarPlanner::hashPosition(const Eigen::Vector2d& position) const {
    // Convert to grid coordinates
    int x = static_cast<int>(std::round(position.x() / grid_resolution_));
    int y = static_cast<int>(std::round(position.y() / grid_resolution_));
    
    // Combine into a single hash (using Cantor pairing function)
    return static_cast<size_t>(0.5 * (x + y) * (x + y + 1) + y);
}

} // namespace asv_planning 