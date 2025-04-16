#include "energy_aware_astar.hpp"
#include "maritime_heuristics.hpp"
#include "../../asv_core/include/asv_dynamics.hpp"
#include "../../../environment_representation/include/multi_resolution_grid.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace asv_planning {

EnergyAwareAStar::EnergyAwareAStar(double resolution)
    : resolution_(resolution),
      energy_weight_(0.5),
      path_energy_(0.0),
      path_length_(0.0) {
}

void EnergyAwareAStar::setMap(std::shared_ptr<MultiResolutionGrid> map) {
    map_ = map;
}

void EnergyAwareAStar::setDynamicsModel(std::shared_ptr<ASVDynamics> dynamics) {
    dynamics_ = dynamics;
}

void EnergyAwareAStar::setStartAndGoal(const Eigen::Vector2d& start, const Eigen::Vector2d& goal) {
    start_ = start;
    goal_ = goal;
}

void EnergyAwareAStar::setEnergyWeight(double weight) {
    if (weight < 0.0 || weight > 1.0) {
        throw std::invalid_argument("Energy weight must be between 0.0 and 1.0");
    }
    energy_weight_ = weight;
}

double EnergyAwareAStar::getPathEnergy() const {
    return path_energy_;
}

double EnergyAwareAStar::getPathLength() const {
    return path_length_;
}

double EnergyAwareAStar::calculateHeuristic(const Node& node, int goal_x, int goal_y) const {
    // Euclidean distance heuristic
    double dx = (node.x - goal_x) * resolution_;
    double dy = (node.y - goal_y) * resolution_;
    
    // Base distance cost
    double distance_cost = std::sqrt(dx * dx + dy * dy);
    
    // Add environmental factor from maritime heuristics if available
    double environmental_factor = 1.0;
    if (map_) {
        Eigen::Vector2d pos(node.x * resolution_, node.y * resolution_);
        environmental_factor = MaritimeHeuristics::getEnvironmentalFactor(*map_, pos);
    }
    
    return distance_cost * environmental_factor;
}

double EnergyAwareAStar::estimateEnergyConsumption(const Node& from, const Node& to, double current_energy) const {
    // If dynamics model is not set, use a simple distance-based estimate
    if (!dynamics_) {
        double dx = (to.x - from.x) * resolution_;
        double dy = (to.y - from.y) * resolution_;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        // Simple model: energy is proportional to distance
        return distance * 10.0;  // Arbitrary energy per meter
    }
    
    // Calculate direction and angle
    double dx = (to.x - from.x) * resolution_;
    double dy = (to.y - from.y) * resolution_;
    double distance = std::sqrt(dx * dx + dy * dy);
    double angle = std::atan2(dy, dx);
    
    // Create a simple state and control vector
    Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
    state(2) = angle;  // Heading aligned with movement direction
    state(3) = 1.0;    // Forward speed of 1 m/s
    
    // Estimate control effort needed (simple model)
    Eigen::VectorXd control(2);
    control << 10.0, 10.0;  // Equal thrust to move forward
    
    // Estimate time to traverse the cell
    double time = distance / state(3);
    
    // Use dynamics model to estimate energy
    double energy = dynamics_->computeEnergyConsumption(control, time);
    
    // Add environmental effects based on force field
    if (map_) {
        Eigen::Vector2d pos(to.x * resolution_, to.y * resolution_);
        double force_magnitude = map_->getForceFieldMagnitude(pos);
        double force_direction = map_->getForceFieldDirection(pos);
        
        // Adjust energy based on relation between movement direction and force direction
        double angle_diff = std::abs(angle - force_direction);
        while (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        
        // Opposing force increases energy, aligned force decreases it
        double energy_factor = 1.0 + force_magnitude * std::cos(angle_diff) * 0.2;
        energy *= std::max(0.5, energy_factor);  // Cap the minimum at 50% to avoid unrealistic values
    }
    
    return energy;
}

std::vector<EnergyAwareAStar::Node> EnergyAwareAStar::getNeighbors(const Node& current) const {
    std::vector<Node> neighbors;
    
    // Check 8 adjacent cells
    static const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    
    for (int i = 0; i < 8; ++i) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        
        // Check if within map bounds
        if (map_ && !map_->isWithinBounds(nx * resolution_, ny * resolution_)) {
            continue;
        }
        
        // Check if the cell is free
        if (map_ && map_->isOccupied(nx * resolution_, ny * resolution_)) {
            continue;
        }
        
        // Calculate movement cost (diagonal movements cost more)
        double distance_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
        distance_cost *= resolution_;
        
        // Estimate energy cost for this move
        double energy_cost = estimateEnergyConsumption(current, Node(nx, ny), current.energy);
        
        // Combined cost with weighting between distance and energy
        double combined_cost = (1.0 - energy_weight_) * distance_cost + 
                               energy_weight_ * (energy_cost / 100.0);  // Normalize energy scale
        
        neighbors.emplace_back(nx, ny, current.g + combined_cost, 0.0, current.energy + energy_cost);
    }
    
    return neighbors;
}

std::vector<Eigen::Vector2d> EnergyAwareAStar::reconstructPath(
    const std::unordered_map<Node, Node, NodeHash>& came_from,
    const Node& current) {
    
    std::vector<Eigen::Vector2d> path;
    
    // Start with the goal
    Node node = current;
    path_energy_ = node.energy;
    
    // Convert grid coordinates to world coordinates
    Eigen::Vector2d point(node.x * resolution_, node.y * resolution_);
    path.emplace_back(point);
    
    // Follow the path back to the start
    while (came_from.find(node) != came_from.end()) {
        Node prev = came_from.at(node);
        Eigen::Vector2d prev_point(prev.x * resolution_, prev.y * resolution_);
        path.emplace_back(prev_point);
        
        // Calculate path segment length
        double dx = prev_point.x() - point.x();
        double dy = prev_point.y() - point.y();
        path_length_ += std::sqrt(dx * dx + dy * dy);
        
        // Move to previous node
        node = prev;
        point = prev_point;
    }
    
    // Reverse the path to get start-to-goal ordering
    std::reverse(path.begin(), path.end());
    
    return path;
}

std::vector<Eigen::Vector2d> EnergyAwareAStar::planPath() {
    if (!map_) {
        throw std::runtime_error("No map set for planning");
    }
    
    // Clear previous planning results
    path_.clear();
    path_energy_ = 0.0;
    path_length_ = 0.0;
    
    // Convert start and goal to grid coordinates
    int start_x = static_cast<int>(std::round(start_.x() / resolution_));
    int start_y = static_cast<int>(std::round(start_.y() / resolution_));
    int goal_x = static_cast<int>(std::round(goal_.x() / resolution_));
    int goal_y = static_cast<int>(std::round(goal_.y() / resolution_));
    
    // Check if start or goal is in collision
    if (map_->isOccupied(start_.x(), start_.y())) {
        throw std::runtime_error("Start position is in collision");
    }
    
    if (map_->isOccupied(goal_.x(), goal_.y())) {
        throw std::runtime_error("Goal position is in collision");
    }
    
    // Initialize A* search
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    std::unordered_map<Node, double, NodeHash> g_score;
    std::unordered_map<Node, Node, NodeHash> came_from;
    
    // Create start node
    Node start_node(start_x, start_y);
    start_node.h = calculateHeuristic(start_node, goal_x, goal_y);
    
    // Add start node to open set
    open_set.push(start_node);
    g_score[start_node] = 0.0;
    
    // Main A* loop
    while (!open_set.empty()) {
        // Get the node with the lowest f-score
        Node current = open_set.top();
        open_set.pop();
        
        // Check if we reached the goal
        if (current.x == goal_x && current.y == goal_y) {
            path_ = reconstructPath(came_from, current);
            return path_;
        }
        
        // Explore neighbors
        for (Node neighbor : getNeighbors(current)) {
            // If we found a better path to the neighbor
            if (g_score.find(neighbor) == g_score.end() || neighbor.g < g_score[neighbor]) {
                // Update the path
                came_from[neighbor] = current;
                g_score[neighbor] = neighbor.g;
                
                // Calculate heuristic if not calculated yet
                neighbor.h = calculateHeuristic(neighbor, goal_x, goal_y);
                
                // Add to open set
                open_set.push(neighbor);
            }
        }
    }
    
    // No path found
    return path_;
}

} // namespace asv_planning 