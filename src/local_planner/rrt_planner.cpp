#include "../../include/local_planner/rrt_planner.hpp"
#include <cmath>
#include <limits>
#include <chrono>
#include <algorithm>

namespace asv_planning {

RRTPlanner::RRTPlanner() 
    : max_iterations_(5000),
      step_size_(3.0),
      goal_bias_(0.1),
      obstacle_bias_(0.05),
      path_bias_(0.3),
      rewire_radius_(10.0),
      max_planning_time_(0.5),
      obstacle_clearance_(5.0),
      local_map_radius_(150.0),
      energy_aware_(true),
      rng_(std::random_device()()),
      dist_(0.0, 1.0) {
}

void RRTPlanner::setMaxIterations(int iterations) {
    max_iterations_ = iterations;
}

void RRTPlanner::setStepSize(double step_size) {
    step_size_ = step_size;
}

void RRTPlanner::setGoalBias(double bias) {
    goal_bias_ = std::max(0.0, std::min(1.0, bias));
}

void RRTPlanner::setObstacleBias(double bias) {
    obstacle_bias_ = std::max(0.0, std::min(1.0, bias));
}

void RRTPlanner::setPathBias(double bias) {
    path_bias_ = std::max(0.0, std::min(1.0, bias));
}

void RRTPlanner::setRewireRadius(double radius) {
    rewire_radius_ = radius;
}

void RRTPlanner::setMaxPlanningTime(double time) {
    max_planning_time_ = time;
}

void RRTPlanner::setObstacleClearance(double clearance) {
    obstacle_clearance_ = clearance;
}

void RRTPlanner::setLocalMapRadius(double radius) {
    local_map_radius_ = radius;
}

void RRTPlanner::setEnergyAware(bool aware) {
    energy_aware_ = aware;
}

void RRTPlanner::setConstraints(std::shared_ptr<MaritimeConstraints> constraints) {
    constraints_ = constraints;
}

void RRTPlanner::updateEnvironmentMap(const EnvironmentMap& map) {
    environment_map_ = map;
}

PlanningResult RRTPlanner::plan(const PlanningParameters& params) {
    PlanningResult result;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Clear the tree
    nodes_.clear();
    
    // Define planning bounds
    double min_x = std::min(params.start.x, params.goal.x) - local_map_radius_;
    double max_x = std::max(params.start.x, params.goal.x) + local_map_radius_;
    double min_y = std::min(params.start.y, params.goal.y) - local_map_radius_;
    double max_y = std::max(params.start.y, params.goal.y) + local_map_radius_;
    std::array<double, 4> bounds = {min_x, max_x, min_y, max_y};
    
    // Check if start and goal are valid
    if (!isPathFree(params.start, params.start)) {
        result.success = false;
        result.error_message = "Start position is invalid (obstacle collision)";
        return result;
    }
    
    // Initialize the tree with the start node
    Node start_node;
    start_node.position = params.start;
    start_node.heading = params.start_heading;
    start_node.cost = 0.0;
    start_node.energy_cost = 0.0;
    start_node.parent_id = -1;
    nodes_.push_back(start_node);
    
    // Goal threshold (distance at which to consider goal reached)
    const double goal_threshold = step_size_ * 1.5;
    
    // Variables to track the best path to goal so far
    int best_goal_idx = -1;
    double best_goal_dist = std::numeric_limits<double>::max();
    
    // Main RRT loop
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // Check time limit
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - start_time;
        if (elapsed.count() > params.time_limit) {
            break;
        }
        
        // Sample a random point (with bias)
        Point2D random_point = samplePoint(params.goal, bounds);
        
        // Find the nearest node in the tree
        int nearest_idx = findNearestNode(random_point);
        if (nearest_idx < 0) {
            continue;
        }
        
        // Extend the tree towards the random point
        int new_idx = extendTree(nearest_idx, random_point);
        if (new_idx < 0) {
            continue;
        }
        
        // Check for rewiring if using RRT* variant
        if (rewire_radius_ > 0.0) {
            std::vector<int> near_indices = findNearNodes(nodes_[new_idx].position, rewire_radius_);
            rewireTree(new_idx, near_indices);
        }
        
        // Check if we're close to the goal
        double dist_to_goal = distance(nodes_[new_idx].position, params.goal);
        if (dist_to_goal < goal_threshold) {
            // If goal is reachable directly, connect
            if (isPathFree(nodes_[new_idx].position, params.goal)) {
                // Direct path to goal is possible
                if (dist_to_goal < best_goal_dist || best_goal_idx < 0) {
                    best_goal_idx = new_idx;
                    best_goal_dist = dist_to_goal;
                }
            }
        }
        
        // Update result iterations
        result.iterations = iter + 1;
    }
    
    // Check if we found a path to the goal
    if (best_goal_idx >= 0) {
        // Extract the path from the tree
        result.path = extractPath(best_goal_idx);
        
        // Add goal to the path if not too close
        if (best_goal_dist > 0.1) {
            result.path.push_back(params.goal);
        }
        
        // Calculate energy cost if enabled
        if (energy_aware_) {
            result.energy_cost = 0.0;
            for (size_t i = 1; i < result.path.size(); ++i) {
                double heading1 = (i > 1) ? calculateHeading(result.path[i-2], result.path[i-1]) : params.start_heading;
                double heading2 = calculateHeading(result.path[i-1], result.path[i]);
                result.energy_cost += calculateEnergyCost(result.path[i-1], result.path[i], heading1, heading2);
            }
        }
        
        result.success = true;
    } else {
        // No path found, but return the closest approach
        int closest_idx = findNearestNode(params.goal);
        if (closest_idx >= 0) {
            result.path = extractPath(closest_idx);
            result.success = false;
            result.error_message = "Could not reach goal, returning best partial path";
        } else {
            result.success = false;
            result.error_message = "Failed to plan a path to the goal";
        }
    }
    
    // Record planning time
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time = end_time - start_time;
    result.planning_time = planning_time.count();
    
    return result;
}

Point2D RRTPlanner::getRandomPoint(const std::array<double, 4>& bounds) {
    Point2D point;
    point.x = bounds[0] + dist_(rng_) * (bounds[1] - bounds[0]);
    point.y = bounds[2] + dist_(rng_) * (bounds[3] - bounds[2]);
    return point;
}

Point2D RRTPlanner::samplePoint(const Point2D& goal, const std::array<double, 4>& bounds) {
    // Randomly choose sampling strategy based on biases
    double r = dist_(rng_);
    
    if (r < goal_bias_) {
        // Sample the goal directly
        return goal;
    } else if (r < goal_bias_ + path_bias_ && !nodes_.empty()) {
        // Sample from the global path if available
        // TODO: Implement more sophisticated path sampling
        return getRandomPoint(bounds);
    } else if (r < goal_bias_ + path_bias_ + obstacle_bias_) {
        // Sample near obstacles (for more interesting paths)
        // TODO: Implement obstacle-biased sampling
        return getRandomPoint(bounds);
    } else {
        // Sample randomly in the bounds
        return getRandomPoint(bounds);
    }
}

int RRTPlanner::findNearestNode(const Point2D& point) {
    if (nodes_.empty()) {
        return -1;
    }
    
    int nearest_idx = 0;
    double min_dist = distance(nodes_[0].position, point);
    
    for (size_t i = 1; i < nodes_.size(); ++i) {
        double dist = distance(nodes_[i].position, point);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = static_cast<int>(i);
        }
    }
    
    return nearest_idx;
}

int RRTPlanner::extendTree(int nearest_idx, const Point2D& point) {
    if (nearest_idx < 0 || nearest_idx >= static_cast<int>(nodes_.size())) {
        return -1;
    }
    
    // Get the nearest node
    const Node& nearest_node = nodes_[nearest_idx];
    
    // Calculate direction towards the random point
    double dx = point.x - nearest_node.position.x;
    double dy = point.y - nearest_node.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    // If already close enough, return current node
    if (dist < 0.01) {
        return nearest_idx;
    }
    
    // Normalize direction and scale by step size
    dx = dx / dist * std::min(step_size_, dist);
    dy = dy / dist * std::min(step_size_, dist);
    
    // Calculate new position
    Point2D new_position;
    new_position.x = nearest_node.position.x + dx;
    new_position.y = nearest_node.position.y + dy;
    
    // Check if the path is collision-free
    if (!isPathFree(nearest_node.position, new_position)) {
        return -1;
    }
    
    // Calculate new heading (direction of travel)
    double new_heading = std::atan2(dy, dx);
    
    // Check motion constraints if available
    if (constraints_) {
        if (!constraints_->checkMotionConstraints(
                nearest_node.position, new_position,
                nearest_node.heading, new_heading, 1.0)) {
            return -1;
        }
    }
    
    // Calculate cost and energy cost
    double segment_cost = calculateCost(nearest_node.position, new_position, 
                                      nearest_node.heading, new_heading);
    double segment_energy = energy_aware_ ? 
        calculateEnergyCost(nearest_node.position, new_position, 
                          nearest_node.heading, new_heading) : 0.0;
    
    // Create the new node
    Node new_node;
    new_node.position = new_position;
    new_node.heading = new_heading;
    new_node.cost = nearest_node.cost + segment_cost;
    new_node.energy_cost = nearest_node.energy_cost + segment_energy;
    new_node.parent_id = nearest_idx;
    
    // Add the new node to the tree
    int new_idx = static_cast<int>(nodes_.size());
    nodes_.push_back(new_node);
    
    // Update the parent's children
    nodes_[nearest_idx].children.push_back(new_idx);
    
    return new_idx;
}

std::vector<int> RRTPlanner::findNearNodes(const Point2D& point, double radius) {
    std::vector<int> near_indices;
    
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (distance(nodes_[i].position, point) <= radius) {
            near_indices.push_back(static_cast<int>(i));
        }
    }
    
    return near_indices;
}

void RRTPlanner::rewireTree(int new_idx, const std::vector<int>& near_indices) {
    if (new_idx < 0 || new_idx >= static_cast<int>(nodes_.size())) {
        return;
    }
    
    Node& new_node = nodes_[new_idx];
    
    // Check each nearby node to see if rerouting through the new node is cheaper
    for (int near_idx : near_indices) {
        // Skip parent of the new node
        if (near_idx == new_node.parent_id || near_idx == new_idx) {
            continue;
        }
        
        Node& near_node = nodes_[near_idx];
        
        // Check if path is collision-free
        if (!isPathFree(new_node.position, near_node.position)) {
            continue;
        }
        
        // Calculate potential new heading
        double new_heading = calculateHeading(new_node.position, near_node.position);
        
        // Check motion constraints
        if (constraints_ && !constraints_->checkMotionConstraints(
                new_node.position, near_node.position,
                new_node.heading, new_heading, 1.0)) {
            continue;
        }
        
        // Calculate cost of the new potential path
        double segment_cost = calculateCost(new_node.position, near_node.position,
                                          new_node.heading, new_heading);
        double new_cost = new_node.cost + segment_cost;
        
        // If the new path is cheaper, rewire
        if (new_cost < near_node.cost) {
            // Remove near_node from its parent's children
            int old_parent = near_node.parent_id;
            if (old_parent >= 0) {
                auto& children = nodes_[old_parent].children;
                children.erase(std::remove(children.begin(), children.end(), near_idx), children.end());
            }
            
            // Update near_node's parent and cost
            near_node.parent_id = new_idx;
            near_node.cost = new_cost;
            
            // Recalculate energy cost if needed
            if (energy_aware_) {
                double segment_energy = calculateEnergyCost(new_node.position, near_node.position,
                                                          new_node.heading, new_heading);
                near_node.energy_cost = new_node.energy_cost + segment_energy;
            }
            
            // Add near_node to new_node's children
            new_node.children.push_back(near_idx);
            
            // Update heading
            near_node.heading = new_heading;
            
            // Recursively update the costs of all descendants
            // Note: This is a simplification and could be optimized
            // TODO: Implement more efficient cost propagation
        }
    }
}

bool RRTPlanner::isPathFree(const Point2D& from, const Point2D& to) {
    // Simple line collision check
    // In a real implementation, this would check against the environment map
    
    // Number of interpolation points
    const int steps = std::max(1, static_cast<int>(distance(from, to) / (step_size_ / 2.0)));
    
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        double x = from.x + t * (to.x - from.x);
        double y = from.y + t * (to.y - from.y);
        
        // Check if point is in collision with obstacles
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        
        // Using environment map to check obstacle distance
        // This is a simplification - in a real system you'd use a proper collision checker
        if (environment_map_.getDistanceToNearestObstacle(point) < obstacle_clearance_) {
            return false;  // Collision detected
        }
    }
    
    return true;  // Path is free
}

double RRTPlanner::calculateCost(const Point2D& from, const Point2D& to, 
                               double from_heading, double to_heading) {
    // Basic cost is the distance
    double dist_cost = distance(from, to);
    
    // Add turning cost (penalize sharp turns)
    double heading_diff = std::abs(to_heading - from_heading);
    while (heading_diff > M_PI) heading_diff = 2.0 * M_PI - heading_diff;
    double turn_cost = heading_diff * 0.5;  // Scale factor for turning cost
    
    // Total cost
    double total_cost = dist_cost + turn_cost;
    
    // If energy-aware, incorporate environmental factors
    if (energy_aware_ && constraints_) {
        // Add energy-related costs based on maritime environment
        // This is a simplification - in a real system you'd use a more sophisticated model
        geometry_msgs::msg::Point midpoint;
        midpoint.x = (from.x + to.x) / 2.0;
        midpoint.y = (from.y + to.y) / 2.0;
        
        // Get safety weight
        double safety_weight = constraints_->getSafetyWeight();
        
        // Distance to nearest obstacle as safety factor
        double obstacle_dist = environment_map_.getDistanceToNearestObstacle(midpoint);
        double safety_factor = std::max(0.0, 1.0 - obstacle_dist / 20.0);
        
        // Add safety cost
        total_cost += safety_weight * safety_factor * dist_cost;
    }
    
    return total_cost;
}

double RRTPlanner::calculateEnergyCost(const Point2D& from, const Point2D& to,
                                     double from_heading, double to_heading) {
    // Basic energy cost model based on distance and turning
    double dist = distance(from, to);
    
    // Base energy consumption proportional to distance
    double base_energy = dist * 1.0;  // 1.0 energy unit per distance unit
    
    // Additional energy for turning (proportional to heading change)
    double heading_diff = std::abs(to_heading - from_heading);
    while (heading_diff > M_PI) heading_diff = 2.0 * M_PI - heading_diff;
    double turn_energy = heading_diff * 0.2 * dist;  // Scale factor for turning energy
    
    // Environmental factors (if environment map is available)
    double env_factor = 1.0;
    
    // Calculate midpoint for environmental sampling
    geometry_msgs::msg::Point midpoint;
    midpoint.x = (from.x + to.x) / 2.0;
    midpoint.y = (from.y + to.y) / 2.0;
    
    // Get current and wave factors
    // This is a simplification - in a real system you'd use a more sophisticated model
    double current_factor = 1.0;
    double wave_factor = 1.0;
    
    // Incorporate current effect on energy (simplified)
    // If traveling against current, energy usage increases
    current_factor = environment_map_.getCurrentFactor(midpoint, to_heading);
    
    // Incorporate wave effect (simplified)
    // Higher waves increase energy usage
    wave_factor = environment_map_.getWaveFactor(midpoint);
    
    // Combine environmental factors
    env_factor = 0.7 * current_factor + 0.3 * wave_factor;
    
    // Total energy cost including environmental effects
    double total_energy = (base_energy + turn_energy) * env_factor;
    
    return total_energy;
}

std::vector<Point2D> RRTPlanner::extractPath(int goal_idx) {
    std::vector<Point2D> path;
    
    if (goal_idx < 0 || goal_idx >= static_cast<int>(nodes_.size())) {
        return path;
    }
    
    // Trace back from goal to start
    int current_idx = goal_idx;
    while (current_idx >= 0) {
        path.push_back(nodes_[current_idx].position);
        current_idx = nodes_[current_idx].parent_id;
    }
    
    // Reverse to get path from start to goal
    std::reverse(path.begin(), path.end());
    
    return path;
}

double RRTPlanner::calculateHeading(const Point2D& from, const Point2D& to) {
    return std::atan2(to.y - from.y, to.x - from.x);
}

double RRTPlanner::calculatePathCost(const std::vector<Point2D>& path) {
    double total_cost = 0.0;
    
    for (size_t i = 1; i < path.size(); ++i) {
        double heading1 = (i > 1) ? calculateHeading(path[i-2], path[i-1]) : 0.0;
        double heading2 = calculateHeading(path[i-1], path[i]);
        total_cost += calculateCost(path[i-1], path[i], heading1, heading2);
    }
    
    return total_cost;
}

double RRTPlanner::distance(const Point2D& p1, const Point2D& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace asv_planning 