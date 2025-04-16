#include "../../include/local_planner/local_planner.hpp"
#include <cmath>

namespace asv_planning {

LocalPlanner::LocalPlanner(const std::string& node_name)
    : Node(node_name),
      update_frequency_(5.0),
      local_map_radius_(150.0),
      max_planning_time_(0.5) {
    
    RCLCPP_INFO(get_logger(), "Creating LocalPlanner base node: %s", node_name.c_str());
    
    // Initialize publishers
    path_pub_ = this->create_publisher<asv_energy_aware_planning::msg::PlanningPath>(
        "local_path", 10);
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", 10);
    
    // Initialize subscribers
    global_path_sub_ = this->create_subscription<asv_energy_aware_planning::msg::PlanningPath>(
        "global_path", 10,
        std::bind(&LocalPlanner::globalPathCallback, this, std::placeholders::_1));
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10,
        std::bind(&LocalPlanner::environmentMapCallback, this, std::placeholders::_1));
    
    asv_state_sub_ = this->create_subscription<asv_energy_aware_planning::msg::ASVState>(
        "asv_state", 10,
        std::bind(&LocalPlanner::asvStateCallback, this, std::placeholders::_1));
    
    // Create environment map
    environment_map_ = std::make_shared<EnvironmentMap>();
    
    RCLCPP_INFO(get_logger(), "LocalPlanner base node created");
}

bool LocalPlanner::initialize() {
    RCLCPP_INFO(get_logger(), "Initializing LocalPlanner base");
    
    // Load common parameters
    this->declare_parameter("update_frequency", 5.0);
    this->declare_parameter("local_map_radius", 150.0);
    this->declare_parameter("max_planning_time", 0.5);
    
    update_frequency_ = this->get_parameter("update_frequency").as_double();
    local_map_radius_ = this->get_parameter("local_map_radius").as_double();
    max_planning_time_ = this->get_parameter("max_planning_time").as_double();
    
    RCLCPP_INFO(get_logger(), "LocalPlanner parameters: update_freq=%.1f Hz, map_radius=%.1f m, plan_time=%.2f s",
             update_frequency_, local_map_radius_, max_planning_time_);
    
    return true;
}

void LocalPlanner::globalPathCallback(
    const asv_energy_aware_planning::msg::PlanningPath::SharedPtr path) {
    
    if (!path) {
        RCLCPP_WARN(get_logger(), "Received null global path");
        return;
    }
    
    if (path->waypoints.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty global path");
        return;
    }
    
    RCLCPP_INFO(get_logger(), "Received global path with %zu waypoints", path->waypoints.size());
    
    // Store the global path
    current_global_path_ = path;
    
    // If we have the ASV state, we can adapt the path immediately
    if (asv_state_) {
        // Extract current pose and velocity
        geometry_msgs::msg::Pose current_pose = asv_state_->pose;
        geometry_msgs::msg::Twist current_velocity = asv_state_->velocity;
        
        // Adapt the path to local conditions
        auto local_path = adaptPath(*current_global_path_, current_pose, current_velocity);
        
        // Publish the adapted path
        path_pub_->publish(local_path);
    }
}

void LocalPlanner::environmentMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    
    if (!map) {
        RCLCPP_WARN(get_logger(), "Received null environment map");
        return;
    }
    
    RCLCPP_INFO(get_logger(), "Received environment map: %dx%d cells, resolution: %.2f",
             map->info.width, map->info.height, map->info.resolution);
    
    // Update the environment map
    environment_map_->updateFromOccupancyGrid(*map);
}

void LocalPlanner::asvStateCallback(
    const asv_energy_aware_planning::msg::ASVState::SharedPtr state) {
    
    if (!state) {
        RCLCPP_WARN(get_logger(), "Received null ASV state");
        return;
    }
    
    // Store the ASV state
    asv_state_ = state;
    
    // If we have a global path, we can adapt it with the new state
    if (current_global_path_) {
        // Extract current pose and velocity
        geometry_msgs::msg::Pose current_pose = state->pose;
        geometry_msgs::msg::Twist current_velocity = state->velocity;
        
        // Adapt the path to local conditions
        auto local_path = adaptPath(*current_global_path_, current_pose, current_velocity);
        
        // Publish the adapted path
        path_pub_->publish(local_path);
        
        // Generate and publish velocity commands
        auto commands = pathToCommandTrajectory(local_path);
        
        // If there are commands available, publish the first one
        if (!commands.empty()) {
            cmd_vel_pub_->publish(commands[0]);
        }
    }
}

std::vector<geometry_msgs::msg::Twist> LocalPlanner::pathToCommandTrajectory(
    const asv_energy_aware_planning::msg::PlanningPath& path) {
    
    std::vector<geometry_msgs::msg::Twist> commands;
    
    if (path.waypoints.empty() || path.headings.empty() || path.velocities.empty()) {
        RCLCPP_WARN(get_logger(), "Cannot convert empty path to commands");
        return commands;
    }
    
    // Ensure we have heading and velocity for each waypoint
    size_t min_size = std::min({
        path.waypoints.size(),
        path.headings.size(),
        path.velocities.size()
    });
    
    for (size_t i = 0; i < min_size; ++i) {
        geometry_msgs::msg::Twist cmd;
        
        // Convert heading and velocity to linear x/y velocities
        double heading = path.headings[i];
        double velocity = path.velocities[i];
        
        cmd.linear.x = velocity * std::cos(heading);
        cmd.linear.y = velocity * std::sin(heading);
        cmd.linear.z = 0.0;
        
        // Set angular velocity if available in the path
        if (i < path.waypoints.size() - 1 && i + 1 < path.headings.size()) {
            // Calculate angular velocity based on heading change and time difference
            double next_heading = path.headings[i + 1];
            
            // Ensure heading difference is in [-pi, pi]
            double heading_diff = next_heading - heading;
            while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
            while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
            
            // Estimate time to reach next waypoint
            double time_diff;
            if (i < path.times.size() - 1) {
                time_diff = path.times[i + 1] - path.times[i];
            } else {
                // Estimate based on distance and velocity
                double dx = path.waypoints[i + 1].pose.position.x - path.waypoints[i].pose.position.x;
                double dy = path.waypoints[i + 1].pose.position.y - path.waypoints[i].pose.position.y;
                double distance = std::sqrt(dx * dx + dy * dy);
                time_diff = distance / std::max(0.1, velocity);
            }
            
            // Set angular velocity (ensure non-zero time_diff)
            if (time_diff > 0.001) {
                cmd.angular.z = heading_diff / time_diff;
            } else {
                cmd.angular.z = 0.0;
            }
        } else {
            cmd.angular.z = 0.0;
        }
        
        commands.push_back(cmd);
    }
    
    return commands;
}

} // namespace asv_planning 