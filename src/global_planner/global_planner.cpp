#include "../../include/global_planner/global_planner.hpp"
#include <cmath>

namespace asv_planning {

GlobalPlanner::GlobalPlanner(const std::string& node_name)
    : Node(node_name),
      plan_count_(0),
      total_planning_time_(0.0),
      max_planning_time_(0.0),
      min_planning_time_(std::numeric_limits<double>::max()) {
    
    RCLCPP_INFO(get_logger(), "Creating GlobalPlanner base node: %s", node_name.c_str());
    
    // Initialize publishers
    path_pub_ = this->create_publisher<asv_energy_aware_planning::msg::PlanningPath>(
        "global_path", 10);
    
    planning_response_pub_ = this->create_publisher<asv_energy_aware_planning::msg::PlanningResponse>(
        "planning_response", 10);
    
    // Initialize subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, 
        std::bind(&GlobalPlanner::updateEnvironmentMap, this, std::placeholders::_1));
    
    planning_request_sub_ = this->create_subscription<asv_energy_aware_planning::msg::PlanningRequest>(
        "planning_request", 10,
        std::bind(&GlobalPlanner::handlePlanningRequest, this, std::placeholders::_1));
    
    // Create environment and energy cost maps
    environment_map_ = std::make_shared<MultiResolutionGrid>();
    energy_cost_map_ = std::make_shared<EnergyCostMap>();
    
    RCLCPP_INFO(get_logger(), "GlobalPlanner base node created");
}

bool GlobalPlanner::initialize() {
    RCLCPP_INFO(get_logger(), "Initializing GlobalPlanner base");
    
    // Load common parameters
    this->declare_parameter("max_planning_time", 1.0);
    max_planning_time_ = this->get_parameter("max_planning_time").as_double();
    
    return true;
}

void GlobalPlanner::updateEnvironmentMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {
    if (!map) {
        RCLCPP_WARN(get_logger(), "Received null environment map");
        return;
    }
    
    RCLCPP_INFO(get_logger(), "Updating environment map: %dx%d cells, resolution: %.2f",
             map->info.width, map->info.height, map->info.resolution);
    
    // Update the multi-resolution grid with the new map data
    environment_map_->updateFromOccupancyGrid(*map);
}

void GlobalPlanner::updateEnergyCostMap(const EnergyCostMap::SharedPtr& energy_map) {
    if (!energy_map) {
        RCLCPP_WARN(get_logger(), "Received null energy cost map");
        return;
    }
    
    RCLCPP_INFO(get_logger(), "Updating energy cost map");
    
    // Update the energy cost map
    energy_cost_map_ = energy_map;
}

void GlobalPlanner::handlePlanningRequest(
    const asv_energy_aware_planning::msg::PlanningRequest::SharedPtr& request) {
    
    if (!request) {
        RCLCPP_ERROR(get_logger(), "Received null planning request");
        return;
    }
    
    RCLCPP_INFO(get_logger(), "Received planning request from (%.2f, %.2f) to (%.2f, %.2f)",
              request->start.position.x, request->start.position.y,
              request->goal.position.x, request->goal.position.y);
    
    // Create response
    auto response = std::make_shared<asv_energy_aware_planning::msg::PlanningResponse>();
    response->request_id = request->request_id;
    response->timestamp = this->now();
    
    try {
        // Call the planPath method to generate a path
        auto path = planPath(request->start, request->goal, request->consider_energy);
        
        // Set response fields
        response->success = true;
        response->path = path;
        response->message = "Path planning successful";
        
        // Publish the path
        path_pub_->publish(path);
    } catch (const std::exception& e) {
        // Handle planning failure
        RCLCPP_ERROR(get_logger(), "Path planning failed: %s", e.what());
        
        response->success = false;
        response->message = "Path planning failed: " + std::string(e.what());
    }
    
    // Publish the response
    planning_response_pub_->publish(*response);
}

double GlobalPlanner::getDistanceToNearestObstacle(const geometry_msgs::msg::Point& point) const {
    if (!environment_map_) {
        return 100.0;  // Default large value if no map is available
    }
    
    // Convert to Eigen vector for the environment map
    Eigen::Vector2d position(point.x, point.y);
    
    return environment_map_->getDistanceToNearestObstacle(position);
}

void GlobalPlanner::extractRPYFromQuaternion(
    const geometry_msgs::msg::Quaternion& q,
    double& roll, double& pitch, double& yaw) const {
    
    // Convert quaternion to Euler angles
    double sqw = q.w * q.w;
    double sqx = q.x * q.x;
    double sqy = q.y * q.y;
    double sqz = q.z * q.z;
    
    // Roll (x-axis rotation)
    double t0 = 2.0 * (q.w * q.x + q.y * q.z);
    double t1 = 1.0 - 2.0 * (sqx + sqy);
    roll = std::atan2(t0, t1);
    
    // Pitch (y-axis rotation)
    double t2 = 2.0 * (q.w * q.y - q.z * q.x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = std::asin(t2);
    
    // Yaw (z-axis rotation)
    double t3 = 2.0 * (q.w * q.z + q.x * q.y);
    double t4 = 1.0 - 2.0 * (sqy + sqz);
    yaw = std::atan2(t3, t4);
}

} // namespace asv_planning 