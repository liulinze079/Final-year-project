#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "asv_energy_aware_planning/msg/planning_path.hpp"
#include "asv_energy_aware_planning/msg/asv_state.hpp"
#include "../environment_representation/multi_resolution_grid.hpp"
#include "../environment_representation/environment_map.hpp"

namespace asv_planning {

/**
 * @brief Base class for local path planners in ASV navigation
 * 
 * This abstract base class defines the interface for local planners
 * that adapt global paths to local conditions and obstacles.
 */
class LocalPlanner : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     * @param node_name Name of the ROS node
     */
    LocalPlanner(const std::string& node_name);
    
    /**
     * @brief Destructor
     */
    virtual ~LocalPlanner() = default;
    
    /**
     * @brief Initialize the planner
     * @return True if initialization was successful
     */
    virtual bool initialize();
    
    /**
     * @brief Adapt a global path to local conditions
     * @param global_path Global path to adapt
     * @param current_pose Current ASV pose
     * @param current_velocity Current ASV velocity
     * @param energy_aware Whether to consider energy in planning
     * @return Adapted local path
     */
    virtual asv_energy_aware_planning::msg::PlanningPath adaptPath(
        const asv_energy_aware_planning::msg::PlanningPath& global_path,
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Twist& current_velocity,
        bool energy_aware = true) = 0;
    
protected:
    /**
     * @brief Callback for global path updates
     * @param path Global path message
     */
    void globalPathCallback(
        const asv_energy_aware_planning::msg::PlanningPath::SharedPtr path);
    
    /**
     * @brief Callback for environment map updates
     * @param map Environment map message
     */
    void environmentMapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    
    /**
     * @brief Callback for ASV state updates
     * @param state ASV state message
     */
    void asvStateCallback(
        const asv_energy_aware_planning::msg::ASVState::SharedPtr state);
    
    /**
     * @brief Convert path to command trajectory for control
     * @param path Path to convert
     * @return Command trajectory
     */
    std::vector<geometry_msgs::msg::Twist> pathToCommandTrajectory(
        const asv_energy_aware_planning::msg::PlanningPath& path);
    
    // ROS communication
    rclcpp::Subscription<asv_energy_aware_planning::msg::PlanningPath>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<asv_energy_aware_planning::msg::ASVState>::SharedPtr asv_state_sub_;
    rclcpp::Publisher<asv_energy_aware_planning::msg::PlanningPath>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // Environment representation
    std::shared_ptr<EnvironmentMap> environment_map_;
    
    // Path tracking
    asv_energy_aware_planning::msg::PlanningPath::SharedPtr current_global_path_;
    asv_energy_aware_planning::msg::ASVState::SharedPtr asv_state_;
    
    // Parameters
    double update_frequency_;
    double local_map_radius_;
    double max_planning_time_;
};

/**
 * @brief Simple 2D point structure
 */
struct Point2D {
    double x;
    double y;
    
    bool operator==(const Point2D& other) const {
        return x == other.x && y == other.y;
    }
    
    bool operator!=(const Point2D& other) const {
        return !(*this == other);
    }
};

/**
 * @brief Planning parameters for local planning
 */
struct PlanningParameters {
    Point2D start;                 // Start position
    Point2D goal;                  // Goal position
    double start_heading = 0.0;    // Start heading (radians)
    std::array<double, 2> start_velocity = {0.0, 0.0}; // Start velocity [vx, vy]
    double time_limit = 0.5;       // Planning time limit (seconds)
    std::vector<Point2D> global_path; // Global path points
    bool energy_aware = true;      // Whether to consider energy
};

/**
 * @brief Result of local planning
 */
struct PlanningResult {
    bool success = false;          // Whether planning succeeded
    std::vector<Point2D> path;     // Planned path
    int iterations = 0;            // Number of iterations
    double planning_time = 0.0;    // Time spent planning (seconds)
    double energy_cost = 0.0;      // Estimated energy cost
    std::string error_message;     // Error message if planning failed
};

} // namespace asv_planning 