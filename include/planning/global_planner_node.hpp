#ifndef ASV_PLANNING_GLOBAL_PLANNER_NODE_HPP
#define ASV_PLANNING_GLOBAL_PLANNER_NODE_HPP

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <asv_interfaces/msg/planning_path.hpp>
#include <asv_interfaces/srv/get_plan.hpp>
#include "planning/global_planner.hpp"
#include "environment_representation/multi_resolution_grid.hpp"

namespace asv_planning {

/**
 * @class GlobalPlannerNode
 * @brief Base class for ROS2 nodes that provide global planning services
 * 
 * This abstract class defines the interface for global planner nodes. It provides
 * functionality for handling service requests, updating the environmental map,
 * and publishing planned paths.
 */
class GlobalPlannerNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     * @param node_name Name of the node
     */
    GlobalPlannerNode(const std::string& node_name);
    
    /**
     * @brief Virtual destructor
     */
    virtual ~GlobalPlannerNode() = default;
    
protected:
    /**
     * @brief Initialize the planner with environment data
     * @return True if initialization is successful
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief Create the appropriate global planner instance
     * @return A shared pointer to the global planner
     */
    virtual std::shared_ptr<GlobalPlanner> createPlanner() = 0;
    
    /**
     * @brief Handle a planning service request
     * @param request The planning request
     * @param response The planning response
     */
    virtual void handlePlanningRequest(
        const std::shared_ptr<asv_interfaces::srv::GetPlan::Request> request,
        std::shared_ptr<asv_interfaces::srv::GetPlan::Response> response);
    
    /**
     * @brief Handle a map update
     * @param msg The occupancy grid message
     */
    virtual void handleMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    /**
     * @brief Convert ROS Path to PlanningPath message with energy and safety info
     * @param path The ROS path message
     * @param start_pose The start pose
     * @param goal_pose The goal pose
     * @param planning_time Time spent planning in seconds
     * @return The planning path message
     */
    virtual asv_interfaces::msg::PlanningPath pathToPlanningPath(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::PoseStamped& start_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose,
        double planning_time);
    
    // Protected members
    std::shared_ptr<GlobalPlanner> planner_;
    std::shared_ptr<MultiResolutionGrid> map_;
    
    // ROS communication
    rclcpp::Service<asv_interfaces::srv::GetPlan>::SharedPtr planning_service_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<asv_interfaces::msg::PlanningPath>::SharedPtr planning_path_publisher_;
    
    // Parameters
    double planning_timeout_;
    double min_obstacle_distance_;
    double max_obstacle_distance_;
    std::string global_frame_;
};

} // namespace asv_planning

#endif // ASV_PLANNING_GLOBAL_PLANNER_NODE_HPP 