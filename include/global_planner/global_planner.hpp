#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "asv_energy_aware_planning/msg/planning_path.hpp"
#include "asv_energy_aware_planning/msg/planning_request.hpp"
#include "asv_energy_aware_planning/msg/planning_response.hpp"
#include "../environment_representation/multi_resolution_grid.hpp"
#include "../environment_representation/energy_cost_map.hpp"

namespace asv_planning {

/**
 * @brief Base class for global path planners in ASV navigation
 * 
 * This abstract base class defines the interface for global planners
 * that generate energy-efficient paths for ASVs in maritime environments.
 */
class GlobalPlanner : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     * @param node_name Name of the ROS node
     */
    GlobalPlanner(const std::string& node_name);
    
    /**
     * @brief Destructor
     */
    virtual ~GlobalPlanner() = default;
    
    /**
     * @brief Initialize the planner
     * @return True if initialization was successful
     */
    virtual bool initialize();
    
    /**
     * @brief Plan a path from start to goal
     * @param start_pose Start pose
     * @param goal_pose Goal pose
     * @param consider_energy Whether to consider energy in planning
     * @return Planned path
     */
    virtual asv_energy_aware_planning::msg::PlanningPath planPath(
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Pose& goal_pose,
        bool consider_energy = true) = 0;
    
protected:
    /**
     * @brief Update the environment map
     * @param map The new environment map
     */
    void updateEnvironmentMap(const nav_msgs::msg::OccupancyGrid::SharedPtr& map);
    
    /**
     * @brief Update the energy cost map
     * @param energy_map The new energy cost map
     */
    void updateEnergyCostMap(const EnergyCostMap::SharedPtr& energy_map);
    
    /**
     * @brief Handle planning requests
     * @param request The planning request
     */
    void handlePlanningRequest(
        const asv_energy_aware_planning::msg::PlanningRequest::SharedPtr& request);
    
    /**
     * @brief Get distance to the nearest obstacle
     * @param point The point to check
     * @return Distance to nearest obstacle
     */
    double getDistanceToNearestObstacle(const geometry_msgs::msg::Point& point) const;
    
    /**
     * @brief Extract roll, pitch, yaw from quaternion
     * @param q Quaternion
     * @param roll Roll angle (output)
     * @param pitch Pitch angle (output)
     * @param yaw Yaw angle (output)
     */
    void extractRPYFromQuaternion(
        const geometry_msgs::msg::Quaternion& q,
        double& roll, double& pitch, double& yaw) const;
    
    // ROS communication
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<asv_energy_aware_planning::msg::PlanningRequest>::SharedPtr planning_request_sub_;
    rclcpp::Publisher<asv_energy_aware_planning::msg::PlanningPath>::SharedPtr path_pub_;
    rclcpp::Publisher<asv_energy_aware_planning::msg::PlanningResponse>::SharedPtr planning_response_pub_;
    
    // Environment representation
    std::shared_ptr<MultiResolutionGrid> environment_map_;
    std::shared_ptr<EnergyCostMap> energy_cost_map_;
    
    // Planning metrics
    unsigned int plan_count_;
    double total_planning_time_;
    double max_planning_time_;
    double min_planning_time_;
};

} // namespace asv_planning 