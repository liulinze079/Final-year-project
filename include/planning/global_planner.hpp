#ifndef ASV_PLANNING_GLOBAL_PLANNER_HPP
#define ASV_PLANNING_GLOBAL_PLANNER_HPP

#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <string>
#include <memory>

namespace asv_planning {

// Forward declaration
class MultiResolutionGrid;

/**
 * @class GlobalPlanner
 * @brief Base class for global path planning algorithms
 * 
 * This abstract class defines the interface for global path planners in the ASV planning framework.
 * It provides methods for initialization and path planning that derived classes must implement.
 */
class GlobalPlanner {
public:
    /**
     * @brief Constructor for GlobalPlanner
     */
    GlobalPlanner() = default;
    
    /**
     * @brief Virtual destructor for GlobalPlanner
     */
    virtual ~GlobalPlanner() = default;
    
    /**
     * @brief Initialize the planner with environmental map
     * @param map The environmental map to use for planning
     * @return True if initialization is successful
     */
    virtual bool initialize(MultiResolutionGrid* map) = 0;
    
    /**
     * @brief Plan a path from start to goal
     * @param start The start pose
     * @param goal The goal pose
     * @param path The resulting path if planning is successful
     * @return True if a path is found
     */
    virtual bool planPath(const geometry_msgs::msg::PoseStamped& start,
                         const geometry_msgs::msg::PoseStamped& goal,
                         std::vector<geometry_msgs::msg::PoseStamped>& path) = 0;
    
    /**
     * @brief Get the name of the planner
     * @return The planner name
     */
    virtual std::string getName() const {
        return "GlobalPlanner";
    }
    
    /**
     * @brief Convert a vector of poses to a ROS path message
     * @param poses Vector of pose stamped messages
     * @return ROS path message
     */
    nav_msgs::msg::Path posesToPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses) const {
        nav_msgs::msg::Path path;
        
        if (!poses.empty()) {
            path.header = poses.front().header;
            path.poses = poses;
        }
        
        return path;
    }
};

} // namespace asv_planning

#endif // ASV_PLANNING_GLOBAL_PLANNER_HPP 