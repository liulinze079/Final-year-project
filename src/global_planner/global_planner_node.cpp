#include "../../include/global_planner/global_planner.hpp"
#include "../../include/global_planner/a_star_planner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

using namespace asv_planning;

/**
 * @brief Implementation of the ASV Global Planner node
 * 
 * This node provides global path planning services for ASV energy-aware navigation.
 * It implements A* search with maritime-specific heuristics to optimize for energy
 * efficiency in dynamic maritime environments.
 */
class AStarGlobalPlannerNode : public GlobalPlannerNode {
public:
    /**
     * @brief Constructor
     */
    AStarGlobalPlannerNode()
        : GlobalPlannerNode("asv_global_planner") {
        RCLCPP_INFO(get_logger(), "Starting ASV Global Planner Node");
        
        // Declare parameters
        this->declare_parameter("heuristic_type", "euclidean");
        this->declare_parameter("heuristic_weight", 1.2);
        this->declare_parameter("allow_diagonal", true);
        this->declare_parameter("min_cell_size", 2.0);
        this->declare_parameter("obstacle_inflation_radius", 5.0);
        this->declare_parameter("current_weight", 0.4);
        this->declare_parameter("wave_weight", 0.3);
        this->declare_parameter("lane_weight", 0.2);
        this->declare_parameter("safety_weight", 0.1);
        this->declare_parameter("shipping_lane_factor", 0.7);
        this->declare_parameter("safety_factor", 1.5);
        this->declare_parameter("energy_aware", true);
        this->declare_parameter("update_frequency", 1.0);
        this->declare_parameter("max_planning_time", 5.0);
        this->declare_parameter("max_path_length", 10000.0);
        this->declare_parameter("planning_horizon", 2000.0);
        
        // Initialize the planner
        initialize();
        
        // Create a timer for updating the planner
        double update_freq = this->get_parameter("update_frequency").as_double();
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_freq)),
            std::bind(&AStarGlobalPlannerNode::update, this));
        
        RCLCPP_INFO(get_logger(), "ASV Global Planner Node initialized");
    }
    
    /**
     * @brief Destructor
     */
    ~AStarGlobalPlannerNode() override = default;
    
    /**
     * @brief Initialize the planner
     * @return True if initialization was successful
     */
    bool initialize() override {
        GlobalPlannerNode::initialize();
        
        // Load parameters
        std::string heuristic_type = this->get_parameter("heuristic_type").as_string();
        double heuristic_weight = this->get_parameter("heuristic_weight").as_double();
        bool allow_diagonal = this->get_parameter("allow_diagonal").as_bool();
        double min_cell_size = this->get_parameter("min_cell_size").as_double();
        double obstacle_inflation = this->get_parameter("obstacle_inflation_radius").as_double();
        
        // Initialize the maritime heuristics
        maritime_heuristics_ = std::make_shared<MaritimeHeuristics>();
        maritime_heuristics_->setCurrentWeight(this->get_parameter("current_weight").as_double());
        maritime_heuristics_->setWaveWeight(this->get_parameter("wave_weight").as_double());
        maritime_heuristics_->setLaneWeight(this->get_parameter("lane_weight").as_double());
        maritime_heuristics_->setSafetyWeight(this->get_parameter("safety_weight").as_double());
        maritime_heuristics_->setShippingLaneFactor(this->get_parameter("shipping_lane_factor").as_double());
        maritime_heuristics_->setSafetyFactor(this->get_parameter("safety_factor").as_double());
        
        // Initialize the A* planner
        planner_ = std::make_shared<AStarPlanner>(maritime_heuristics_);
        planner_->setHeuristicType(heuristic_type);
        planner_->setHeuristicWeight(heuristic_weight);
        planner_->setAllowDiagonal(allow_diagonal);
        planner_->setMinCellSize(min_cell_size);
        planner_->setObstacleInflationRadius(obstacle_inflation);
        planner_->setEnergyAware(this->get_parameter("energy_aware").as_bool());
        planner_->setMaxPlanningTime(this->get_parameter("max_planning_time").as_double());
        planner_->setMaxPathLength(this->get_parameter("max_path_length").as_double());
        planner_->setPlanningHorizon(this->get_parameter("planning_horizon").as_double());
        
        RCLCPP_INFO(get_logger(), "A* Planner initialized with heuristic: %s, weight: %.2f", 
                  heuristic_type.c_str(), heuristic_weight);
        
        return true;
    }
    
    /**
     * @brief Plan a path from start to goal
     * @param start Start pose
     * @param goal Goal pose
     * @param energy_aware Whether to consider energy in planning
     * @return Planned path
     */
    asv_energy_aware_planning::msg::PlanningPath planPath(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal,
        bool energy_aware = true) override {
        
        RCLCPP_INFO(get_logger(), "Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
                  start.position.x, start.position.y,
                  goal.position.x, goal.position.y);
        
        // Update energy awareness setting
        planner_->setEnergyAware(energy_aware);
        
        // Convert start and goal to planner format
        Point2D start_point = {start.position.x, start.position.y};
        Point2D goal_point = {goal.position.x, goal.position.y};
        
        // Plan the path
        auto planning_start_time = std::chrono::high_resolution_clock::now();
        
        PlanningParameters params;
        params.start = start_point;
        params.goal = goal_point;
        params.time_limit = this->get_parameter("max_planning_time").as_double();
        params.distance_threshold = 1.0; // 1 meter threshold
        
        PlanningResult result = planner_->plan(params);
        
        auto planning_end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> planning_time = planning_end_time - planning_start_time;
        
        // Create path message
        asv_energy_aware_planning::msg::PlanningPath path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        path_msg.path_id = "global_path_" + std::to_string(path_id_counter_++);
        path_msg.path_type = "global";
        
        // Set path properties
        path_msg.planning_time = planning_time.count();
        path_msg.iterations = result.iterations;
        path_msg.algorithm_used = "A*";
        path_msg.is_feasible = result.success;
        
        if (result.success) {
            // Convert path to ROS message
            for (const auto& point : result.path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = this->now();
                pose.header.frame_id = "map";
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = 0.0;
                
                // Set orientation (assuming heading is along path)
                // Will be improved with actual heading calculations
                path_msg.waypoints.push_back(pose);
            }
            
            // Set velocities, headings, and times
            // For now, use placeholder values that will be refined by the local planner
            path_msg.velocities.resize(result.path.size(), 2.0); // 2 m/s default
            path_msg.headings.resize(result.path.size(), 0.0);   // 0 rad default
            
            // Calculate times based on distances and velocities
            path_msg.times.resize(result.path.size(), 0.0);
            double cumulative_time = 0.0;
            path_msg.times[0] = 0.0;
            
            for (size_t i = 1; i < result.path.size(); ++i) {
                double dx = result.path[i].x - result.path[i-1].x;
                double dy = result.path[i].y - result.path[i-1].y;
                double distance = std::sqrt(dx*dx + dy*dy);
                double velocity = path_msg.velocities[i-1];
                double segment_time = distance / velocity;
                
                cumulative_time += segment_time;
                path_msg.times[i] = cumulative_time;
            }
            
            // Set the path properties
            path_msg.total_length = result.path_length;
            path_msg.estimated_duration = cumulative_time;
            path_msg.estimated_energy_usage = result.energy_cost;
            path_msg.smoothness = calculatePathSmoothness(result.path);
            path_msg.safety_score = calculatePathSafety(result.path);
            
            RCLCPP_INFO(get_logger(), "Path planning succeeded with %zu points, length: %.2f m, time: %.2f s",
                      result.path.size(), result.path_length, cumulative_time);
        } else {
            RCLCPP_WARN(get_logger(), "Path planning failed: %s", result.error_message.c_str());
        }
        
        // Publish the path
        path_pub_->publish(path_msg);
        
        return path_msg;
    }

private:
    /**
     * @brief Update callback for periodic updates
     */
    void update() {
        // This method will handle periodic updates such as updating the environment map
        // or replanning if necessary
        auto current_time = this->now();
        
        if (environment_map_ && planner_) {
            // Update the planner with the latest environment data
            planner_->updateEnvironmentMap(*environment_map_);
        }
    }
    
    /**
     * @brief Calculate the smoothness of a path
     * @param path The path as a vector of points
     * @return Smoothness metric (0-1, higher is smoother)
     */
    double calculatePathSmoothness(const std::vector<Point2D>& path) const {
        if (path.size() < 3) {
            return 1.0;  // Short paths are considered smooth
        }
        
        double total_angle_change = 0.0;
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            // Calculate vectors for adjacent segments
            double dx1 = path[i].x - path[i-1].x;
            double dy1 = path[i].y - path[i-1].y;
            double dx2 = path[i+1].x - path[i].x;
            double dy2 = path[i+1].y - path[i].y;
            
            // Calculate dot product
            double dot_product = dx1 * dx2 + dy1 * dy2;
            
            // Calculate magnitudes
            double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            double mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            
            // Calculate angle (avoid division by zero)
            if (mag1 > 1e-6 && mag2 > 1e-6) {
                double cos_angle = dot_product / (mag1 * mag2);
                // Clamp to valid range due to potential numerical issues
                cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
                double angle = std::acos(cos_angle);
                total_angle_change += angle;
            }
        }
        
        // Normalize to 0-1 range (0 being the worst, 1 being perfectly smooth)
        double max_possible_angle_change = M_PI * (path.size() - 2);
        return 1.0 - (total_angle_change / max_possible_angle_change);
    }
    
    /**
     * @brief Calculate the safety score of a path
     * @param path The path as a vector of points
     * @return Safety score (0-1, higher is safer)
     */
    double calculatePathSafety(const std::vector<Point2D>& path) const {
        if (!environment_map_) {
            return 0.5;  // Default safety without environment info
        }
        
        double min_obstacle_distance = std::numeric_limits<double>::max();
        double avg_obstacle_distance = 0.0;
        
        for (const auto& point : path) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            
            double distance = getDistanceToNearestObstacle(p);
            min_obstacle_distance = std::min(min_obstacle_distance, distance);
            avg_obstacle_distance += distance;
        }
        
        if (path.size() > 0) {
            avg_obstacle_distance /= path.size();
        }
        
        // Calculate safety score (1.0 is safest)
        double safety_threshold = 20.0;  // Distance considered maximally safe
        double critical_distance = 5.0;  // Distance considered minimally safe
        
        double safety_score = (min_obstacle_distance - critical_distance) / 
                            (safety_threshold - critical_distance);
        
        return std::max(0.0, std::min(1.0, safety_score));
    }
    
    /**
     * @brief Get the distance to the nearest obstacle
     * @param point The point to check
     * @return Distance to nearest obstacle
     */
    double getDistanceToNearestObstacle(const geometry_msgs::msg::Point& point) const {
        // This is a placeholder implementation
        // In a real system, this would use the environment map to compute distance
        return 10.0;  // Default 10 meters
    }

    // A* planner
    std::shared_ptr<AStarPlanner> planner_;
    
    // Maritime heuristics
    std::shared_ptr<MaritimeHeuristics> maritime_heuristics_;
    
    // Timer for periodic updates
    rclcpp::TimerBase::SharedPtr update_timer_;
    
    // Path ID counter for unique identification
    unsigned int path_id_counter_ = 0;
    
    // Path buffer for storing recent paths
    std::vector<asv_energy_aware_planning::msg::PlanningPath> path_buffer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AStarGlobalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 