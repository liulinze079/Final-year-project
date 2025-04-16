#include "../../include/local_planner/local_planner.hpp"
#include "../../include/local_planner/rrt_planner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <chrono>
#include <string>
#include <limits>
#include <yaml-cpp/yaml.h>

using namespace asv_planning;

/**
 * @brief Implementation of the ASV Local Planner node
 * 
 * This node provides reactive local path planning for ASV energy-aware navigation.
 * It adapts global paths to avoid dynamic obstacles and optimize energy usage
 * based on real-time environmental information.
 */
class RRTLocalPlannerNode : public LocalPlanner {
public:
    /**
     * @brief Constructor
     */
    RRTLocalPlannerNode()
        : LocalPlanner("asv_local_planner") {
        RCLCPP_INFO(get_logger(), "Starting ASV Local Planner Node");
        
        // Declare parameters
        this->declare_parameter("max_iterations", 5000);
        this->declare_parameter("step_size", 3.0);
        this->declare_parameter("goal_bias", 0.1);
        this->declare_parameter("obstacle_bias", 0.05);
        this->declare_parameter("path_bias", 0.3);
        this->declare_parameter("max_planning_time", 0.5);
        this->declare_parameter("obstacle_clearance", 5.0);
        this->declare_parameter("update_frequency", 5.0);
        this->declare_parameter("local_map_radius", 150.0);
        this->declare_parameter("waypoint_threshold", 5.0);
        this->declare_parameter("rewire_radius", 10.0);
        this->declare_parameter("max_planner_speed", 5.0);
        this->declare_parameter("max_planner_turning_rate", 0.5);
        this->declare_parameter("energy_aware", true);
        this->declare_parameter("smoothing_factor", 0.3);
        this->declare_parameter("safety_weight", 0.6);
        this->declare_parameter("efficiency_weight", 0.4);
        
        // Initialize the planner
        initialize();
        
        // Create a timer for updating the planner
        double update_freq = this->get_parameter("update_frequency").as_double();
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_freq)),
            std::bind(&RRTLocalPlannerNode::update, this));
        
        RCLCPP_INFO(get_logger(), "ASV Local Planner Node initialized");
    }
    
    /**
     * @brief Destructor
     */
    ~RRTLocalPlannerNode() override = default;
    
    /**
     * @brief Initialize the planner
     * @return True if initialization was successful
     */
    bool initialize() override {
        LocalPlanner::initialize();
        
        // Load parameters
        int max_iterations = this->get_parameter("max_iterations").as_int();
        double step_size = this->get_parameter("step_size").as_double();
        double goal_bias = this->get_parameter("goal_bias").as_double();
        double obstacle_bias = this->get_parameter("obstacle_bias").as_double();
        double path_bias = this->get_parameter("path_bias").as_double();
        double planning_time = this->get_parameter("max_planning_time").as_double();
        double obstacle_clearance = this->get_parameter("obstacle_clearance").as_double();
        double local_map_radius = this->get_parameter("local_map_radius").as_double();
        double rewire_radius = this->get_parameter("rewire_radius").as_double();
        
        // Initialize the RRT planner
        planner_ = std::make_shared<RRTPlanner>();
        planner_->setMaxIterations(max_iterations);
        planner_->setStepSize(step_size);
        planner_->setGoalBias(goal_bias);
        planner_->setObstacleBias(obstacle_bias);
        planner_->setPathBias(path_bias);
        planner_->setRewireRadius(rewire_radius);
        planner_->setMaxPlanningTime(planning_time);
        planner_->setObstacleClearance(obstacle_clearance);
        planner_->setLocalMapRadius(local_map_radius);
        planner_->setEnergyAware(this->get_parameter("energy_aware").as_bool());
        
        // Initialize maritime navigation constraints
        maritime_constraints_ = std::make_shared<MaritimeConstraints>();
        maritime_constraints_->setMaxSpeed(this->get_parameter("max_planner_speed").as_double());
        maritime_constraints_->setMaxTurningRate(this->get_parameter("max_planner_turning_rate").as_double());
        maritime_constraints_->setSafetyWeight(this->get_parameter("safety_weight").as_double());
        maritime_constraints_->setEfficiencyWeight(this->get_parameter("efficiency_weight").as_double());
        
        // Apply constraints to planner
        planner_->setConstraints(maritime_constraints_);
        
        RCLCPP_INFO(get_logger(), "RRT Planner initialized with iterations: %d, step: %.2f, goal bias: %.2f",
                  max_iterations, step_size, goal_bias);
        
        return true;
    }
    
    /**
     * @brief Adapt the global path to local conditions
     * @param global_path Global path to adapt
     * @param current_pose Current ASV pose
     * @param current_velocity Current ASV velocity
     * @param energy_aware Whether to consider energy in planning
     * @return Adapted local path
     */
    asv_energy_aware_planning::msg::PlanningPath adaptPath(
        const asv_energy_aware_planning::msg::PlanningPath& global_path,
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Twist& current_velocity,
        bool energy_aware = true) override {
        
        RCLCPP_INFO(get_logger(), "Adapting global path to local conditions from (%.2f, %.2f)",
                  current_pose.position.x, current_pose.position.y);
        
        // Update energy awareness setting
        planner_->setEnergyAware(energy_aware);
        
        // Find the next waypoint to follow from the global path
        size_t next_waypoint_idx = findNextWaypoint(global_path, current_pose);
        
        // Extract relevant portion of the global path
        std::vector<Point2D> relevant_path;
        std::vector<double> path_headings;
        std::vector<double> path_velocities;
        
        // Get waypoint threshold
        double waypoint_threshold = this->get_parameter("waypoint_threshold").as_double();
        
        // Extract relevant portion of the global path (current + next few waypoints)
        for (size_t i = next_waypoint_idx; i < global_path.waypoints.size() && 
             i < next_waypoint_idx + 10; ++i) {
            Point2D point = {
                global_path.waypoints[i].pose.position.x,
                global_path.waypoints[i].pose.position.y
            };
            relevant_path.push_back(point);
            
            // Also extract headings and velocities if available
            if (i < global_path.headings.size()) {
                path_headings.push_back(global_path.headings[i]);
            } else {
                path_headings.push_back(0.0); // Default
            }
            
            if (i < global_path.velocities.size()) {
                path_velocities.push_back(global_path.velocities[i]);
            } else {
                path_velocities.push_back(2.0); // Default 2 m/s
            }
        }
        
        // Set up start and goal
        Point2D start_point = {current_pose.position.x, current_pose.position.y};
        Point2D goal_point;
        
        // If no relevant path or at the end, use last point
        if (relevant_path.empty()) {
            goal_point = {
                global_path.waypoints.back().pose.position.x,
                global_path.waypoints.back().pose.position.y
            };
        } else {
            // Use the last point in the relevant path as the goal
            goal_point = relevant_path.back();
        }
        
        // Set up planning parameters
        PlanningParameters params;
        params.start = start_point;
        params.goal = goal_point;
        params.time_limit = this->get_parameter("max_planning_time").as_double();
        params.global_path = relevant_path;
        
        // Get current velocity vector
        params.start_velocity = {current_velocity.linear.x, current_velocity.linear.y};
        
        // Current heading from quaternion
        double roll, pitch, yaw;
        extractRPYFromQuaternion(current_pose.orientation, roll, pitch, yaw);
        params.start_heading = yaw;
        
        // Plan the path
        auto planning_start_time = std::chrono::high_resolution_clock::now();
        PlanningResult result = planner_->plan(params);
        auto planning_end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> planning_time = planning_end_time - planning_start_time;
        
        // Create the local path message
        asv_energy_aware_planning::msg::PlanningPath local_path;
        local_path.header.stamp = this->now();
        local_path.header.frame_id = "map";
        local_path.path_id = "local_path_" + std::to_string(path_id_counter_++);
        local_path.path_type = "local";
        local_path.related_path_id = global_path.path_id;
        
        // Set path properties
        local_path.planning_time = planning_time.count();
        local_path.iterations = result.iterations;
        local_path.algorithm_used = "RRT";
        local_path.is_feasible = result.success;
        
        if (result.success) {
            // Smooth the path if needed
            std::vector<Point2D> smooth_path;
            if (this->get_parameter("smoothing_factor").as_double() > 0.0) {
                smooth_path = smoothPath(result.path, this->get_parameter("smoothing_factor").as_double());
            } else {
                smooth_path = result.path;
            }
            
            // Convert path to ROS message
            for (const auto& point : smooth_path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = this->now();
                pose.header.frame_id = "map";
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = 0.0;
                
                // Add to path
                local_path.waypoints.push_back(pose);
            }
            
            // Calculate path metrics
            double path_length = 0.0;
            for (size_t i = 1; i < smooth_path.size(); ++i) {
                double dx = smooth_path[i].x - smooth_path[i-1].x;
                double dy = smooth_path[i].y - smooth_path[i-1].y;
                path_length += std::sqrt(dx*dx + dy*dy);
            }
            
            // Calculate headings along the path
            std::vector<double> headings;
            for (size_t i = 0; i < smooth_path.size(); ++i) {
                if (i < smooth_path.size() - 1) {
                    // Calculate heading from current point to next point
                    double dx = smooth_path[i+1].x - smooth_path[i].x;
                    double dy = smooth_path[i+1].y - smooth_path[i].y;
                    double heading = std::atan2(dy, dx);
                    headings.push_back(heading);
                } else if (!headings.empty()) {
                    // For the last point, use the last calculated heading
                    headings.push_back(headings.back());
                } else {
                    // If there's only one point, use the current heading
                    headings.push_back(params.start_heading);
                }
            }
            
            // Calculate velocities along the path
            std::vector<double> velocities;
            double max_speed = this->get_parameter("max_planner_speed").as_double();
            
            // Start with current velocity
            double current_speed = std::sqrt(current_velocity.linear.x * current_velocity.linear.x + 
                                          current_velocity.linear.y * current_velocity.linear.y);
            
            for (size_t i = 0; i < smooth_path.size(); ++i) {
                // Calculate a reasonable velocity based on curvature and obstacles
                double speed;
                
                if (i < smooth_path.size() - 1) {
                    // Check for upcoming turns
                    double curvature = 0.0;
                    if (i < smooth_path.size() - 2) {
                        curvature = calculateCurvature(
                            smooth_path[i], smooth_path[i+1], smooth_path[i+2]);
                    }
                    
                    // Reduce speed for curves
                    double curvature_factor = 1.0 / (1.0 + 5.0 * curvature);
                    
                    // Check for obstacles
                    Point2D point = smooth_path[i];
                    geometry_msgs::msg::Point p;
                    p.x = point.x;
                    p.y = point.y;
                    double obstacle_distance = getDistanceToNearestObstacle(p);
                    
                    // Reduce speed near obstacles
                    double obstacle_factor = std::min(1.0, obstacle_distance / 20.0);
                    
                    // Calculate speed
                    speed = max_speed * std::min(curvature_factor, obstacle_factor);
                } else {
                    // For the last point, slow down to half the previous speed
                    speed = velocities.empty() ? current_speed : velocities.back() * 0.5;
                }
                
                // Ensure speed is within limits
                speed = std::max(0.5, std::min(speed, max_speed));
                velocities.push_back(speed);
            }
            
            // Calculate times based on distances and velocities
            std::vector<double> times(smooth_path.size(), 0.0);
            double cumulative_time = 0.0;
            
            for (size_t i = 1; i < smooth_path.size(); ++i) {
                double dx = smooth_path[i].x - smooth_path[i-1].x;
                double dy = smooth_path[i].y - smooth_path[i-1].y;
                double distance = std::sqrt(dx*dx + dy*dy);
                double velocity = velocities[i-1];
                double segment_time = distance / velocity;
                
                cumulative_time += segment_time;
                times[i] = cumulative_time;
            }
            
            // Add path details to the message
            local_path.headings = headings;
            local_path.velocities = velocities;
            local_path.times = times;
            local_path.total_length = path_length;
            local_path.estimated_duration = cumulative_time;
            local_path.estimated_energy_usage = result.energy_cost;
            local_path.smoothness = calculatePathSmoothness(smooth_path);
            local_path.safety_score = calculatePathSafety(smooth_path);
            
            RCLCPP_INFO(get_logger(), "Local path planning succeeded with %zu points, length: %.2f m, time: %.2f s",
                      smooth_path.size(), path_length, cumulative_time);
        } else {
            RCLCPP_WARN(get_logger(), "Local path planning failed: %s", result.error_message.c_str());
            
            // In case of failure, use a fallback strategy
            // For example, follow the global path directly or implement emergency stop
            if (!global_path.waypoints.empty()) {
                RCLCPP_INFO(get_logger(), "Using global path as fallback");
                local_path = generateFallbackPath(global_path, current_pose, next_waypoint_idx);
            }
        }
        
        // Publish the path
        path_pub_->publish(local_path);
        
        return local_path;
    }

private:
    /**
     * @brief Extract roll, pitch, yaw from quaternion
     * @param q Quaternion
     * @param roll Roll angle (output)
     * @param pitch Pitch angle (output)
     * @param yaw Yaw angle (output)
     */
    void extractRPYFromQuaternion(const geometry_msgs::msg::Quaternion& q,
                                double& roll, double& pitch, double& yaw) const {
        // Convert quaternion to Euler angles
        // This is a simplified version using quaternion components directly
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
    
    /**
     * @brief Find the next waypoint to follow on the global path
     * @param global_path Global path
     * @param current_pose Current ASV pose
     * @return Index of the next waypoint
     */
    size_t findNextWaypoint(
        const asv_energy_aware_planning::msg::PlanningPath& global_path,
        const geometry_msgs::msg::Pose& current_pose) const {
        
        if (global_path.waypoints.empty()) {
            return 0;
        }
        
        double waypoint_threshold = this->get_parameter("waypoint_threshold").as_double();
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        
        // Find the closest waypoint
        for (size_t i = 0; i < global_path.waypoints.size(); ++i) {
            double dx = global_path.waypoints[i].pose.position.x - current_pose.position.x;
            double dy = global_path.waypoints[i].pose.position.y - current_pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        // Find the first waypoint ahead of the closest one
        size_t next_idx = closest_idx;
        bool found_ahead = false;
        
        // Get current heading
        double roll, pitch, yaw;
        extractRPYFromQuaternion(current_pose.orientation, roll, pitch, yaw);
        
        // Unit vector in direction of heading
        double hx = std::cos(yaw);
        double hy = std::sin(yaw);
        
        for (size_t i = closest_idx; i < global_path.waypoints.size(); ++i) {
            double dx = global_path.waypoints[i].pose.position.x - current_pose.position.x;
            double dy = global_path.waypoints[i].pose.position.y - current_pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            // Check if waypoint is ahead (positive dot product with heading vector)
            double dot_product = dx * hx + dy * hy;
            
            if (dot_product > 0 && dist > waypoint_threshold) {
                next_idx = i;
                found_ahead = true;
                break;
            }
        }
        
        // If no waypoint ahead, use the closest one
        if (!found_ahead) {
            next_idx = closest_idx;
        }
        
        return next_idx;
    }
    
    /**
     * @brief Smooth a path using path smoothing algorithm
     * @param path Original path
     * @param smoothing_factor Smoothing factor (0-1)
     * @return Smoothed path
     */
    std::vector<Point2D> smoothPath(
        const std::vector<Point2D>& path,
        double smoothing_factor) const {
        
        if (path.size() <= 2) {
            return path;  // No need to smooth very short paths
        }
        
        std::vector<Point2D> smoothed_path = path;
        
        // Smooth the path using a moving average
        for (int iter = 0; iter < 5; ++iter) {  // Multiple iterations for better smoothing
            std::vector<Point2D> temp_path = smoothed_path;
            
            for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
                // Weight for current point
                double weight = 1.0 - smoothing_factor;
                
                // Apply weighted average
                temp_path[i].x = weight * smoothed_path[i].x + 
                               (1.0 - weight) * 0.5 * (smoothed_path[i-1].x + smoothed_path[i+1].x);
                temp_path[i].y = weight * smoothed_path[i].y + 
                               (1.0 - weight) * 0.5 * (smoothed_path[i-1].y + smoothed_path[i+1].y);
            }
            
            smoothed_path = temp_path;
        }
        
        return smoothed_path;
    }
    
    /**
     * @brief Generate a fallback path when planning fails
     * @param global_path Original global path
     * @param current_pose Current ASV pose
     * @param next_waypoint_idx Index of the next waypoint
     * @return Fallback path
     */
    asv_energy_aware_planning::msg::PlanningPath generateFallbackPath(
        const asv_energy_aware_planning::msg::PlanningPath& global_path,
        const geometry_msgs::msg::Pose& current_pose,
        size_t next_waypoint_idx) const {
        
        // Create a simplified path message
        asv_energy_aware_planning::msg::PlanningPath fallback_path;
        fallback_path.header.stamp = this->now();
        fallback_path.header.frame_id = "map";
        fallback_path.path_id = "fallback_path_" + std::to_string(path_id_counter_);
        fallback_path.path_type = "fallback";
        fallback_path.related_path_id = global_path.path_id;
        fallback_path.algorithm_used = "Direct";
        fallback_path.is_feasible = true;
        
        // Add the current position as the first waypoint
        geometry_msgs::msg::PoseStamped current_waypoint;
        current_waypoint.header.stamp = this->now();
        current_waypoint.header.frame_id = "map";
        current_waypoint.pose = current_pose;
        fallback_path.waypoints.push_back(current_waypoint);
        
        // Add a subset of global path waypoints
        for (size_t i = next_waypoint_idx; 
             i < global_path.waypoints.size() && i < next_waypoint_idx + 5; ++i) {
            fallback_path.waypoints.push_back(global_path.waypoints[i]);
        }
        
        // Calculate basic path metrics
        double path_length = 0.0;
        std::vector<double> velocities;
        std::vector<double> headings;
        
        // Default speed and first point heading
        double default_speed = 1.0;  // Slower speed for safety
        double first_heading = 0.0;
        
        // Calculate heading of first segment
        if (fallback_path.waypoints.size() > 1) {
            double dx = fallback_path.waypoints[1].pose.position.x - fallback_path.waypoints[0].pose.position.x;
            double dy = fallback_path.waypoints[1].pose.position.y - fallback_path.waypoints[0].pose.position.y;
            first_heading = std::atan2(dy, dx);
        } else {
            // Extract yaw from current pose
            double roll, pitch, yaw;
            extractRPYFromQuaternion(current_pose.orientation, roll, pitch, yaw);
            first_heading = yaw;
        }
        
        headings.push_back(first_heading);
        velocities.push_back(default_speed);
        
        // Calculate path length, headings, and velocities
        for (size_t i = 1; i < fallback_path.waypoints.size(); ++i) {
            double dx = fallback_path.waypoints[i].pose.position.x - fallback_path.waypoints[i-1].pose.position.x;
            double dy = fallback_path.waypoints[i].pose.position.y - fallback_path.waypoints[i-1].pose.position.y;
            double segment_length = std::sqrt(dx*dx + dy*dy);
            path_length += segment_length;
            
            // Calculate heading for this segment
            double heading = std::atan2(dy, dx);
            headings.push_back(heading);
            
            // Use a constant reduced velocity for fallback
            velocities.push_back(default_speed);
        }
        
        // Calculate times based on distances and velocities
        std::vector<double> times(fallback_path.waypoints.size(), 0.0);
        double cumulative_time = 0.0;
        
        for (size_t i = 1; i < fallback_path.waypoints.size(); ++i) {
            double dx = fallback_path.waypoints[i].pose.position.x - fallback_path.waypoints[i-1].pose.position.x;
            double dy = fallback_path.waypoints[i].pose.position.y - fallback_path.waypoints[i-1].pose.position.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            double segment_time = distance / default_speed;
            
            cumulative_time += segment_time;
            times[i] = cumulative_time;
        }
        
        // Set path details
        fallback_path.headings = headings;
        fallback_path.velocities = velocities;
        fallback_path.times = times;
        fallback_path.total_length = path_length;
        fallback_path.estimated_duration = cumulative_time;
        fallback_path.estimated_energy_usage = path_length * 1.5;  // Rough estimate
        fallback_path.smoothness = 0.7;  // Assumed value
        fallback_path.safety_score = 0.5;  // Assumed value
        
        return fallback_path;
    }
    
    /**
     * @brief Calculate the curvature at a point based on three consecutive points
     * @param p1 First point
     * @param p2 Middle point
     * @param p3 Third point
     * @return Curvature
     */
    double calculateCurvature(
        const Point2D& p1,
        const Point2D& p2,
        const Point2D& p3) const {
        
        // Calculate vectors
        double v1x = p2.x - p1.x;
        double v1y = p2.y - p1.y;
        double v2x = p3.x - p2.x;
        double v2y = p3.y - p2.y;
        
        // Calculate segment lengths
        double len1 = std::sqrt(v1x*v1x + v1y*v1y);
        double len2 = std::sqrt(v2x*v2x + v2y*v2y);
        
        // Avoid division by zero
        if (len1 < 1e-6 || len2 < 1e-6) {
            return 0.0;
        }
        
        // Normalize vectors
        v1x /= len1;
        v1y /= len1;
        v2x /= len2;
        v2y /= len2;
        
        // Calculate dot product and cross product
        double dot_product = v1x * v2x + v1y * v2y;
        double cross_product = v1x * v2y - v1y * v2x;
        
        // Clamp to valid range due to potential numerical issues
        dot_product = std::max(-1.0, std::min(1.0, dot_product));
        
        // Calculate angle
        double angle = std::acos(dot_product);
        
        // Adjust for the direction of turn
        if (cross_product < 0) {
            angle = -angle;
        }
        
        // Curvature is the angle divided by the average segment length
        double avg_length = (len1 + len2) / 2.0;
        return std::abs(angle) / avg_length;
    }
    
    /**
     * @brief Update callback for periodic updates
     */
    void update() {
        // This method will handle periodic updates such as updating the environment map
        // or adapting the path if needed
        auto current_time = this->now();
        
        if (environment_map_ && planner_) {
            // Update the planner with the latest environment data
            planner_->updateEnvironmentMap(*environment_map_);
        }
        
        // Check if we need to replan based on new obstacles or path deviations
        if (current_path_ && asv_state_) {
            // Potential replanning logic goes here
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

    // RRT planner
    std::shared_ptr<RRTPlanner> planner_;
    
    // Maritime navigation constraints
    std::shared_ptr<MaritimeConstraints> maritime_constraints_;
    
    // Timer for periodic updates
    rclcpp::TimerBase::SharedPtr update_timer_;
    
    // Path ID counter for unique identification
    mutable unsigned int path_id_counter_ = 0;
    
    // Current path being followed
    std::shared_ptr<asv_energy_aware_planning::msg::PlanningPath> current_path_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRTLocalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 