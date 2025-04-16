#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <random>
#include <functional>
#include <unordered_map>
#include <asv_dynamics.hpp>
#include <map_representation.hpp>

namespace asv_planning {

/**
 * @brief Configuration parameters for the RRT-based local planner
 */
struct RRTLocalPlannerConfig {
    // RRT parameters
    double max_extension_distance = 2.0;    // Maximum distance to extend the tree (m)
    double goal_bias = 0.2;                 // Probability of sampling the goal (0-1)
    double obstacle_bias = 0.1;             // Probability of sampling near obstacles (0-1)
    double waypoint_bias = 0.1;             // Probability of sampling near global path waypoints (0-1)
    int max_iterations = 1000;              // Maximum number of iterations
    double goal_tolerance = 1.0;            // Distance tolerance to consider goal reached (m)
    double heading_tolerance = 0.2;         // Heading tolerance to consider goal reached (rad)
    
    // Sampling parameters
    double x_min = -100.0;                  // Minimum x-coordinate for sampling (m)
    double x_max = 100.0;                   // Maximum x-coordinate for sampling (m)
    double y_min = -100.0;                  // Minimum y-coordinate for sampling (m)
    double y_max = 100.0;                   // Maximum y-coordinate for sampling (m)
    
    // Kinematics and dynamics
    double dt = 0.1;                        // Time step for forward simulation (s)
    int num_control_samples = 10;           // Number of control samples
    double min_thrust = 0.0;                // Minimum thrust (N)
    double max_thrust = 500.0;              // Maximum thrust (N)
    double min_rudder = -0.5;               // Minimum rudder angle (rad, ~-30 degrees)
    double max_rudder = 0.5;                // Maximum rudder angle (rad, ~30 degrees)
    int forward_sim_steps = 5;              // Number of steps for forward simulation
    bool use_dynamics = true;               // Whether to use full dynamics or simplified kinematics
    
    // Energy-aware parameters
    bool energy_aware = true;               // Whether to consider energy in the cost function
    double energy_weight = 0.5;             // Weight for energy component in cost function (0-1)
    double distance_weight = 0.3;           // Weight for distance component in cost function (0-1)
    double time_weight = 0.2;               // Weight for time component in cost function (0-1)
    
    // Planning horizons
    double planning_horizon = 20.0;         // Planning horizon distance (m)
    double replanning_trigger = 5.0;        // Distance to trigger replanning (m)
    
    // Collision avoidance
    double safety_distance = 2.0;           // Safety distance from obstacles (m)
    double collision_check_resolution = 0.5; // Resolution for collision checking (m)
    
    // Path smoothing
    bool smooth_path = true;                // Whether to smooth the path
    int smoothing_iterations = 5;           // Number of smoothing iterations
    double smoothing_alpha = 0.1;           // Smoothing parameter (0-1)
};

/**
 * @brief RRT Node structure for tree building
 */
struct RRTNode {
    Eigen::VectorXd state;      // State vector (x, y, psi, u, v, r)
    RRTNode* parent;            // Parent node
    ASVControl control;         // Control to reach this node from parent
    double cost_to_come;        // Cost to reach this node from start
    double est_cost_to_go;      // Estimated cost to reach goal from this node
    double total_cost;          // Total cost (cost_to_come + est_cost_to_go)
    double energy;              // Energy consumption to reach this node
    double time;                // Time to reach this node
    
    // Constructor
    RRTNode(const Eigen::VectorXd& _state, RRTNode* _parent = nullptr,
            const ASVControl& _control = {0.0, 0.0},
            double _cost_to_come = 0.0, double _energy = 0.0, double _time = 0.0)
        : state(_state), parent(_parent), control(_control),
          cost_to_come(_cost_to_come), est_cost_to_go(0.0),
          total_cost(_cost_to_come), energy(_energy), time(_time) {}
};

/**
 * @brief Path segment between two nodes
 */
struct PathSegment {
    Eigen::VectorXd start_state;   // Start state
    Eigen::VectorXd end_state;     // End state
    ASVControl control;            // Control input
    double duration;               // Duration of segment (s)
    double energy;                 // Energy consumption (J)
    double cost;                   // Cost of segment
};

/**
 * @brief Path representation with waypoints and control inputs
 */
struct LocalPath {
    std::vector<Eigen::VectorXd> states;    // Path waypoints (state vectors)
    std::vector<ASVControl> controls;       // Control inputs
    std::vector<double> durations;          // Duration of each segment (s)
    std::vector<double> energy;             // Energy consumption of each segment (J)
    double total_energy;                    // Total energy consumption (J)
    double total_time;                      // Total time (s)
    double total_distance;                  // Total distance (m)
    bool valid;                             // Whether the path is valid
};

/**
 * @brief RRT-based local planner for ASV navigation
 * 
 * This class implements an energy-aware RRT-based local planner for ASVs.
 * It generates feasible and energy-efficient trajectories that respect
 * the ASV dynamics and avoid obstacles in the environment.
 */
class RRTLocalPlanner {
public:
    /**
     * @brief Constructor
     * 
     * @param config Configuration parameters
     * @param dynamics ASV dynamics model
     * @param map_ptr Pointer to the environment map representation
     */
    RRTLocalPlanner(const RRTLocalPlannerConfig& config,
                   std::shared_ptr<ASVDynamics> dynamics,
                   std::shared_ptr<MapRepresentation> map_ptr);
    
    /**
     * @brief Destructor
     */
    ~RRTLocalPlanner();
    
    /**
     * @brief Update the configuration parameters
     * 
     * @param config New configuration parameters
     */
    void updateConfig(const RRTLocalPlannerConfig& config);
    
    /**
     * @brief Get the current configuration parameters
     * 
     * @return Current configuration parameters
     */
    const RRTLocalPlannerConfig& getConfig() const;
    
    /**
     * @brief Set the global path for the planner to follow
     * 
     * @param global_path Vector of waypoints (x, y) from the global planner
     */
    void setGlobalPath(const std::vector<Eigen::Vector2d>& global_path);
    
    /**
     * @brief Set the current state of the ASV
     * 
     * @param current_state Current ASV state
     */
    void setCurrentState(const Eigen::VectorXd& current_state);
    
    /**
     * @brief Plan a local trajectory from the current state towards the global path
     * 
     * @param start_state Start state of the ASV
     * @param goal_state Goal state to reach
     * @param time_budget Maximum computation time budget (s)
     * @return Planned local path
     */
    LocalPath planTrajectory(const Eigen::VectorXd& start_state, 
                            const Eigen::VectorXd& goal_state,
                            double time_budget = 0.1);
    
    /**
     * @brief Check if replanning is needed
     * 
     * @param current_state Current ASV state
     * @param current_path Current local path
     * @return True if replanning is needed, false otherwise
     */
    bool needReplanning(const Eigen::VectorXd& current_state, const LocalPath& current_path);
    
    /**
     * @brief Get the nearest waypoint on the global path
     * 
     * @param current_state Current ASV state
     * @param look_ahead_distance Look-ahead distance (m)
     * @return Nearest waypoint state (position, heading)
     */
    Eigen::VectorXd getNearestGlobalWaypoint(const Eigen::VectorXd& current_state, 
                                           double look_ahead_distance);
    
    /**
     * @brief Get the tree nodes for visualization
     * 
     * @return Vector of node states
     */
    std::vector<Eigen::VectorXd> getTreeNodes() const;
    
    /**
     * @brief Get the explored edges for visualization
     * 
     * @return Vector of edges (pairs of states)
     */
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> getTreeEdges() const;
    
private:
    /**
     * @brief Initialize the RRT tree with the start node
     * 
     * @param start_state Start state
     */
    void initTree(const Eigen::VectorXd& start_state);
    
    /**
     * @brief Sample a random state in the configuration space
     * 
     * @param goal_state Goal state (for biasing)
     * @return Sampled state
     */
    Eigen::VectorXd sampleState(const Eigen::VectorXd& goal_state);
    
    /**
     * @brief Find the nearest node in the tree
     * 
     * @param state State to find the nearest node to
     * @return Nearest node
     */
    RRTNode* findNearestNode(const Eigen::VectorXd& state);
    
    /**
     * @brief Extend the tree towards the sampled state
     * 
     * @param nearest_node Nearest node in the tree
     * @param sampled_state Sampled state to extend towards
     * @return New node if extension was successful, nullptr otherwise
     */
    RRTNode* extendTree(RRTNode* nearest_node, const Eigen::VectorXd& sampled_state);
    
    /**
     * @brief Check if the path between two states is collision-free
     * 
     * @param from_state Start state
     * @param to_state End state
     * @param control Control input
     * @return True if collision-free, false otherwise
     */
    bool isCollisionFree(const Eigen::VectorXd& from_state, 
                         const Eigen::VectorXd& to_state,
                         const ASVControl& control);
    
    /**
     * @brief Sample control inputs to extend the tree
     * 
     * @param from_node Node to extend from
     * @param target_state Target state to extend towards
     * @return Vector of sampled control inputs
     */
    std::vector<ASVControl> sampleControls(RRTNode* from_node, 
                                          const Eigen::VectorXd& target_state);
    
    /**
     * @brief Compute the cost of a path segment
     * 
     * @param from_state Start state
     * @param to_state End state
     * @param control Control input
     * @param energy Energy consumption
     * @param duration Duration of segment
     * @return Cost of the segment
     */
    double computeSegmentCost(const Eigen::VectorXd& from_state,
                             const Eigen::VectorXd& to_state,
                             const ASVControl& control,
                             double energy,
                             double duration);
    
    /**
     * @brief Estimate the cost-to-go from a state to the goal
     * 
     * @param state Current state
     * @param goal_state Goal state
     * @return Estimated cost to reach the goal
     */
    double estimateCostToGo(const Eigen::VectorXd& state, const Eigen::VectorXd& goal_state);
    
    /**
     * @brief Extract the path from the tree
     * 
     * @param goal_node Goal node
     * @return Extracted path
     */
    LocalPath extractPath(RRTNode* goal_node);
    
    /**
     * @brief Smooth the path using iterative gradient descent
     * 
     * @param path Path to smooth
     * @return Smoothed path
     */
    LocalPath smoothPath(const LocalPath& path);
    
    /**
     * @brief Clean up the tree nodes
     */
    void clearTree();
    
    // Configuration parameters
    RRTLocalPlannerConfig config_;
    
    // ASV dynamics model
    std::shared_ptr<ASVDynamics> dynamics_;
    
    // Map representation
    std::shared_ptr<MapRepresentation> map_;
    
    // Tree nodes
    std::vector<RRTNode*> tree_nodes_;
    
    // Global path
    std::vector<Eigen::Vector2d> global_path_;
    
    // Random number generator
    std::mt19937 rng_;
    
    // Current best path
    LocalPath current_path_;
    
    // Goal region reached flag
    bool goal_reached_;
    
    // Current ASV state
    Eigen::VectorXd current_state_;
};

} // namespace asv_planning 