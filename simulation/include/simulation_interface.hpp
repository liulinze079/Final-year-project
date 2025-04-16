#pragma once

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <mutex>
#include <Eigen/Dense>

#include "asv_dynamics.hpp"
#include "global_planner.hpp"
#include "rrt_local_planner.hpp"
#include "mpc_controller.hpp"
#include "maritime_environment.hpp"

namespace asv_planning {

/**
 * @brief Configuration parameters for the simulation interface
 */
struct SimulationConfig {
    // Timing parameters
    double sim_time_step = 0.01;        // Simulation time step (s)
    double planning_time_step = 0.5;    // Planning time step (s)
    double control_time_step = 0.1;     // Control time step (s)
    
    // Logging parameters
    bool enable_logging = true;         // Enable logging
    std::string log_directory = "logs"; // Log directory
    double log_frequency = 10.0;        // Logging frequency (Hz)
    
    // Visualization parameters
    bool enable_visualization = true;   // Enable visualization
    double viz_frequency = 10.0;        // Visualization update frequency (Hz)
    
    // Environment parameters
    std::string environment_file = "";  // Path to environment file
    
    // Mission parameters
    double mission_timeout = 3600.0;    // Mission timeout (s)
    double goal_tolerance = 2.0;        // Goal tolerance (m)
    
    // Energy parameters
    double initial_energy = 1000000.0;  // Initial energy (J)
    double critical_energy = 100000.0;  // Critical energy level (J)
    
    // Planning hierarchy parameters
    bool enable_replanning = true;      // Enable replanning when needed
    double replan_distance = 10.0;      // Distance to trigger replanning (m)
    double replan_time = 30.0;          // Time to trigger replanning (s)
};

/**
 * @brief Simulation metrics
 */
struct SimulationMetrics {
    // Path metrics
    double path_length = 0.0;           // Total path length (m)
    double path_time = 0.0;             // Total path time (s)
    double energy_consumed = 0.0;       // Total energy consumed (J)
    
    // Planning metrics
    int global_replans = 0;             // Number of global replans
    int local_replans = 0;              // Number of local replans
    double global_planning_time = 0.0;  // Total global planning time (s)
    double local_planning_time = 0.0;   // Total local planning time (s)
    
    // Control metrics
    double avg_cross_track_error = 0.0; // Average cross-track error (m)
    double max_cross_track_error = 0.0; // Maximum cross-track error (m)
    double avg_control_effort = 0.0;    // Average control effort
    
    // Safety metrics
    double min_obstacle_distance = 0.0; // Minimum distance to obstacles (m)
    int collision_count = 0;            // Number of collisions
    
    // Mission metrics
    bool mission_completed = false;     // Whether the mission was completed
    bool energy_depleted = false;       // Whether energy was depleted
    bool timeout = false;               // Whether the mission timed out
};

/**
 * @brief Callback function type for state updates
 */
using StateUpdateCallback = std::function<void(const Eigen::VectorXd&, double)>;

/**
 * @brief Callback function type for path updates
 */
using PathUpdateCallback = std::function<void(const std::vector<Eigen::Vector2d>&)>;

/**
 * @brief Callback function type for metrics updates
 */
using MetricsUpdateCallback = std::function<void(const SimulationMetrics&)>;

/**
 * @brief Simulation interface for autonomous surface vehicle planning and control
 * 
 * This class provides an interface between the planning and control modules
 * and the simulation environment. It manages the simulation loop, updates
 * the vehicle state, and coordinates the planning and control hierarchy.
 */
class SimulationInterface {
public:
    /**
     * @brief Constructor
     * 
     * @param config Simulation configuration
     * @param dynamics ASV dynamics model
     * @param global_planner Global planner
     * @param local_planner Local planner
     * @param controller Controller
     * @param environment Maritime environment
     */
    SimulationInterface(
        const SimulationConfig& config,
        std::shared_ptr<ASVDynamics> dynamics,
        std::shared_ptr<GlobalPlanner> global_planner,
        std::shared_ptr<RRTLocalPlanner> local_planner,
        std::shared_ptr<MPCController> controller,
        std::shared_ptr<MaritimeEnvironment> environment);
    
    /**
     * @brief Destructor
     */
    ~SimulationInterface();
    
    /**
     * @brief Initialize the simulation
     * 
     * @param initial_state Initial ASV state
     * @param start_position Start position
     * @param goal_position Goal position
     * @return True if initialization was successful
     */
    bool initialize(
        const Eigen::VectorXd& initial_state,
        const Eigen::Vector2d& start_position,
        const Eigen::Vector2d& goal_position);
    
    /**
     * @brief Run the simulation for a specified duration
     * 
     * @param duration Simulation duration (s), 0 means run until completion
     * @return True if simulation completed successfully
     */
    bool run(double duration = 0.0);
    
    /**
     * @brief Pause the simulation
     */
    void pause();
    
    /**
     * @brief Resume the simulation
     */
    void resume();
    
    /**
     * @brief Stop the simulation
     */
    void stop();
    
    /**
     * @brief Step the simulation forward by one time step
     * 
     * @return True if step was successful
     */
    bool step();
    
    /**
     * @brief Get the current simulation time
     * 
     * @return Current simulation time (s)
     */
    double getCurrentTime() const;
    
    /**
     * @brief Get the current ASV state
     * 
     * @return Current ASV state
     */
    Eigen::VectorXd getCurrentState() const;
    
    /**
     * @brief Get the current simulation metrics
     * 
     * @return Current simulation metrics
     */
    SimulationMetrics getMetrics() const;
    
    /**
     * @brief Get the global path
     * 
     * @return Global path waypoints
     */
    std::vector<Eigen::Vector2d> getGlobalPath() const;
    
    /**
     * @brief Get the local path
     * 
     * @return Local path waypoints
     */
    std::vector<Eigen::Vector2d> getLocalPath() const;
    
    /**
     * @brief Get the current energy level
     * 
     * @return Current energy level (J)
     */
    double getCurrentEnergy() const;
    
    /**
     * @brief Register callback for state updates
     * 
     * @param callback State update callback function
     */
    void registerStateUpdateCallback(StateUpdateCallback callback);
    
    /**
     * @brief Register callback for path updates
     * 
     * @param callback Path update callback function
     */
    void registerPathUpdateCallback(PathUpdateCallback callback);
    
    /**
     * @brief Register callback for metrics updates
     * 
     * @param callback Metrics update callback function
     */
    void registerMetricsUpdateCallback(MetricsUpdateCallback callback);
    
    /**
     * @brief Update the simulation configuration
     * 
     * @param config New configuration parameters
     */
    void updateConfig(const SimulationConfig& config);
    
    /**
     * @brief Get the current simulation configuration
     * 
     * @return Current configuration parameters
     */
    const SimulationConfig& getConfig() const;
    
private:
    /**
     * @brief Update the ASV state based on the current control input
     * 
     * @param control Control input
     * @param dt Time step (s)
     */
    void updateState(const ASVControl& control, double dt);
    
    /**
     * @brief Update the planning hierarchy if needed
     * 
     * @return True if planning was updated
     */
    bool updatePlanning();
    
    /**
     * @brief Update the control input based on the current state and path
     * 
     * @return Control input
     */
    ASVControl updateControl();
    
    /**
     * @brief Update the simulation metrics
     */
    void updateMetrics();
    
    /**
     * @brief Check if the mission is complete
     * 
     * @return True if the mission is complete
     */
    bool checkMissionComplete();
    
    /**
     * @brief Check if replanning is needed
     * 
     * @return True if replanning is needed
     */
    bool needsReplanning() const;
    
    /**
     * @brief Log the current simulation state
     */
    void logState();
    
    /**
     * @brief Update visualization if enabled
     */
    void updateVisualization();
    
    /**
     * @brief Compute the cross-track error from the path
     * 
     * @return Cross-track error (m)
     */
    double computeCrossTrackError() const;
    
    /**
     * @brief Compute the distance to the nearest obstacle
     * 
     * @return Distance to the nearest obstacle (m)
     */
    double computeMinObstacleDistance() const;
    
    // Simulation configuration
    SimulationConfig config_;
    
    // Component instances
    std::shared_ptr<ASVDynamics> dynamics_;
    std::shared_ptr<GlobalPlanner> global_planner_;
    std::shared_ptr<RRTLocalPlanner> local_planner_;
    std::shared_ptr<MPCController> controller_;
    std::shared_ptr<MaritimeEnvironment> environment_;
    
    // Simulation state
    Eigen::VectorXd current_state_;
    double current_time_;
    double current_energy_;
    bool is_running_;
    bool is_paused_;
    
    // Mission state
    Eigen::Vector2d start_position_;
    Eigen::Vector2d goal_position_;
    std::vector<Eigen::Vector2d> global_path_;
    std::vector<Eigen::Vector2d> local_path_;
    ASVControl current_control_;
    
    // Timing variables
    double last_planning_time_;
    double last_control_time_;
    double last_log_time_;
    double last_viz_time_;
    
    // Metrics
    SimulationMetrics metrics_;
    
    // Callbacks
    std::vector<StateUpdateCallback> state_callbacks_;
    std::vector<PathUpdateCallback> path_callbacks_;
    std::vector<MetricsUpdateCallback> metrics_callbacks_;
    
    // Thread safety
    mutable std::mutex state_mutex_;
    mutable std::mutex path_mutex_;
    mutable std::mutex metrics_mutex_;
};

} // namespace asv_planning 