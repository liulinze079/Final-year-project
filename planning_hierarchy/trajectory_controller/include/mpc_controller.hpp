#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "asv_dynamics.hpp"
#include "cost_functions.hpp"
#include "trajectory.hpp"

namespace asv_planning {

/**
 * @brief Configuration parameters for the MPC controller
 */
struct MPCConfig {
    int prediction_horizon = 10;       // Prediction horizon
    double dt = 0.1;                   // Time step
    
    // Weight matrices for different cost components
    double pos_weight = 1.0;           // Position tracking weight
    double heading_weight = 1.0;       // Heading tracking weight
    double vel_weight = 0.5;           // Velocity tracking weight
    double control_weight = 0.1;       // Control effort weight
    double control_rate_weight = 0.05; // Control rate weight
    double energy_weight = 0.2;        // Energy consumption weight
    
    // State and control limits
    double max_surge = 5.0;            // Maximum surge velocity (m/s)
    double max_sway = 2.0;             // Maximum sway velocity (m/s)
    double max_yaw_rate = 0.5;         // Maximum yaw rate (rad/s)
    double max_thrust = 500.0;         // Maximum thrust force (N)
    double max_rudder = 0.7;           // Maximum rudder angle (rad)
    
    // Constraints
    double max_pos_error = 5.0;        // Maximum position error (m)
    double max_heading_error = 0.5;    // Maximum heading error (rad)
    
    // Optimization parameters
    int max_iterations = 100;          // Maximum solver iterations
    double convergence_tolerance = 1e-6; // Solver convergence tolerance
};

/**
 * @brief Model Predictive Control (MPC) controller for ASV trajectory tracking
 * 
 * This class implements an MPC controller that generates optimal control inputs
 * to track a reference trajectory while considering energy efficiency.
 */
class MPCController {
public:
    /**
     * @brief Constructor
     * 
     * @param dynamics ASV dynamics model
     * @param config MPC configuration parameters
     */
    MPCController(std::shared_ptr<ASVDynamics> dynamics, const MPCConfig& config = MPCConfig());
    
    /**
     * @brief Destructor
     */
    ~MPCController();
    
    /**
     * @brief Set the reference trajectory
     * 
     * @param trajectory Reference trajectory
     */
    void setReferenceTrajectory(const Trajectory& trajectory);
    
    /**
     * @brief Set the cost functions
     * 
     * @param cost_functions Cost functions for optimization
     */
    void setCostFunctions(std::shared_ptr<CostFunctions> cost_functions);
    
    /**
     * @brief Update the MPC configuration
     * 
     * @param config MPC configuration parameters
     */
    void updateConfig(const MPCConfig& config);
    
    /**
     * @brief Get the current MPC configuration
     * 
     * @return MPC configuration parameters
     */
    const MPCConfig& getConfig() const;
    
    /**
     * @brief Compute the optimal control input
     * 
     * @param current_state Current ASV state (x, y, heading, surge, sway, yaw_rate)
     * @param current_time Current time
     * @param control_input Output optimal control input (thrust, rudder)
     * @return true if optimization successful, false otherwise
     */
    bool computeControlInput(const Eigen::VectorXd& current_state, double current_time, Eigen::VectorXd& control_input);
    
    /**
     * @brief Get the predicted trajectory based on the last optimization
     * 
     * @return Predicted trajectory (sequence of states)
     */
    std::vector<Eigen::VectorXd> getPredictedTrajectory() const;
    
    /**
     * @brief Get the predicted control inputs based on the last optimization
     * 
     * @return Predicted control inputs
     */
    std::vector<Eigen::VectorXd> getPredictedControls() const;
    
    /**
     * @brief Get the MPC cost value from the last optimization
     * 
     * @return Cost value
     */
    double getLastCostValue() const;
    
    /**
     * @brief Get the ASV dynamics model
     * 
     * @return ASV dynamics model
     */
    std::shared_ptr<ASVDynamics> getDynamicsModel() const;
    
private:
    /**
     * @brief Set up the optimization problem
     * 
     * @param current_state Current ASV state
     * @param current_time Current time
     */
    void setupOptimizationProblem(const Eigen::VectorXd& current_state, double current_time);
    
    /**
     * @brief Solve the optimization problem
     * 
     * @return true if optimization successful, false otherwise
     */
    bool solveOptimizationProblem();
    
    /**
     * @brief Extract solution from the optimization result
     * 
     * @param control_input Output optimal control input
     */
    void extractSolution(Eigen::VectorXd& control_input);
    
    /**
     * @brief Get the reference states for the prediction horizon
     * 
     * @param current_time Current time
     * @return Reference states
     */
    std::vector<Eigen::VectorXd> getReferenceStates(double current_time) const;
    
    /**
     * @brief Apply constraints to the optimization problem
     * 
     * @param current_state Current ASV state
     */
    void applyConstraints(const Eigen::VectorXd& current_state);
    
    std::shared_ptr<ASVDynamics> dynamics_;
    std::shared_ptr<CostFunctions> cost_functions_;
    Trajectory reference_trajectory_;
    
    MPCConfig config_;
    
    // Optimization variables
    Eigen::VectorXd initial_state_;
    std::vector<Eigen::VectorXd> predicted_states_;
    std::vector<Eigen::VectorXd> predicted_controls_;
    std::vector<Eigen::VectorXd> reference_states_;
    double last_cost_value_;
    
    // Previous control input for control rate calculation
    Eigen::VectorXd previous_control_;
    
    // Flag to indicate if the controller is initialized
    bool initialized_;
};

} // namespace asv_planning 