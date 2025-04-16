#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <memory>

namespace OsqpEigen {
    class Solver;
}

namespace asv_planning {

class ASVDynamics;
class CostFunctions;

/**
 * @brief Model Predictive Control class for trajectory following
 * 
 * This class implements a linear MPC controller that uses linearized ASV dynamics
 * to predict and optimize control inputs over a prediction horizon. It minimizes
 * a cost function that includes tracking error, control effort, control rate, and
 * energy consumption costs.
 */
class ModelPredictiveControl {
public:
    /**
     * @brief Constructor
     */
    ModelPredictiveControl();
    
    /**
     * @brief Destructor
     */
    ~ModelPredictiveControl();
    
    /**
     * @brief Initialize the MPC controller
     * 
     * @param dynamics ASV dynamics model
     * @param cost_functions Cost functions for optimization
     * @param prediction_horizon Number of steps in prediction horizon
     * @param control_interval Time interval between control steps (seconds)
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize(std::shared_ptr<ASVDynamics> dynamics,
                   std::shared_ptr<CostFunctions> cost_functions,
                   int prediction_horizon = 10,
                   double control_interval = 0.1);
    
    /**
     * @brief Set reference trajectory for the MPC to follow
     * 
     * @param reference_states Vector of reference states along the trajectory
     * @param time_points Vector of time points corresponding to reference states
     */
    void setReferenceTrajectory(const std::vector<Eigen::VectorXd>& reference_states,
                              const std::vector<double>& time_points);
    
    /**
     * @brief Set current state of the vehicle
     * 
     * @param state Current state vector
     */
    void setState(const Eigen::VectorXd& state);
    
    /**
     * @brief Set current control inputs
     * 
     * @param control Current control vector
     */
    void setControl(const Eigen::VectorXd& control);
    
    /**
     * @brief Set state constraints (position limits)
     * 
     * @param x_min Minimum x position
     * @param x_max Maximum x position
     * @param y_min Minimum y position
     * @param y_max Maximum y position
     */
    void setStateConstraints(double x_min, double x_max,
                           double y_min, double y_max);
    
    /**
     * @brief Set control constraints
     * 
     * @param u_min Minimum control input vector
     * @param u_max Maximum control input vector
     */
    void setControlConstraints(const Eigen::VectorXd& u_min,
                             const Eigen::VectorXd& u_max);
    
    /**
     * @brief Set weights for the cost function
     * 
     * @param position_weight Weight for position tracking error
     * @param heading_weight Weight for heading tracking error
     * @param velocity_weight Weight for velocity tracking error
     * @param control_weight Weight for control effort
     * @param control_rate_weight Weight for control rate
     * @param energy_weight Weight for energy consumption
     */
    void setWeights(double position_weight, double heading_weight,
                  double velocity_weight, double control_weight,
                  double control_rate_weight, double energy_weight);
    
    /**
     * @brief Solve the MPC optimization problem
     * 
     * @param current_time Current time (seconds) for reference trajectory lookup
     * @return true if optimization succeeded, false otherwise
     */
    bool solve(double current_time);
    
    /**
     * @brief Get the optimal control input for the current step
     * 
     * @return Optimal control input vector
     */
    Eigen::VectorXd getOptimalControl() const;
    
    /**
     * @brief Get the predicted states over the prediction horizon
     * 
     * @return Vector of predicted state vectors
     */
    const std::vector<Eigen::VectorXd>& getPredictedStates() const;
    
    /**
     * @brief Get the optimal control sequence over the prediction horizon
     * 
     * @return Vector of optimal control vectors
     */
    const std::vector<Eigen::VectorXd>& getOptimalControls() const;
    
private:
    /**
     * @brief Generate linearized dynamics matrices over the prediction horizon
     * 
     * @param state Current state for linearization
     * @param control Current control for linearization
     * @param A_matrices Output vector of state transition matrices
     * @param B_matrices Output vector of control matrices
     * @param c_vectors Output vector of constant term vectors
     */
    void linearizeDynamics(const Eigen::VectorXd& state,
                         const Eigen::VectorXd& control,
                         std::vector<Eigen::MatrixXd>& A_matrices,
                         std::vector<Eigen::MatrixXd>& B_matrices,
                         std::vector<Eigen::VectorXd>& c_vectors);
    
    /**
     * @brief Generate reference states for each step in the prediction horizon
     * 
     * @param current_time Current time for reference trajectory lookup
     * @param reference_trajectory Output vector of reference states
     */
    void generateReferenceStates(double current_time,
                               std::vector<Eigen::VectorXd>& reference_trajectory);
    
    /**
     * @brief Interpolate reference state for a specific time
     * 
     * @param target_time Target time for interpolation
     * @return Interpolated reference state
     */
    Eigen::VectorXd interpolateReferenceState(double target_time);
    
    /**
     * @brief Set up the quadratic programming problem for the MPC
     * 
     * @param reference_states Reference states for the prediction horizon
     * @param A_matrices State transition matrices
     * @param B_matrices Control matrices
     * @param c_vectors Constant term vectors
     * @return true if setup succeeded, false otherwise
     */
    bool setupQPProblem(const std::vector<Eigen::VectorXd>& reference_states,
                      const std::vector<Eigen::MatrixXd>& A_matrices,
                      const std::vector<Eigen::MatrixXd>& B_matrices,
                      const std::vector<Eigen::VectorXd>& c_vectors);
    
    /**
     * @brief Extract optimal states and controls from solution vector
     * 
     * @param solution Solution vector from QP solver
     */
    void extractSolution(const Eigen::VectorXd& solution);
    
    // MPC parameters
    int prediction_horizon_;
    double control_interval_;
    int state_dim_;
    int control_dim_;
    
    // State constraints
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
    
    // Control constraints
    Eigen::VectorXd u_min_;
    Eigen::VectorXd u_max_;
    
    // Cost weights
    double position_weight_;
    double heading_weight_;
    double velocity_weight_;
    double control_weight_;
    double control_rate_weight_;
    double energy_weight_;
    
    // Current state and control
    Eigen::VectorXd current_state_;
    Eigen::VectorXd current_control_;
    
    // Reference trajectory
    std::vector<Eigen::VectorXd> reference_states_;
    std::vector<double> reference_times_;
    
    // MPC solution
    std::vector<Eigen::VectorXd> predicted_states_;
    std::vector<Eigen::VectorXd> optimal_controls_;
    
    // Components
    std::unique_ptr<OsqpEigen::Solver> solver_;
    std::shared_ptr<ASVDynamics> dynamics_;
    std::shared_ptr<CostFunctions> cost_functions_;
};

} // namespace asv_planning 