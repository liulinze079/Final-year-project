#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <functional>
#include <asv_dynamics.hpp>
#include <OsqpEigen/OsqpEigen.h>

namespace asv_planning {

/**
 * @brief Configuration parameters for the MPC controller
 */
struct MPCConfig {
    // MPC parameters
    int prediction_horizon = 10;           // Prediction horizon steps
    double dt = 0.1;                       // Sampling time (s)
    
    // Weights for the cost function
    double position_weight = 1.0;          // Weight for position error
    double heading_weight = 0.5;           // Weight for heading error
    double velocity_weight = 0.2;          // Weight for velocity error
    double energy_weight = 0.5;            // Weight for energy consumption
    double control_weight = 0.1;           // Weight for control effort
    double control_rate_weight = 0.2;      // Weight for control rate
    
    // Constraints
    double max_thrust = 500.0;             // Maximum thrust force (N)
    double min_thrust = 0.0;               // Minimum thrust force (N)
    double max_rudder = 0.5;               // Maximum rudder angle (rad, ~30 degrees)
    double min_rudder = -0.5;              // Minimum rudder angle (rad, ~-30 degrees)
    double max_thrust_rate = 100.0;        // Maximum thrust rate (N/s)
    double max_rudder_rate = 0.2;          // Maximum rudder rate (rad/s)
    
    // Solver parameters
    int max_iterations = 100;              // Maximum iterations for QP solver
    double solver_tolerance = 1e-6;        // Solver tolerance
    
    // Energy optimization
    bool energy_aware = true;              // Whether to consider energy in the cost function
    
    // Safety
    double safety_margin = 1.0;            // Safety margin for obstacle avoidance (m)
    double collision_avoidance_weight = 10.0; // Weight for collision avoidance
    bool use_collision_avoidance = true;   // Whether to use collision avoidance
    
    // Reference tracking
    double look_ahead_distance = 2.0;      // Look-ahead distance for path tracking (m)
    double cross_track_error_weight = 1.0; // Weight for cross-track error
    double along_track_weight = 0.5;       // Weight for along-track progress
};

/**
 * @brief Reference trajectory point for MPC
 */
struct ReferencePoint {
    double x;         // X position (m)
    double y;         // Y position (m)
    double psi;       // Heading angle (rad)
    double u;         // Surge velocity (m/s)
    double v;         // Sway velocity (m/s)
    double r;         // Yaw rate (rad/s)
    
    // Constructor
    ReferencePoint(double _x = 0.0, double _y = 0.0, double _psi = 0.0,
                  double _u = 0.0, double _v = 0.0, double _r = 0.0)
        : x(_x), y(_y), psi(_psi), u(_u), v(_v), r(_r) {}
    
    // Convert to state vector
    Eigen::VectorXd toStateVector() const {
        Eigen::VectorXd state(6);
        state << x, y, psi, u, v, r;
        return state;
    }
};

/**
 * @brief Reference trajectory for MPC
 */
struct ReferenceTrajectory {
    std::vector<ReferencePoint> points;
    std::vector<double> timestamps;
    bool valid;
};

/**
 * @brief MPC solution data
 */
struct MPCSolution {
    std::vector<ASVControl> control_sequence;  // Sequence of control inputs
    ASVControl first_control;                 // First control input to apply
    bool solved;                              // Whether the solution was found
    double solve_time;                        // Time taken to solve (s)
    double predicted_energy;                  // Predicted energy consumption (J)
    std::vector<Eigen::VectorXd> predicted_states;  // Predicted state trajectory
};

/**
 * @brief Model Predictive Controller for ASV control
 * 
 * This class implements an energy-aware Model Predictive Controller (MPC)
 * for ASV control. It generates optimal control inputs that minimize a
 * cost function considering tracking error, energy consumption, and control effort,
 * while respecting the ASV dynamics and constraints.
 */
class MPCController {
public:
    /**
     * @brief Constructor
     * 
     * @param config MPC configuration parameters
     * @param dynamics ASV dynamics model
     */
    MPCController(const MPCConfig& config, std::shared_ptr<ASVDynamics> dynamics);
    
    /**
     * @brief Destructor
     */
    ~MPCController();
    
    /**
     * @brief Update the MPC configuration
     * 
     * @param config New configuration parameters
     */
    void updateConfig(const MPCConfig& config);
    
    /**
     * @brief Get the current MPC configuration
     * 
     * @return Current configuration parameters
     */
    const MPCConfig& getConfig() const;
    
    /**
     * @brief Set the reference trajectory for tracking
     * 
     * @param trajectory Reference trajectory
     */
    void setReferenceTrajectory(const ReferenceTrajectory& trajectory);
    
    /**
     * @brief Add obstacles to the MPC problem
     * 
     * @param obstacles Vector of obstacle positions and radii (x, y, radius)
     */
    void setObstacles(const std::vector<Eigen::Vector3d>& obstacles);
    
    /**
     * @brief Solve the MPC problem and get control inputs
     * 
     * @param current_state Current ASV state
     * @param current_control Current control input
     * @return MPC solution
     */
    MPCSolution solve(const Eigen::VectorXd& current_state, const ASVControl& current_control);
    
    /**
     * @brief Get the predicted state trajectory from the last MPC solution
     * 
     * @return Vector of predicted states
     */
    const std::vector<Eigen::VectorXd>& getPredictedTrajectory() const;
    
    /**
     * @brief Get the reference trajectory used in the last MPC solution
     * 
     * @return Reference trajectory
     */
    const ReferenceTrajectory& getReferenceTrajectory() const;
    
    /**
     * @brief Get the linearized dynamics matrices from the last linearization
     * 
     * @return Pair of A and B matrices
     */
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> getLinearizedDynamics() const;
    
private:
    /**
     * @brief Setup the QP problem for OSQP
     */
    void setupQP();
    
    /**
     * @brief Update the QP problem for the current state and reference
     * 
     * @param current_state Current ASV state
     * @param current_control Current control input
     */
    void updateQP(const Eigen::VectorXd& current_state, const ASVControl& current_control);
    
    /**
     * @brief Linearize the ASV dynamics around the current operating point
     * 
     * @param state Current state
     * @param control Current control
     * @return Pair of A and B matrices for the linearized system
     */
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> linearizeDynamics(
        const Eigen::VectorXd& state, const ASVControl& control);
    
    /**
     * @brief Form the quadratic cost matrices for the MPC problem
     * 
     * @param reference_traj Reference trajectory
     * @param Q State weight matrix
     * @param R Control weight matrix
     * @return Pair of H (hessian) and g (gradient) matrices
     */
    std::pair<Eigen::SparseMatrix<double>, Eigen::VectorXd> formCostMatrices(
        const ReferenceTrajectory& reference_traj,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R);
    
    /**
     * @brief Form the constraint matrices for the MPC problem
     * 
     * @param current_state Current ASV state
     * @param current_control Current control input
     * @param A Dynamics A matrix
     * @param B Dynamics B matrix
     * @return Tuple of A_constraint, lower_bound, upper_bound
     */
    std::tuple<Eigen::SparseMatrix<double>, Eigen::VectorXd, Eigen::VectorXd> formConstraintMatrices(
        const Eigen::VectorXd& current_state,
        const ASVControl& current_control,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& B);
    
    /**
     * @brief Extract the control sequence from the QP solution
     * 
     * @param qp_solution QP solution vector
     * @return Control sequence
     */
    std::vector<ASVControl> extractControlSequence(const Eigen::VectorXd& qp_solution);
    
    /**
     * @brief Extract the predicted state trajectory from the QP solution
     * 
     * @param qp_solution QP solution vector
     * @param current_state Current state
     * @return Predicted state trajectory
     */
    std::vector<Eigen::VectorXd> extractPredictedTrajectory(
        const Eigen::VectorXd& qp_solution, const Eigen::VectorXd& current_state);
    
    /**
     * @brief Compute the predicted energy consumption from the control sequence
     * 
     * @param control_sequence Control sequence
     * @param state_trajectory Predicted state trajectory
     * @return Predicted energy consumption (J)
     */
    double computePredictedEnergy(
        const std::vector<ASVControl>& control_sequence,
        const std::vector<Eigen::VectorXd>& state_trajectory);
    
    /**
     * @brief Generate reference trajectory from path waypoints
     * 
     * @param waypoints Path waypoints
     * @param current_state Current ASV state
     * @param prediction_horizon Prediction horizon steps
     * @return Reference trajectory
     */
    ReferenceTrajectory generateReferenceTrajectory(
        const std::vector<Eigen::Vector2d>& waypoints,
        const Eigen::VectorXd& current_state,
        int prediction_horizon);
    
    // MPC configuration
    MPCConfig config_;
    
    // ASV dynamics model
    std::shared_ptr<ASVDynamics> dynamics_;
    
    // OSQP solver
    OsqpEigen::Solver osqp_solver_;
    
    // Problem dimensions
    int n_states_;           // Number of states
    int n_controls_;         // Number of controls
    int n_horizon_;          // Prediction horizon steps
    
    // QP problem size
    int n_variables_;        // Number of decision variables
    int n_constraints_;      // Number of constraints
    
    // Reference trajectory
    ReferenceTrajectory reference_trajectory_;
    
    // Obstacle list
    std::vector<Eigen::Vector3d> obstacles_;
    
    // Last MPC solution
    MPCSolution last_solution_;
    
    // Cached matrices
    Eigen::MatrixXd Q_;      // State weight matrix
    Eigen::MatrixXd R_;      // Control weight matrix
    Eigen::MatrixXd A_lin_;  // Linearized A matrix
    Eigen::MatrixXd B_lin_;  // Linearized B matrix
    
    // Flag to indicate if the QP has been initialized
    bool is_initialized_;
};

} // namespace asv_planning 