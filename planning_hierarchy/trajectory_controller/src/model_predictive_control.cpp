#include "model_predictive_control.hpp"
#include "cost_functions.hpp"
#include "../../asv_core/include/asv_dynamics.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <iostream>
#include <stdexcept>
#include <cmath>

namespace asv_planning {

ModelPredictiveControl::ModelPredictiveControl()
    : prediction_horizon_(10),
      control_interval_(0.1),
      solver_(nullptr),
      cost_functions_(nullptr),
      dynamics_(nullptr) {
}

ModelPredictiveControl::~ModelPredictiveControl() = default;

bool ModelPredictiveControl::initialize(std::shared_ptr<ASVDynamics> dynamics,
                                    std::shared_ptr<CostFunctions> cost_functions,
                                    int prediction_horizon,
                                    double control_interval) {
    // Store parameters
    dynamics_ = dynamics;
    cost_functions_ = cost_functions;
    prediction_horizon_ = prediction_horizon;
    control_interval_ = control_interval;
    
    // Initialize OSQP solver
    solver_ = std::make_unique<OsqpEigen::Solver>();
    solver_->settings()->setWarmStart(true);
    solver_->settings()->setVerbosity(false);
    solver_->settings()->setAbsoluteTolerance(1e-4);
    solver_->settings()->setRelativeTolerance(1e-4);
    solver_->settings()->setMaxIteration(2000);
    
    // Get dimensions
    state_dim_ = cost_functions_->getStateDimension();
    control_dim_ = cost_functions_->getControlDimension();
    
    // Initialize default state constraints (large values, effectively no constraints)
    x_min_ = -1000.0;
    x_max_ = 1000.0;
    y_min_ = -1000.0;
    y_max_ = 1000.0;
    
    // Initialize default control constraints
    u_min_ = Eigen::VectorXd::Constant(control_dim_, -100.0);
    u_max_ = Eigen::VectorXd::Constant(control_dim_, 100.0);
    
    // Initialize default weights
    position_weight_ = 10.0;
    heading_weight_ = 5.0;
    velocity_weight_ = 1.0;
    control_weight_ = 0.1;
    control_rate_weight_ = 0.5;
    energy_weight_ = 1.0;
    
    // Apply weights to cost functions
    cost_functions_->setWeights(position_weight_, heading_weight_, velocity_weight_,
                             control_weight_, control_rate_weight_, energy_weight_);
    
    return true;
}

void ModelPredictiveControl::setReferenceTrajectory(const std::vector<Eigen::VectorXd>& reference_states,
                                               const std::vector<double>& time_points) {
    reference_states_ = reference_states;
    reference_times_ = time_points;
}

void ModelPredictiveControl::setState(const Eigen::VectorXd& state) {
    current_state_ = state;
}

void ModelPredictiveControl::setControl(const Eigen::VectorXd& control) {
    current_control_ = control;
}

void ModelPredictiveControl::setStateConstraints(double x_min, double x_max,
                                            double y_min, double y_max) {
    x_min_ = x_min;
    x_max_ = x_max;
    y_min_ = y_min;
    y_max_ = y_max;
}

void ModelPredictiveControl::setControlConstraints(const Eigen::VectorXd& u_min,
                                              const Eigen::VectorXd& u_max) {
    if (u_min.size() != control_dim_ || u_max.size() != control_dim_) {
        throw std::invalid_argument("Control constraints must match control dimension");
    }
    
    u_min_ = u_min;
    u_max_ = u_max;
}

void ModelPredictiveControl::setWeights(double position_weight, double heading_weight,
                                   double velocity_weight, double control_weight,
                                   double control_rate_weight, double energy_weight) {
    position_weight_ = position_weight;
    heading_weight_ = heading_weight;
    velocity_weight_ = velocity_weight;
    control_weight_ = control_weight;
    control_rate_weight_ = control_rate_weight;
    energy_weight_ = energy_weight;
    
    cost_functions_->setWeights(position_weight_, heading_weight_, velocity_weight_,
                             control_weight_, control_rate_weight_, energy_weight_);
}

bool ModelPredictiveControl::solve(double current_time) {
    if (!dynamics_ || !cost_functions_ || !solver_) {
        std::cerr << "MPC not properly initialized" << std::endl;
        return false;
    }
    
    // Generate reference states for prediction horizon
    std::vector<Eigen::VectorXd> reference_trajectory;
    generateReferenceStates(current_time, reference_trajectory);
    
    // Get linearized dynamics matrices over the prediction horizon
    std::vector<Eigen::MatrixXd> A_matrices;
    std::vector<Eigen::MatrixXd> B_matrices;
    std::vector<Eigen::VectorXd> c_vectors;
    
    linearizeDynamics(current_state_, current_control_, 
                     A_matrices, B_matrices, c_vectors);
    
    // Setup the QP problem
    bool success = setupQPProblem(reference_trajectory, A_matrices, B_matrices, c_vectors);
    if (!success) {
        std::cerr << "Failed to set up QP problem" << std::endl;
        return false;
    }
    
    // Solve the QP problem
    if (solver_->solve() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "QP solver failed" << std::endl;
        return false;
    }
    
    // Extract solution
    Eigen::VectorXd solution = solver_->getSolution();
    extractSolution(solution);
    
    return true;
}

Eigen::VectorXd ModelPredictiveControl::getOptimalControl() const {
    if (optimal_controls_.empty()) {
        return Eigen::VectorXd::Zero(control_dim_);
    }
    
    return optimal_controls_[0];
}

const std::vector<Eigen::VectorXd>& ModelPredictiveControl::getPredictedStates() const {
    return predicted_states_;
}

const std::vector<Eigen::VectorXd>& ModelPredictiveControl::getOptimalControls() const {
    return optimal_controls_;
}

void ModelPredictiveControl::linearizeDynamics(const Eigen::VectorXd& state,
                                         const Eigen::VectorXd& control,
                                         std::vector<Eigen::MatrixXd>& A_matrices,
                                         std::vector<Eigen::MatrixXd>& B_matrices,
                                         std::vector<Eigen::VectorXd>& c_vectors) {
    // Clear output vectors
    A_matrices.clear();
    B_matrices.clear();
    c_vectors.clear();
    
    // Current state and control for linearization
    Eigen::VectorXd current_lin_state = state;
    Eigen::VectorXd current_lin_control = control;
    
    // For each step in the prediction horizon
    for (int i = 0; i < prediction_horizon_; ++i) {
        // Get linearized dynamics at current state and control
        Eigen::MatrixXd A, B;
        Eigen::VectorXd c;
        
        dynamics_->getLinearizedDiscreteModel(current_lin_state, current_lin_control, 
                                           control_interval_, A, B, c);
        
        // Store linearized matrices
        A_matrices.push_back(A);
        B_matrices.push_back(B);
        c_vectors.push_back(c);
        
        // Predict next state for next linearization point
        current_lin_state = A * current_lin_state + B * current_lin_control + c;
    }
}

void ModelPredictiveControl::generateReferenceStates(double current_time,
                                               std::vector<Eigen::VectorXd>& reference_trajectory) {
    // Clear output vector
    reference_trajectory.clear();
    
    // Check if reference trajectory is available
    if (reference_states_.empty() || reference_times_.empty()) {
        // No reference trajectory, use current state as reference
        for (int i = 0; i < prediction_horizon_; ++i) {
            reference_trajectory.push_back(current_state_);
        }
        return;
    }
    
    // Generate reference states for each step in the prediction horizon
    for (int i = 0; i < prediction_horizon_; ++i) {
        double target_time = current_time + i * control_interval_;
        Eigen::VectorXd reference = interpolateReferenceState(target_time);
        reference_trajectory.push_back(reference);
    }
}

Eigen::VectorXd ModelPredictiveControl::interpolateReferenceState(double target_time) {
    // Find the two closest time points
    int idx = 0;
    while (idx < reference_times_.size() - 1 && reference_times_[idx + 1] < target_time) {
        idx++;
    }
    
    // If target time is before the first reference point or after the last one
    if (idx >= reference_times_.size() - 1) {
        return reference_states_.back();
    }
    if (target_time <= reference_times_[0]) {
        return reference_states_[0];
    }
    
    // Linear interpolation between two reference states
    double t0 = reference_times_[idx];
    double t1 = reference_times_[idx + 1];
    double alpha = (target_time - t0) / (t1 - t0);
    
    Eigen::VectorXd state0 = reference_states_[idx];
    Eigen::VectorXd state1 = reference_states_[idx + 1];
    
    // Special handling for heading angle to interpolate correctly
    double heading0 = state0(2);
    double heading1 = state1(2);
    
    // Ensure heading difference is in [-pi, pi]
    double heading_diff = heading1 - heading0;
    while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
    while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
    
    // Interpolate heading
    double interpolated_heading = heading0 + alpha * heading_diff;
    while (interpolated_heading > M_PI) interpolated_heading -= 2.0 * M_PI;
    while (interpolated_heading < -M_PI) interpolated_heading += 2.0 * M_PI;
    
    // Linear interpolation for other states
    Eigen::VectorXd interpolated_state = state0 + alpha * (state1 - state0);
    
    // Override heading with correctly interpolated value
    interpolated_state(2) = interpolated_heading;
    
    return interpolated_state;
}

bool ModelPredictiveControl::setupQPProblem(const std::vector<Eigen::VectorXd>& reference_states,
                                      const std::vector<Eigen::MatrixXd>& A_matrices,
                                      const std::vector<Eigen::MatrixXd>& B_matrices,
                                      const std::vector<Eigen::VectorXd>& c_vectors) {
    // Calculate problem dimensions
    int n_states = state_dim_ * prediction_horizon_;
    int n_controls = control_dim_ * prediction_horizon_;
    int n_variables = n_states + n_controls;
    
    // Calculate number of constraints: dynamics + state bounds + control bounds
    int n_eq_constraints = state_dim_ * prediction_horizon_;  // Dynamics constraints
    int n_ineq_constraints = 2 * (2 + control_dim_) * prediction_horizon_;  // x, y bounds + control bounds
    int n_constraints = n_eq_constraints + n_ineq_constraints;
    
    // Create and populate matrices for the QP problem
    Eigen::SparseMatrix<double> P(n_variables, n_variables);  // Quadratic cost matrix
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_variables);   // Linear cost vector
    Eigen::SparseMatrix<double> A(n_constraints, n_variables); // Constraint matrix
    Eigen::VectorXd l(n_constraints);                          // Lower bound vector
    Eigen::VectorXd u(n_constraints);                          // Upper bound vector
    
    // Build cost matrices
    cost_functions_->buildQPCostMatrices(prediction_horizon_, reference_states, current_control_, P, q);
    
    // Initialize A, l, u for constraints
    std::vector<Eigen::Triplet<double>> A_triplets;
    
    // 1. Dynamics constraints (equality): x_{k+1} = A_k * x_k + B_k * u_k + c_k
    int constraint_idx = 0;
    
    for (int k = 0; k < prediction_horizon_ - 1; ++k) {
        // For each state variable
        for (int i = 0; i < state_dim_; ++i) {
            // x_{k+1,i} term (with coefficient 1)
            A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, (k+1) * state_dim_ + i, 1.0));
            
            // -A_k * x_k terms
            for (int j = 0; j < state_dim_; ++j) {
                A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, k * state_dim_ + j, -A_matrices[k](i, j)));
            }
            
            // -B_k * u_k terms
            for (int j = 0; j < control_dim_; ++j) {
                A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, n_states + k * control_dim_ + j, -B_matrices[k](i, j)));
            }
            
            // c_k term (constant term in dynamics)
            l(constraint_idx) = c_vectors[k](i);
            u(constraint_idx) = c_vectors[k](i);
            
            constraint_idx++;
        }
    }
    
    // Initial state constraint: x_0 = current_state_
    for (int i = 0; i < state_dim_; ++i) {
        A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, i, 1.0));
        l(constraint_idx) = current_state_(i);
        u(constraint_idx) = current_state_(i);
        constraint_idx++;
    }
    
    // 2. State bounds constraints (inequality): x_min <= x <= x_max, y_min <= y <= y_max
    for (int k = 0; k < prediction_horizon_; ++k) {
        // X position lower bound
        A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, k * state_dim_ + 0, 1.0));
        l(constraint_idx) = x_min_;
        u(constraint_idx) = OSQP_INFTY;
        constraint_idx++;
        
        // X position upper bound
        A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, k * state_dim_ + 0, 1.0));
        l(constraint_idx) = -OSQP_INFTY;
        u(constraint_idx) = x_max_;
        constraint_idx++;
        
        // Y position lower bound
        A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, k * state_dim_ + 1, 1.0));
        l(constraint_idx) = y_min_;
        u(constraint_idx) = OSQP_INFTY;
        constraint_idx++;
        
        // Y position upper bound
        A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, k * state_dim_ + 1, 1.0));
        l(constraint_idx) = -OSQP_INFTY;
        u(constraint_idx) = y_max_;
        constraint_idx++;
    }
    
    // 3. Control bounds constraints (inequality): u_min <= u <= u_max
    for (int k = 0; k < prediction_horizon_; ++k) {
        for (int i = 0; i < control_dim_; ++i) {
            // Control lower bound
            A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, n_states + k * control_dim_ + i, 1.0));
            l(constraint_idx) = u_min_(i);
            u(constraint_idx) = OSQP_INFTY;
            constraint_idx++;
            
            // Control upper bound
            A_triplets.push_back(Eigen::Triplet<double>(constraint_idx, n_states + k * control_dim_ + i, 1.0));
            l(constraint_idx) = -OSQP_INFTY;
            u(constraint_idx) = u_max_(i);
            constraint_idx++;
        }
    }
    
    // Set constraint matrix from triplets
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());
    
    // Initialize the solver with problem size
    solver_->data()->setNumberOfVariables(n_variables);
    solver_->data()->setNumberOfConstraints(n_constraints);
    
    // Set the quadratic cost and linear cost
    if (!solver_->data()->setHessianMatrix(P)) return false;
    if (!solver_->data()->setGradient(q)) return false;
    
    // Set the linear constraints
    if (!solver_->data()->setLinearConstraintsMatrix(A)) return false;
    if (!solver_->data()->setLowerBound(l)) return false;
    if (!solver_->data()->setUpperBound(u)) return false;
    
    // Initialize the solver
    return solver_->initSolver();
}

void ModelPredictiveControl::extractSolution(const Eigen::VectorXd& solution) {
    // Clear previous solutions
    predicted_states_.clear();
    optimal_controls_.clear();
    
    // Extract predicted states
    for (int k = 0; k < prediction_horizon_; ++k) {
        Eigen::VectorXd state = solution.segment(k * state_dim_, state_dim_);
        predicted_states_.push_back(state);
    }
    
    // Extract optimal controls
    for (int k = 0; k < prediction_horizon_; ++k) {
        Eigen::VectorXd control = solution.segment(state_dim_ * prediction_horizon_ + k * control_dim_, control_dim_);
        optimal_controls_.push_back(control);
    }
}

} // namespace asv_planning 