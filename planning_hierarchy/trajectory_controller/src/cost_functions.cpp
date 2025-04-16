#include "cost_functions.hpp"
#include "../../asv_core/include/asv_dynamics.hpp"
#include "../../../environment_representation/include/multi_resolution_grid.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace asv_planning {

CostFunctions::CostFunctions()
    : position_weight_(10.0),
      heading_weight_(5.0),
      velocity_weight_(1.0),
      control_weight_(0.1),
      control_rate_weight_(0.5),
      energy_weight_(1.0),
      state_dim_(6),
      control_dim_(2) {
}

void CostFunctions::setDynamicsModel(std::shared_ptr<ASVDynamics> dynamics) {
    dynamics_ = dynamics;
}

void CostFunctions::setEnvironmentMap(std::shared_ptr<MultiResolutionGrid> map) {
    map_ = map;
}

void CostFunctions::setWeights(double position_weight, double heading_weight,
                            double velocity_weight, double control_weight,
                            double control_rate_weight, double energy_weight) {
    position_weight_ = position_weight;
    heading_weight_ = heading_weight;
    velocity_weight_ = velocity_weight;
    control_weight_ = control_weight;
    control_rate_weight_ = control_rate_weight;
    energy_weight_ = energy_weight;
}

void CostFunctions::getStateCostMatrix(Eigen::MatrixXd& Q) const {
    Q = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
    
    // Position tracking weights for x and y
    Q(0, 0) = position_weight_;
    Q(1, 1) = position_weight_;
    
    // Heading tracking weight
    Q(2, 2) = heading_weight_;
    
    // Velocity tracking weights
    Q(3, 3) = velocity_weight_;  // Surge
    Q(4, 4) = velocity_weight_;  // Sway
    Q(5, 5) = velocity_weight_;  // Yaw rate
}

void CostFunctions::getControlCostMatrix(Eigen::MatrixXd& R) const {
    R = Eigen::MatrixXd::Zero(control_dim_, control_dim_);
    
    // Control effort weights for left and right thrusters
    R(0, 0) = control_weight_;
    R(1, 1) = control_weight_;
}

void CostFunctions::getControlRateCostMatrix(Eigen::MatrixXd& S) const {
    S = Eigen::MatrixXd::Zero(control_dim_, control_dim_);
    
    // Control rate weights for left and right thrusters
    S(0, 0) = control_rate_weight_;
    S(1, 1) = control_rate_weight_;
}

double CostFunctions::calculateTrackingCost(const Eigen::VectorXd& state,
                                        const Eigen::VectorXd& reference) const {
    Eigen::VectorXd error = state - reference;
    
    // Normalize heading error to [-pi, pi]
    while (error(2) > M_PI) error(2) -= 2.0 * M_PI;
    while (error(2) < -M_PI) error(2) += 2.0 * M_PI;
    
    // Get state cost matrix
    Eigen::MatrixXd Q;
    getStateCostMatrix(Q);
    
    return calculateQuadraticCost(error, Q);
}

double CostFunctions::calculateControlCost(const Eigen::VectorXd& control) const {
    Eigen::MatrixXd R;
    getControlCostMatrix(R);
    
    return calculateQuadraticCost(control, R);
}

double CostFunctions::calculateControlRateCost(const Eigen::VectorXd& control,
                                          const Eigen::VectorXd& prev_control) const {
    Eigen::VectorXd rate = control - prev_control;
    
    Eigen::MatrixXd S;
    getControlRateCostMatrix(S);
    
    return calculateQuadraticCost(rate, S);
}

double CostFunctions::calculateEnergyCost(const Eigen::VectorXd& control, double dt) const {
    if (!dynamics_) {
        return 0.0;  // No dynamics model, can't calculate energy
    }
    
    double energy = dynamics_->computeEnergyConsumption(control, dt);
    
    return energy_weight_ * energy;
}

void CostFunctions::buildQPCostMatrices(int horizon_steps,
                                    const std::vector<Eigen::VectorXd>& reference_states,
                                    const Eigen::VectorXd& current_control,
                                    Eigen::SparseMatrix<double>& P,
                                    Eigen::VectorXd& q) {
    int n_states = state_dim_ * horizon_steps;
    int n_controls = control_dim_ * horizon_steps;
    int total_vars = n_states + n_controls;
    
    // Initialize cost matrices
    P.resize(total_vars, total_vars);
    q = Eigen::VectorXd::Zero(total_vars);
    
    // Get cost weight matrices
    Eigen::MatrixXd Q, R, S;
    getStateCostMatrix(Q);
    getControlCostMatrix(R);
    getControlRateCostMatrix(S);
    
    // Build sparse P matrix (block diagonal)
    std::vector<Eigen::Triplet<double>> triplets;
    
    // Add state cost terms to P
    for (int k = 0; k < horizon_steps; ++k) {
        for (int i = 0; i < state_dim_; ++i) {
            for (int j = 0; j < state_dim_; ++j) {
                if (Q(i, j) != 0.0) {
                    int row = k * state_dim_ + i;
                    int col = k * state_dim_ + j;
                    triplets.push_back(Eigen::Triplet<double>(row, col, Q(i, j)));
                }
            }
        }
    }
    
    // Add control cost terms to P
    for (int k = 0; k < horizon_steps; ++k) {
        for (int i = 0; i < control_dim_; ++i) {
            for (int j = 0; j < control_dim_; ++j) {
                if (R(i, j) != 0.0) {
                    int row = n_states + k * control_dim_ + i;
                    int col = n_states + k * control_dim_ + j;
                    triplets.push_back(Eigen::Triplet<double>(row, col, R(i, j)));
                }
            }
        }
    }
    
    // Add control rate cost terms to P
    for (int k = 0; k < horizon_steps; ++k) {
        for (int i = 0; i < control_dim_; ++i) {
            for (int j = 0; j < control_dim_; ++j) {
                if (S(i, j) != 0.0) {
                    int row = n_states + k * control_dim_ + i;
                    int col = n_states + k * control_dim_ + j;
                    
                    // If not the first control step, add rate term
                    if (k > 0) {
                        triplets.push_back(Eigen::Triplet<double>(row, col, S(i, j) * 2.0));
                        triplets.push_back(Eigen::Triplet<double>(row, col - control_dim_, -S(i, j)));
                        triplets.push_back(Eigen::Triplet<double>(row - control_dim_, col, -S(i, j)));
                    }
                }
            }
        }
    }
    
    // Set sparse P matrix
    P.setFromTriplets(triplets.begin(), triplets.end());
    
    // Build q vector (linear terms)
    for (int k = 0; k < horizon_steps; ++k) {
        // Reference tracking linear terms
        if (k < reference_states.size()) {
            Eigen::VectorXd ref_term = -2.0 * Q * reference_states[k];
            q.segment(k * state_dim_, state_dim_) = ref_term;
        }
        
        // Control rate linear terms (for first step only)
        if (k == 0) {
            Eigen::VectorXd rate_term = -2.0 * S * current_control;
            q.segment(n_states, control_dim_) = rate_term;
        }
    }
}

double CostFunctions::calculateEnvironmentalCost(const Eigen::VectorXd& state) const {
    if (!map_) {
        return 0.0;  // No map, no environmental cost
    }
    
    double x = state(0);
    double y = state(1);
    double heading = state(2);
    
    Eigen::Vector2d position(x, y);
    
    // Get environmental factors from map
    double current_magnitude = map_->getForceFieldMagnitude(position);
    double current_direction = map_->getForceFieldDirection(position);
    double wave_height = map_->getWaveHeight(position);
    
    // Calculate angle between heading and current
    double angle_diff = std::abs(heading - current_direction);
    while (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;
    
    // Energy cost increases when moving against current
    double current_cost = current_magnitude * std::cos(angle_diff) * (-1.0);
    
    // Energy cost increases with wave height
    double wave_cost = wave_height * 0.5;
    
    // Lower cost in shipping lanes
    double lane_factor = map_->isInShippingLane(position) ? -0.2 : 0.0;
    
    // Combined environmental cost
    return energy_weight_ * (current_cost + wave_cost + lane_factor);
}

int CostFunctions::getStateDimension() const {
    return state_dim_;
}

int CostFunctions::getControlDimension() const {
    return control_dim_;
}

double CostFunctions::calculateQuadraticCost(const Eigen::VectorXd& error,
                                        const Eigen::MatrixXd& weight) const {
    return error.transpose() * weight * error;
}

} // namespace asv_planning 