#include "asv_dynamics.hpp"
#include <cmath>

namespace asv_planning {

ASVDynamics::ASVDynamics(double mass, double inertia_z, double length)
    : mass_(mass), inertia_z_(inertia_z), length_(length),
      propeller_coefficient_(1.0), energy_coefficient_(1.0) {
    
    // Default damping coefficients (should be set properly based on vehicle characteristics)
    linear_damping_ = Eigen::Vector3d(50.0, 100.0, 30.0);  // [X_u, Y_v, N_r]
    quadratic_damping_ = Eigen::Vector3d(100.0, 200.0, 50.0);  // [X_uu, Y_vv, N_rr]
}

Eigen::VectorXd ASVDynamics::updateState(
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& control,
    double dt) {
    
    // State: [x, y, theta, u, v, r]
    // Control: [thrust_left, thrust_right]
    
    // Extract state variables
    double theta = state(2);  // Heading angle
    double u = state(3);      // Surge velocity (forward)
    double v = state(4);      // Sway velocity (lateral)
    double r = state(5);      // Yaw rate (angular velocity)
    
    // Calculate forces and moments
    Eigen::Vector3d forces = calculateForces(state, control);
    double X = forces(0);  // Surge force
    double Y = forces(1);  // Sway force
    double N = forces(2);  // Yaw moment
    
    // Acceleration
    double u_dot = X / mass_ + v * r;  // Surge acceleration
    double v_dot = Y / mass_ - u * r;  // Sway acceleration
    double r_dot = N / inertia_z_;     // Yaw acceleration
    
    // Euler integration for velocity
    double u_new = u + u_dot * dt;
    double v_new = v + v_dot * dt;
    double r_new = r + r_dot * dt;
    
    // Rotation matrix from body to world frame
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    
    // Position update in world frame
    double x_dot = u * cos_theta - v * sin_theta;
    double y_dot = u * sin_theta + v * cos_theta;
    double theta_dot = r;
    
    // Euler integration for position
    double x_new = state(0) + x_dot * dt;
    double y_new = state(1) + y_dot * dt;
    double theta_new = theta + theta_dot * dt;
    
    // Normalize angle to [-pi, pi]
    while (theta_new > M_PI) theta_new -= 2.0 * M_PI;
    while (theta_new < -M_PI) theta_new += 2.0 * M_PI;
    
    // Create and return new state vector
    Eigen::VectorXd new_state(6);
    new_state << x_new, y_new, theta_new, u_new, v_new, r_new;
    
    return new_state;
}

double ASVDynamics::computeEnergyConsumption(
    const Eigen::VectorXd& control,
    double dt) {
    
    // Simple model: energy = k * (thrust_left^2 + thrust_right^2) * dt
    double thrust_left = control(0);
    double thrust_right = control(1);
    
    double energy = energy_coefficient_ * (thrust_left * thrust_left + thrust_right * thrust_right) * dt;
    
    return energy;
}

void ASVDynamics::setDampingCoefficients(
    const Eigen::Vector3d& linear_damping,
    const Eigen::Vector3d& quadratic_damping) {
    
    linear_damping_ = linear_damping;
    quadratic_damping_ = quadratic_damping;
}

void ASVDynamics::setPropulsionParameters(
    double propeller_coefficient,
    double energy_coefficient) {
    
    propeller_coefficient_ = propeller_coefficient;
    energy_coefficient_ = energy_coefficient;
}

double ASVDynamics::getMass() const {
    return mass_;
}

double ASVDynamics::getInertiaZ() const {
    return inertia_z_;
}

Eigen::Vector3d ASVDynamics::calculateForces(
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& control) {
    
    // Extract state variables
    double u = state(3);  // Surge velocity
    double v = state(4);  // Sway velocity
    double r = state(5);  // Yaw rate
    
    // Extract control inputs
    double thrust_left = control(0);
    double thrust_right = control(1);
    
    // Damping forces
    double X_damping = -linear_damping_(0) * u - quadratic_damping_(0) * u * std::abs(u);
    double Y_damping = -linear_damping_(1) * v - quadratic_damping_(1) * v * std::abs(v);
    double N_damping = -linear_damping_(2) * r - quadratic_damping_(2) * r * std::abs(r);
    
    // Propulsion forces
    double X_prop = propeller_coefficient_ * (thrust_left + thrust_right);
    
    // Propulsion moment (yaw)
    double lever_arm = length_ / 2.0;  // Distance from center to propeller
    double N_prop = propeller_coefficient_ * lever_arm * (thrust_right - thrust_left);
    
    // Total forces and moment
    double X = X_damping + X_prop;
    double Y = Y_damping;
    double N = N_damping + N_prop;
    
    return Eigen::Vector3d(X, Y, N);
}

} // namespace asv_planning 