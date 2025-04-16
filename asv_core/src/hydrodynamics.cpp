#include "../include/hydrodynamics.hpp"
#include <cmath>

namespace asv_planning {

Hydrodynamics::Hydrodynamics(double vessel_length, double vessel_width, double vessel_mass)
    : length_(vessel_length), width_(vessel_width), mass_(vessel_mass) {
    // Initialize hydrodynamic coefficients based on vessel geometry
    calculateDefaultCoefficients();
}

Eigen::Matrix3d Hydrodynamics::getAddedMassMatrix(const Eigen::Vector3d& velocity) const {
    // For most applications, added mass can be modeled as constant
    // In more advanced models, it might depend on velocity or frequency
    return added_mass_matrix_;
}

Eigen::Vector3d Hydrodynamics::getDampingForces(const Eigen::Vector3d& velocity) const {
    // Damping forces include linear and quadratic components
    // F_damping = -D_linear * velocity - D_quadratic * velocity * |velocity|
    
    Eigen::Vector3d linear_damping = -linear_damping_matrix_ * velocity;
    
    // Apply quadratic damping (element-wise multiplication with sign consideration)
    Eigen::Vector3d quadratic_damping;
    for (int i = 0; i < 3; ++i) {
        double v = velocity(i);
        double v_abs = std::abs(v);
        quadratic_damping(i) = -quadratic_damping_matrix_(i, i) * v * v_abs;
    }
    
    return linear_damping + quadratic_damping;
}

Eigen::Vector3d Hydrodynamics::getRestoringForces(double roll, double pitch) const {
    // For a surface vessel, restoring forces primarily affect roll and pitch
    // For path planning, these are often simplified
    
    // Simplified model assuming small angles
    // Weight and buoyancy are balanced in heave
    double X_r = -mass_ * 9.81 * std::sin(pitch);  // Restoring force in surge
    double Y_r = mass_ * 9.81 * std::sin(roll);    // Restoring force in sway
    double N_r = 0.0;  // No direct restoring moment in yaw for a symmetric vessel
    
    return Eigen::Vector3d(X_r, Y_r, N_r);
}

Eigen::Vector3d Hydrodynamics::getHydrodynamicForces(
    const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& acceleration) const {
    
    // Inertial forces (added mass)
    Eigen::Vector3d inertial_forces = -added_mass_matrix_ * acceleration;
    
    // Damping forces
    Eigen::Vector3d damping_forces = getDampingForces(velocity);
    
    // For a complete model, we would also include:
    // - Wave-induced forces
    // - Current-induced forces
    // - Other environmental effects
    
    // Return total hydrodynamic forces
    return inertial_forces + damping_forces;
}

void Hydrodynamics::setVesselParameters(double vessel_length, double vessel_width, double vessel_mass) {
    length_ = vessel_length;
    width_ = vessel_width;
    mass_ = vessel_mass;
    
    // Recalculate hydrodynamic coefficients
    calculateDefaultCoefficients();
}

void Hydrodynamics::setHydrodynamicCoefficients(
    const Eigen::Matrix3d& added_mass,
    const Eigen::Matrix3d& linear_damping,
    const Eigen::Matrix3d& quadratic_damping) {
    
    added_mass_matrix_ = added_mass;
    linear_damping_matrix_ = linear_damping;
    quadratic_damping_matrix_ = quadratic_damping;
}

void Hydrodynamics::calculateDefaultCoefficients() {
    // Calculate reasonable default values based on vessel geometry
    // These are simplified estimates; for accurate values, vessel-specific
    // hydrodynamic analysis or experiments are required
    
    // Added mass estimates (simplified for a surface vessel)
    // - Surge: ~10-20% of vessel mass
    // - Sway: ~40-70% of vessel mass
    // - Yaw: ~10-30% of vessel mass * (length/2)^2
    double X_added = 0.15 * mass_;
    double Y_added = 0.6 * mass_;
    double N_added = 0.2 * mass_ * std::pow(length_ / 2.0, 2);
    
    // Cross-coupling terms
    double N_v_added = 0.05 * mass_ * length_ / 2.0;
    double Y_r_added = N_v_added;  // Symmetric for many vessels
    
    // Create added mass matrix
    added_mass_matrix_ << X_added, 0, 0,
                          0, Y_added, Y_r_added,
                          0, N_v_added, N_added;
    
    // Linear damping (simplified)
    double X_u = 0.02 * mass_;  // Surge linear damping
    double Y_v = 0.10 * mass_;  // Sway linear damping
    double N_r = 0.05 * mass_ * std::pow(length_ / 2.0, 2);  // Yaw linear damping
    
    // Create linear damping matrix
    linear_damping_matrix_ << X_u, 0, 0,
                             0, Y_v, 0,
                             0, 0, N_r;
    
    // Quadratic damping (simplified)
    double X_uu = 0.5 * 1000 * length_ * width_ * 0.4;  // Surge quadratic damping
    double Y_vv = 0.5 * 1000 * length_ * width_ * 0.9;  // Sway quadratic damping
    double N_rr = 0.5 * 1000 * std::pow(length_, 4) * 0.008;  // Yaw quadratic damping
    
    // Create quadratic damping matrix
    quadratic_damping_matrix_ << X_uu, 0, 0,
                                0, Y_vv, 0,
                                0, 0, N_rr;
}

} // namespace asv_planning 