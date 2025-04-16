#ifndef ASV_DYNAMICS_HPP
#define ASV_DYNAMICS_HPP

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace asv_planning {

/**
 * @brief 3-DOF ASV dynamics model for surge, sway, and yaw
 * 
 * This class implements a simplified 3-DOF model of an autonomous surface vehicle,
 * including inertia, damping, and propulsion effects. The model can be used for
 * both simulation and for the prediction in the MPC controller.
 */
class ASVDynamics {
public:
    /**
     * @brief Constructor with model parameters
     * 
     * @param mass Vehicle mass (kg)
     * @param inertia_z Moment of inertia around z-axis (kg⋅m²)
     * @param length Vehicle length (m)
     */
    ASVDynamics(double mass, double inertia_z, double length);
    
    /**
     * @brief Default destructor
     */
    virtual ~ASVDynamics() = default;
    
    /**
     * @brief State update function to integrate dynamics
     * 
     * @param state Current state vector (x, y, theta, u, v, r)
     * @param control Control vector (thrust_left, thrust_right)
     * @param dt Time step (s)
     * @return Updated state vector
     */
    Eigen::VectorXd updateState(
        const Eigen::VectorXd& state,
        const Eigen::VectorXd& control,
        double dt);
    
    /**
     * @brief Compute energy consumption for a given control input
     * 
     * @param control Control vector (thrust_left, thrust_right)
     * @param dt Time step (s)
     * @return Energy consumed in Joules
     */
    double computeEnergyConsumption(
        const Eigen::VectorXd& control,
        double dt);
    
    /**
     * @brief Set hydrodynamic damping coefficients
     * 
     * @param linear_damping Linear damping coefficients [X_u, Y_v, N_r]
     * @param quadratic_damping Quadratic damping coefficients [X_uu, Y_vv, N_rr]
     */
    void setDampingCoefficients(
        const Eigen::Vector3d& linear_damping,
        const Eigen::Vector3d& quadratic_damping);
    
    /**
     * @brief Set propulsion system parameters
     * 
     * @param propeller_coefficient Propeller thrust coefficient
     * @param energy_coefficient Energy consumption coefficient
     */
    void setPropulsionParameters(
        double propeller_coefficient,
        double energy_coefficient);
    
    /**
     * @brief Get the vehicle mass
     * 
     * @return Vehicle mass (kg)
     */
    double getMass() const;
    
    /**
     * @brief Get the vehicle inertia around z-axis
     * 
     * @return Moment of inertia (kg⋅m²)
     */
    double getInertiaZ() const;
    
private:
    // Vehicle physical parameters
    double mass_;                  ///< Vehicle mass (kg)
    double inertia_z_;             ///< Moment of inertia around z-axis (kg⋅m²)
    double length_;                ///< Vehicle length (m)
    
    // Hydrodynamic parameters
    Eigen::Vector3d linear_damping_;      ///< Linear damping coefficients [X_u, Y_v, N_r]
    Eigen::Vector3d quadratic_damping_;   ///< Quadratic damping coefficients [X_uu, Y_vv, N_rr]
    
    // Propulsion parameters
    double propeller_coefficient_; ///< Propeller thrust coefficient
    double energy_coefficient_;    ///< Energy consumption coefficient
    
    /**
     * @brief Calculate forces and moments based on state and control
     * 
     * @param state Current state vector (x, y, theta, u, v, r)
     * @param control Control vector (thrust_left, thrust_right)
     * @return Vector of forces and moments [X, Y, N]
     */
    Eigen::Vector3d calculateForces(
        const Eigen::VectorXd& state,
        const Eigen::VectorXd& control);
};

} // namespace asv_planning

#endif // ASV_DYNAMICS_HPP 