#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <functional>

namespace asv_planning {

/**
 * @brief Configuration parameters for the ASV dynamics model
 */
struct ASVDynamicsConfig {
    // Vehicle parameters
    double mass = 500.0;            // Mass of the vehicle (kg)
    double length = 5.0;            // Length of the vehicle (m)
    double width = 2.0;             // Width of the vehicle (m)
    
    // Hydrodynamic parameters
    double X_u_dot = -50.0;         // Added mass in surge (kg)
    double Y_v_dot = -200.0;        // Added mass in sway (kg)
    double N_r_dot = -80.0;         // Added mass in yaw (kg.m^2)
    double I_z = 400.0;             // Moment of inertia about z-axis (kg.m^2)
    
    // Linear damping coefficients
    double X_u = -25.0;             // Linear damping in surge (kg/s)
    double Y_v = -80.0;             // Linear damping in sway (kg/s)
    double N_r = -50.0;             // Linear damping in yaw (kg.m^2/s)
    
    // Quadratic damping coefficients
    double X_uu = -15.0;            // Quadratic damping in surge (kg/m)
    double Y_vv = -60.0;            // Quadratic damping in sway (kg/m)
    double N_rr = -40.0;            // Quadratic damping in yaw (kg.m^2)
    
    // Cross-coupling terms
    double Y_r = -10.0;             // Coupling between sway and yaw rate (kg.m/s)
    double N_v = -10.0;             // Coupling between yaw and sway velocity (kg.m/s)
    double X_vr = 0.0;              // Coupling term (kg.m/s)
    double Y_ur = 0.0;              // Coupling term (kg.m/s)
    double N_ur = -10.0;            // Coupling term (kg.m^2/s)
    
    // Control parameters
    double X_rudder = -30.0;        // Rudder force coefficient in surge (kg/s^2)
    double Y_rudder = -80.0;        // Rudder force coefficient in sway (kg/s^2)
    double N_rudder = -40.0;        // Rudder torque coefficient (kg.m/s^2)
    
    // Actuator efficiency
    double thrust_efficiency = 0.7; // Thrust actuator efficiency
    double rudder_efficiency = 0.8; // Rudder actuator efficiency
    
    // Environmental parameters
    double water_density = 1025.0;  // Water density (kg/m^3)
    
    // Simulation parameters
    double dt = 0.1;                // Default time step for simulation (s)
    
    // Energy consumption parameters
    double k_thrust_power = 0.5;    // Power coefficient for thrust (W/N^2)
    double k_rudder_power = 0.1;    // Power coefficient for rudder (W/rad^2)
    double base_power = 100.0;      // Base power consumption (W)
};

/**
 * @brief ASV state representation
 */
struct ASVState {
    double x;        // X position (m)
    double y;        // Y position (m)
    double psi;      // Heading angle (rad)
    double u;        // Surge velocity (m/s)
    double v;        // Sway velocity (m/s)
    double r;        // Yaw rate (rad/s)
    
    // Convert to Eigen vector
    Eigen::VectorXd toEigen() const {
        Eigen::VectorXd state(6);
        state << x, y, psi, u, v, r;
        return state;
    }
    
    // Create from Eigen vector
    static ASVState fromEigen(const Eigen::VectorXd& state) {
        return {state(0), state(1), state(2), state(3), state(4), state(5)};
    }
};

/**
 * @brief Control input for the ASV
 */
struct ASVControl {
    double thrust;   // Thrust force (N)
    double rudder;   // Rudder angle (rad)
    
    // Convert to Eigen vector
    Eigen::VectorXd toEigen() const {
        Eigen::VectorXd control(2);
        control << thrust, rudder;
        return control;
    }
    
    // Create from Eigen vector
    static ASVControl fromEigen(const Eigen::VectorXd& control) {
        return {control(0), control(1)};
    }
};

/**
 * @brief ASV dynamics model for simulation and control
 * 
 * This class implements a 3-DOF (surge, sway, yaw) dynamic model of an ASV.
 * It includes both kinematics and dynamics, as well as methods to compute
 * energy consumption and simulate the ASV behavior over time.
 */
class ASVDynamics {
public:
    /**
     * @brief Constructor
     * 
     * @param config Configuration parameters for the dynamics model
     */
    explicit ASVDynamics(const ASVDynamicsConfig& config = ASVDynamicsConfig());
    
    /**
     * @brief Destructor
     */
    ~ASVDynamics();
    
    /**
     * @brief Set the configuration parameters
     * 
     * @param config Configuration parameters
     */
    void setConfig(const ASVDynamicsConfig& config);
    
    /**
     * @brief Get the current configuration parameters
     * 
     * @return Configuration parameters
     */
    const ASVDynamicsConfig& getConfig() const;
    
    /**
     * @brief Compute the derivatives of the state vector
     * 
     * @param state Current state (x, y, psi, u, v, r)
     * @param control Control input (thrust, rudder)
     * @return Derivatives of the state (dx/dt, dy/dt, dpsi/dt, du/dt, dv/dt, dr/dt)
     */
    Eigen::VectorXd stateDerivative(const Eigen::VectorXd& state, const Eigen::VectorXd& control) const;
    
    /**
     * @brief Simulate one step forward using Euler integration
     * 
     * @param state Current state
     * @param control Control input
     * @param dt Time step (s)
     * @return Next state
     */
    Eigen::VectorXd step(const Eigen::VectorXd& state, const Eigen::VectorXd& control, double dt) const;
    
    /**
     * @brief Compute the energy consumption for a given state and control
     * 
     * @param state Current state
     * @param control Control input
     * @return Energy consumption rate (W)
     */
    double computeEnergyConsumption(const Eigen::VectorXd& state, const Eigen::VectorXd& control) const;
    
    /**
     * @brief Set the environmental disturbance function
     * 
     * The function should take the current state and time as input and return
     * an additional force/torque vector to be added to the dynamics.
     * 
     * @param disturbance_function Function that computes environmental disturbances
     */
    void setDisturbanceFunction(
        std::function<Eigen::Vector3d(const Eigen::VectorXd&, double)> disturbance_function);
    
    /**
     * @brief Check if the ASV state is within feasible operation range
     * 
     * @param state ASV state to check
     * @return true if state is feasible, false otherwise
     */
    bool isStateFeasible(const Eigen::VectorXd& state) const;
    
    /**
     * @brief Check if the control input is within feasible range
     * 
     * @param control Control input to check
     * @return true if control is feasible, false otherwise
     */
    bool isControlFeasible(const Eigen::VectorXd& control) const;
    
    /**
     * @brief Linearize the dynamics around an operating point
     * 
     * @param state Operating point state
     * @param control Operating point control
     * @param A Output state matrix (dx/dx)
     * @param B Output control matrix (dx/du)
     */
    void linearize(const Eigen::VectorXd& state, const Eigen::VectorXd& control,
                   Eigen::MatrixXd& A, Eigen::MatrixXd& B) const;
    
private:
    /**
     * @brief Compute the damping forces and moments
     * 
     * @param velocity Velocity vector (u, v, r)
     * @return Damping forces and moment (X, Y, N)
     */
    Eigen::Vector3d computeDamping(const Eigen::Vector3d& velocity) const;
    
    /**
     * @brief Compute the Coriolis and centripetal forces
     * 
     * @param velocity Velocity vector (u, v, r)
     * @return Coriolis forces and moment (X, Y, N)
     */
    Eigen::Vector3d computeCoriolis(const Eigen::Vector3d& velocity) const;
    
    /**
     * @brief Compute the control forces from thrust and rudder
     * 
     * @param velocity Velocity vector (u, v, r)
     * @param control Control input (thrust, rudder)
     * @return Control forces and moment (X, Y, N)
     */
    Eigen::Vector3d computeControlForces(const Eigen::Vector3d& velocity, const Eigen::VectorXd& control) const;
    
    // Mass and inertia matrix (M)
    Eigen::Matrix3d mass_matrix_;
    // Inverse of mass matrix (M^-1) for efficient computation
    Eigen::Matrix3d inv_mass_matrix_;
    
    // Configuration parameters
    ASVDynamicsConfig config_;
    
    // Environmental disturbance function
    std::function<Eigen::Vector3d(const Eigen::VectorXd&, double)> disturbance_function_;
    
    // State and control limits
    double max_surge_ = 10.0;       // Maximum surge velocity (m/s)
    double max_sway_ = 5.0;         // Maximum sway velocity (m/s)
    double max_yaw_rate_ = 1.0;     // Maximum yaw rate (rad/s)
    double max_thrust_ = 1000.0;    // Maximum thrust force (N)
    double max_rudder_ = 0.78;      // Maximum rudder angle (rad, ~45 degrees)
};

} // namespace asv_planning 