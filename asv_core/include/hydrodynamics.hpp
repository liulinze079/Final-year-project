#pragma once

#include <Eigen/Core>
#include <vector>
#include <memory>

namespace asv_planning {

/**
 * @class Hydrodynamics
 * @brief Models hydrodynamic forces acting on an ASV
 * 
 * This class implements models for hydrodynamic forces including added mass,
 * damping, and other water-related forces that affect ASV motion.
 */
class Hydrodynamics {
public:
    /**
     * @brief Constructor
     * @param vessel_length Length of the vessel in meters
     * @param vessel_width Width of the vessel in meters
     * @param vessel_mass Mass of the vessel in kg
     */
    Hydrodynamics(double vessel_length, double vessel_width, double vessel_mass);
    
    /**
     * @brief Destructor
     */
    ~Hydrodynamics() = default;
    
    /**
     * @brief Calculate added mass matrix
     * @param velocity Current velocity vector [u, v, r] (surge, sway, yaw rate)
     * @return Added mass matrix (3x3)
     */
    Eigen::Matrix3d getAddedMassMatrix(const Eigen::Vector3d& velocity) const;
    
    /**
     * @brief Calculate damping forces
     * @param velocity Current velocity vector [u, v, r] (surge, sway, yaw rate)
     * @return Damping force vector [X_d, Y_d, N_d]
     */
    Eigen::Vector3d getDampingForces(const Eigen::Vector3d& velocity) const;
    
    /**
     * @brief Calculate restoring forces (gravity, buoyancy)
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @return Restoring force vector [X_r, Y_r, N_r]
     */
    Eigen::Vector3d getRestoringForces(double roll, double pitch) const;
    
    /**
     * @brief Calculate hydrodynamic forces based on hull shape and motion
     * @param velocity Current velocity vector [u, v, r]
     * @param acceleration Current acceleration vector [u_dot, v_dot, r_dot]
     * @return Total hydrodynamic force vector [X_h, Y_h, N_h]
     */
    Eigen::Vector3d getHydrodynamicForces(
        const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& acceleration) const;
    
    /**
     * @brief Set vessel parameters
     * @param vessel_length Length of the vessel in meters
     * @param vessel_width Width of the vessel in meters
     * @param vessel_mass Mass of the vessel in kg
     */
    void setVesselParameters(double vessel_length, double vessel_width, double vessel_mass);
    
    /**
     * @brief Set hydrodynamic coefficients directly
     * @param added_mass Added mass coefficient matrix (3x3)
     * @param linear_damping Linear damping coefficient matrix (3x3)
     * @param quadratic_damping Quadratic damping coefficient matrix (3x3)
     */
    void setHydrodynamicCoefficients(
        const Eigen::Matrix3d& added_mass,
        const Eigen::Matrix3d& linear_damping,
        const Eigen::Matrix3d& quadratic_damping);

private:
    // Vessel parameters
    double length_;
    double width_;
    double mass_;
    
    // Hydrodynamic coefficients
    Eigen::Matrix3d added_mass_matrix_;
    Eigen::Matrix3d linear_damping_matrix_;
    Eigen::Matrix3d quadratic_damping_matrix_;
    
    // Calculate default hydrodynamic coefficients based on vessel geometry
    void calculateDefaultCoefficients();
};

} // namespace asv_planning 