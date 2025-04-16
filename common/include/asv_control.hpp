#pragma once

#include <Eigen/Dense>

namespace asv_planning {

/**
 * @brief Control inputs for the ASV
 * 
 * This struct represents the control inputs for the ASV, including
 * thrust and rudder angle.
 */
struct ASVControl {
    /**
     * @brief Default constructor
     */
    ASVControl()
        : thrust(0.0), rudder(0.0) {}
    
    /**
     * @brief Constructor with parameters
     * 
     * @param _thrust Thrust force (Newtons)
     * @param _rudder Rudder angle (radians)
     */
    ASVControl(double _thrust, double _rudder)
        : thrust(_thrust), rudder(_rudder) {}
    
    /**
     * @brief Convert to an Eigen vector
     * 
     * @return Control vector [thrust, rudder]
     */
    Eigen::Vector2d toVector() const {
        Eigen::Vector2d vec;
        vec << thrust, rudder;
        return vec;
    }
    
    /**
     * @brief Create from an Eigen vector
     * 
     * @param vec Control vector [thrust, rudder]
     * @return ASVControl object
     */
    static ASVControl fromVector(const Eigen::Vector2d& vec) {
        return ASVControl(vec(0), vec(1));
    }
    
    /**
     * @brief Check if control is within valid bounds
     * 
     * @param min_thrust Minimum thrust (Newtons)
     * @param max_thrust Maximum thrust (Newtons)
     * @param min_rudder Minimum rudder angle (radians)
     * @param max_rudder Maximum rudder angle (radians)
     * @return True if control is valid, false otherwise
     */
    bool isValid(double min_thrust, double max_thrust,
               double min_rudder, double max_rudder) const {
        return thrust >= min_thrust && thrust <= max_thrust &&
               rudder >= min_rudder && rudder <= max_rudder;
    }
    
    /**
     * @brief Clamp control values to valid ranges
     * 
     * @param min_thrust Minimum thrust (Newtons)
     * @param max_thrust Maximum thrust (Newtons)
     * @param min_rudder Minimum rudder angle (radians)
     * @param max_rudder Maximum rudder angle (radians)
     * @return Clamped ASVControl
     */
    ASVControl clamped(double min_thrust, double max_thrust,
                     double min_rudder, double max_rudder) const {
        return ASVControl(
            std::max(min_thrust, std::min(thrust, max_thrust)),
            std::max(min_rudder, std::min(rudder, max_rudder))
        );
    }
    
    /**
     * @brief Calculate the energy consumption for this control input
     * 
     * Simple model where energy is proportional to thrust magnitude
     * and duration.
     * 
     * @param duration Duration of control application (seconds)
     * @return Energy consumption (Joules)
     */
    double calculateEnergy(double duration) const {
        // Simple model: energy = thrust * efficiency_factor * duration
        double efficiency_factor = 1.0;
        return std::abs(thrust) * efficiency_factor * duration;
    }
    
    /**
     * @brief Interpolate between two control inputs
     * 
     * @param other Other control input
     * @param alpha Interpolation factor (0.0 to 1.0)
     * @return Interpolated control
     */
    ASVControl interpolate(const ASVControl& other, double alpha) const {
        return ASVControl(
            thrust * (1.0 - alpha) + other.thrust * alpha,
            rudder * (1.0 - alpha) + other.rudder * alpha
        );
    }
    
    double thrust;  ///< Thrust force (Newtons)
    double rudder;  ///< Rudder angle (radians)
};

} // namespace asv_planning 