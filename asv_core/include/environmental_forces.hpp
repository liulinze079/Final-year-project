#pragma once

#include <Eigen/Core>
#include <vector>
#include <memory>
#include <functional>

namespace asv_planning {

/**
 * @class EnvironmentalForces
 * @brief Models environmental forces acting on an ASV
 * 
 * This class implements models for environmental forces including
 * currents, waves, and wind that affect ASV motion and energy consumption.
 */
class EnvironmentalForces {
public:
    /**
     * @brief Constructor
     */
    EnvironmentalForces();
    
    /**
     * @brief Destructor
     */
    ~EnvironmentalForces() = default;
    
    /**
     * @brief Calculate current-induced forces
     * @param position ASV position [x, y]
     * @param velocity ASV velocity vector [u, v, r]
     * @param heading ASV heading in radians
     * @return Current-induced force vector [X_c, Y_c, N_c]
     */
    Eigen::Vector3d getCurrentForces(
        const Eigen::Vector2d& position,
        const Eigen::Vector3d& velocity,
        double heading) const;
    
    /**
     * @brief Calculate wave-induced forces
     * @param position ASV position [x, y]
     * @param velocity ASV velocity vector [u, v, r]
     * @param heading ASV heading in radians
     * @return Wave-induced force vector [X_w, Y_w, N_w]
     */
    Eigen::Vector3d getWaveForces(
        const Eigen::Vector2d& position,
        const Eigen::Vector3d& velocity,
        double heading) const;
    
    /**
     * @brief Calculate wind-induced forces
     * @param velocity ASV velocity vector [u, v, r]
     * @param heading ASV heading in radians
     * @return Wind-induced force vector [X_wind, Y_wind, N_wind]
     */
    Eigen::Vector3d getWindForces(
        const Eigen::Vector3d& velocity,
        double heading) const;
    
    /**
     * @brief Set current model parameters
     * @param current_velocity Current velocity magnitude (m/s)
     * @param current_direction Current direction in radians
     */
    void setCurrentModel(double current_velocity, double current_direction);
    
    /**
     * @brief Set spatially varying current model
     * @param current_field_function Function that returns current vector [vx, vy] given position [x, y]
     */
    void setSpatialCurrentModel(
        std::function<Eigen::Vector2d(const Eigen::Vector2d&)> current_field_function);
    
    /**
     * @brief Set wave model parameters
     * @param wave_height Significant wave height (m)
     * @param wave_period Wave period (s)
     * @param wave_direction Primary wave direction in radians
     */
    void setWaveModel(double wave_height, double wave_period, double wave_direction);
    
    /**
     * @brief Set wind model parameters
     * @param wind_speed Wind speed (m/s)
     * @param wind_direction Wind direction in radians
     */
    void setWindModel(double wind_speed, double wind_direction);
    
    /**
     * @brief Get the total environmental forces
     * @param position ASV position [x, y]
     * @param velocity ASV velocity vector [u, v, r]
     * @param heading ASV heading in radians
     * @return Total environmental force vector [X_env, Y_env, N_env]
     */
    Eigen::Vector3d getTotalEnvironmentalForces(
        const Eigen::Vector2d& position,
        const Eigen::Vector3d& velocity,
        double heading) const;
    
    /**
     * @brief Calculate energy cost due to environmental forces
     * @param from_position Start position
     * @param to_position End position
     * @param velocity ASV velocity
     * @param heading ASV heading
     * @return Energy cost factor (1.0 = neutral, >1.0 = increased cost, <1.0 = decreased cost)
     */
    double calculateEnvironmentalEnergyCost(
        const Eigen::Vector2d& from_position,
        const Eigen::Vector2d& to_position,
        double velocity,
        double heading) const;

private:
    // Current model parameters
    double current_velocity_;
    double current_direction_;
    bool use_spatial_current_model_;
    std::function<Eigen::Vector2d(const Eigen::Vector2d&)> current_field_function_;
    
    // Wave model parameters
    double wave_height_;
    double wave_period_;
    double wave_direction_;
    
    // Wind model parameters
    double wind_speed_;
    double wind_direction_;
    
    // Vessel geometric parameters (for force calculations)
    double length_;
    double width_;
    double draft_;
    
    // Helper methods for force calculations
    Eigen::Vector2d getCurrentVector(const Eigen::Vector2d& position) const;
    double calculateRelativeAngle(double direction1, double direction2) const;
};

} // namespace asv_planning 