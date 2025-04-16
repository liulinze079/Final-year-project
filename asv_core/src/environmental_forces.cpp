#include "../include/environmental_forces.hpp"
#include <cmath>

namespace asv_planning {

EnvironmentalForces::EnvironmentalForces()
    : current_velocity_(0.0), current_direction_(0.0),
      use_spatial_current_model_(false),
      wave_height_(0.0), wave_period_(0.0), wave_direction_(0.0),
      wind_speed_(0.0), wind_direction_(0.0),
      length_(10.0), width_(3.0), draft_(1.5) {
    // Initialize with default values
}

Eigen::Vector3d EnvironmentalForces::getCurrentForces(
    const Eigen::Vector2d& position,
    const Eigen::Vector3d& velocity,
    double heading) const {
    
    // Get current vector at the specified position
    Eigen::Vector2d current = getCurrentVector(position);
    
    // Calculate relative velocity between ASV and current
    double u_rel = velocity(0) - current(0);
    double v_rel = velocity(1) - current(1);
    
    // Simplified current force model
    // Forces are proportional to the square of relative velocity
    const double water_density = 1025.0;  // kg/m^3
    const double drag_coef_x = 0.5;       // Longitudinal drag coefficient
    const double drag_coef_y = 0.8;       // Lateral drag coefficient
    
    // Calculate projected areas
    double A_x = width_ * draft_;         // Frontal area
    double A_y = length_ * draft_;        // Lateral area
    
    // Calculate forces
    double X_current = -0.5 * water_density * drag_coef_x * A_x * u_rel * std::abs(u_rel);
    double Y_current = -0.5 * water_density * drag_coef_y * A_y * v_rel * std::abs(v_rel);
    
    // Calculate turning moment (simplified)
    // Assume center of pressure is at L/4 from bow for lateral force
    double N_current = -Y_current * (length_ / 4.0);
    
    return Eigen::Vector3d(X_current, Y_current, N_current);
}

Eigen::Vector3d EnvironmentalForces::getWaveForces(
    const Eigen::Vector2d& position,
    const Eigen::Vector3d& velocity,
    double heading) const {
    
    // Only compute wave forces if waves are present
    if (wave_height_ < 0.1) {
        return Eigen::Vector3d::Zero();
    }
    
    // Calculate relative angle between wave direction and vessel heading
    double relative_angle = calculateRelativeAngle(wave_direction_, heading);
    
    // Simplified wave force model
    // Forces depend on wave height, period, and relative angle
    const double water_density = 1025.0;  // kg/m^3
    
    // Calculate wave forces (simplified)
    // For more accurate models, complex wave-structure interaction should be considered
    double wave_energy = 0.5 * water_density * 9.81 * wave_height_ * wave_height_;
    
    // Force coefficients based on relative angle
    double C_x = 0.4 * std::cos(relative_angle);
    double C_y = 0.8 * std::sin(relative_angle);
    
    // Projected areas
    double A_x = width_ * draft_;
    double A_y = length_ * draft_;
    
    // Calculate forces
    double X_wave = C_x * wave_energy * A_x;
    double Y_wave = C_y * wave_energy * A_y;
    
    // Calculate turning moment (simplified)
    // Assume center of pressure varies with relative angle
    double lever_arm = (length_ / 4.0) * std::sin(relative_angle);
    double N_wave = Y_wave * lever_arm;
    
    return Eigen::Vector3d(X_wave, Y_wave, N_wave);
}

Eigen::Vector3d EnvironmentalForces::getWindForces(
    const Eigen::Vector3d& velocity,
    double heading) const {
    
    // Only compute wind forces if wind is present
    if (wind_speed_ < 0.5) {
        return Eigen::Vector3d::Zero();
    }
    
    // Calculate relative angle between wind direction and vessel heading
    double relative_angle = calculateRelativeAngle(wind_direction_, heading);
    
    // Simplified wind force model
    const double air_density = 1.225;  // kg/m^3
    
    // Estimate above-water profile (simplified)
    double height_above_water = 2.0;  // m
    double A_x_wind = width_ * height_above_water;    // Frontal area
    double A_y_wind = length_ * height_above_water;   // Lateral area
    
    // Wind coefficients based on relative angle
    // These should be determined from wind tunnel tests or CFD for accurate models
    double C_x_wind = 0.5 * std::cos(relative_angle);
    double C_y_wind = 0.9 * std::sin(relative_angle);
    
    // Calculate wind forces
    double X_wind = C_x_wind * 0.5 * air_density * wind_speed_ * wind_speed_ * A_x_wind;
    double Y_wind = C_y_wind * 0.5 * air_density * wind_speed_ * wind_speed_ * A_y_wind;
    
    // Calculate turning moment
    double lever_arm_wind = (length_ / 4.0) * std::sin(relative_angle);
    double N_wind = Y_wind * lever_arm_wind;
    
    return Eigen::Vector3d(X_wind, Y_wind, N_wind);
}

void EnvironmentalForces::setCurrentModel(double current_velocity, double current_direction) {
    current_velocity_ = current_velocity;
    current_direction_ = current_direction;
    use_spatial_current_model_ = false;
}

void EnvironmentalForces::setSpatialCurrentModel(
    std::function<Eigen::Vector2d(const Eigen::Vector2d&)> current_field_function) {
    
    current_field_function_ = current_field_function;
    use_spatial_current_model_ = true;
}

void EnvironmentalForces::setWaveModel(double wave_height, double wave_period, double wave_direction) {
    wave_height_ = wave_height;
    wave_period_ = wave_period;
    wave_direction_ = wave_direction;
}

void EnvironmentalForces::setWindModel(double wind_speed, double wind_direction) {
    wind_speed_ = wind_speed;
    wind_direction_ = wind_direction;
}

Eigen::Vector3d EnvironmentalForces::getTotalEnvironmentalForces(
    const Eigen::Vector2d& position,
    const Eigen::Vector3d& velocity,
    double heading) const {
    
    // Combine all environmental forces
    Eigen::Vector3d current_forces = getCurrentForces(position, velocity, heading);
    Eigen::Vector3d wave_forces = getWaveForces(position, velocity, heading);
    Eigen::Vector3d wind_forces = getWindForces(velocity, heading);
    
    return current_forces + wave_forces + wind_forces;
}

double EnvironmentalForces::calculateEnvironmentalEnergyCost(
    const Eigen::Vector2d& from_position,
    const Eigen::Vector2d& to_position,
    double velocity,
    double heading) const {
    
    // Calculate midpoint of the path segment
    Eigen::Vector2d midpoint = (from_position + to_position) / 2.0;
    
    // Get current at midpoint
    Eigen::Vector2d current = getCurrentVector(midpoint);
    
    // Calculate angle between current and heading
    double current_direction = std::atan2(current.y(), current.x());
    double relative_angle = calculateRelativeAngle(current_direction, heading);
    
    // Calculate current magnitude
    double current_magnitude = current.norm();
    
    // Energy cost factor based on current
    // - Moving against current: higher energy cost
    // - Moving with current: lower energy cost
    double current_cost_factor = 1.0 + 0.5 * (current_magnitude / velocity) * std::cos(relative_angle);
    
    // Factor for wave impact on energy
    double wave_cost_factor = 1.0;
    if (wave_height_ > 0.1) {
        double wave_rel_angle = calculateRelativeAngle(wave_direction_, heading);
        // Waves from side require more energy to maintain course
        wave_cost_factor += 0.2 * wave_height_ * std::sin(wave_rel_angle) * std::sin(wave_rel_angle);
    }
    
    // Combine factors
    return current_cost_factor * wave_cost_factor;
}

Eigen::Vector2d EnvironmentalForces::getCurrentVector(const Eigen::Vector2d& position) const {
    if (use_spatial_current_model_ && current_field_function_) {
        return current_field_function_(position);
    } else {
        // Uniform current field
        return Eigen::Vector2d(
            current_velocity_ * std::cos(current_direction_),
            current_velocity_ * std::sin(current_direction_)
        );
    }
}

double EnvironmentalForces::calculateRelativeAngle(double direction1, double direction2) const {
    double relative_angle = direction1 - direction2;
    
    // Normalize to [-pi, pi]
    while (relative_angle > M_PI) relative_angle -= 2.0 * M_PI;
    while (relative_angle < -M_PI) relative_angle += 2.0 * M_PI;
    
    return relative_angle;
}

} // namespace asv_planning 