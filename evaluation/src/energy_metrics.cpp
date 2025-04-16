#include "../include/energy_metrics.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>

namespace asv_planning {

EnergyMetrics::EnergyMetrics(const std::string& vessel_params)
    : vessel_mass_(1000.0), // Default values
      propulsion_efficiency_(0.7) {
    
    // Initialize drag coefficients (default values)
    drag_coefficients_ = {0.4, 0.7, 0.05}; // X, Y, N directions
    
    // Load vessel parameters if available
    try {
        // Implementation for loading parameters from file would go here
        // This is a placeholder
        std::cout << "Loading vessel parameters from: " << vessel_params << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load vessel parameters: " << e.what() << std::endl;
        std::cerr << "Using default values instead." << std::endl;
    }
}

double EnergyMetrics::calculatePathEnergy(
    const std::vector<Eigen::Vector2d>& path,
    const std::vector<double>& velocities,
    const std::vector<std::map<std::string, double>>& environmental_data) {
    
    if (path.size() < 2 || path.size() != velocities.size() || 
        path.size() != environmental_data.size() + 1) {
        std::cerr << "Invalid input dimensions for path energy calculation." << std::endl;
        return 0.0;
    }
    
    double total_energy = 0.0;
    
    // Calculate energy for each path segment
    for (size_t i = 0; i < path.size() - 1; ++i) {
        // Extract environmental data
        Eigen::Vector2d current_vector(0.0, 0.0);
        double wave_height = 0.0;
        
        if (i < environmental_data.size()) {
            auto& env_data = environmental_data[i];
            
            if (env_data.count("current_x") && env_data.count("current_y")) {
                current_vector = Eigen::Vector2d(
                    env_data.at("current_x"),
                    env_data.at("current_y")
                );
            }
            
            if (env_data.count("wave_height")) {
                wave_height = env_data.at("wave_height");
            }
        }
        
        // Average velocity for the segment
        double velocity = velocities[i];
        
        // Calculate segment energy
        double segment_energy = calculateSegmentEnergy(
            path[i], path[i+1], velocity, current_vector, wave_height);
        
        total_energy += segment_energy;
    }
    
    return total_energy;
}

double EnergyMetrics::calculateSegmentEnergy(
    const Eigen::Vector2d& from,
    const Eigen::Vector2d& to,
    double velocity,
    const Eigen::Vector2d& current_vector,
    double wave_height) {
    
    // Calculate segment distance
    double distance = (to - from).norm();
    
    // Calculate segment heading
    double heading = std::atan2(to.y() - from.y(), to.x() - from.x());
    
    // Calculate power requirement
    double power_kW = calculatePowerRequirement(velocity, current_vector, heading);
    
    // Adjust for wave effects (simplified model)
    // In a real implementation, this would be more sophisticated
    double wave_factor = 1.0 + 0.2 * wave_height;
    power_kW *= wave_factor;
    
    // Calculate time to traverse segment (in hours)
    double time_hours = calculateSegmentTime(distance, velocity) / 3600.0;
    
    // Calculate energy (power * time)
    double energy_kWh = power_kW * time_hours;
    
    return energy_kWh;
}

double EnergyMetrics::computeEnergyEfficiency(
    const std::vector<Eigen::Vector2d>& path,
    double total_energy) {
    
    if (path.size() < 2 || total_energy <= 0.0) {
        return 0.0;
    }
    
    // Calculate total path distance
    double total_distance = calculatePathDistance(path);
    
    // Calculate efficiency (meters per kWh)
    return total_distance / total_energy;
}

double EnergyMetrics::comparePathEnergy(
    const std::vector<Eigen::Vector2d>& path1,
    const std::vector<double>& velocities1,
    const std::vector<Eigen::Vector2d>& path2,
    const std::vector<double>& velocities2,
    const std::vector<std::map<std::string, double>>& environmental_data) {
    
    // Calculate energy for both paths
    double energy1 = calculatePathEnergy(path1, velocities1, environmental_data);
    double energy2 = calculatePathEnergy(path2, velocities2, environmental_data);
    
    if (energy2 <= 0.0) {
        return 0.0; // Avoid division by zero
    }
    
    // Calculate percentage improvement
    double improvement = (energy2 - energy1) / energy2 * 100.0;
    
    return improvement;
}

void EnergyMetrics::recordEnergyData(
    const std::string& log_file,
    const std::vector<Eigen::Vector2d>& path,
    const std::vector<double>& energy_consumption,
    const std::string& timestamp) {
    
    std::ofstream file;
    file.open(log_file, std::ios_base::app); // Append mode
    
    if (!file.is_open()) {
        std::cerr << "Failed to open log file: " << log_file << std::endl;
        return;
    }
    
    // Write header if file is empty
    file.seekp(0, std::ios::end);
    if (file.tellp() == 0) {
        file << "Timestamp,Waypoint,X,Y,Energy,CumulativeEnergy" << std::endl;
    }
    
    // Write data rows
    double cumulative_energy = 0.0;
    for (size_t i = 0; i < path.size(); ++i) {
        double energy = (i < energy_consumption.size()) ? energy_consumption[i] : 0.0;
        cumulative_energy += energy;
        
        file << timestamp << ","
             << i << ","
             << path[i].x() << ","
             << path[i].y() << ","
             << energy << ","
             << cumulative_energy << std::endl;
    }
    
    file.close();
}

void EnergyMetrics::setVesselParameters(
    double mass,
    const std::vector<double>& drag_coefficients,
    double propulsion_efficiency) {
    
    vessel_mass_ = mass;
    
    if (drag_coefficients.size() >= 3) {
        drag_coefficients_ = drag_coefficients;
    }
    
    propulsion_efficiency_ = std::max(0.0, std::min(1.0, propulsion_efficiency));
}

double EnergyMetrics::calculatePowerRequirement(
    double velocity,
    const Eigen::Vector2d& current_vector,
    double heading) {
    
    // Convert heading to direction vector
    Eigen::Vector2d heading_vector(std::cos(heading), std::sin(heading));
    
    // Calculate relative velocity components
    double velocity_rel = velocity - current_vector.dot(heading_vector);
    
    // Ensure minimum velocity
    velocity_rel = std::max(0.1, velocity_rel);
    
    // Calculate power required to overcome drag
    // P = F * v where F = 0.5 * rho * Cd * A * v^2
    // Simplified: P = k * v^3
    // Where k incorporates water density, drag coefficient, frontal area
    
    double drag_coefficient = drag_coefficients_[0]; // Using X-direction coefficient
    const double water_density = 1025.0; // kg/m^3
    
    // Rough approximation of frontal area based on vessel mass
    double frontal_area = 0.01 * vessel_mass_; // m^2
    
    // Calculate power in Watts
    double power_W = 0.5 * water_density * drag_coefficient * frontal_area * 
                     std::pow(velocity_rel, 3);
    
    // Account for propulsion efficiency
    power_W /= propulsion_efficiency_;
    
    // Convert to kilowatts
    double power_kW = power_W / 1000.0;
    
    return power_kW;
}

double EnergyMetrics::calculateSegmentTime(double distance, double velocity) {
    if (velocity <= 0.0) {
        return 0.0; // Avoid division by zero
    }
    
    // Time in seconds
    return distance / velocity;
}

double EnergyMetrics::calculatePathDistance(const std::vector<Eigen::Vector2d>& path) {
    double total_distance = 0.0;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        total_distance += (path[i+1] - path[i]).norm();
    }
    
    return total_distance;
}

} // namespace asv_planning 