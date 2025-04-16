#pragma once

#include <vector>
#include <Eigen/Core>
#include <string>
#include <memory>

namespace asv_planning {

/**
 * @class EnergyMetrics
 * @brief Evaluates and records energy consumption metrics for ASV operations
 * 
 * This class provides functionality to measure, record, and analyze the energy
 * consumption of an ASV during mission execution. It supports different energy
 * models and can compute metrics like total energy used, energy efficiency,
 * and comparisons between different planning approaches.
 */
class EnergyMetrics {
public:
    /**
     * @brief Constructor
     * @param vessel_params File path to vessel parameters
     */
    EnergyMetrics(const std::string& vessel_params);
    
    /**
     * @brief Destructor
     */
    ~EnergyMetrics() = default;
    
    /**
     * @brief Calculate energy consumption for a given path
     * @param path Vector of waypoints in the path
     * @param velocities Velocities at each waypoint
     * @param environmental_data Environmental data along the path
     * @return Total energy consumption in kWh
     */
    double calculatePathEnergy(
        const std::vector<Eigen::Vector2d>& path,
        const std::vector<double>& velocities,
        const std::vector<std::map<std::string, double>>& environmental_data);
    
    /**
     * @brief Calculate energy consumption for a given segment
     * @param from Start point
     * @param to End point
     * @param velocity Velocity during the segment
     * @param current_vector Current vector during the segment
     * @param wave_height Wave height during the segment
     * @return Energy consumption for the segment in kWh
     */
    double calculateSegmentEnergy(
        const Eigen::Vector2d& from,
        const Eigen::Vector2d& to,
        double velocity,
        const Eigen::Vector2d& current_vector,
        double wave_height);
    
    /**
     * @brief Compute energy efficiency (distance traveled per unit energy)
     * @param path Vector of waypoints in the path
     * @param total_energy Total energy consumption
     * @return Energy efficiency in meters per kWh
     */
    double computeEnergyEfficiency(
        const std::vector<Eigen::Vector2d>& path,
        double total_energy);
    
    /**
     * @brief Compare energy usage between two different paths
     * @param path1 First path to compare
     * @param velocities1 Velocities for first path
     * @param path2 Second path to compare
     * @param velocities2 Velocities for second path
     * @param environmental_data Environmental data along both paths
     * @return Percentage improvement of path1 over path2
     */
    double comparePathEnergy(
        const std::vector<Eigen::Vector2d>& path1,
        const std::vector<double>& velocities1,
        const std::vector<Eigen::Vector2d>& path2,
        const std::vector<double>& velocities2,
        const std::vector<std::map<std::string, double>>& environmental_data);
    
    /**
     * @brief Record energy consumption data to a log file
     * @param log_file Path to the log file
     * @param path Path waypoints
     * @param energy_consumption Energy consumption values
     * @param timestamp Timestamp for the log entry
     */
    void recordEnergyData(
        const std::string& log_file,
        const std::vector<Eigen::Vector2d>& path,
        const std::vector<double>& energy_consumption,
        const std::string& timestamp);
    
    /**
     * @brief Set vessel-specific parameters for energy calculations
     * @param mass Vessel mass in kg
     * @param drag_coefficients Drag coefficients for different directions
     * @param propulsion_efficiency Propulsion system efficiency (0-1)
     */
    void setVesselParameters(
        double mass,
        const std::vector<double>& drag_coefficients,
        double propulsion_efficiency);

private:
    // Vessel parameters
    double vessel_mass_;
    std::vector<double> drag_coefficients_;
    double propulsion_efficiency_;
    
    // Calculate power requirements based on vessel dynamics
    double calculatePowerRequirement(
        double velocity,
        const Eigen::Vector2d& current_vector,
        double heading);
    
    // Calculate time to traverse a segment
    double calculateSegmentTime(
        double distance,
        double velocity);
    
    // Calculate total distance of a path
    double calculatePathDistance(
        const std::vector<Eigen::Vector2d>& path);
};

} // namespace asv_planning 