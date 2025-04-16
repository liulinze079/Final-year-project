#include "../include/path_quality.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <sstream>

namespace asv_planning {

PathQuality::PathQuality(const std::string& config_file)
    : smoothness_weight_(1.0), safety_weight_(1.5), shipping_lane_weight_(0.8) {
    
    // Load configuration if a file is provided
    if (!config_file.empty()) {
        // Implementation for loading configuration would go here
        std::cout << "Loading configuration from: " << config_file << std::endl;
    }
}

double PathQuality::calculateSmoothness(const std::vector<Eigen::Vector2d>& path) {
    if (path.size() < 3) {
        return 1.0; // Not enough points to calculate smoothness
    }
    
    // Calculate smoothness based on heading changes
    std::vector<double> heading_changes = calculateHeadingChanges(path);
    
    // Calculate the average squared heading change
    double sum_squared_changes = 0.0;
    for (const double& change : heading_changes) {
        sum_squared_changes += change * change;
    }
    
    double avg_squared_change = sum_squared_changes / heading_changes.size();
    
    // Convert to a smoothness score (0 to 1, where 1 is perfectly smooth)
    // Using an exponential decay function: e^(-k*x)
    const double k = 5.0; // Scaling factor
    double smoothness = std::exp(-k * avg_squared_change);
    
    return smoothness;
}

double PathQuality::calculateSafetyScore(
    const std::vector<Eigen::Vector2d>& path,
    const std::shared_ptr<void>& obstacle_map) {
    
    if (path.empty() || !obstacle_map) {
        return 0.0;
    }
    
    // Calculate minimum distance to obstacles for each point
    std::vector<double> distances;
    distances.reserve(path.size());
    
    for (const auto& point : path) {
        double distance = calculateMinDistanceToObstacles(point, obstacle_map);
        distances.push_back(distance);
    }
    
    // Calculate the minimum distance along the path
    double min_distance = *std::min_element(distances.begin(), distances.end());
    
    // Calculate the average distance
    double avg_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    
    // Calculate safety score based on minimum and average distances
    // Weighted combination of min and avg distances, normalized to 0-1
    
    // Parameters
    const double critical_distance = 5.0;     // Distance below which safety is critical
    const double safe_distance = 20.0;        // Distance above which safety is maximized
    const double min_weight = 0.7;            // Weight for minimum distance
    const double avg_weight = 0.3;            // Weight for average distance
    
    // Normalize distances to 0-1 range
    double min_safety = std::min(1.0, std::max(0.0, (min_distance - critical_distance) / 
                                                    (safe_distance - critical_distance)));
    
    double avg_safety = std::min(1.0, std::max(0.0, (avg_distance - critical_distance) / 
                                                    (safe_distance - critical_distance)));
    
    // Weighted combination
    double safety_score = min_weight * min_safety + avg_weight * avg_safety;
    
    return safety_score;
}

bool PathQuality::isPathFeasible(
    const std::vector<Eigen::Vector2d>& path,
    const std::vector<double>& velocities,
    double turning_radius) {
    
    if (path.size() < 3 || path.size() != velocities.size() + 1) {
        return false; // Not enough points or invalid velocities
    }
    
    // Calculate curvature at each point
    std::vector<double> curvatures = calculateCurvature(path);
    
    // Check if any curvature exceeds the maximum allowed by turning radius
    const double max_curvature = 1.0 / turning_radius;
    
    for (size_t i = 0; i < curvatures.size(); ++i) {
        if (curvatures[i] > max_curvature) {
            // Current curvature exceeds maximum allowed by turning radius
            return false;
        }
        
        // Additional check: velocity-dependent curvature constraint
        // Higher speeds require larger turning radii
        if (i < velocities.size()) {
            double speed_factor = 1.0 + 0.1 * velocities[i]; // Simple linear model
            double velocity_adjusted_max_curvature = max_curvature / speed_factor;
            
            if (curvatures[i] > velocity_adjusted_max_curvature) {
                return false;
            }
        }
    }
    
    return true;
}

std::vector<double> PathQuality::calculateCurvature(const std::vector<Eigen::Vector2d>& path) {
    if (path.size() < 3) {
        return std::vector<double>(path.size(), 0.0); // Not enough points
    }
    
    std::vector<double> curvatures(path.size(), 0.0);
    
    // Curvature is not well-defined for the first and last points
    // so we'll only calculate it for the interior points
    for (size_t i = 1; i < path.size() - 1; ++i) {
        // Use finite differences to approximate curvature
        Eigen::Vector2d p_prev = path[i - 1];
        Eigen::Vector2d p_curr = path[i];
        Eigen::Vector2d p_next = path[i + 1];
        
        // Calculate tangent vectors
        Eigen::Vector2d t1 = (p_curr - p_prev).normalized();
        Eigen::Vector2d t2 = (p_next - p_curr).normalized();
        
        // Calculate change in angle
        double dot_product = t1.dot(t2);
        // Clamp to prevent numerical issues
        dot_product = std::max(-1.0, std::min(1.0, dot_product));
        double angle_change = std::acos(dot_product);
        
        // Calculate distance between points
        double segment_length = ((p_next - p_prev) / 2.0).norm();
        
        // Curvature = rate of change of tangent angle with respect to arc length
        if (segment_length > 1e-6) {
            curvatures[i] = angle_change / segment_length;
        } else {
            curvatures[i] = 0.0;
        }
    }
    
    return curvatures;
}

std::vector<double> PathQuality::calculateHeadingChanges(const std::vector<Eigen::Vector2d>& path) {
    if (path.size() < 3) {
        return std::vector<double>(); // Not enough points
    }
    
    std::vector<double> heading_changes;
    heading_changes.reserve(path.size() - 2);
    
    for (size_t i = 0; i < path.size() - 2; ++i) {
        Eigen::Vector2d segment1 = path[i + 1] - path[i];
        Eigen::Vector2d segment2 = path[i + 2] - path[i + 1];
        
        double heading1 = std::atan2(segment1.y(), segment1.x());
        double heading2 = std::atan2(segment2.y(), segment2.x());
        
        // Calculate heading change and normalize to [-pi, pi]
        double change = heading2 - heading1;
        while (change > M_PI) change -= 2.0 * M_PI;
        while (change < -M_PI) change += 2.0 * M_PI;
        
        heading_changes.push_back(std::abs(change));
    }
    
    return heading_changes;
}

double PathQuality::calculateShippingLaneAdherence(
    const std::vector<Eigen::Vector2d>& path,
    const std::vector<std::vector<Eigen::Vector2d>>& shipping_lanes) {
    
    if (path.empty() || shipping_lanes.empty()) {
        return 0.0;
    }
    
    // Count points that are within any shipping lane
    int points_in_lanes = 0;
    
    for (const auto& point : path) {
        bool in_any_lane = false;
        
        for (const auto& lane : shipping_lanes) {
            if (isPointInPolygon(point, lane)) {
                in_any_lane = true;
                break;
            }
        }
        
        if (in_any_lane) {
            points_in_lanes++;
        }
    }
    
    // Calculate percentage of path within shipping lanes
    return static_cast<double>(points_in_lanes) / path.size();
}

void PathQuality::generateQualityReport(
    const std::vector<Eigen::Vector2d>& path,
    const std::vector<double>& velocities,
    const std::shared_ptr<void>& obstacle_map,
    const std::vector<std::vector<Eigen::Vector2d>>& shipping_lanes,
    const std::string& output_file) {
    
    // Calculate quality metrics
    double smoothness = calculateSmoothness(path);
    double safety = calculateSafetyScore(path, obstacle_map);
    bool feasible = isPathFeasible(path, velocities, 15.0); // Assume 15m turning radius
    double lane_adherence = calculateShippingLaneAdherence(path, shipping_lanes);
    
    // Calculate overall quality score
    double overall_score = 
        smoothness_weight_ * smoothness +
        safety_weight_ * safety +
        shipping_lane_weight_ * lane_adherence;
    
    // Normalize to 0-1 range
    double max_possible_score = smoothness_weight_ + safety_weight_ + shipping_lane_weight_;
    overall_score /= max_possible_score;
    
    // Format the report
    std::ostringstream report;
    report << "PATH QUALITY REPORT\n";
    report << "==================\n\n";
    report << "Path length: " << path.size() << " points\n";
    report << "Path feasible: " << (feasible ? "Yes" : "No") << "\n\n";
    report << "Quality Metrics:\n";
    report << "  Smoothness:       " << smoothness << " (weight: " << smoothness_weight_ << ")\n";
    report << "  Safety:           " << safety << " (weight: " << safety_weight_ << ")\n";
    report << "  Shipping Lanes:   " << lane_adherence * 100.0 << "% (weight: " << shipping_lane_weight_ << ")\n";
    report << "\nOVERALL QUALITY:   " << overall_score << "\n";
    
    // Output the report
    if (output_file.empty()) {
        // Print to console
        std::cout << report.str() << std::endl;
    } else {
        // Write to file
        std::ofstream file(output_file);
        if (file.is_open()) {
            file << report.str();
            file.close();
            std::cout << "Quality report written to " << output_file << std::endl;
        } else {
            std::cerr << "Failed to open output file: " << output_file << std::endl;
            std::cout << report.str() << std::endl;
        }
    }
}

double PathQuality::comparePaths(
    const std::vector<Eigen::Vector2d>& path1,
    const std::vector<Eigen::Vector2d>& path2,
    const std::shared_ptr<void>& obstacle_map,
    const std::vector<double>& weights) {
    
    // Ensure we have valid weights
    double w_smoothness = (weights.size() > 0) ? weights[0] : 1.0;
    double w_safety = (weights.size() > 1) ? weights[1] : 1.0;
    double w_length = (weights.size() > 2) ? weights[2] : 1.0;
    
    // Calculate metrics for both paths
    double smoothness1 = calculateSmoothness(path1);
    double smoothness2 = calculateSmoothness(path2);
    
    double safety1 = calculateSafetyScore(path1, obstacle_map);
    double safety2 = calculateSafetyScore(path2, obstacle_map);
    
    // Calculate path lengths
    double length1 = 0.0;
    for (size_t i = 0; i < path1.size() - 1; ++i) {
        length1 += (path1[i+1] - path1[i]).norm();
    }
    
    double length2 = 0.0;
    for (size_t i = 0; i < path2.size() - 1; ++i) {
        length2 += (path2[i+1] - path2[i]).norm();
    }
    
    // Normalize length scores (inverse, since shorter is better)
    double max_length = std::max(length1, length2);
    double length_score1 = 1.0 - (length1 / max_length);
    double length_score2 = 1.0 - (length2 / max_length);
    
    // Calculate weighted scores
    double score1 = w_smoothness * smoothness1 + w_safety * safety1 + w_length * length_score1;
    double score2 = w_smoothness * smoothness2 + w_safety * safety2 + w_length * length_score2;
    
    // Return difference (positive if path1 is better)
    return score1 - score2;
}

double PathQuality::calculateMinDistanceToObstacles(
    const Eigen::Vector2d& point,
    const std::shared_ptr<void>& obstacle_map) {
    
    // This is a placeholder implementation since we don't have
    // access to the actual obstacle map structure
    // In a real implementation, this would query the obstacle map
    
    // For demonstration, return a random value between 5 and 30
    return 5.0 + (std::rand() % 26);
}

bool PathQuality::isPointInPolygon(
    const Eigen::Vector2d& point,
    const std::vector<Eigen::Vector2d>& polygon) {
    
    if (polygon.size() < 3) {
        return false;
    }
    
    // Implementation of ray casting algorithm
    bool inside = false;
    
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        bool intersect = ((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) &&
                         (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) /
                                        (polygon[j].y() - polygon[i].y()) + polygon[i].x());
        
        if (intersect) {
            inside = !inside;
        }
    }
    
    return inside;
}

} // namespace asv_planning 