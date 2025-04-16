#include "sampling_strategies.hpp"
#include "../../../environment_representation/include/multi_resolution_grid.hpp"
#include <cmath>
#include <random>
#include <algorithm>

namespace asv_planning {

// Base SamplingStrategy implementation

SamplingStrategy::SamplingStrategy()
    : rng_(std::random_device()()) {
}

void SamplingStrategy::setMap(std::shared_ptr<MultiResolutionGrid> map) {
    map_ = map;
}

Eigen::Vector2d SamplingStrategy::sampleUniform(const Eigen::Vector4d& bounds) {
    std::uniform_real_distribution<double> x_dist(bounds[0], bounds[2]);
    std::uniform_real_distribution<double> y_dist(bounds[1], bounds[3]);
    
    return Eigen::Vector2d(x_dist(rng_), y_dist(rng_));
}

// UniformSampling implementation

UniformSampling::UniformSampling()
    : SamplingStrategy() {
}

Eigen::Vector2d UniformSampling::samplePosition(
    const Eigen::Vector2d& current_position,
    double heading,
    const Eigen::Vector4d& bounds,
    const Eigen::Vector2d& target_bias,
    double bias_probability) {
    
    // Sample from uniform distribution with bias toward target
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    if (dist(rng_) < bias_probability) {
        return target_bias;
    }
    
    return sampleUniform(bounds);
}

// GaussianSampling implementation

GaussianSampling::GaussianSampling(double std_dev_scale)
    : SamplingStrategy(), std_dev_scale_(std_dev_scale) {
}

Eigen::Vector2d GaussianSampling::samplePosition(
    const Eigen::Vector2d& current_position,
    double heading,
    const Eigen::Vector4d& bounds,
    const Eigen::Vector2d& target_bias,
    double bias_probability) {
    
    // With some probability, sample directly toward target
    std::uniform_real_distribution<double> bias_dist(0.0, 1.0);
    if (bias_dist(rng_) < bias_probability) {
        return target_bias;
    }
    
    // Calculate distance to target for scaling standard deviation
    double distance_to_target = (target_bias - current_position).norm();
    
    // Standard deviation proportional to distance (with some minimum)
    double std_dev = std::max(1.0, distance_to_target * std_dev_scale_);
    
    // Create Gaussian distributions centered on current position
    std::normal_distribution<double> x_dist(current_position.x(), std_dev);
    std::normal_distribution<double> y_dist(current_position.y(), std_dev);
    
    // Sample until we get a point within bounds
    Eigen::Vector2d sample;
    int max_attempts = 10;
    for (int i = 0; i < max_attempts; ++i) {
        double x = x_dist(rng_);
        double y = y_dist(rng_);
        
        // Check if within bounds
        if (x >= bounds[0] && x <= bounds[2] && y >= bounds[1] && y <= bounds[3]) {
            return Eigen::Vector2d(x, y);
        }
    }
    
    // If failed to sample within bounds, fall back to uniform
    return sampleUniform(bounds);
}

// AdaptiveSampling implementation

AdaptiveSampling::AdaptiveSampling()
    : SamplingStrategy() {
}

void AdaptiveSampling::setMap(std::shared_ptr<MultiResolutionGrid> map) {
    SamplingStrategy::setMap(map);
}

Eigen::Vector2d AdaptiveSampling::samplePosition(
    const Eigen::Vector2d& current_position,
    double heading,
    const Eigen::Vector4d& bounds,
    const Eigen::Vector2d& target_bias,
    double bias_probability) {
    
    // With some probability, sample directly toward target
    std::uniform_real_distribution<double> bias_dist(0.0, 1.0);
    if (bias_dist(rng_) < bias_probability) {
        return target_bias;
    }
    
    // If no map available, fall back to uniform sampling
    if (!map_) {
        return sampleUniform(bounds);
    }
    
    // Try to adaptively sample based on obstacles
    int max_attempts = 20;
    std::vector<Eigen::Vector2d> candidates;
    std::vector<double> weights;
    
    // Generate candidate samples
    for (int i = 0; i < max_attempts; ++i) {
        Eigen::Vector2d candidate = sampleUniform(bounds);
        
        // Skip if occupied
        if (map_->isOccupied(candidate.x(), candidate.y())) {
            continue;
        }
        
        double weight = calculateSamplingWeight(candidate);
        
        candidates.push_back(candidate);
        weights.push_back(weight);
    }
    
    // If no valid candidates, fall back to uniform
    if (candidates.empty()) {
        return sampleUniform(bounds);
    }
    
    // Select based on weights
    std::discrete_distribution<int> weighted_dist(weights.begin(), weights.end());
    int selected = weighted_dist(rng_);
    
    return candidates[selected];
}

double AdaptiveSampling::calculateSamplingWeight(const Eigen::Vector2d& position) const {
    if (!map_) {
        return 1.0;  // No map, uniform weight
    }
    
    // Get distance to nearest obstacle
    double obstacle_distance = map_->getDistanceToNearestObstacle(position);
    
    // Calculate weight based on obstacle distance
    // Higher weight for positions that are not too close or too far from obstacles
    double optimal_distance = 3.0;  // Target distance from obstacles
    double distance_factor = std::exp(-std::pow(obstacle_distance - optimal_distance, 2) / (2.0 * 3.0));
    
    // Consider force field magnitude (higher weight in areas with strong currents)
    double force_magnitude = map_->getForceFieldMagnitude(position);
    double force_factor = 1.0 + force_magnitude * 0.5;
    
    // Consider shipping lanes (higher weight in shipping lanes)
    double lane_factor = map_->isInShippingLane(position) ? 1.5 : 1.0;
    
    // Combined weight
    double weight = distance_factor * force_factor * lane_factor;
    
    return weight;
}

} // namespace asv_planning 