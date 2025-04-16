#ifndef SAMPLING_STRATEGIES_HPP
#define SAMPLING_STRATEGIES_HPP

#include <Eigen/Dense>
#include <memory>
#include <random>
#include <vector>

namespace asv_planning {

// Forward declarations
class MultiResolutionGrid;

/**
 * @brief Base class for RRT sampling strategies
 * 
 * This class defines the interface for sampling strategies used by RRT planners.
 * Different implementations can provide different sampling distributions to
 * optimize RRT performance in various environments.
 */
class SamplingStrategy {
public:
    /**
     * @brief Constructor
     */
    SamplingStrategy();
    
    /**
     * @brief Virtual destructor
     */
    virtual ~SamplingStrategy() = default;
    
    /**
     * @brief Sample a random position
     * 
     * @param current_position Current ASV position
     * @param heading Current ASV heading
     * @param bounds Sampling bounds (min_x, min_y, max_x, max_y)
     * @param target_bias Target position for biased sampling
     * @param bias_probability Probability of sampling toward target
     * @return Sampled position
     */
    virtual Eigen::Vector2d samplePosition(const Eigen::Vector2d& current_position,
                                      double heading,
                                      const Eigen::Vector4d& bounds,
                                      const Eigen::Vector2d& target_bias,
                                      double bias_probability) = 0;
    
    /**
     * @brief Set the environment map
     * 
     * @param map Pointer to environment map
     */
    virtual void setMap(std::shared_ptr<MultiResolutionGrid> map);
    
protected:
    std::mt19937 rng_;  ///< Random number generator
    std::shared_ptr<MultiResolutionGrid> map_; ///< Environment map
    
    /**
     * @brief Sample a random position within bounds
     * 
     * @param bounds Sampling bounds (min_x, min_y, max_x, max_y)
     * @return Random position
     */
    Eigen::Vector2d sampleUniform(const Eigen::Vector4d& bounds);
};

/**
 * @brief Uniform sampling strategy
 * 
 * Samples positions uniformly within the given bounds, with optional
 * bias toward a target position.
 */
class UniformSampling : public SamplingStrategy {
public:
    /**
     * @brief Constructor
     */
    UniformSampling();
    
    /**
     * @brief Sample a random position
     * 
     * @param current_position Current ASV position
     * @param heading Current ASV heading
     * @param bounds Sampling bounds (min_x, min_y, max_x, max_y)
     * @param target_bias Target position for biased sampling
     * @param bias_probability Probability of sampling toward target
     * @return Sampled position
     */
    Eigen::Vector2d samplePosition(const Eigen::Vector2d& current_position,
                               double heading,
                               const Eigen::Vector4d& bounds,
                               const Eigen::Vector2d& target_bias,
                               double bias_probability) override;
};

/**
 * @brief Gaussian sampling strategy
 * 
 * Samples positions using a Gaussian distribution centered on the ASV,
 * with standard deviation proportional to the distance to the goal.
 */
class GaussianSampling : public SamplingStrategy {
public:
    /**
     * @brief Constructor
     * 
     * @param std_dev_scale Scale factor for standard deviation
     */
    GaussianSampling(double std_dev_scale = 0.3);
    
    /**
     * @brief Sample a random position
     * 
     * @param current_position Current ASV position
     * @param heading Current ASV heading
     * @param bounds Sampling bounds (min_x, min_y, max_x, max_y)
     * @param target_bias Target position for biased sampling
     * @param bias_probability Probability of sampling toward target
     * @return Sampled position
     */
    Eigen::Vector2d samplePosition(const Eigen::Vector2d& current_position,
                               double heading,
                               const Eigen::Vector4d& bounds,
                               const Eigen::Vector2d& target_bias,
                               double bias_probability) override;
                               
private:
    double std_dev_scale_;  ///< Scale factor for standard deviation
};

/**
 * @brief Adaptive sampling strategy based on obstacles
 * 
 * Adjusts sampling based on obstacle density, sampling more densely
 * in regions with obstacles that need to be navigated around.
 */
class AdaptiveSampling : public SamplingStrategy {
public:
    /**
     * @brief Constructor
     */
    AdaptiveSampling();
    
    /**
     * @brief Sample a random position
     * 
     * @param current_position Current ASV position
     * @param heading Current ASV heading
     * @param bounds Sampling bounds (min_x, min_y, max_x, max_y)
     * @param target_bias Target position for biased sampling
     * @param bias_probability Probability of sampling toward target
     * @return Sampled position
     */
    Eigen::Vector2d samplePosition(const Eigen::Vector2d& current_position,
                               double heading,
                               const Eigen::Vector4d& bounds,
                               const Eigen::Vector2d& target_bias,
                               double bias_probability) override;
                               
    /**
     * @brief Set the environment map
     * 
     * @param map Pointer to environment map
     */
    void setMap(std::shared_ptr<MultiResolutionGrid> map) override;
    
private:
    /**
     * @brief Calculate sampling weight based on obstacle density
     * 
     * @param position Position to evaluate
     * @return Sampling weight
     */
    double calculateSamplingWeight(const Eigen::Vector2d& position) const;
};

} // namespace asv_planning

#endif // SAMPLING_STRATEGIES_HPP 