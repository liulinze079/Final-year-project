#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <functional>
#include <vector>

namespace asv_planning {

// Forward declarations
class ASVDynamics;
class MultiResolutionGrid;

/**
 * @brief Cost functions for MPC optimization
 * 
 * This class defines various cost functions used in the MPC optimization,
 * including tracking error costs, control effort costs, and energy consumption costs.
 */
class CostFunctions {
public:
    /**
     * @brief Constructor
     * 
     * @param dynamics ASV dynamics model for energy calculations
     */
    CostFunctions(std::shared_ptr<ASVDynamics> dynamics);
    
    /**
     * @brief Destructor
     */
    ~CostFunctions();
    
    /**
     * @brief Calculate tracking error cost for position
     * 
     * @param state Current state
     * @param reference Reference state
     * @param weight Weight factor
     * @return Cost value
     */
    double positionTrackingCost(const Eigen::VectorXd& state,
                              const Eigen::VectorXd& reference,
                              double weight) const;
    
    /**
     * @brief Calculate tracking error cost for heading
     * 
     * @param state Current state
     * @param reference Reference state
     * @param weight Weight factor
     * @return Cost value
     */
    double headingTrackingCost(const Eigen::VectorXd& state,
                             const Eigen::VectorXd& reference,
                             double weight) const;
    
    /**
     * @brief Calculate tracking error cost for velocity
     * 
     * @param state Current state
     * @param reference Reference state
     * @param weight Weight factor
     * @return Cost value
     */
    double velocityTrackingCost(const Eigen::VectorXd& state,
                              const Eigen::VectorXd& reference,
                              double weight) const;
    
    /**
     * @brief Calculate control effort cost
     * 
     * @param control Control inputs
     * @param weight Weight factor
     * @return Cost value
     */
    double controlEffortCost(const Eigen::VectorXd& control,
                           double weight) const;
    
    /**
     * @brief Calculate control rate cost (penalizes rapid control changes)
     * 
     * @param control Current control inputs
     * @param prev_control Previous control inputs
     * @param weight Weight factor
     * @return Cost value
     */
    double controlRateCost(const Eigen::VectorXd& control,
                         const Eigen::VectorXd& prev_control,
                         double weight) const;
    
    /**
     * @brief Calculate energy consumption cost
     * 
     * @param state Current state
     * @param control Control inputs
     * @param weight Weight factor
     * @return Cost value
     */
    double energyCost(const Eigen::VectorXd& state,
                    const Eigen::VectorXd& control,
                    double weight) const;
    
    /**
     * @brief Get quadratic cost matrices for MPC formulation
     * 
     * @param Q_out Output state cost matrix
     * @param R_out Output control cost matrix
     * @param position_weight Weight for position tracking
     * @param heading_weight Weight for heading tracking
     * @param velocity_weight Weight for velocity tracking
     * @param control_weight Weight for control effort
     * @param energy_weight Weight for energy consumption
     */
    void getQuadraticCostMatrices(Eigen::MatrixXd& Q_out,
                                Eigen::MatrixXd& R_out,
                                double position_weight,
                                double heading_weight,
                                double velocity_weight,
                                double control_weight,
                                double energy_weight) const;
    
    /**
     * @brief Set environment map
     * 
     * @param map Environment map
     */
    void setEnvironmentMap(std::shared_ptr<MultiResolutionGrid> map);
    
    /**
     * @brief Calculate environmental cost component
     * 
     * @param state Current state
     * @return Environmental cost
     */
    double calculateEnvironmentalCost(const Eigen::VectorXd& state) const;
    
    /**
     * @brief Get state dimension
     * 
     * @return State dimension
     */
    int getStateDimension() const;
    
    /**
     * @brief Get control dimension
     * 
     * @return Control dimension
     */
    int getControlDimension() const;
    
private:
    std::shared_ptr<ASVDynamics> dynamics_;
    std::shared_ptr<MultiResolutionGrid> map_;
    
    // Cost weights
    double position_weight_;
    double heading_weight_;
    double velocity_weight_;
    double control_weight_;
    double control_rate_weight_;
    double energy_weight_;
    
    // Dimensions
    int state_dim_;
    int control_dim_;
    
    /**
     * @brief Calculate a quadratic cost term
     * 
     * @param error Error vector
     * @param weight Weight matrix
     * @return Quadratic cost value
     */
    double calculateQuadraticCost(const Eigen::VectorXd& error,
                                const Eigen::MatrixXd& weight) const;
};

} // namespace asv_planning 