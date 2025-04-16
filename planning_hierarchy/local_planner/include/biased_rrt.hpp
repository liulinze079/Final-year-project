#ifndef BIASED_RRT_HPP
#define BIASED_RRT_HPP

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <random>

namespace asv_planning {

// Forward declarations
class MultiResolutionGrid;
class ASVDynamics;
class SamplingStrategy;

/**
 * @brief RRT-based local planner with global path bias
 * 
 * This class implements a local path planner based on RRT that adapts the global
 * path to avoid dynamic obstacles. The sampling is biased toward the global path
 * to encourage path following while still allowing for reactive avoidance.
 */
class BiasedRRT {
public:
    /**
     * @brief Constructor
     * 
     * @param lookahead_distance Maximum planning distance (meters)
     * @param max_iterations Maximum number of RRT iterations
     * @param step_size Step size for RRT extension (meters)
     */
    BiasedRRT(double lookahead_distance = 30.0, 
            int max_iterations = 1000,
            double step_size = 1.0);
    
    /**
     * @brief Destructor
     */
    virtual ~BiasedRRT() = default;
    
    /**
     * @brief Set the environment map
     * 
     * @param map Pointer to environment map
     */
    void setMap(std::shared_ptr<MultiResolutionGrid> map);
    
    /**
     * @brief Set the global path to follow
     * 
     * @param global_path Global path waypoints
     */
    void setGlobalPath(const std::vector<Eigen::Vector2d>& global_path);
    
    /**
     * @brief Set the current ASV position and orientation
     * 
     * @param position Current position (x,y)
     * @param heading Current heading (radians)
     */
    void setCurrentPose(const Eigen::Vector2d& position, double heading);
    
    /**
     * @brief Set the ASV dynamics model
     * 
     * @param dynamics Pointer to ASV dynamics model
     */
    void setDynamicsModel(std::shared_ptr<ASVDynamics> dynamics);
    
    /**
     * @brief Set the sampling strategy
     * 
     * @param strategy Pointer to sampling strategy
     */
    void setSamplingStrategy(std::shared_ptr<SamplingStrategy> strategy);
    
    /**
     * @brief Set the goal bias probability
     * 
     * @param bias Probability of sampling toward goal (0.0-1.0)
     */
    void setGoalBias(double bias);
    
    /**
     * @brief Set the global path bias probability
     * 
     * @param bias Probability of sampling toward global path (0.0-1.0)
     */
    void setGlobalPathBias(double bias);
    
    /**
     * @brief Plan a local path
     * 
     * @return Vector of waypoints forming the local path
     */
    std::vector<Eigen::Vector2d> planPath();
    
    /**
     * @brief Get the closest point on global path
     * 
     * @param position Current position
     * @return Closest point on global path and its index
     */
    std::pair<Eigen::Vector2d, int> getClosestPointOnGlobalPath(
        const Eigen::Vector2d& position) const;
    
private:
    struct Node {
        Eigen::Vector2d position;
        int parent_idx;
        
        Node(const Eigen::Vector2d& pos, int parent = -1)
            : position(pos), parent_idx(parent) {}
    };
    
    // RRT parameters
    double lookahead_distance_;    ///< Maximum planning distance (m)
    int max_iterations_;           ///< Maximum RRT iterations
    double step_size_;             ///< Step size for extension (m)
    double goal_bias_;             ///< Goal sampling bias probability
    double global_path_bias_;      ///< Global path sampling bias probability
    
    // Current state
    Eigen::Vector2d current_position_;  ///< Current ASV position
    double current_heading_;            ///< Current ASV heading
    
    // Path data
    std::vector<Eigen::Vector2d> global_path_;   ///< Global path to follow
    std::vector<Eigen::Vector2d> local_path_;    ///< Resulting local path
    
    // Components
    std::shared_ptr<MultiResolutionGrid> map_;           ///< Environment map
    std::shared_ptr<ASVDynamics> dynamics_;             ///< ASV dynamics model
    std::shared_ptr<SamplingStrategy> sampling_strategy_; ///< Sampling strategy
    
    // Random number generator
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_dist_;
    
    /**
     * @brief Sample a random position
     * 
     * @return Random position
     */
    Eigen::Vector2d sampleRandomPosition();
    
    /**
     * @brief Find the nearest node in the tree
     * 
     * @param position Position to find nearest to
     * @param nodes Current RRT nodes
     * @return Index of nearest node
     */
    int findNearestNode(const Eigen::Vector2d& position, 
                     const std::vector<Node>& nodes) const;
    
    /**
     * @brief Extend the tree toward the sampled position
     * 
     * @param nearest_idx Index of nearest node
     * @param sample_pos Sampled position
     * @param nodes Current RRT nodes
     * @return True if extension succeeded
     */
    bool extend(int nearest_idx, const Eigen::Vector2d& sample_pos, 
             std::vector<Node>& nodes);
    
    /**
     * @brief Check if a path segment is collision-free
     * 
     * @param from Start position
     * @param to End position
     * @return True if collision-free
     */
    bool isCollisionFree(const Eigen::Vector2d& from, 
                       const Eigen::Vector2d& to) const;
    
    /**
     * @brief Extract path from RRT nodes
     * 
     * @param nodes RRT nodes
     * @param goal_idx Index of goal node
     * @return Vector of waypoints forming the path
     */
    std::vector<Eigen::Vector2d> extractPath(const std::vector<Node>& nodes, 
                                           int goal_idx) const;
    
    /**
     * @brief Get a target point on the global path
     * 
     * @return Target point for local planning
     */
    Eigen::Vector2d getGlobalPathTarget() const;
};

} // namespace asv_planning

#endif // BIASED_RRT_HPP 