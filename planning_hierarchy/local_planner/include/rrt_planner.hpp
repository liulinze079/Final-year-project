#pragma once

#include <vector>
#include <memory>
#include <random>
#include <Eigen/Dense>
#include <functional>

#include "grid/multi_resolution_grid.hpp"

namespace asv_planning {

struct RRTNode {
    Eigen::Vector3d state;  // x, y, heading
    int parent_id;
    double cost;
    
    RRTNode(const Eigen::Vector3d& state, int parent_id, double cost)
        : state(state), parent_id(parent_id), cost(cost) {}
};

/**
 * @brief RRT-based local planner for ASV navigation
 * 
 * This class implements a sampling-based RRT (Rapidly-exploring Random Tree)
 * algorithm for local planning around obstacles. It incorporates energy awareness
 * and maritime domain-specific constraints.
 */
class RRTPlanner {
public:
    /**
     * @brief Constructor
     */
    RRTPlanner();
    
    /**
     * @brief Destructor
     */
    ~RRTPlanner();
    
    /**
     * @brief Initialize the planner
     * 
     * @param map Environment map
     * @param max_iterations Maximum number of iterations
     * @param step_size Step size for extending the tree
     * @param goal_bias Goal bias probability
     * @param goal_tolerance Goal tolerance
     * @param obstacle_clearance Obstacle clearance
     * @param domain_min Minimum domain boundaries
     * @param domain_max Maximum domain boundaries
     */
    void initialize(std::shared_ptr<MultiResolutionGrid> map,
                  int max_iterations = 1000,
                  double step_size = 1.0,
                  double goal_bias = 0.1,
                  double goal_tolerance = 0.5,
                  double obstacle_clearance = 1.0,
                  const Eigen::Vector2d& domain_min = Eigen::Vector2d(-100, -100),
                  const Eigen::Vector2d& domain_max = Eigen::Vector2d(100, 100));
    
    /**
     * @brief Set the cost function for edge evaluation
     * 
     * @param cost_function Function that evaluates cost between two states
     */
    void setCostFunction(std::function<double(const Eigen::Vector3d&, const Eigen::Vector3d&)> cost_function);
    
    /**
     * @brief Set the state validity checker
     * 
     * @param validity_checker Function that checks if a state is valid
     */
    void setStateValidityChecker(std::function<bool(const Eigen::Vector3d&)> validity_checker);
    
    /**
     * @brief Set the dynamics constraints
     * 
     * @param dynamics_constraints Function that enforces dynamics constraints
     */
    void setDynamicsConstraints(std::function<Eigen::Vector3d(const Eigen::Vector3d&, const Eigen::Vector3d&)> dynamics_constraints);
    
    /**
     * @brief Plan a path from start to goal
     * 
     * @param start Start state (x, y, heading)
     * @param goal Goal state (x, y, heading)
     * @param path Output path as a sequence of states
     * @return true if path found, false otherwise
     */
    bool plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, std::vector<Eigen::Vector3d>& path);
    
    /**
     * @brief Get the tree nodes
     * 
     * @return Vector of RRT nodes
     */
    const std::vector<RRTNode>& getTree() const;
    
    /**
     * @brief Clear the tree
     */
    void clearTree();
    
    /**
     * @brief Set the random seed
     * 
     * @param seed Random seed
     */
    void setSeed(unsigned int seed);
    
private:
    /**
     * @brief Sample a random state
     * 
     * @param goal Goal state
     * @return Random state
     */
    Eigen::Vector3d sampleRandomState(const Eigen::Vector3d& goal);
    
    /**
     * @brief Find the nearest node to a state
     * 
     * @param state Query state
     * @return Index of the nearest node
     */
    int findNearestNode(const Eigen::Vector3d& state) const;
    
    /**
     * @brief Find nodes within a radius of a state
     * 
     * @param state Query state
     * @param radius Search radius
     * @return Indices of nodes within radius
     */
    std::vector<int> findNodesInRadius(const Eigen::Vector3d& state, double radius) const;
    
    /**
     * @brief Extend the tree towards a state
     * 
     * @param nearest_node_idx Index of the nearest node
     * @param target_state Target state
     * @return New node if extension successful, nullptr otherwise
     */
    Eigen::Vector3d extend(int nearest_node_idx, const Eigen::Vector3d& target_state);
    
    /**
     * @brief Check if a straight-line path between two states is collision-free
     * 
     * @param from_state Source state
     * @param to_state Target state
     * @return true if path is collision-free, false otherwise
     */
    bool isPathCollisionFree(const Eigen::Vector3d& from_state, const Eigen::Vector3d& to_state) const;
    
    /**
     * @brief Compute the cost of moving between two states
     * 
     * @param from_state Source state
     * @param to_state Target state
     * @return Cost value
     */
    double computeCost(const Eigen::Vector3d& from_state, const Eigen::Vector3d& to_state) const;
    
    /**
     * @brief Extract path from the tree
     * 
     * @param goal_node_idx Index of the goal node
     * @param path Output path
     */
    void extractPath(int goal_node_idx, std::vector<Eigen::Vector3d>& path) const;
    
    /**
     * @brief Smooth the path
     * 
     * @param path Input/output path
     */
    void smoothPath(std::vector<Eigen::Vector3d>& path) const;
    
    std::vector<RRTNode> tree_;
    std::shared_ptr<MultiResolutionGrid> map_;
    
    // RRT parameters
    int max_iterations_;
    double step_size_;
    double goal_bias_;
    double goal_tolerance_;
    double obstacle_clearance_;
    Eigen::Vector2d domain_min_;
    Eigen::Vector2d domain_max_;
    
    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;
    
    // Function objects
    std::function<double(const Eigen::Vector3d&, const Eigen::Vector3d&)> cost_function_;
    std::function<bool(const Eigen::Vector3d&)> validity_checker_;
    std::function<Eigen::Vector3d(const Eigen::Vector3d&, const Eigen::Vector3d&)> dynamics_constraints_;
};

} // namespace asv_planning 