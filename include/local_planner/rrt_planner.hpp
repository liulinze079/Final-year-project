#pragma once

#include <vector>
#include <memory>
#include <random>
#include <functional>
#include "../../include/local_planner/local_planner.hpp"
#include "../../include/planning/maritime_constraints.hpp"
#include "../environment_representation/environment_map.hpp"

namespace asv_planning {

/**
 * @brief Rapidly-exploring Random Tree (RRT) planner for ASV local planning
 * 
 * This class implements a RRT-based local path planner with energy awareness
 * for autonomous surface vehicles. It adapts global paths to local conditions
 * while avoiding obstacles and optimizing energy usage.
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
    ~RRTPlanner() = default;
    
    /**
     * @brief Set the maximum number of iterations
     * @param iterations Maximum iterations
     */
    void setMaxIterations(int iterations);
    
    /**
     * @brief Set the step size for extending the tree
     * @param step_size Step size (m)
     */
    void setStepSize(double step_size);
    
    /**
     * @brief Set the goal bias probability
     * @param bias Goal bias (0-1)
     */
    void setGoalBias(double bias);
    
    /**
     * @brief Set the obstacle bias probability
     * @param bias Obstacle bias (0-1)
     */
    void setObstacleBias(double bias);
    
    /**
     * @brief Set the path bias probability
     * @param bias Path bias (0-1)
     */
    void setPathBias(double bias);
    
    /**
     * @brief Set the rewire radius for RRT* optimization
     * @param radius Rewire radius (m)
     */
    void setRewireRadius(double radius);
    
    /**
     * @brief Set the maximum planning time
     * @param time Maximum planning time (seconds)
     */
    void setMaxPlanningTime(double time);
    
    /**
     * @brief Set the obstacle clearance
     * @param clearance Obstacle clearance (m)
     */
    void setObstacleClearance(double clearance);
    
    /**
     * @brief Set the local map radius
     * @param radius Local map radius (m)
     */
    void setLocalMapRadius(double radius);
    
    /**
     * @brief Set whether to consider energy in planning
     * @param aware Whether to consider energy
     */
    void setEnergyAware(bool aware);
    
    /**
     * @brief Set maritime constraints
     * @param constraints Maritime constraints
     */
    void setConstraints(std::shared_ptr<MaritimeConstraints> constraints);
    
    /**
     * @brief Plan a path from start to goal
     * @param params Planning parameters
     * @return Planning result
     */
    PlanningResult plan(const PlanningParameters& params);
    
    /**
     * @brief Update the environment map
     * @param map New environment map
     */
    void updateEnvironmentMap(const EnvironmentMap& map);
    
private:
    /**
     * @brief Node in the RRT
     */
    struct Node {
        Point2D position;          // Position
        double heading = 0.0;      // Heading (radians)
        double cost = 0.0;         // Cost to reach this node
        double energy_cost = 0.0;  // Energy cost to reach this node
        int parent_id = -1;        // Index of parent node (-1 for root)
        std::vector<int> children; // Indices of child nodes
    };
    
    /**
     * @brief Get a random point for tree extension
     * @param bounds Map bounds
     * @return Random point
     */
    Point2D getRandomPoint(const std::array<double, 4>& bounds);
    
    /**
     * @brief Sample a point biased towards the goal, obstacles, or global path
     * @param goal Goal position
     * @param bounds Map bounds
     * @return Sampled point
     */
    Point2D samplePoint(const Point2D& goal, const std::array<double, 4>& bounds);
    
    /**
     * @brief Find the nearest node in the tree to a given point
     * @param point Query point
     * @return Index of the nearest node
     */
    int findNearestNode(const Point2D& point);
    
    /**
     * @brief Extend the tree towards a point
     * @param nearest_idx Index of the nearest node
     * @param point Target point
     * @return Index of the new node, or -1 if extension failed
     */
    int extendTree(int nearest_idx, const Point2D& point);
    
    /**
     * @brief Find nodes near a point for rewiring
     * @param point Query point
     * @param radius Search radius
     * @return Indices of nearby nodes
     */
    std::vector<int> findNearNodes(const Point2D& point, double radius);
    
    /**
     * @brief Rewire the tree to optimize paths
     * @param new_idx Index of the new node
     * @param near_indices Indices of nearby nodes
     */
    void rewireTree(int new_idx, const std::vector<int>& near_indices);
    
    /**
     * @brief Check if a path between two points is collision-free
     * @param from Start point
     * @param to End point
     * @return True if collision-free
     */
    bool isPathFree(const Point2D& from, const Point2D& to);
    
    /**
     * @brief Calculate the cost of moving between two points
     * @param from Start point
     * @param to End point
     * @param from_heading Heading at start
     * @param to_heading Heading at end
     * @return Cost of the movement
     */
    double calculateCost(const Point2D& from, const Point2D& to, 
                       double from_heading, double to_heading);
    
    /**
     * @brief Calculate the energy cost of moving between two points
     * @param from Start point
     * @param to End point
     * @param from_heading Heading at start
     * @param to_heading Heading at end
     * @return Energy cost of the movement
     */
    double calculateEnergyCost(const Point2D& from, const Point2D& to,
                             double from_heading, double to_heading);
    
    /**
     * @brief Extract the path from the tree
     * @param goal_idx Index of the goal node
     * @return Vector of points along the path
     */
    std::vector<Point2D> extractPath(int goal_idx);
    
    /**
     * @brief Calculate heading between two points
     * @param from Start point
     * @param to End point
     * @return Heading in radians
     */
    double calculateHeading(const Point2D& from, const Point2D& to);
    
    /**
     * @brief Calculate path cost including energy if enabled
     * @param path Path to evaluate
     * @return Total cost of the path
     */
    double calculatePathCost(const std::vector<Point2D>& path);
    
    /**
     * @brief Calculate the distance between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Euclidean distance
     */
    double distance(const Point2D& p1, const Point2D& p2);
    
    // RRT parameters
    int max_iterations_;
    double step_size_;
    double goal_bias_;
    double obstacle_bias_;
    double path_bias_;
    double rewire_radius_;
    double max_planning_time_;
    double obstacle_clearance_;
    double local_map_radius_;
    bool energy_aware_;
    
    // Tree data
    std::vector<Node> nodes_;
    
    // Environment and constraints
    EnvironmentMap environment_map_;
    std::shared_ptr<MaritimeConstraints> constraints_;
    
    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;
};

} // namespace asv_planning 