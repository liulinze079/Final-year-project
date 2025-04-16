#ifndef ASV_PLANNING_A_STAR_PLANNER_HPP
#define ASV_PLANNING_A_STAR_PLANNER_HPP

#include <vector>
#include <unordered_map>
#include <queue>
#include <functional>
#include <Eigen/Dense>
#include "../environment_representation/multi_resolution_grid.hpp"
#include "../environment_representation/energy_cost_map.hpp"
#include "../planning/maritime_heuristics.hpp"

namespace asv_planning {

/**
 * @class AStarPlanner
 * @brief A* algorithm implementation for energy-aware global path planning in maritime environments
 * 
 * This class implements the A* search algorithm with maritime-specific heuristics for
 * path planning in ASV (Autonomous Surface Vehicle) navigation. It considers energy
 * efficiency, environmental factors (currents, waves), and safety constraints.
 */
class AStarPlanner {
public:
    /**
     * @brief Constructor for AStarPlanner
     */
    AStarPlanner();
    
    /**
     * @brief Default destructor
     */
    ~AStarPlanner() = default;
    
    /**
     * @struct PlanningParams
     * @brief Parameters for the A* planning algorithm
     */
    struct PlanningParams {
        Eigen::Vector2d start;            // Start position
        Eigen::Vector2d goal;             // Goal position
        double start_heading = 0.0;       // Start heading in radians
        double goal_heading = 0.0;        // Goal heading in radians
        double planning_timeout = 10.0;   // Planning timeout in seconds
        bool consider_energy = true;      // Whether to consider energy in planning
        double heuristic_weight = 1.0;    // Heuristic weight (1.0 = standard A*)
        bool allow_diagonal = true;       // Whether to allow diagonal movements
        double safety_weight = 0.7;       // Weight for safety constraints
        double energy_weight = 0.3;       // Weight for energy efficiency
        std::string heuristic_type = "euclidean"; // Heuristic type: "euclidean", "manhattan", "maritime"
    };
    
    /**
     * @struct PlanningResult
     * @brief Result of the A* planning algorithm
     */
    struct PlanningResult {
        bool success = false;                     // Whether planning was successful
        std::vector<Eigen::Vector2d> path;        // Planned path
        std::vector<double> headings;             // Headings along the path
        double path_length = 0.0;                 // Path length in meters
        double energy_cost = 0.0;                 // Estimated energy cost
        double safety_score = 0.0;                // Safety score (0-1, higher is better)
        double planning_time = 0.0;               // Planning time in seconds
        int iterations = 0;                       // Number of iterations
        std::string error_message;                // Error message if planning failed
    };
    
    /**
     * @brief Sets the environment map
     * @param map The multi-resolution grid representing the environment
     */
    void setEnvironmentMap(const MultiResolutionGrid& map);
    
    /**
     * @brief Sets the energy cost map
     * @param energy_map The energy cost map
     */
    void setEnergyCostMap(const EnergyCostMap& energy_map);
    
    /**
     * @brief Sets the maritime heuristics
     * @param heuristics The maritime heuristics
     */
    void setMaritimeHeuristics(const MaritimeHeuristics& heuristics);
    
    /**
     * @brief Plans a path from start to goal
     * @param params Planning parameters
     * @return Planning result
     */
    PlanningResult planPath(const PlanningParams& params);
    
    /**
     * @brief Smooths a planned path
     * @param path The original path
     * @param headings The original headings
     * @return Smoothed path and headings
     */
    std::pair<std::vector<Eigen::Vector2d>, std::vector<double>> 
    smoothPath(const std::vector<Eigen::Vector2d>& path,
               const std::vector<double>& headings);
    
private:
    /**
     * @struct Node
     * @brief A node in the A* search tree
     */
    struct Node {
        Eigen::Vector2d position;    // Position
        double g_cost = 0.0;         // Cost from start
        double h_cost = 0.0;         // Heuristic cost to goal
        double f_cost = 0.0;         // Total cost (g + h)
        double heading = 0.0;        // Heading at this node
        double energy_cost = 0.0;    // Energy cost to reach this node
        double safety_score = 1.0;   // Safety score at this node
        Eigen::Vector2d parent;      // Parent node position
        
        // Comparison for priority queue
        bool operator>(const Node& other) const {
            return f_cost > other.f_cost;
        }
    };
    
    /**
     * @brief Calculates the heuristic cost from a position to the goal
     * @param position Current position
     * @param goal Goal position
     * @param heuristic_type Type of heuristic to use
     * @return Heuristic cost
     */
    double calculateHeuristic(
        const Eigen::Vector2d& position,
        const Eigen::Vector2d& goal,
        const std::string& heuristic_type);
    
    /**
     * @brief Calculates the maritime heuristic cost
     * @param position Current position
     * @param goal Goal position
     * @return Maritime heuristic cost
     */
    double calculateMaritimeHeuristic(
        const Eigen::Vector2d& position,
        const Eigen::Vector2d& goal);
    
    /**
     * @brief Gets the neighbors of a node
     * @param current Current node
     * @param allow_diagonal Whether to allow diagonal movements
     * @return List of neighbor positions
     */
    std::vector<Eigen::Vector2d> getNeighbors(
        const Node& current,
        bool allow_diagonal);
    
    /**
     * @brief Calculates the movement cost between two nodes
     * @param from Start position
     * @param to End position
     * @param from_heading Heading at start position
     * @param params Planning parameters
     * @return Movement cost
     */
    double calculateMovementCost(
        const Eigen::Vector2d& from,
        const Eigen::Vector2d& to,
        double from_heading,
        const PlanningParams& params);
    
    /**
     * @brief Calculates the heading between two positions
     * @param from Start position
     * @param to End position
     * @return Heading in radians
     */
    double calculateHeading(
        const Eigen::Vector2d& from,
        const Eigen::Vector2d& to);
    
    /**
     * @brief Checks if a path between two positions is collision-free
     * @param from Start position
     * @param to End position
     * @return True if path is collision-free
     */
    bool isPathCollisionFree(
        const Eigen::Vector2d& from,
        const Eigen::Vector2d& to);
    
    /**
     * @brief Calculates the safety score for a position
     * @param position Position to check
     * @return Safety score (0-1, higher is better)
     */
    double calculateSafetyScore(const Eigen::Vector2d& position);
    
    /**
     * @brief Reconstructs the path from the goal to the start
     * @param goal_node Goal node
     * @param came_from Map of nodes to their parents
     * @return Path and headings
     */
    std::pair<std::vector<Eigen::Vector2d>, std::vector<double>> 
    reconstructPath(
        const Node& goal_node,
        const std::unordered_map<size_t, Node>& came_from);
    
    /**
     * @brief Hash function for 2D positions
     * @param position Position to hash
     * @return Hash value
     */
    size_t hashPosition(const Eigen::Vector2d& position) const;
    
    // Environment data
    MultiResolutionGrid environment_map_;
    EnergyCostMap energy_cost_map_;
    MaritimeHeuristics maritime_heuristics_;
    
    // Grid resolution for discretization
    double grid_resolution_;
    
    // Safety parameters
    double obstacle_clearance_;
    double min_safety_distance_;
};

} // namespace asv_planning

#endif // ASV_PLANNING_A_STAR_PLANNER_HPP 