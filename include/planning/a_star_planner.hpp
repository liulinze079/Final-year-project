#ifndef ASV_PLANNING_A_STAR_PLANNER_HPP
#define ASV_PLANNING_A_STAR_PLANNER_HPP

#include "planning/global_planner.hpp"
#include "planning/maritime_heuristics.hpp"
#include "environment_representation/multi_resolution_grid.hpp"
#include <Eigen/Core>
#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <memory>

namespace asv_planning {

/**
 * @class AStarPlanner
 * @brief A* search algorithm implementation for global path planning in maritime environments.
 * 
 * This class implements the A* search algorithm with maritime-specific heuristics
 * for energy-aware path planning for autonomous surface vehicles (ASVs).
 */
class AStarPlanner : public GlobalPlanner {
public:
    /**
     * @brief Construct a new AStarPlanner object
     */
    AStarPlanner();

    /**
     * @brief Destroy the AStarPlanner object
     */
    ~AStarPlanner() override = default;

    /**
     * @brief Initialize the planner with environmental data
     * @param map The multi-resolution grid representing the environment
     * @return true if initialization was successful
     */
    bool initialize(MultiResolutionGrid* map) override;

    /**
     * @brief Plan a path from start to goal
     * @param start Start pose
     * @param goal Goal pose
     * @param path Output path
     * @return true if planning was successful
     */
    bool planPath(const geometry_msgs::msg::PoseStamped& start,
                 const geometry_msgs::msg::PoseStamped& goal,
                 std::vector<geometry_msgs::msg::PoseStamped>& path) override;

    /**
     * @brief Set the heuristic type
     * @param type Type of heuristic (0: Manhattan, 1: Euclidean, 2: Diagonal, 3: Maritime)
     */
    void setHeuristicType(int type);

    /**
     * @brief Set the heuristic weight
     * @param weight Weight factor for the heuristic
     */
    void setHeuristicWeight(double weight);

    /**
     * @brief Set whether diagonal movement is allowed
     * @param allow true to allow diagonal movement
     */
    void setAllowDiagonal(bool allow);

    /**
     * @brief Get the number of iterations performed in the last planning
     * @return Number of iterations
     */
    int getIterations() const;

    /**
     * @brief Get the path cost of the last planned path
     * @return Path cost
     */
    double getPathCost() const;

    /**
     * @brief Set maritime heuristics object for environmental aware planning
     * @param maritime_heuristics Maritime heuristics object
     */
    void setMaritimeHeuristics(std::shared_ptr<MaritimeHeuristics> maritime_heuristics);

private:
    /**
     * @brief Struct to represent a node in the A* search
     */
    struct Node {
        Eigen::Vector2d position;  // Cell position
        double g_cost;             // Cost from start to current
        double h_cost;             // Estimated cost from current to goal
        double f_cost;             // Total cost (g_cost + h_cost)
        std::shared_ptr<Node> parent; // Parent node
        
        Node(const Eigen::Vector2d& pos) 
            : position(pos), g_cost(0.0), h_cost(0.0), f_cost(0.0), parent(nullptr) {}
    };

    /**
     * @brief Calculate heuristic cost between two positions
     * @param from Start position
     * @param to Goal position
     * @return Heuristic cost
     */
    double calculateHeuristic(const Eigen::Vector2d& from, const Eigen::Vector2d& to);

    /**
     * @brief Calculate the movement cost between adjacent cells
     * @param from Start position
     * @param to Goal position
     * @return Movement cost
     */
    double calculateMovementCost(const Eigen::Vector2d& from, const Eigen::Vector2d& to);

    /**
     * @brief Get neighboring cells for a given position
     * @param position Current position
     * @return Vector of neighboring positions
     */
    std::vector<Eigen::Vector2d> getNeighbors(const Eigen::Vector2d& position);

    /**
     * @brief Reconstruct path from A* search result
     * @param end_node End node of the search
     * @param start Start pose
     * @param goal Goal pose
     * @param path Output path
     */
    void reconstructPath(const std::shared_ptr<Node>& end_node,
                         const geometry_msgs::msg::PoseStamped& start,
                         const geometry_msgs::msg::PoseStamped& goal,
                         std::vector<geometry_msgs::msg::PoseStamped>& path);

    /**
     * @brief Convert grid node position to world pose
     * @param position Grid position
     * @param heading Heading angle
     * @return Pose in world coordinates
     */
    geometry_msgs::msg::PoseStamped poseToPoseStamped(const Eigen::Vector2d& position, double heading);

    // Planner parameters
    int heuristic_type_;           // Type of heuristic
    double heuristic_weight_;      // Weight of heuristic
    bool allow_diagonal_;          // Whether diagonal movement is allowed
    double grid_resolution_;       // Resolution of the grid for planning

    // Performance metrics
    int iterations_;               // Number of iterations in last planning
    double path_cost_;             // Cost of the last planned path

    // Environment representation
    MultiResolutionGrid* map_;     // Pointer to the environment map
    std::shared_ptr<MaritimeHeuristics> maritime_heuristics_; // Maritime-specific heuristics

    // Comparison function for the priority queue
    struct NodeComparison {
        bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
            return a->f_cost > b->f_cost;
        }
    };
};

} // namespace asv_planning

#endif // ASV_PLANNING_A_STAR_PLANNER_HPP 