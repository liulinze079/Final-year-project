#ifndef ENERGY_AWARE_ASTAR_HPP
#define ENERGY_AWARE_ASTAR_HPP

#include <Eigen/Dense>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include <functional>
#include <string>

namespace asv_planning {

// Forward declarations
class MultiResolutionGrid;
class ASVDynamics;

/**
 * @brief Energy-aware A* global path planner
 * 
 * This class implements a global path planner based on the A* algorithm,
 * with energy consumption as a component of the cost function. It uses
 * a simplified model to estimate energy costs between grid cells.
 */
class EnergyAwareAStar {
public:
    /**
     * @brief Constructor
     * 
     * @param resolution Grid resolution for planning (meters)
     */
    EnergyAwareAStar(double resolution = 1.0);
    
    /**
     * @brief Destructor
     */
    virtual ~EnergyAwareAStar() = default;
    
    /**
     * @brief Set the map for planning
     * 
     * @param map Pointer to the environment grid map
     */
    void setMap(std::shared_ptr<MultiResolutionGrid> map);
    
    /**
     * @brief Set ASV dynamics model for energy estimation
     * 
     * @param dynamics Pointer to ASV dynamics model
     */
    void setDynamicsModel(std::shared_ptr<ASVDynamics> dynamics);
    
    /**
     * @brief Set start and goal positions
     * 
     * @param start Start position (x,y)
     * @param goal Goal position (x,y)
     */
    void setStartAndGoal(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);
    
    /**
     * @brief Set energy weight in the cost function
     * 
     * @param weight Weight for energy term (0.0 to 1.0)
     */
    void setEnergyWeight(double weight);
    
    /**
     * @brief Plan global path from start to goal
     * 
     * @return Vector of waypoints forming the path
     */
    std::vector<Eigen::Vector2d> planPath();
    
    /**
     * @brief Get the estimated energy consumption for the path
     * 
     * @return Energy consumption in Joules
     */
    double getPathEnergy() const;
    
    /**
     * @brief Get the estimated path length
     * 
     * @return Path length in meters
     */
    double getPathLength() const;
    
private:
    // Structure to represent a node in the search graph
    struct Node {
        int x, y;        // Grid coordinates
        double g;        // Cost from start
        double h;        // Heuristic (estimated cost to goal)
        double energy;   // Energy consumption from start
        
        Node(int x, int y, double g = 0.0, double h = 0.0, double energy = 0.0)
            : x(x), y(y), g(g), h(h), energy(energy) {}
        
        // Total cost (f = g + h)
        double f() const { return g + h; }
        
        // Comparison operator for priority queue
        bool operator>(const Node& other) const {
            return f() > other.f();
        }
        
        // Equality operator for hash lookups
        bool operator==(const Node& other) const {
            return x == other.x && y == other.y;
        }
    };
    
    // Hash function for Node
    struct NodeHash {
        std::size_t operator()(const Node& node) const {
            return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
        }
    };
    
    // Planning parameters
    double resolution_;                     ///< Grid resolution for planning (m)
    double energy_weight_;                  ///< Weight for energy term in cost function
    Eigen::Vector2d start_;                 ///< Start position (x,y)
    Eigen::Vector2d goal_;                  ///< Goal position (x,y)
    
    // Path results
    std::vector<Eigen::Vector2d> path_;     ///< Resulting path waypoints
    double path_energy_;                    ///< Estimated total energy consumption
    double path_length_;                    ///< Total path length
    
    // Components
    std::shared_ptr<MultiResolutionGrid> map_;  ///< Environment grid map
    std::shared_ptr<ASVDynamics> dynamics_;    ///< ASV dynamics model
    
    /**
     * @brief Calculate heuristic (estimated cost) from node to goal
     * 
     * @param node Current node
     * @param goal_x Goal x-coordinate
     * @param goal_y Goal y-coordinate
     * @return Estimated cost to goal
     */
    double calculateHeuristic(const Node& node, int goal_x, int goal_y) const;
    
    /**
     * @brief Estimate energy consumption between two neighboring cells
     * 
     * @param from Current node
     * @param to Neighbor node
     * @param current_energy Energy consumed so far
     * @return Estimated additional energy consumption
     */
    double estimateEnergyConsumption(const Node& from, const Node& to, double current_energy) const;
    
    /**
     * @brief Get neighboring nodes of the current node
     * 
     * @param current Current node
     * @return Vector of neighboring nodes
     */
    std::vector<Node> getNeighbors(const Node& current) const;
    
    /**
     * @brief Reconstruct path from goal to start using parent map
     * 
     * @param came_from Map of node parents
     * @param current Final node (goal)
     * @return Vector of waypoints forming the path
     */
    std::vector<Eigen::Vector2d> reconstructPath(
        const std::unordered_map<Node, Node, NodeHash>& came_from,
        const Node& current);
};

} // namespace asv_planning

#endif // ENERGY_AWARE_ASTAR_HPP 