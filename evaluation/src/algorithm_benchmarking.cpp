#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <random>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"

// Core planning components
#include "../../environment_representation/include/map_representation.hpp"
#include "../../environment_representation/include/grid_map_representation.hpp"
#include "../../planning_hierarchy/global_planner/include/energy_aware_astar.hpp"
#include "../../include/local_planner/rrt_planner.hpp"
#include "../../control/include/mpc_controller.hpp"
#include "../../common/include/asv_dynamics.hpp"
#include "../../common/include/asv_control.hpp"

// Evaluation metrics
#include "../include/energy_metrics.hpp"
#include "../include/path_quality.hpp"
#include "../include/obstacle_avoidance.hpp"

namespace asv_planning {

/**
 * @brief Container for algorithm benchmark results
 */
struct BenchmarkResult {
    // Algorithm identification
    std::string algorithm_name;
    std::string variant;
    std::map<std::string, double> parameters;
    
    // Performance metrics
    double computation_time_ms;
    int iterations;
    bool success;
    
    // Solution quality
    double path_length;
    double path_smoothness;
    double min_obstacle_distance;
    double avg_obstacle_distance;
    
    // Energy metrics
    double energy_consumption;
    double energy_efficiency;  // distance/energy
    
    // Algorithm-specific metrics
    // A*
    int nodes_expanded;
    double memory_usage_kb;
    double optimality_ratio;
    
    // RRT
    int tree_size;
    double convergence_rate;
    double environment_adaptation;
    
    // MPC
    double tracking_error;
    double control_effort;
    double prediction_accuracy;
    
    // Constructor with default values
    BenchmarkResult(const std::string& algo_name = "unknown") 
        : algorithm_name(algo_name),
          variant("default"),
          computation_time_ms(0.0),
          iterations(0),
          success(false),
          path_length(0.0),
          path_smoothness(0.0),
          min_obstacle_distance(0.0),
          avg_obstacle_distance(0.0),
          energy_consumption(0.0),
          energy_efficiency(0.0),
          nodes_expanded(0),
          memory_usage_kb(0.0),
          optimality_ratio(0.0),
          tree_size(0),
          convergence_rate(0.0),
          environment_adaptation(0.0),
          tracking_error(0.0),
          control_effort(0.0),
          prediction_accuracy(0.0) {}
};

/**
 * @brief Container for test scenario
 */
struct TestScenario {
    std::string name;
    std::string description;
    Eigen::Vector2d start;
    Eigen::Vector2d goal;
    double start_heading;
    std::shared_ptr<GridMapRepresentation> map;
    std::vector<Eigen::Vector2d> obstacles;
    double difficulty_level;
    std::vector<std::vector<Eigen::Vector2d>> shipping_lanes;
    std::map<std::string, double> environment_factors;
};

/**
 * @brief Comprehensive algorithm benchmarking framework
 */
class AlgorithmBenchmarking {
public:
    AlgorithmBenchmarking() {
        initializeMetrics();
    }

    /**
     * @brief Initialize evaluation metrics
     */
    void initializeMetrics() {
        energy_metrics_ = std::make_shared<EnergyMetrics>("config/vessel_params.yaml");
        path_quality_ = std::make_shared<PathQuality>();
    }

    /**
     * @brief Generate test scenarios with varying complexity
     * @param num_scenarios Number of scenarios to generate
     * @param min_difficulty Minimum difficulty level (0-1)
     * @param max_difficulty Maximum difficulty level (0-1)
     * @return Vector of test scenarios
     */
    std::vector<TestScenario> generateTestScenarios(
        int num_scenarios = 10,
        double min_difficulty = 0.1,
        double max_difficulty = 0.9) {
        
        std::vector<TestScenario> scenarios;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> difficulty_dist(min_difficulty, max_difficulty);
        std::uniform_real_distribution<> pos_dist(-500.0, 500.0);
        
        for (int i = 0; i < num_scenarios; ++i) {
            TestScenario scenario;
            
            // Assign basic properties
            scenario.name = "scenario_" + std::to_string(i);
            scenario.difficulty_level = difficulty_dist(gen);
            
            // Create start and goal
            scenario.start = Eigen::Vector2d(pos_dist(gen), pos_dist(gen));
            scenario.goal = Eigen::Vector2d(pos_dist(gen), pos_dist(gen));
            scenario.start_heading = std::atan2(scenario.goal.y() - scenario.start.y(), 
                                               scenario.goal.x() - scenario.start.x());
            
            // Create map with complexity based on difficulty
            auto map = std::make_shared<GridMapRepresentation>(1.0, 1000.0, 1000.0, -500.0, -500.0);
            
            // Generate obstacles based on difficulty
            int num_obstacles = static_cast<int>(50 * scenario.difficulty_level);
            std::vector<Eigen::Vector2d> obstacles;
            
            for (int j = 0; j < num_obstacles; ++j) {
                Eigen::Vector2d obstacle(pos_dist(gen), pos_dist(gen));
                obstacles.push_back(obstacle);
                
                // Add obstacle to map
                map->setOccupied(obstacle.x(), obstacle.y(), true);
                
                // Add some surrounding cells as occupied for larger obstacles
                if (j % 3 == 0) {  // Make every 3rd obstacle larger
                    for (int dx = -2; dx <= 2; ++dx) {
                        for (int dy = -2; dy <= 2; ++dy) {
                            if (dx == 0 && dy == 0) continue;
                            map->setOccupied(obstacle.x() + dx, obstacle.y() + dy, true);
                        }
                    }
                }
            }
            
            // Make sure start and goal are not on obstacles
            map->setOccupied(scenario.start.x(), scenario.start.y(), false);
            map->setOccupied(scenario.goal.x(), scenario.goal.y(), false);
            
            // Compute distance grid for fast collision checking
            map->computeDistanceGrid();
            
            // Add environmental forces (currents) based on difficulty
            for (int x = -500; x < 500; x += 50) {
                for (int y = -500; y < 500; y += 50) {
                    double magnitude = 2.0 * scenario.difficulty_level * (std::sin(x/100.0) + std::cos(y/100.0));
                    double direction = std::atan2(y, x);
                    map->setForceField(x, y, magnitude, direction);
                }
            }
            
            // Store map and obstacles
            scenario.map = map;
            scenario.obstacles = obstacles;
            
            // Add shipping lanes if applicable
            if (i % 3 == 0) {  // Add shipping lanes to every 3rd scenario
                std::vector<Eigen::Vector2d> lane1, lane2;
                
                // Create two crossing shipping lanes
                for (int j = -400; j <= 400; j += 20) {
                    lane1.push_back(Eigen::Vector2d(j, j * 0.5));
                    lane2.push_back(Eigen::Vector2d(j, -j * 0.5));
                }
                
                scenario.shipping_lanes.push_back(lane1);
                scenario.shipping_lanes.push_back(lane2);
            }
            
            // Add environment factors
            scenario.environment_factors["wave_height"] = 1.0 * scenario.difficulty_level;
            scenario.environment_factors["wind_speed"] = 5.0 * scenario.difficulty_level;
            scenario.environment_factors["current_strength"] = 2.0 * scenario.difficulty_level;
            
            // Add description
            scenario.description = "Test scenario with difficulty " + 
                                  std::to_string(scenario.difficulty_level) +
                                  " and " + std::to_string(num_obstacles) + " obstacles";
            
            scenarios.push_back(scenario);
        }
        
        return scenarios;
    }

    /**
     * @brief Load test scenarios from files
     * @param directory Directory containing scenario files
     * @return Vector of test scenarios
     */
    std::vector<TestScenario> loadTestScenarios(const std::string& directory) {
        std::vector<TestScenario> scenarios;
        
        // Check if directory exists
        if (!std::filesystem::exists(directory)) {
            std::cerr << "Directory does not exist: " << directory << std::endl;
            return scenarios;
        }
        
        // Iterate through files in directory
        for (const auto& entry : std::filesystem::directory_iterator(directory)) {
            if (entry.path().extension() == ".scenario") {
                // TODO: Implement scenario file parsing
                TestScenario scenario;
                // Parse scenario file
                scenario.name = entry.path().stem().string();
                
                scenarios.push_back(scenario);
            }
        }
        
        return scenarios;
    }

    /**
     * @brief Benchmark A* algorithm
     * @param scenario Test scenario
     * @param energy_aware Whether to use energy-aware mode
     * @return Benchmark results
     */
    BenchmarkResult benchmarkAStar(
        const TestScenario& scenario,
        bool energy_aware = true) {
        
        BenchmarkResult result("A*");
        result.variant = energy_aware ? "energy_aware" : "standard";
        result.parameters["energy_weight"] = energy_aware ? 0.3 : 0.0;
        
        // Create A* planner
        auto dynamics = std::make_shared<ASVDynamics>(500.0, 400.0, 5.0);
        auto a_star = std::make_shared<EnergyAwareAStar>(1.0);
        a_star->setMap(std::dynamic_pointer_cast<MultiResolutionGrid>(scenario.map));
        a_star->setDynamicsModel(dynamics);
        a_star->setStartAndGoal(scenario.start, scenario.goal);
        a_star->setEnergyWeight(result.parameters["energy_weight"]);
        
        // Start timing
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Plan path
        std::vector<Eigen::Vector2d> path;
        try {
            path = a_star->planPath();
            result.success = !path.empty();
        } catch (const std::exception& e) {
            result.success = false;
            std::cerr << "A* planning failed: " << e.what() << std::endl;
        }
        
        // End timing
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end_time - start_time;
        result.computation_time_ms = duration.count();
        
        // If planning successful, evaluate path
        if (result.success) {
            // Calculate path metrics
            result.path_length = 0.0;
            for (size_t i = 1; i < path.size(); ++i) {
                result.path_length += (path[i] - path[i-1]).norm();
            }
            
            // Calculate path smoothness
            result.path_smoothness = path_quality_->calculateSmoothness(path);
            
            // Calculate obstacle distance
            result.min_obstacle_distance = std::numeric_limits<double>::max();
            double total_distance = 0.0;
            for (const auto& point : path) {
                double distance = scenario.map->getDistanceToClosestObstacle(point.x(), point.y());
                result.min_obstacle_distance = std::min(result.min_obstacle_distance, distance);
                total_distance += distance;
            }
            result.avg_obstacle_distance = total_distance / path.size();
            
            // Calculate energy metrics
            result.energy_consumption = a_star->getPathEnergy();
            result.energy_efficiency = result.path_length / result.energy_consumption;
            
            // A*-specific metrics
            // TODO: Extract from A* implementation or estimate
            result.nodes_expanded = 1000;  // Placeholder
            result.optimality_ratio = 1.0;  // Placeholder
        }
        
        return result;
    }

    /**
     * @brief Benchmark RRT planner
     * @param scenario Test scenario
     * @param energy_aware Whether to use energy-aware mode
     * @return Benchmark results
     */
    BenchmarkResult benchmarkRRT(
        const TestScenario& scenario,
        bool energy_aware = true) {
        
        BenchmarkResult result("RRT");
        result.variant = energy_aware ? "energy_aware" : "standard";
        
        // Create RRT planner
        auto rrt = std::make_shared<RRTPlanner>();
        rrt->setEnergyAware(energy_aware);
        rrt->setMaxIterations(5000);
        rrt->setStepSize(3.0);
        rrt->setGoalBias(0.1);
        rrt->setObstacleBias(0.05);
        rrt->setPathBias(0.3);
        rrt->setRewireRadius(10.0);
        rrt->setMaxPlanningTime(1.0);  // 1 second timeout
        rrt->setObstacleClearance(2.0);
        
        // Store parameters
        result.parameters["goal_bias"] = 0.1;
        result.parameters["step_size"] = 3.0;
        result.parameters["max_iterations"] = 5000;
        
        // Convert map for RRT
        EnvironmentMap env_map;
        // TODO: Properly convert GridMapRepresentation to EnvironmentMap
        
        rrt->updateEnvironmentMap(env_map);
        
        // Setup planning parameters
        PlanningParameters params;
        params.start = Point2D(scenario.start.x(), scenario.start.y());
        params.goal = Point2D(scenario.goal.x(), scenario.goal.y());
        params.start_heading = scenario.start_heading;
        
        // Start timing
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Plan path
        PlanningResult plan_result = rrt->plan(params);
        result.success = plan_result.success;
        
        // End timing
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end_time - start_time;
        result.computation_time_ms = duration.count();
        
        // Extract metrics from result
        if (result.success) {
            // Convert path to Eigen vectors
            std::vector<Eigen::Vector2d> path;
            for (const auto& point : plan_result.path) {
                path.push_back(Eigen::Vector2d(point.x, point.y));
            }
            
            // Calculate path metrics
            result.path_length = plan_result.path_length;
            result.iterations = plan_result.iterations;
            
            // Calculate path smoothness
            result.path_smoothness = path_quality_->calculateSmoothness(path);
            
            // Calculate obstacle distance
            result.min_obstacle_distance = std::numeric_limits<double>::max();
            double total_distance = 0.0;
            for (const auto& point : path) {
                double distance = scenario.map->getDistanceToClosestObstacle(point.x(), point.y());
                result.min_obstacle_distance = std::min(result.min_obstacle_distance, distance);
                total_distance += distance;
            }
            result.avg_obstacle_distance = total_distance / path.size();
            
            // Energy metrics
            result.energy_consumption = plan_result.energy_cost;
            result.energy_efficiency = result.path_length / result.energy_consumption;
            
            // RRT-specific metrics
            result.tree_size = plan_result.iterations;  // Approximation
            result.convergence_rate = result.path_length / result.iterations;
            // Environment adaptation measurement would require more detailed analysis
        }
        
        return result;
    }

    /**
     * @brief Benchmark MPC controller
     * @param scenario Test scenario
     * @param reference_path Reference path to follow
     * @param energy_aware Whether to use energy-aware mode
     * @return Benchmark results
     */
    BenchmarkResult benchmarkMPC(
        const TestScenario& scenario,
        const std::vector<Eigen::Vector2d>& reference_path,
        bool energy_aware = true) {
        
        BenchmarkResult result("MPC");
        result.variant = energy_aware ? "energy_aware" : "standard";
        
        // Create MPC configuration
        MPCConfig config;
        config.position_weight = 1.0;
        config.heading_weight = 0.5;
        config.velocity_weight = 0.2;
        config.energy_weight = energy_aware ? 0.5 : 0.0;
        config.control_weight = 0.1;
        config.control_rate_weight = 0.2;
        config.prediction_horizon = 10;
        config.dt = 0.1;
        config.max_thrust = 500.0;
        config.max_rudder = 0.5;
        config.energy_aware = energy_aware;
        
        // Store parameters
        result.parameters["prediction_horizon"] = config.prediction_horizon;
        result.parameters["energy_weight"] = config.energy_weight;
        result.parameters["dt"] = config.dt;
        
        // Create ASV dynamics model
        auto dynamics = std::make_shared<ASVDynamics>(500.0, 400.0, 5.0);
        
        // Create MPC controller
        auto mpc = std::make_shared<MPCController>(config, dynamics);
        
        // Create reference trajectory from path
        ReferenceTrajectory ref_trajectory;
        ref_trajectory.valid = true;
        
        double velocity = 2.0;  // m/s
        double time = 0.0;
        
        for (size_t i = 0; i < reference_path.size(); ++i) {
            // Calculate heading if not the last point
            double heading = 0.0;
            if (i < reference_path.size() - 1) {
                Eigen::Vector2d diff = reference_path[i+1] - reference_path[i];
                heading = std::atan2(diff.y(), diff.x());
            } else if (i > 0) {
                Eigen::Vector2d diff = reference_path[i] - reference_path[i-1];
                heading = std::atan2(diff.y(), diff.x());
            }
            
            // Create reference point
            ReferencePoint point(
                reference_path[i].x(),
                reference_path[i].y(),
                heading,
                velocity,
                0.0,  // Sway velocity
                0.0   // Yaw rate
            );
            
            ref_trajectory.points.push_back(point);
            ref_trajectory.timestamps.push_back(time);
            
            // Update time for next point (simple approximation)
            if (i < reference_path.size() - 1) {
                double distance = (reference_path[i+1] - reference_path[i]).norm();
                time += distance / velocity;
            }
        }
        
        // Set reference trajectory
        mpc->setReferenceTrajectory(ref_trajectory);
        
        // Create initial state and control
        Eigen::VectorXd initial_state(6);
        initial_state << reference_path[0].x(), reference_path[0].y(), 0.0, 0.0, 0.0, 0.0;
        
        ASVControl initial_control(0.0, 0.0);
        
        // Start timing
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Run MPC simulation
        double sim_time = 0.0;
        double sim_duration = 10.0;  // Simulate for 10 seconds
        double dt = config.dt;
        
        std::vector<Eigen::VectorXd> actual_trajectory;
        std::vector<ASVControl> control_inputs;
        double total_energy = 0.0;
        
        Eigen::VectorXd current_state = initial_state;
        ASVControl current_control = initial_control;
        
        while (sim_time < sim_duration) {
            // Solve MPC optimization
            MPCSolution solution = mpc->solve(current_state, current_control);
            
            if (!solution.solved) {
                break;
            }
            
            // Apply first control input
            current_control = solution.first_control;
            control_inputs.push_back(current_control);
            
            // Simulate system for one step
            Eigen::VectorXd next_state = dynamics->updateState(current_state, current_control.toVector(), dt);
            actual_trajectory.push_back(next_state);
            
            // Update energy consumption
            total_energy += current_control.calculateEnergy(dt);
            
            // Update state for next iteration
            current_state = next_state;
            sim_time += dt;
        }
        
        // End timing
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end_time - start_time;
        result.computation_time_ms = duration.count();
        
        // Calculate metrics
        result.success = !actual_trajectory.empty();
        
        if (result.success) {
            // Calculate path length
            result.path_length = 0.0;
            for (size_t i = 1; i < actual_trajectory.size(); ++i) {
                Eigen::Vector2d p1(actual_trajectory[i-1][0], actual_trajectory[i-1][1]);
                Eigen::Vector2d p2(actual_trajectory[i][0], actual_trajectory[i][1]);
                result.path_length += (p2 - p1).norm();
            }
            
            // Calculate tracking error
            double total_error = 0.0;
            for (size_t i = 0; i < actual_trajectory.size(); ++i) {
                // Find closest reference point
                double min_dist = std::numeric_limits<double>::max();
                for (const auto& ref_point : ref_trajectory.points) {
                    Eigen::Vector2d actual(actual_trajectory[i][0], actual_trajectory[i][1]);
                    Eigen::Vector2d ref(ref_point.x, ref_point.y);
                    double dist = (actual - ref).norm();
                    min_dist = std::min(min_dist, dist);
                }
                total_error += min_dist;
            }
            result.tracking_error = total_error / actual_trajectory.size();
            
            // Calculate control effort
            double total_effort = 0.0;
            for (const auto& control : control_inputs) {
                total_effort += std::abs(control.thrust) + std::abs(control.rudder);
            }
            result.control_effort = total_effort / control_inputs.size();
            
            // Energy metrics
            result.energy_consumption = total_energy;
            result.energy_efficiency = result.path_length / total_energy;
            
            // Convert trajectory to path for smoothness calculation
            std::vector<Eigen::Vector2d> path;
            for (const auto& state : actual_trajectory) {
                path.push_back(Eigen::Vector2d(state[0], state[1]));
            }
            result.path_smoothness = path_quality_->calculateSmoothness(path);
            
            // Calculate obstacle distance
            result.min_obstacle_distance = std::numeric_limits<double>::max();
            double total_distance = 0.0;
            for (const auto& state : actual_trajectory) {
                Eigen::Vector2d point(state[0], state[1]);
                double distance = scenario.map->getDistanceToClosestObstacle(point.x(), point.y());
                result.min_obstacle_distance = std::min(result.min_obstacle_distance, distance);
                total_distance += distance;
            }
            result.avg_obstacle_distance = total_distance / actual_trajectory.size();
        }
        
        return result;
    }

    /**
     * @brief Run benchmark for all algorithms on a test scenario
     * @param scenario Test scenario to benchmark
     * @param output_file Output CSV file path
     * @return Map of algorithm names to benchmark results
     */
    std::map<std::string, BenchmarkResult> runBenchmark(
        const TestScenario& scenario,
        const std::string& output_file = "") {
        
        std::map<std::string, BenchmarkResult> results;
        
        std::cout << "Running benchmark on scenario: " << scenario.name << std::endl;
        
        // Benchmark A* (standard and energy-aware)
        std::cout << "Benchmarking A* (standard)..." << std::endl;
        results["A*_standard"] = benchmarkAStar(scenario, false);
        
        std::cout << "Benchmarking A* (energy-aware)..." << std::endl;
        results["A*_energy_aware"] = benchmarkAStar(scenario, true);
        
        // Use A* path for RRT and MPC benchmarks
        std::vector<Eigen::Vector2d> reference_path;
        if (results["A*_energy_aware"].success) {
            // Use the path from energy-aware A* as reference
            // Note: Actual implementation would need to extract path from A* result
            auto a_star = std::make_shared<EnergyAwareAStar>(1.0);
            a_star->setMap(std::dynamic_pointer_cast<MultiResolutionGrid>(scenario.map));
            a_star->setStartAndGoal(scenario.start, scenario.goal);
            a_star->setEnergyWeight(0.3);
            
            try {
                reference_path = a_star->planPath();
            } catch (...) {
                // Fallback to a straight line if A* fails
                reference_path = {scenario.start, scenario.goal};
            }
        } else {
            // Fallback to a straight line if A* fails
            reference_path = {scenario.start, scenario.goal};
        }
        
        // Benchmark RRT (standard and energy-aware)
        std::cout << "Benchmarking RRT (standard)..." << std::endl;
        results["RRT_standard"] = benchmarkRRT(scenario, false);
        
        std::cout << "Benchmarking RRT (energy-aware)..." << std::endl;
        results["RRT_energy_aware"] = benchmarkRRT(scenario, true);
        
        // Benchmark MPC (standard and energy-aware)
        std::cout << "Benchmarking MPC (standard)..." << std::endl;
        results["MPC_standard"] = benchmarkMPC(scenario, reference_path, false);
        
        std::cout << "Benchmarking MPC (energy-aware)..." << std::endl;
        results["MPC_energy_aware"] = benchmarkMPC(scenario, reference_path, true);
        
        // Save results to file if requested
        if (!output_file.empty()) {
            saveResultsToCSV(results, output_file);
        }
        
        return results;
    }

    /**
     * @brief Save benchmark results to CSV file
     * @param results Benchmark results to save
     * @param filename Output CSV file path
     */
    void saveResultsToCSV(
        const std::map<std::string, BenchmarkResult>& results,
        const std::string& filename) {
        
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
            return;
        }
        
        // Write CSV header
        file << "Algorithm,Variant,Success,ComputationTime(ms),Iterations,PathLength(m),Smoothness,"
             << "MinObstacleDistance,AvgObstacleDistance,EnergyConsumption,EnergyEfficiency,"
             << "NodesExpanded,MemoryUsage(KB),OptimalityRatio,TreeSize,"
             << "ConvergenceRate,TrackingError,ControlEffort" << std::endl;
        
        // Write results
        for (const auto& [name, result] : results) {
            file << result.algorithm_name << ","
                 << result.variant << ","
                 << (result.success ? "true" : "false") << ","
                 << result.computation_time_ms << ","
                 << result.iterations << ","
                 << result.path_length << ","
                 << result.path_smoothness << ","
                 << result.min_obstacle_distance << ","
                 << result.avg_obstacle_distance << ","
                 << result.energy_consumption << ","
                 << result.energy_efficiency << ","
                 << result.nodes_expanded << ","
                 << result.memory_usage_kb << ","
                 << result.optimality_ratio << ","
                 << result.tree_size << ","
                 << result.convergence_rate << ","
                 << result.tracking_error << ","
                 << result.control_effort << std::endl;
        }
        
        file.close();
        std::cout << "Results saved to " << filename << std::endl;
    }

private:
    std::shared_ptr<EnergyMetrics> energy_metrics_;
    std::shared_ptr<PathQuality> path_quality_;
};

} // namespace asv_planning

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("algorithm_benchmarking");
    
    asv_planning::AlgorithmBenchmarking benchmarking;
    
    // Generate test scenarios
    std::cout << "Generating test scenarios..." << std::endl;
    auto scenarios = benchmarking.generateTestScenarios(3, 0.2, 0.8);
    std::cout << "Generated " << scenarios.size() << " test scenarios." << std::endl;
    
    // Create output directory
    std::filesystem::create_directories("results");
    
    // Run benchmarks for each scenario
    for (size_t i = 0; i < scenarios.size(); ++i) {
        const auto& scenario = scenarios[i];
        std::string output_file = "results/scenario_" + std::to_string(i) + "_results.csv";
        
        std::cout << "\nBenchmarking scenario " << i << ": " << scenario.name << std::endl;
        std::cout << "Difficulty: " << scenario.difficulty_level << std::endl;
        std::cout << "Description: " << scenario.description << std::endl;
        
        auto results = benchmarking.runBenchmark(scenario, output_file);
    }
    
    std::cout << "\nAll benchmarks completed." << std::endl;
    
    rclcpp::shutdown();
    return 0;
} 