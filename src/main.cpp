#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

/**
 * @brief Main entry point for the ASV energy-aware planning system
 * 
 * This file serves as the main entry point for the ASV energy-aware planning system.
 * It initializes the ROS2 system and launches the planning nodes.
 */
int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Create the main planning node
    auto planning_node = std::make_shared<rclcpp::Node>("asv_energy_aware_planning");
    RCLCPP_INFO(planning_node->get_logger(), "Starting ASV Energy-Aware Planning System");
    
    // Create executor to handle callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(planning_node);
    
    // Start executor
    executor.spin();
    
    // Shutdown
    rclcpp::shutdown();
    
    return 0;
} 