cmake_minimum_required(VERSION 3.8)
project(asv_energy_aware_planning)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ASVState.msg"
  "msg/EnergyMetrics.msg"
  "msg/EnvironmentalForce.msg"
  "msg/EnvironmentMap.msg"
  "msg/PlannerStatus.msg"
  "msg/PlanningPath.msg"
  "srv/ControllerCommand.srv"
  "srv/EnvironmentQuery.srv"
  "srv/GlobalPathRequest.srv"
  "srv/LocalPathAdaptation.srv"
  "srv/MapUpdate.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Build executables
add_executable(global_planner_node 
  src/global_planner/global_planner_node.cpp
)
target_link_libraries(global_planner_node
  Eigen3::Eigen
)
ament_target_dependencies(global_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
rosidl_target_interfaces(global_planner_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(local_planner_node 
  src/local_planner/local_planner_node.cpp
)
target_link_libraries(local_planner_node
  Eigen3::Eigen
)
ament_target_dependencies(local_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
rosidl_target_interfaces(local_planner_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(mpc_controller_node 
  src/mpc_controller/mpc_controller_node.cpp
)
target_link_libraries(mpc_controller_node
  Eigen3::Eigen
)
ament_target_dependencies(mpc_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
rosidl_target_interfaces(mpc_controller_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(environment_node 
  src/environment_representation/environment_node.cpp
)
target_link_libraries(environment_node
  Eigen3::Eigen
)
ament_target_dependencies(environment_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
rosidl_target_interfaces(environment_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(planning_coordinator_node 
  src/planning_hierarchy/planning_coordinator_node.cpp
)
ament_target_dependencies(planning_coordinator_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
rosidl_target_interfaces(planning_coordinator_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(path_visualizer_node 
  src/visualization/path_visualizer_node.cpp
)
target_link_libraries(path_visualizer_node
  Eigen3::Eigen
)
ament_target_dependencies(path_visualizer_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
rosidl_target_interfaces(path_visualizer_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Add evaluation executables
add_executable(algorithm_benchmarking
  evaluation/src/algorithm_benchmarking.cpp
)

target_include_directories(algorithm_benchmarking PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/environment_representation/include
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_hierarchy/global_planner/include
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_hierarchy/local_planner/include
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_hierarchy/trajectory_controller/include
  ${CMAKE_CURRENT_SOURCE_DIR}/evaluation/include
)

target_link_libraries(algorithm_benchmarking
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${OSQP_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
)

# Install executables
install(TARGETS
  global_planner_node
  local_planner_node
  mpc_controller_node
  environment_node
  planning_coordinator_node
  path_visualizer_node
  algorithm_benchmarking
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install evaluation scripts
install(DIRECTORY evaluation/scripts/
  DESTINATION lib/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.py"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(FILES evaluation/run_benchmarks.sh
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Install evaluation results directory structure
install(DIRECTORY DESTINATION lib/${PROJECT_NAME}/results)
install(DIRECTORY DESTINATION lib/${PROJECT_NAME}/analysis)
install(DIRECTORY DESTINATION lib/${PROJECT_NAME}/test_environments)

# Add evaluation target
add_custom_target(run_benchmarks
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/evaluation/run_benchmarks.sh
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/evaluation
  COMMENT "Running algorithm benchmarks"
)

# Export dependencies
ament_export_dependencies(rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs eigen3_cmake_module Eigen3)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_package() 