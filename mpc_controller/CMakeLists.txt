cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)
get_filename_component(PROJECT_ROOT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
include(${PROJECT_ROOT_DIR}/3rd/conanbuildinfo.cmake)
conan_basic_setup()

# Find required ROS packages
find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
  base_local_planner
  controller
)

# Find additional required packages
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)  # Add this line

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /usr/local/include/osqp  # Add OSQP include directory
)

add_library(${PROJECT_NAME}
  src/mpc_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CONAN_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CONAN_LIBS}
  osqp  # Add OSQP library
)
