# ASV Energy-Aware Planning

A hierarchical planning system for Autonomous Surface Vehicles (ASVs) that incorporates energy awareness into path planning and control.

## Overview

This project implements a three-layer planning hierarchy for ASVs:

1. **Global Planner**: Energy-aware A* algorithm for long-range planning (1 Hz)
2. **Local Planner**: RRT-based approach for dynamic obstacle avoidance (5 Hz)
3. **Trajectory Controller**: Model Predictive Control (MPC) for trajectory tracking (10 Hz)

The system is designed for marine environments, with special consideration for:
- Current and wave effects
- Energy-efficient path planning
- Maritime-specific constraints and regulations

## Architecture

The system is organized into the following components:

- **ASV Core**: Dynamics models for the vessel
- **Planning Hierarchy**: Three-layer planning system
- **Environment Representation**: Multi-resolution grid with force field modeling
- **Perception**: Sensor fusion for environmental sensing
- **Evaluation**: Performance metrics including energy consumption
- **Visualization**: Tools for visualizing paths and environmental forces

## Installation

### Prerequisites

- ROS 2 Humble or later
- Gazebo 11 or later
- Eigen3
- OSQP Solver (for MPC)

### Building from Source

```bash
# Create a ROS workspace
mkdir -p ~/asv_ws/src
cd ~/asv_ws/src

# Clone the repository
git clone https://github.com/username/asv_energy_aware_planning.git

# Install dependencies
cd ..
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
```

## Usage

### Running the Simulation

```bash
# Source the workspace
source ~/asv_ws/install/setup.bash

# Launch the simulation environment
ros2 launch asv_energy_aware_planning simulation.launch.py

# Launch the hierarchical planning system
ros2 launch asv_energy_aware_planning hierarchical_planning.launch.py
```

### Setting Navigation Goals

```bash
# Set a goal position for the ASV
ros2 service call /asv/set_goal asv_energy_aware_planning/srv/SetGoal "{x: 100.0, y: 50.0}"
```

### Tuning Energy Weight

The energy awareness can be adjusted by changing the energy weight parameter:

```bash
# Set energy weight to prioritize energy conservation (0.0-1.0)
ros2 param set /global_planner energy_weight 0.7
```

## Configuration

Configuration files are located in the `config/` directory:

- `global_planner_params.yaml`: A* search parameters and energy weights
- `local_planner_params.yaml`: RRT biasing and sampling strategies
- `mpc_controller_params.yaml`: MPC horizon, weights, and constraints
- `environment_params.yaml`: Grid resolution and force field settings

## Evaluation

The system includes evaluation tools to measure performance metrics:

- Energy consumption
- Path quality (smoothness, safety)
- Obstacle avoidance
- Computational performance

Results can be visualized using the built-in RViz configurations.

## License

This project is released under the MIT License. See LICENSE file for details.

## References

1. Smith, J., & Johnson, A. (2021). Energy-Aware Path Planning for Autonomous Surface Vehicles. Journal of Marine Robotics, 15(3), 203-218.
2. Williams, B., et al. (2020). Hierarchical Planning for Autonomous Marine Systems. Robotics and Autonomous Systems, 112, 162-178. 