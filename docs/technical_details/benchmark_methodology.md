# ASV Energy-Aware Planning Benchmark Methodology

This document details the technical approach used in benchmarking ASV path planning algorithms, with special focus on the Energy-Aware Planning algorithm.

## Benchmark Architecture

The benchmark system consists of three main components:

1. **Scenario Generation**: Creates test environments with varying obstacle densities
2. **Algorithm Execution**: Runs each path planning algorithm on the scenarios
3. **Results Analysis**: Processes raw results and generates comparative visualizations

## Scenario Types

Three types of scenarios are used to evaluate algorithm performance:

### Simple Scenarios
- Basic grid-based environments
- Obstacles placed in regular patterns
- Tests basic pathfinding capabilities

### Complex Scenarios
- Irregular obstacle patterns
- Narrow passages
- Tests complex navigational decision-making

### Real-World Scenarios
- Based on actual maritime environments
- Includes currents, wind effects, and varying water depths
- Tests performance in realistic conditions

## Obstacle Density Variations

Each scenario type is tested with three obstacle densities:
- **Low (0.1)**: 10% of navigable space contains obstacles
- **Medium (0.5)**: 50% of navigable space contains obstacles
- **High (0.9)**: 90% of navigable space contains obstacles

## Evaluation Metrics

### Path Length
- Measured in meters
- Calculated as the sum of Euclidean distances between consecutive waypoints
- Lower values indicate shorter paths

### Computation Time
- Measured in milliseconds
- Time taken for the algorithm to compute a complete path
- Lower values indicate faster algorithms

### Energy Consumption
- Modeled using a physics-based approach
- Accounts for:
  - Distance traveled
  - Acceleration/deceleration events
  - Environmental factors (currents, wind)
  - Vehicle dynamics
- Lower values indicate more energy-efficient paths

### Success Rate
- Percentage of trials where a valid path was found
- Calculated over multiple runs with randomly generated start/goal positions
- Higher values indicate more robust algorithms

## Statistical Analysis

- Each scenario is run 10 times with different random seeds
- Results include mean and standard deviation for each metric
- Statistical significance is assessed using paired t-tests (p < 0.05)

## Visualization Methodology

The analysis generates multiple visualization types:

1. **Line plots**: Show performance trends across obstacle densities
2. **Bar charts**: Compare algorithms on specific metrics
3. **Error bars**: Indicate variability in performance
4. **Heat maps**: Visualize performance across scenario combinations

## Energy Model Details

The energy consumption model is based on:

```
E = Σ (Pt + Pr + Pe)dt

Where:
- Pt = thrust power
- Pr = resistance power
- Pe = environmental factors
- dt = time step
```

This model accounts for both the energy required to follow the path and the additional energy needed for obstacle avoidance maneuvers. 