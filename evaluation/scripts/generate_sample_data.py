#!/usr/bin/env python3

import os
import numpy as np
import json
import argparse
import random
from pathlib import Path

def generate_sample_data(output_dir):
    """Generate sample benchmark results for demonstration."""
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Define algorithms
    algorithms = ['AStar', 'Dijkstra', 'RRT', 'EnergyAware']
    
    # Define scenarios
    scenario_types = ['simple', 'complex', 'real_world']
    obstacle_densities = [0.1, 0.5, 0.9]
    
    # Generate random results for each scenario and algorithm
    for scenario_type in scenario_types:
        for density in obstacle_densities:
            scenario_name = f"{scenario_type}_{density}"
            scenario_dir = os.path.join(output_dir, scenario_name)
            os.makedirs(scenario_dir, exist_ok=True)
            
            for algorithm in algorithms:
                # Generate random metrics
                num_runs = 10
                path_length = np.random.normal(100 * (1 + obstacle_densities.index(density)), 
                                             20, num_runs)
                computation_time = np.random.normal(50 * (1 + algorithms.index(algorithm) * 0.5), 
                                                 10, num_runs)
                energy_consumption = np.random.normal(200 * (1 + obstacle_densities.index(density) * 0.3), 
                                                   40, num_runs)
                success_rate = min(1.0, max(0.5, np.random.normal(0.9 - (obstacle_densities.index(density) * 0.1), 0.1)))
                
                # Make the EnergyAware algorithm perform better in energy metrics
                if algorithm == 'EnergyAware':
                    energy_consumption = energy_consumption * 0.7
                
                # Create results dictionary
                results = {
                    "algorithm": algorithm,
                    "scenario": scenario_name,
                    "metrics": {
                        "path_length": path_length.tolist(),
                        "computation_time": computation_time.tolist(),
                        "energy_consumption": energy_consumption.tolist(),
                        "success_rate": success_rate
                    }
                }
                
                # Save to JSON file
                result_file = os.path.join(scenario_dir, f"{algorithm}.json")
                with open(result_file, 'w') as f:
                    json.dump(results, f, indent=2)
                
                print(f"Generated sample results for {algorithm} on {scenario_name}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate sample benchmark results')
    parser.add_argument('--output_dir', type=str, 
                        default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'results'),
                        help='Directory to store the generated results')
    args = parser.parse_args()
    
    # Print the output directory for debugging
    print(f"Writing results to: {args.output_dir}")
    
    generate_sample_data(args.output_dir)
    print(f"Sample data generation complete. Results saved to {args.output_dir}") 