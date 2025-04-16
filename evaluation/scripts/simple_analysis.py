#!/usr/bin/env python3

import os
import json
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from pathlib import Path

def load_results(results_dir):
    """Load benchmark results from the specified directory."""
    results = []
    for scenario_dir in os.listdir(results_dir):
        scenario_path = os.path.join(results_dir, scenario_dir)
        if os.path.isdir(scenario_path):
            for result_file in os.listdir(scenario_path):
                if result_file.endswith(".json"):
                    file_path = os.path.join(scenario_path, result_file)
                    with open(file_path, 'r') as f:
                        result_data = json.load(f)
                        results.append(result_data)
    return results

def calculate_statistics(results):
    """Calculate statistics for each algorithm and scenario."""
    stats = {}
    
    for result in results:
        algorithm = result['algorithm']
        scenario = result['scenario']
        metrics = result['metrics']
        
        key = (algorithm, scenario)
        if key not in stats:
            stats[key] = {}
        
        for metric_name, metric_values in metrics.items():
            if metric_name != 'success_rate':  # success_rate is already a scalar
                stats[key][f"{metric_name}_mean"] = np.mean(metric_values)
                stats[key][f"{metric_name}_std"] = np.std(metric_values)
            else:
                stats[key][metric_name] = metric_values
    
    return stats

def plot_metric_by_scenario_type(results, metric, output_dir):
    """Create plots comparing algorithms across different scenario types."""
    scenario_types = ['simple', 'complex', 'real_world']
    obstacle_densities = [0.1, 0.5, 0.9]
    algorithms = list(set(result['algorithm'] for result in results))
    
    os.makedirs(output_dir, exist_ok=True)
    
    for scenario_type in scenario_types:
        plt.figure(figsize=(12, 8))
        
        for algorithm in sorted(algorithms):
            x_values = []
            y_values = []
            std_values = []
            
            for density in obstacle_densities:
                scenario = f"{scenario_type}_{density}"
                for result in results:
                    if result['algorithm'] == algorithm and result['scenario'] == scenario:
                        if metric == 'success_rate':
                            y_values.append(result['metrics'][metric])
                            std_values.append(0)  # No std for success rate
                        else:
                            metric_values = result['metrics'][metric]
                            y_values.append(np.mean(metric_values))
                            std_values.append(np.std(metric_values))
                        x_values.append(density)
                        break
            
            plt.errorbar(x_values, y_values, yerr=std_values, marker='o', label=algorithm)
        
        plt.xlabel('Obstacle Density')
        plt.ylabel(metric.replace('_', ' ').title())
        plt.title(f'{metric.replace("_", " ").title()} vs Obstacle Density ({scenario_type.replace("_", " ").title()} Scenario)')
        plt.grid(True)
        plt.legend()
        
        # Save the figure
        output_path = os.path.join(output_dir, f"{scenario_type}_{metric}.png")
        plt.savefig(output_path)
        plt.close()
        
        print(f"Generated plot: {output_path}")

def plot_metrics_by_algorithm(results, output_dir):
    """Create plots comparing algorithm performance across different metrics."""
    metrics = ['path_length', 'computation_time', 'energy_consumption', 'success_rate']
    
    for metric in metrics:
        plot_metric_by_scenario_type(results, metric, output_dir)

def generate_performance_summary(results, output_dir):
    """Generate a performance summary for each algorithm across all scenarios."""
    algorithms = list(set(result['algorithm'] for result in results))
    metrics = ['path_length', 'computation_time', 'energy_consumption', 'success_rate']
    
    summary_path = os.path.join(output_dir, "performance_summary.txt")
    
    with open(summary_path, 'w') as f:
        f.write("Performance Summary\n")
        f.write("===================\n\n")
        
        for algorithm in sorted(algorithms):
            f.write(f"{algorithm}\n")
            f.write("-" * len(algorithm) + "\n")
            
            for metric in metrics:
                f.write(f"\n{metric.replace('_', ' ').title()}:\n")
                
                scenario_results = {}
                for result in results:
                    if result['algorithm'] == algorithm:
                        scenario = result['scenario']
                        if metric == 'success_rate':
                            value = result['metrics'][metric]
                            scenario_results[scenario] = value
                        else:
                            metric_values = result['metrics'][metric]
                            mean_value = np.mean(metric_values)
                            std_value = np.std(metric_values)
                            scenario_results[scenario] = (mean_value, std_value)
                
                for scenario, value in sorted(scenario_results.items()):
                    if metric == 'success_rate':
                        f.write(f"  {scenario}: {value:.2f}\n")
                    else:
                        mean, std = value
                        f.write(f"  {scenario}: {mean:.2f} ± {std:.2f}\n")
                
            f.write("\n\n")
    
    print(f"Generated performance summary: {summary_path}")

def main():
    parser = argparse.ArgumentParser(description='Analyze benchmark results')
    parser.add_argument('--results_dir', type=str, required=True,
                       help='Directory containing benchmark results')
    parser.add_argument('--output_dir', type=str, required=True,
                       help='Directory to store analysis outputs')
    args = parser.parse_args()
    
    # Ensure output directory exists
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Load results
    print(f"Loading results from {args.results_dir}")
    results = load_results(args.results_dir)
    print(f"Loaded {len(results)} results")
    
    # Calculate statistics
    stats = calculate_statistics(results)
    
    # Generate plots
    print("Generating plots...")
    plot_metrics_by_algorithm(results, args.output_dir)
    
    # Generate summary
    print("Generating performance summary...")
    generate_performance_summary(results, args.output_dir)
    
    print(f"Analysis complete. Results saved to {args.output_dir}")

if __name__ == "__main__":
    main() 