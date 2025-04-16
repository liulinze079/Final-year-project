#!/usr/bin/env python3

import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import argparse

def load_results(results_dir):
    """
    Load all benchmark result CSV files from the given directory
    
    Args:
        results_dir: Directory containing benchmark CSV files
        
    Returns:
        DataFrame containing all results
    """
    all_results = []
    
    # Find all CSV files in the results directory
    csv_files = glob.glob(os.path.join(results_dir, "*.csv"))
    
    if not csv_files:
        print(f"No CSV files found in {results_dir}")
        return None
    
    # Load each CSV file and add scenario info
    for file in csv_files:
        base_name = os.path.basename(file)
        scenario_id = base_name.split("_")[1]  # Extract scenario ID from filename
        
        df = pd.read_csv(file)
        df['ScenarioID'] = scenario_id
        
        all_results.append(df)
    
    # Combine all results
    if all_results:
        combined_df = pd.concat(all_results, ignore_index=True)
        return combined_df
    
    return None

def calculate_statistics(df):
    """
    Calculate summary statistics for benchmark results
    
    Args:
        df: DataFrame with benchmark results
        
    Returns:
        DataFrame with statistics
    """
    # Group by algorithm and variant
    grouped = df.groupby(['Algorithm', 'Variant'])
    
    # Calculate statistics
    stats_df = grouped.agg({
        'Success': 'mean',
        'ComputationTime(ms)': ['mean', 'std', 'min', 'max'],
        'PathLength(m)': ['mean', 'std'],
        'Smoothness': ['mean', 'std'],
        'MinObstacleDistance': ['mean', 'std'],
        'EnergyConsumption': ['mean', 'std'],
        'EnergyEfficiency': ['mean', 'std']
    })
    
    # Flatten the multi-level columns
    stats_df.columns = [f'{col[0]}_{col[1]}' if col[1] else col[0] for col in stats_df.columns]
    
    return stats_df

def run_statistical_tests(df):
    """
    Run statistical tests to compare algorithm variants
    
    Args:
        df: DataFrame with benchmark results
        
    Returns:
        DataFrame with test results
    """
    test_results = []
    
    # For each algorithm, compare standard vs energy-aware variants
    for algo in df['Algorithm'].unique():
        # Filter by algorithm
        algo_df = df[df['Algorithm'] == algo]
        
        # Get standard and energy-aware results
        standard = algo_df[algo_df['Variant'] == 'standard']
        energy_aware = algo_df[algo_df['Variant'] == 'energy_aware']
        
        if len(standard) == 0 or len(energy_aware) == 0:
            continue
        
        # Run t-tests for different metrics
        metrics = [
            'ComputationTime(ms)', 
            'PathLength(m)', 
            'Smoothness', 
            'MinObstacleDistance', 
            'EnergyConsumption',
            'EnergyEfficiency'
        ]
        
        for metric in metrics:
            # Skip if any group has no successful runs
            if standard['Success'].sum() == 0 or energy_aware['Success'].sum() == 0:
                continue
                
            # Filter only successful runs
            standard_data = standard[standard['Success'] == True][metric].dropna()
            energy_data = energy_aware[energy_aware['Success'] == True][metric].dropna()
            
            if len(standard_data) < 2 or len(energy_data) < 2:
                # Not enough samples for t-test
                continue
                
            # Run t-test
            t_stat, p_val = stats.ttest_ind(standard_data, energy_data)
            
            # Calculate effect size (Cohen's d)
            mean_diff = energy_data.mean() - standard_data.mean()
            pooled_std = np.sqrt((standard_data.std()**2 + energy_data.std()**2) / 2)
            effect_size = mean_diff / pooled_std if pooled_std != 0 else 0
            
            # Add result
            test_results.append({
                'Algorithm': algo,
                'Metric': metric,
                't_statistic': t_stat,
                'p_value': p_val,
                'effect_size': effect_size,
                'significant': p_val < 0.05
            })
    
    return pd.DataFrame(test_results)

def plot_comparison(df, output_dir):
    """
    Create comparison plots for algorithm variants
    
    Args:
        df: DataFrame with benchmark results
        output_dir: Directory to save plots
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Plot computation time
    plt.figure(figsize=(12, 8))
    sns.set_style("whitegrid")
    
    ax = sns.boxplot(x='Algorithm', y='ComputationTime(ms)', hue='Variant', data=df)
    plt.title('Computation Time Comparison')
    plt.ylabel('Time (ms)')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'computation_time.png'))
    
    # Plot energy consumption
    plt.figure(figsize=(12, 8))
    
    # Filter successful runs
    success_df = df[df['Success'] == True]
    
    ax = sns.boxplot(x='Algorithm', y='EnergyConsumption', hue='Variant', data=success_df)
    plt.title('Energy Consumption Comparison')
    plt.ylabel('Energy Consumption')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'energy_consumption.png'))
    
    # Plot energy efficiency
    plt.figure(figsize=(12, 8))
    
    ax = sns.boxplot(x='Algorithm', y='EnergyEfficiency', hue='Variant', data=success_df)
    plt.title('Energy Efficiency Comparison')
    plt.ylabel('Energy Efficiency (distance/energy)')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'energy_efficiency.png'))
    
    # Plot path smoothness
    plt.figure(figsize=(12, 8))
    
    ax = sns.boxplot(x='Algorithm', y='Smoothness', hue='Variant', data=success_df)
    plt.title('Path Smoothness Comparison')
    plt.ylabel('Smoothness')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'path_smoothness.png'))
    
    # Plot min obstacle distance
    plt.figure(figsize=(12, 8))
    
    ax = sns.boxplot(x='Algorithm', y='MinObstacleDistance', hue='Variant', data=success_df)
    plt.title('Minimum Obstacle Distance Comparison')
    plt.ylabel('Distance')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'min_obstacle_distance.png'))
    
    # Plot success rate
    plt.figure(figsize=(12, 8))
    
    # Calculate success rate by algorithm and variant
    success_rate = df.groupby(['Algorithm', 'Variant'])['Success'].mean().reset_index()
    
    ax = sns.barplot(x='Algorithm', y='Success', hue='Variant', data=success_rate)
    plt.title('Success Rate Comparison')
    plt.ylabel('Success Rate')
    plt.ylim(0, 1)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'success_rate.png'))

def plot_scenario_comparison(df, output_dir):
    """
    Plot performance across different scenarios
    
    Args:
        df: DataFrame with benchmark results
        output_dir: Directory to save plots
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Filter successful runs
    success_df = df[df['Success'] == True]
    
    # Plot energy consumption by scenario
    plt.figure(figsize=(14, 8))
    
    # Group by scenario, algorithm, and variant
    pivot_df = success_df.pivot_table(
        index='ScenarioID', 
        columns=['Algorithm', 'Variant'], 
        values='EnergyConsumption'
    )
    
    ax = pivot_df.plot(kind='bar')
    plt.title('Energy Consumption by Scenario')
    plt.ylabel('Energy Consumption')
    plt.xlabel('Scenario ID')
    plt.legend(title='Algorithm + Variant')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'energy_by_scenario.png'))
    
    # Plot computation time by scenario
    plt.figure(figsize=(14, 8))
    
    # Group by scenario, algorithm, and variant
    pivot_df = df.pivot_table(
        index='ScenarioID', 
        columns=['Algorithm', 'Variant'], 
        values='ComputationTime(ms)'
    )
    
    ax = pivot_df.plot(kind='bar')
    plt.title('Computation Time by Scenario')
    plt.ylabel('Time (ms)')
    plt.xlabel('Scenario ID')
    plt.legend(title='Algorithm + Variant')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'time_by_scenario.png'))

def create_radar_chart(df, output_dir):
    """
    Create radar chart comparing the algorithms
    
    Args:
        df: DataFrame with benchmark results
        output_dir: Directory to save plots
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Filter successful runs
    success_df = df[df['Success'] == True]
    
    # Group by algorithm and variant
    grouped = success_df.groupby(['Algorithm', 'Variant'])
    
    # Normalize metrics for radar chart
    metrics = [
        'ComputationTime(ms)', 
        'PathLength(m)', 
        'Smoothness', 
        'MinObstacleDistance', 
        'EnergyConsumption',
        'EnergyEfficiency'
    ]
    
    # Calculate mean values
    means = grouped[metrics].mean().reset_index()
    
    # Create a normalized version where higher is always better
    normalized = means.copy()
    
    # Normalize values to 0-1 range, ensuring higher is better
    for metric in metrics:
        if metric in ['ComputationTime(ms)', 'PathLength(m)', 'EnergyConsumption']:
            # Lower is better, so invert
            max_val = normalized[metric].max()
            normalized[metric] = 1 - (normalized[metric] / max_val)
        else:
            # Higher is better
            max_val = normalized[metric].max()
            if max_val > 0:
                normalized[metric] = normalized[metric] / max_val
    
    # Create radar chart
    plt.figure(figsize=(10, 10))
    
    # Number of metrics
    N = len(metrics)
    
    # Create angle for each metric
    angles = np.linspace(0, 2*np.pi, N, endpoint=False).tolist()
    angles += angles[:1]  # Close the polygon
    
    # Create radar plot
    ax = plt.subplot(111, polar=True)
    
    # Add metrics labels
    metric_labels = ['Computation\nTime', 'Path\nLength', 'Smoothness', 
                    'Obstacle\nAvoidance', 'Energy\nConsumption', 'Energy\nEfficiency']
    plt.xticks(angles[:-1], metric_labels)
    
    # Add each algorithm variant
    for _, row in normalized.iterrows():
        values = row[metrics].tolist()
        values += values[:1]  # Close the polygon
        
        label = f"{row['Algorithm']} ({row['Variant']})"
        ax.plot(angles, values, linewidth=2, label=label)
        ax.fill(angles, values, alpha=0.1)
    
    plt.title('Algorithm Performance Comparison')
    plt.legend(loc='upper right', bbox_to_anchor=(0.1, 0.1))
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'radar_chart.png'))

def generate_report(df, stats_df, test_df, output_dir):
    """
    Generate an HTML report summarizing the benchmark results
    
    Args:
        df: DataFrame with benchmark results
        stats_df: DataFrame with statistics
        test_df: DataFrame with statistical test results
        output_dir: Directory to save report
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Create report file
    report_path = os.path.join(output_dir, 'benchmark_report.html')
    
    with open(report_path, 'w') as f:
        f.write('''
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="UTF-8">
            <title>Algorithm Benchmark Report</title>
            <style>
                body { font-family: Arial, sans-serif; margin: 20px; }
                h1, h2, h3 { color: #333; }
                table { border-collapse: collapse; margin: 20px 0; }
                th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
                th { background-color: #f2f2f2; }
                tr:nth-child(even) { background-color: #f9f9f9; }
                .significant { color: green; font-weight: bold; }
                .not-significant { color: gray; }
                img { max-width: 100%; margin: 20px 0; }
            </style>
        </head>
        <body>
            <h1>Algorithm Benchmark Report</h1>
        ''')
        
        # Summary statistics
        f.write('<h2>Summary Statistics</h2>')
        f.write(stats_df.to_html())
        
        # Statistical test results
        f.write('<h2>Statistical Tests</h2>')
        f.write('<p>Comparing energy-aware vs standard variants (p < 0.05 is significant)</p>')
        
        # Format p-values and highlight significant results
        test_html = test_df.copy()
        test_html['p_value'] = test_html['p_value'].apply(lambda p: f'<span class="{"significant" if p < 0.05 else "not-significant"}">{p:.4f}</span>')
        
        f.write(test_html.to_html(escape=False))
        
        # Add plots
        f.write('<h2>Performance Comparison</h2>')
        
        plot_files = [
            'computation_time.png',
            'energy_consumption.png',
            'energy_efficiency.png',
            'path_smoothness.png',
            'min_obstacle_distance.png',
            'success_rate.png',
            'energy_by_scenario.png',
            'time_by_scenario.png',
            'radar_chart.png'
        ]
        
        for plot_file in plot_files:
            plot_path = os.path.join(output_dir, plot_file)
            if os.path.exists(plot_path):
                plot_title = ' '.join(plot_file.replace('.png', '').split('_')).title()
                f.write(f'<h3>{plot_title}</h3>')
                f.write(f'<img src="{plot_file}" alt="{plot_title}">')
        
        # Key findings
        f.write('<h2>Key Findings</h2>')
        
        # Check if energy-aware variants save energy
        energy_findings = []
        
        for algo in df['Algorithm'].unique():
            # Filter by algorithm
            algo_df = df[df['Algorithm'] == algo]
            
            # Get standard and energy-aware results for successful runs
            standard = algo_df[(algo_df['Variant'] == 'standard') & (algo_df['Success'] == True)]
            energy_aware = algo_df[(algo_df['Variant'] == 'energy_aware') & (algo_df['Success'] == True)]
            
            if len(standard) == 0 or len(energy_aware) == 0:
                continue
                
            # Calculate average energy savings
            std_energy = standard['EnergyConsumption'].mean()
            ea_energy = energy_aware['EnergyConsumption'].mean()
            
            if std_energy > 0:
                savings = (std_energy - ea_energy) / std_energy * 100
                significant = False
                
                # Check if the difference is statistically significant
                test_row = test_df[(test_df['Algorithm'] == algo) & (test_df['Metric'] == 'EnergyConsumption')]
                if not test_row.empty:
                    significant = test_row.iloc[0]['significant']
                
                energy_findings.append({
                    'Algorithm': algo,
                    'Savings': savings,
                    'Significant': significant
                })
        
        if energy_findings:
            f.write('<h3>Energy Savings</h3>')
            f.write('<ul>')
            
            for finding in energy_findings:
                sig_text = "statistically significant" if finding['Significant'] else "not statistically significant"
                f.write(f'<li><strong>{finding["Algorithm"]}</strong>: Energy-aware version saves {finding["Savings"]:.2f}% energy ({sig_text})</li>')
            
            f.write('</ul>')
        
        # Check computation time impact
        time_findings = []
        
        for algo in df['Algorithm'].unique():
            # Filter by algorithm
            algo_df = df[df['Algorithm'] == algo]
            
            # Get standard and energy-aware results
            standard = algo_df[algo_df['Variant'] == 'standard']
            energy_aware = algo_df[algo_df['Variant'] == 'energy_aware']
            
            if len(standard) == 0 or len(energy_aware) == 0:
                continue
                
            # Calculate average time difference
            std_time = standard['ComputationTime(ms)'].mean()
            ea_time = energy_aware['ComputationTime(ms)'].mean()
            
            if std_time > 0:
                diff = (ea_time - std_time) / std_time * 100
                
                # Check if the difference is statistically significant
                significant = False
                test_row = test_df[(test_df['Algorithm'] == algo) & (test_df['Metric'] == 'ComputationTime(ms)')]
                if not test_row.empty:
                    significant = test_row.iloc[0]['significant']
                
                time_findings.append({
                    'Algorithm': algo,
                    'Difference': diff,
                    'Significant': significant
                })
        
        if time_findings:
            f.write('<h3>Computation Time Impact</h3>')
            f.write('<ul>')
            
            for finding in time_findings:
                sig_text = "statistically significant" if finding['Significant'] else "not statistically significant"
                if finding['Difference'] > 0:
                    f.write(f'<li><strong>{finding["Algorithm"]}</strong>: Energy-aware version takes {finding["Difference"]:.2f}% more computation time ({sig_text})</li>')
                else:
                    f.write(f'<li><strong>{finding["Algorithm"]}</strong>: Energy-aware version takes {abs(finding["Difference"]):.2f}% less computation time ({sig_text})</li>')
            
            f.write('</ul>')
        
        # Success rate comparison
        f.write('<h3>Success Rate Comparison</h3>')
        
        success_rates = df.groupby(['Algorithm', 'Variant'])['Success'].mean().reset_index()
        
        f.write('<ul>')
        for _, row in success_rates.iterrows():
            f.write(f'<li><strong>{row["Algorithm"]} ({row["Variant"]})</strong>: {row["Success"]*100:.1f}% success rate</li>')
        f.write('</ul>')
        
        f.write('<h3>Conclusion</h3>')
        f.write('<p>Based on the benchmark results, we can draw the following conclusions:</p>')
        
        # Generate automatic conclusions based on data
        f.write('<ul>')
        
        # Best energy efficiency
        if not success_df.empty:
            best_energy = success_df.loc[success_df['EnergyEfficiency'].idxmax()]
            f.write(f'<li>The most energy-efficient algorithm is <strong>{best_energy["Algorithm"]} ({best_energy["Variant"]})</strong> with an average efficiency of {best_energy["EnergyEfficiency"]:.2f}.</li>')
        
        # Fastest algorithm
        fastest = df.loc[df['ComputationTime(ms)'].idxmin()]
        f.write(f'<li>The fastest algorithm is <strong>{fastest["Algorithm"]} ({fastest["Variant"]})</strong> with an average computation time of {fastest["ComputationTime(ms)"]:.2f} ms.</li>')
        
        # Generate conclusions about energy-aware variants
        if energy_findings:
            savings = [finding for finding in energy_findings if finding['Savings'] > 0]
            if savings:
                max_saving = max(savings, key=lambda x: x['Savings'])
                f.write(f'<li>Energy-aware variants provide the highest energy savings in <strong>{max_saving["Algorithm"]}</strong> (up to {max_saving["Savings"]:.2f}%).</li>')
            
            # Determine if there's a trade-off
            if time_findings:
                has_tradeoff = any(finding['Difference'] > 5 and finding['Significant'] for finding in time_findings)
                if has_tradeoff:
                    f.write('<li>There is a significant computation time trade-off when using energy-aware variants.</li>')
                else:
                    f.write('<li>Energy-aware variants do not significantly impact computation time in most cases.</li>')
        
        f.write('</ul>')
        
        f.write('''
        </body>
        </html>
        ''')
    
    print(f"Report generated at {report_path}")

def main():
    parser = argparse.ArgumentParser(description='Analyze benchmark results')
    parser.add_argument('--results_dir', type=str, default='results', 
                        help='Directory containing benchmark CSV files')
    parser.add_argument('--output_dir', type=str, default='analysis', 
                        help='Directory to save analysis results')
    args = parser.parse_args()
    
    # Load all results
    df = load_results(args.results_dir)
    
    if df is None or df.empty:
        print("No results found. Please run benchmarks first.")
        return
    
    # Calculate statistics
    stats_df = calculate_statistics(df)
    
    # Run statistical tests
    test_df = run_statistical_tests(df)
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Create plots
    plot_comparison(df, args.output_dir)
    plot_scenario_comparison(df, args.output_dir)
    create_radar_chart(df, args.output_dir)
    
    # Generate report
    generate_report(df, stats_df, test_df, args.output_dir)
    
if __name__ == "__main__":
    main() 