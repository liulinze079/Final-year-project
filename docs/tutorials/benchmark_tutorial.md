# ASV Energy-Aware Planning Benchmark Tutorial

This tutorial guides you through the process of running benchmarks and analyzing the results for the ASV Energy-Aware Planning algorithm.

## Prerequisites

- ROS 2 environment set up
- ASV Energy-Aware Planning package built
- Python 3 with required dependencies (pandas, matplotlib, numpy, scipy)

## Running Benchmarks

1. **Navigate to the evaluation directory**:
   ```bash
   cd /path/to/asv_energy_aware_planning/evaluation
   ```

2. **Generate test scenarios** (if needed):
   ```bash
   python3 scripts/generate_test_scenarios.py
   ```

3. **Run the benchmarks**:
   ```bash
   ./run_benchmarks.sh
   ```
   This will execute all benchmark tests and save results to the `results` directory.

## Analyzing Results

1. **Run the analysis script**:
   ```bash
   python3 scripts/analyze_results.py --results_dir results --output_dir analysis/outputs
   ```

2. **Generate performance report**:
   ```bash
   python3 scripts/generate_pdf_report.py --images_dir analysis/outputs --report_md analysis/outputs/benchmark_report.md --output_pdf analysis/outputs/asv_benchmark_report.pdf
   ```

## Understanding the Results

The benchmark results include:

- **Path Length**: The total length of the generated path
- **Computation Time**: Time required to compute the path
- **Energy Consumption**: Energy required to follow the path
- **Success Rate**: Percentage of successful path planning attempts

The analysis compares different algorithms (AStar, Dijkstra, RRT, EnergyAware) across various scenario types and obstacle densities.

## Visualizing Results

The analysis generates several visualizations:
- Comparison plots for each metric
- Performance by scenario type
- Performance by obstacle density

All visualizations are saved to the `analysis/outputs` directory and compiled into a PDF report.

## Troubleshooting

If you encounter errors:

1. Ensure all Python dependencies are installed:
   ```bash
   pip install pandas matplotlib numpy scipy seaborn
   ```

2. Check that test scenarios were properly generated
3. Verify that the benchmark executable was built successfully 