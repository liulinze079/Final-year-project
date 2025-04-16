#!/bin/bash

# ASV Energy-Aware Planning Algorithm Benchmarking Suite
# This script runs the entire benchmarking process for the planning algorithms

# Set directory paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$SCRIPT_DIR/results"
ANALYSIS_DIR="$SCRIPT_DIR/analysis"
TEST_ENV_DIR="$SCRIPT_DIR/test_environments"

# Make directories if they don't exist
mkdir -p "$RESULTS_DIR"
mkdir -p "$ANALYSIS_DIR"

# Print header
echo "===================================================="
echo "   ASV Energy-Aware Planning Algorithm Benchmark    "
echo "===================================================="
echo

# 1. Generate test scenarios
echo "[1/4] Generating test scenarios..."
echo "----------------------------------------------------"
python "$SCRIPT_DIR/scripts/generate_test_scenarios.py" --simple 3 --complex 3 --real_world 3 --seed 42
echo "Test scenarios generated."
echo

# 2. Build benchmark executable
echo "[2/4] Building benchmark executable..."
echo "----------------------------------------------------"
cd "$PROJECT_DIR"
if [ ! -d "build" ]; then
    mkdir -p build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4 algorithm_benchmarking
if [ $? -ne 0 ]; then
    echo "Failed to build benchmark executable."
    exit 1
fi
echo "Build successful."
echo

# 3. Run benchmarks
echo "[3/4] Running algorithm benchmarks..."
echo "----------------------------------------------------"
cd "$SCRIPT_DIR"
"$PROJECT_DIR/build/algorithm_benchmarking"
if [ $? -ne 0 ]; then
    echo "Benchmark execution failed."
    exit 1
fi
echo "Benchmarks completed successfully."
echo

# 4. Analyze results
echo "[4/4] Analyzing benchmark results..."
echo "----------------------------------------------------"
python "$SCRIPT_DIR/scripts/analyze_results.py" --results_dir "$RESULTS_DIR" --output_dir "$ANALYSIS_DIR"
if [ $? -ne 0 ]; then
    echo "Results analysis failed."
    exit 1
fi
echo "Analysis completed successfully."
echo

# Print summary
echo "===================================================="
echo "   Benchmarking Complete                            "
echo "===================================================="
echo "Results are available in: $RESULTS_DIR"
echo "Analysis report: $ANALYSIS_DIR/benchmark_report.html"
echo
echo "To view the report, open the following URL in your browser:"
echo "file://$ANALYSIS_DIR/benchmark_report.html"
echo

# Open report in default browser if on a desktop system
if command -v xdg-open &> /dev/null; then
    echo "Opening report in browser..."
    xdg-open "$ANALYSIS_DIR/benchmark_report.html"
elif command -v open &> /dev/null; then
    echo "Opening report in browser..."
    open "$ANALYSIS_DIR/benchmark_report.html"
fi 