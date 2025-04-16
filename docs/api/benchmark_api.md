# ASV Benchmark Analysis API Reference

This document describes the API for the ASV benchmark analysis tools.

## Script: analyze_results.py

### Main Function
```python
analyze_results(results_dir, output_dir)
```

**Parameters:**
- `results_dir`: Directory containing benchmark results (JSON files)
- `output_dir`: Directory where analysis outputs will be saved

**Returns:** None

### Key Functions

#### load_results
```python
load_results(results_dir)
```
Loads benchmark results from JSON files.

**Parameters:**
- `results_dir`: Directory containing result files

**Returns:**
- List of result dictionaries

#### calculate_statistics
```python
calculate_statistics(results)
```
Calculates statistical measures for benchmark results.

**Parameters:**
- `results`: List of result dictionaries

**Returns:**
- Dictionary of statistics for each algorithm/scenario combination

#### plot_metrics
```python
plot_metrics_by_algorithm(results, output_dir)
```
Generates comparison plots for all metrics.

**Parameters:**
- `results`: List of result dictionaries
- `output_dir`: Directory where plots will be saved

**Returns:** None

#### generate_performance_summary
```python
generate_performance_summary(results, output_dir)
```
Creates a text summary of algorithm performance.

**Parameters:**
- `results`: List of result dictionaries
- `output_dir`: Directory where the summary will be saved

**Returns:** None

## Script: generate_sample_data.py

### Main Function
```python
generate_sample_data(output_dir)
```

**Parameters:**
- `output_dir`: Directory where sample data will be saved

**Returns:** None

## Script: generate_pdf_report.py

### Main Function
```python
generate_pdf_report(images_dir, report_md, output_pdf)
```

**Parameters:**
- `images_dir`: Directory containing visualization images
- `report_md`: Path to Markdown report file
- `output_pdf`: Path where the PDF report will be saved

**Returns:** None

## Data Formats

### Benchmark Result Format
```json
{
  "algorithm": "AlgorithmName",
  "scenario": "scenario_type_density",
  "metrics": {
    "path_length": [value1, value2, ...],
    "computation_time": [value1, value2, ...],
    "energy_consumption": [value1, value2, ...],
    "success_rate": value
  }
}
```

### Statistics Format
```
{
  (algorithm, scenario): {
    "path_length_mean": value,
    "path_length_std": value,
    "computation_time_mean": value,
    "computation_time_std": value,
    "energy_consumption_mean": value,
    "energy_consumption_std": value,
    "success_rate": value
  }
}
```

## Usage Examples

### Analyzing Benchmark Results
```python
from analyze_results import analyze_results

# Analyze results and generate visualizations
analyze_results("path/to/results", "path/to/output")
```

### Generating Sample Data
```python
from generate_sample_data import generate_sample_data

# Generate sample benchmark data
generate_sample_data("path/to/output")
```

### Creating PDF Report
```python
from generate_pdf_report import generate_pdf_report

# Generate comprehensive PDF report
generate_pdf_report("path/to/images", "path/to/report.md", "path/to/output.pdf")
``` 