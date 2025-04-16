# PlannerStatus.msg
# Represents the status of planning components

# Header with timestamp and frame id
std_msgs/Header header

# Planner identifier
string planner_id

# Planner type (global, local, trajectory)
string planner_type

# Current planning status
uint8 status
uint8 STATUS_IDLE = 0
uint8 STATUS_PLANNING = 1
uint8 STATUS_SUCCESS = 2
uint8 STATUS_FAILURE = 3
uint8 STATUS_EMERGENCY = 4

# Error code if status is FAILURE
uint8 error_code
string error_message

# Performance metrics
float64 planning_time         # Time taken for planning (seconds)
float64 cpu_usage             # CPU usage percentage
float64 memory_usage          # Memory usage in MB
uint32 iterations             # Number of iterations
uint32 nodes_explored         # Number of nodes explored in search

# Path metrics (if status is SUCCESS)
float64 path_length           # Path length in meters
float64 path_duration         # Path duration in seconds
float64 path_energy           # Estimated energy consumption in kWh
float64 path_smoothness       # Path smoothness metric (0-1)
float64 path_safety           # Path safety metric (0-1)

# Planning parameters used
float64 time_horizon          # Planning time horizon (seconds)
float64 distance_horizon      # Planning distance horizon (meters)
float64 obstacle_threshold    # Obstacle safety threshold (meters)

# Replanning trigger
string replanning_reason      # Reason for replanning if applicable 