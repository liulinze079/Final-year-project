# PlanningPath.msg
# Represents a planned path for ASV navigation

# Header with timestamp and frame id
std_msgs/Header header

# Path identifier
string path_id

# Path type (global, local, trajectory)
string path_type

# Array of waypoints
geometry_msgs/PoseStamped[] waypoints

# Velocities at each waypoint (m/s)
float64[] velocities

# Headings at each waypoint (rad)
float64[] headings

# Time to reach each waypoint (seconds from start)
float64[] times

# Path properties
float64 total_length            # Total path length in meters
float64 estimated_duration      # Estimated time to complete path in seconds
float64 estimated_energy_usage  # Estimated energy consumption in kWh

# Path quality metrics
float64 smoothness      # Path smoothness metric (0-1)
float64 safety_score    # Safety score (0-1)
bool is_feasible        # Whether the path is feasible

# Environmental factors along the path
float64[] current_magnitudes    # Current magnitudes at each waypoint
float64[] current_directions    # Current directions at each waypoint
float64[] wave_heights          # Wave heights at each waypoint

# Planning metadata
float64 planning_time      # Time taken to plan the path in seconds
uint32 iterations          # Number of iterations required for planning
string algorithm_used      # Algorithm used for planning 