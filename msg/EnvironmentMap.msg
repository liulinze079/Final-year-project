# EnvironmentMap.msg
# Representation of the environment for energy-aware ASV planning

# Map metadata
Header header
string map_id                   # Unique identifier for this map
time timestamp                  # When the map was created/updated
string map_name                 # Human-readable name
string description              # Description of what this map represents
uint32 version                  # Version number of the map

# Geographical bounds of the map
float64 origin_latitude         # Latitude of map origin (degrees)
float64 origin_longitude        # Longitude of map origin (degrees)
float64 width                   # Width of the map (meters)
float64 height                  # Height of the map (meters)
float64 resolution              # Map resolution (meters/cell)

# Map representation types
uint8 GRID = 0                  # Cell-based grid representation
uint8 MESH = 1                  # Triangular mesh representation
uint8 VECTOR = 2                # Vector-based representation
uint8 map_type                  # Type of the map representation

# Occupancy grid (if using GRID type)
nav_msgs/OccupancyGrid occupancy_grid  # Standard occupancy grid (-1=unknown, 0-100=occupancy)

# Depth map (bathymetry)
float64[] depth_data            # Depth values at each cell (meters)
float64 min_depth               # Minimum depth in the map
float64 max_depth               # Maximum depth in the map
bool depth_available            # Whether depth data is available

# Environmental forces
bool force_field_available      # Whether force field data is available
uint32 force_field_width        # Width of force field grid
uint32 force_field_height       # Height of force field grid
EnvironmentalForce[] forces     # Array of environmental forces

# Obstacles
geometry_msgs/Polygon[] static_obstacles       # Polygonal representation of static obstacles
geometry_msgs/Polygon[] predicted_obstacles    # Predicted future obstacle positions
float64[] obstacle_probabilities               # Probability of each predicted obstacle

# Metadata about data sources
string[] data_sources           # Sources of map data
float64[] data_confidences      # Confidence in each data source (0.0-1.0)
time[] data_timestamps          # When each data source was collected

# Energy characteristics
float64[] energy_cost_map       # Energy cost multiplier for each cell (1.0 = normal)
bool energy_data_available      # Whether energy data is available

# Dynamic map properties
bool is_dynamic                 # Whether the map changes over time
float64 update_frequency        # How often the map is updated (Hz)
time last_update                # When the map was last updated
time next_update                # When the map will be updated next 