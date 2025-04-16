# EnvironmentalForce.msg
# Message representing environmental forces affecting ASV navigation

# Type of environmental force
uint8 force_type
uint8 CURRENT = 0     # Water current
uint8 WIND = 1        # Wind force
uint8 WAVE = 2        # Wave force
uint8 TIDE = 3        # Tidal force
uint8 COMBINED = 10   # Combined forces

# Force vector components (in global frame)
float64 force_x       # X component of force (N/m² or appropriate units)
float64 force_y       # Y component of force (N/m² or appropriate units)
float64 force_z       # Z component of force (N/m² or appropriate units for 3D)

# Force magnitude and direction (alternative representation)
float64 magnitude     # Magnitude of the force
float64 direction     # Direction in radians (0 = East, π/2 = North)

# Position information
geometry_msgs/Point position     # Position where force is measured/applied

# Metadata
time timestamp                   # When force data was measured/predicted
float64 confidence               # Confidence in the force data (0.0-1.0)
string source                    # Source of the force data (sensor, forecast, etc.)

# Temporal information
float64 duration                 # Expected duration of force (in seconds)
float64 time_variance            # Variance in force over time

# For dynamic forces that change over an area
bool is_field                    # True if this is part of a force field
uint32 field_id                  # ID of the force field this belongs to
uint32 grid_x                    # X grid coordinate in force field
uint32 grid_y                    # Y grid coordinate in force field 