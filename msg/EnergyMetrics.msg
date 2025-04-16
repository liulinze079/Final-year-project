# EnergyMetrics.msg
# Represents energy consumption metrics for ASV operations

# Header with timestamp and frame id
std_msgs/Header header

# Path identifier
string path_id

# Total energy consumed (kWh)
float64 total_energy

# Energy efficiency (meters/kWh)
float64 energy_efficiency

# Distance traveled (meters)
float64 distance_traveled

# Average power (kW)
float64 average_power

# Current power (kW)
float64 current_power

# Environmental factors affecting energy consumption
float64 current_magnitude  # m/s
float64 current_direction  # rad
float64 wave_height        # m
float64 wave_direction     # rad

# Energy breakdown by source (%)
float64 propulsion_energy_percent
float64 electronics_energy_percent
float64 hotel_load_energy_percent

# Energy remaining (estimated)
float64 energy_remaining_kwh
float64 estimated_range_remaining_m

# Energy consumption rate (kWh/km)
float64 energy_consumption_rate 