# ASVState.msg
# Represents the state of an Autonomous Surface Vessel

# Header with timestamp and frame id
std_msgs/Header header

# Position in global frame (x, y, z)
geometry_msgs/Point position

# Orientation as quaternion
geometry_msgs/Quaternion orientation

# Linear velocity (vx, vy, vz)
geometry_msgs/Vector3 linear_velocity

# Angular velocity (wx, wy, wz)
geometry_msgs/Vector3 angular_velocity

# Euler angles (roll, pitch, yaw) in radians
geometry_msgs/Vector3 euler

# Linear acceleration (ax, ay, az)
geometry_msgs/Vector3 linear_acceleration

# Surge, sway, and yaw rate in body frame
float64 surge  # m/s
float64 sway   # m/s
float64 yaw_rate  # rad/s

# Control inputs
float64 thrust_left   # N
float64 thrust_right  # N
float64 rudder_angle  # rad

# Battery status
float64 battery_voltage  # V
float64 battery_current  # A
float64 battery_charge   # % (0-100)

# Energy consumption
float64 energy_consumption  # kWh

# System status
bool auto_mode
bool emergency_stop
uint8 navigation_status  # 0: idle, 1: executing, 2: completed, 3: error

# Mission status
string mission_id
uint32 waypoint_index
uint32 total_waypoints 