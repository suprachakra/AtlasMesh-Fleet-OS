# AtlasMesh Vehicle State Message
# Comprehensive vehicle state information for real-time monitoring

# Header with timestamp and frame information
std_msgs/Header header

# Vehicle identification
string vehicle_id
string vehicle_type
string profile_version

# Position and motion
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
geometry_msgs/AccelWithCovariance accel

# Vehicle dynamics
float64 steering_angle          # radians
float64 wheel_speed_fl          # m/s - front left
float64 wheel_speed_fr          # m/s - front right  
float64 wheel_speed_rl          # m/s - rear left
float64 wheel_speed_rr          # m/s - rear right
float64 brake_pressure          # bar
float64 throttle_position       # 0.0 to 1.0
int8 gear_position             # -1=reverse, 0=neutral, 1+=forward gears

# Power and energy
float64 battery_voltage         # volts
float64 battery_current         # amperes
float64 battery_soc            # state of charge (0.0 to 1.0)
float64 battery_temperature    # celsius
float64 fuel_level             # liters or percentage
float64 engine_rpm             # revolutions per minute
float64 engine_temperature     # celsius

# Environmental sensors
float64 ambient_temperature    # celsius
float64 humidity              # percentage
float64 barometric_pressure   # hPa
float64 wind_speed            # m/s
float64 wind_direction        # radians

# Safety systems status
bool emergency_stop_active
bool autonomous_mode_active
bool manual_override_active
bool safety_systems_ok
bool[] sensor_health          # indexed by sensor type
string[] active_warnings
string[] active_errors

# Operational status
uint8 OPERATIONAL_MODE_MANUAL = 0
uint8 OPERATIONAL_MODE_ASSISTED = 1
uint8 OPERATIONAL_MODE_AUTONOMOUS = 2
uint8 OPERATIONAL_MODE_EMERGENCY = 3
uint8 OPERATIONAL_MODE_MAINTENANCE = 4
uint8 operational_mode

uint8 VEHICLE_STATUS_IDLE = 0
uint8 VEHICLE_STATUS_READY = 1
uint8 VEHICLE_STATUS_ACTIVE = 2
uint8 VEHICLE_STATUS_ERROR = 3
uint8 VEHICLE_STATUS_MAINTENANCE = 4
uint8 vehicle_status

# Trip information
string current_trip_id
string current_route_id
geometry_msgs/Point next_waypoint
float64 distance_to_waypoint   # meters
float64 estimated_arrival_time # seconds from now

# Performance metrics
float64 control_loop_frequency # Hz
float64 sensor_fusion_latency  # milliseconds
float64 policy_evaluation_time # milliseconds
float64 communication_latency  # milliseconds to cloud

# Diagnostic information
diagnostic_msgs/DiagnosticStatus[] diagnostics

# Custom metadata for sector-specific data
string sector_type
string metadata_json          # JSON string for extensibility
