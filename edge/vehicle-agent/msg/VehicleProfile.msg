# AtlasMesh Vehicle Profile Message
# Defines vehicle capabilities, constraints, and operational parameters

# Header and identification
std_msgs/Header header
string profile_id
string profile_version
string vehicle_id
string vehicle_type

# Physical specifications
float64 length                 # meters
float64 width                  # meters  
float64 height                 # meters
float64 wheelbase             # meters
float64 track_width           # meters
float64 ground_clearance      # meters
float64 curb_weight           # kg
float64 max_payload           # kg
float64 max_gross_weight      # kg

# Performance characteristics
float64 max_speed             # m/s
float64 max_acceleration      # m/s²
float64 max_deceleration      # m/s²
float64 max_lateral_acceleration # m/s²
float64 min_turning_radius    # meters
float64 max_grade_capability  # percentage
float64 max_side_slope        # degrees

# Powertrain specifications
uint8 POWERTRAIN_ICE = 0
uint8 POWERTRAIN_HYBRID = 1
uint8 POWERTRAIN_ELECTRIC = 2
uint8 POWERTRAIN_FUEL_CELL = 3
uint8 powertrain_type

float64 fuel_capacity         # liters or kWh
float64 fuel_consumption_rate # l/100km or kWh/100km
float64 range_km             # maximum range
float64 charging_power_max   # kW (for electric vehicles)

# Brake system
float64 brake_force_max       # Newtons
float64 brake_response_time   # seconds
float64 emergency_brake_distance # meters at max speed
bool abs_available
bool ebs_available           # Electronic Braking System

# Steering system
float64 steering_ratio        # degrees wheel / degrees road wheel
float64 steering_response_time # seconds
float64 max_steering_angle   # degrees
bool power_steering_available
bool steer_by_wire_available

# Sensor configuration
sensor_msgs/PointCloud2 lidar_specs[]
sensor_msgs/CameraInfo camera_specs[]
sensor_msgs/Range radar_specs[]
sensor_msgs/Imu imu_specs
sensor_msgs/NavSatFix gnss_specs

# Sensor mounting positions (relative to vehicle center)
geometry_msgs/Transform[] sensor_transforms
string[] sensor_names
uint8[] sensor_types
bool[] sensor_critical        # Critical for safe operation

# Environmental operating limits
float64 temperature_min       # celsius
float64 temperature_max       # celsius
float64 humidity_max         # percentage
float64 wind_speed_max       # m/s
float64 precipitation_max    # mm/h
float64 visibility_min       # meters

# Operational Design Domain (ODD) constraints
string[] supported_road_types
string[] supported_weather_conditions
string[] supported_traffic_scenarios
float64 max_operational_speed # m/s (may be less than max_speed)
bool highway_capable
bool urban_capable
bool off_road_capable
bool night_operation_capable

# Safety systems
bool emergency_stop_available
bool collision_avoidance_available
bool lane_keeping_available
bool adaptive_cruise_available
bool blind_spot_monitoring_available
bool automatic_parking_available

# Autonomy capabilities
uint8 AUTONOMY_LEVEL_0 = 0    # No automation
uint8 AUTONOMY_LEVEL_1 = 1    # Driver assistance
uint8 AUTONOMY_LEVEL_2 = 2    # Partial automation
uint8 AUTONOMY_LEVEL_3 = 3    # Conditional automation
uint8 AUTONOMY_LEVEL_4 = 4    # High automation
uint8 AUTONOMY_LEVEL_5 = 5    # Full automation
uint8 autonomy_level

string[] supported_maneuvers
string[] supported_scenarios
float64 reaction_time         # seconds for autonomous systems

# Communication capabilities
string[] communication_protocols # MQTT, gRPC, WebSocket, etc.
string[] network_types        # 4G, 5G, WiFi, Satellite
float64 communication_range   # meters for V2V
bool v2v_capable
bool v2i_capable
bool v2x_capable

# Maintenance and diagnostics
float64 service_interval_km   # kilometers between services
float64 service_interval_hours # hours between services
string[] diagnostic_protocols # OBD-II, J1939, etc.
string[] maintenance_sensors  # Sensors for predictive maintenance

# Regulatory compliance
string[] certifications       # ISO 26262, FMVSS, etc.
string[] regulatory_approvals
string homologation_region
builtin_interfaces/Time certification_expiry

# Sector-specific configurations
string sector_type
string sector_profile_json    # JSON for sector-specific parameters

# Update and versioning
builtin_interfaces/Time profile_created
builtin_interfaces/Time profile_updated
string profile_checksum
uint32 profile_revision
string[] compatible_software_versions

# Validation status
bool profile_validated
string validation_authority
builtin_interfaces/Time validation_date
string validation_certificate
