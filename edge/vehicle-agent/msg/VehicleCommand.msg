# AtlasMesh Vehicle Command Message
# Commands sent to vehicle for control and coordination

# Header with timestamp and command ID
std_msgs/Header header
string command_id              # Unique command identifier for tracking
string issuer_id              # Who issued this command (cloud service, operator, etc.)

# Command types
uint8 COMMAND_TYPE_MOTION = 0
uint8 COMMAND_TYPE_SAFETY = 1
uint8 COMMAND_TYPE_SYSTEM = 2
uint8 COMMAND_TYPE_TRIP = 3
uint8 COMMAND_TYPE_OTA = 4
uint8 command_type

# Priority levels
uint8 PRIORITY_LOW = 0
uint8 PRIORITY_NORMAL = 1
uint8 PRIORITY_HIGH = 2
uint8 PRIORITY_EMERGENCY = 3
uint8 priority

# Motion commands
geometry_msgs/Twist target_velocity
geometry_msgs/Point target_position
float64 target_heading         # radians
float64 max_speed             # m/s
float64 max_acceleration      # m/s²
float64 max_deceleration      # m/s²
trajectory_msgs/JointTrajectory trajectory

# Safety commands
bool execute_emergency_stop
bool enable_autonomous_mode
bool disable_autonomous_mode
bool enable_manual_override
bool disable_manual_override
string safety_reason

# System commands
uint8 SYSTEM_CMD_NONE = 0
uint8 SYSTEM_CMD_SHUTDOWN = 1
uint8 SYSTEM_CMD_RESTART = 2
uint8 SYSTEM_CMD_CALIBRATE = 3
uint8 SYSTEM_CMD_DIAGNOSTICS = 4
uint8 SYSTEM_CMD_UPDATE_PROFILE = 5
uint8 system_command

# Trip commands
uint8 TRIP_CMD_NONE = 0
uint8 TRIP_CMD_START = 1
uint8 TRIP_CMD_PAUSE = 2
uint8 TRIP_CMD_RESUME = 3
uint8 TRIP_CMD_CANCEL = 4
uint8 TRIP_CMD_UPDATE_ROUTE = 5
uint8 trip_command

string trip_id
string route_id
geometry_msgs/Point[] waypoints
string route_metadata_json

# OTA commands
uint8 OTA_CMD_NONE = 0
uint8 OTA_CMD_CHECK_UPDATES = 1
uint8 OTA_CMD_DOWNLOAD_UPDATE = 2
uint8 OTA_CMD_INSTALL_UPDATE = 3
uint8 OTA_CMD_ROLLBACK = 4
uint8 OTA_CMD_VERIFY_INSTALL = 5
uint8 ota_command

string update_package_url
string update_signature
string update_version
string rollback_version

# Timing constraints
builtin_interfaces/Time execution_deadline
builtin_interfaces/Duration max_execution_time
bool require_acknowledgment

# Policy context
string[] applicable_policies
string policy_evaluation_context

# Validation and safety
string command_hash           # For integrity verification
string authorization_token   # For command authorization
bool bypass_safety_checks    # Emergency override (requires special authorization)

# Response requirements
bool require_status_updates
float64 status_update_frequency # Hz
string[] required_confirmations

# Sector-specific extensions
string sector_type
string sector_metadata_json   # JSON for sector-specific command data

# Fallback behavior
uint8 FALLBACK_IGNORE = 0
uint8 FALLBACK_SAFE_STOP = 1
uint8 FALLBACK_RETURN_HOME = 2
uint8 FALLBACK_MANUAL_CONTROL = 3
uint8 fallback_behavior
