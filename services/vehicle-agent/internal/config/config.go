package config

import (
	"fmt"
	"os"
	"strconv"
	"strings"
)

// Config represents the vehicle agent configuration
type Config struct {
	Server       ServerConfig       `yaml:"server"`
	Database     DatabaseConfig     `yaml:"database"`
	Metrics      MetricsConfig      `yaml:"metrics"`
	LogLevel     string             `yaml:"log_level"`
	VehicleID    string             `yaml:"vehicle_id"`
	ROS2         ROS2Config         `yaml:"ros2"`
	Perception   PerceptionConfig   `yaml:"perception"`
	Localization LocalizationConfig `yaml:"localization"`
	Control      ControlConfig      `yaml:"control"`
	Safety       SafetyConfig       `yaml:"safety"`
	Cloud        CloudConfig        `yaml:"cloud"`
}

// ServerConfig represents server configuration
type ServerConfig struct {
	Port int `yaml:"port"`
}

// DatabaseConfig represents database configuration
type DatabaseConfig struct {
	Host     string `yaml:"host"`
	Port     int    `yaml:"port"`
	Database string `yaml:"database"`
	Username string `yaml:"username"`
	Password string `yaml:"password"`
	SSLMode  string `yaml:"ssl_mode"`
}

// MetricsConfig represents metrics configuration
type MetricsConfig struct {
	Enabled bool   `yaml:"enabled"`
	Port    int    `yaml:"port"`
	Path    string `yaml:"path"`
}

// ROS2Config represents ROS2 configuration
type ROS2Config struct {
	NodeName        string `yaml:"node_name"`
	DomainID        int    `yaml:"domain_id"`
	QoSProfile      string `yaml:"qos_profile"`
	ExecutorType    string `yaml:"executor_type"`
	SpinRate        int    `yaml:"spin_rate_hz"`
	LogLevel        string `yaml:"log_level"`
	EnableIntraProcess bool `yaml:"enable_intra_process"`
}

// PerceptionConfig represents perception module configuration
type PerceptionConfig struct {
	LidarTopic          string  `yaml:"lidar_topic"`
	CameraTopic         string  `yaml:"camera_topic"`
	RadarTopic          string  `yaml:"radar_topic"`
	ObjectTopic         string  `yaml:"object_topic"`
	ObstacleTopic       string  `yaml:"obstacle_topic"`
	LanesTopic          string  `yaml:"lanes_topic"`
	TrafficLightTopic   string  `yaml:"traffic_light_topic"`
	ProcessingRate      int     `yaml:"processing_rate_hz"`
	MaxDetectionRange   float64 `yaml:"max_detection_range_m"`
	MinObjectSize       float64 `yaml:"min_object_size_m"`
	ConfidenceThreshold float64 `yaml:"confidence_threshold"`
	EnableTracking      bool    `yaml:"enable_tracking"`
	TrackingTimeout     float64 `yaml:"tracking_timeout_s"`
}

// LocalizationConfig represents localization module configuration
type LocalizationConfig struct {
	GPSTopic            string  `yaml:"gps_topic"`
	IMUTopic            string  `yaml:"imu_topic"`
	OdometryTopic       string  `yaml:"odometry_topic"`
	MapTopic            string  `yaml:"map_topic"`
	PoseTopic           string  `yaml:"pose_topic"`
	OdometryOutTopic    string  `yaml:"odometry_out_topic"`
	PathTopic           string  `yaml:"path_topic"`
	ProcessingRate      int     `yaml:"processing_rate_hz"`
	MinGPSAccuracy      float64 `yaml:"min_gps_accuracy_m"`
	MinSatellites       int     `yaml:"min_satellites"`
	EnableSLAM          bool    `yaml:"enable_slam"`
	EnableMapMatching   bool    `yaml:"enable_map_matching"`
	MapMatchingRadius   float64 `yaml:"map_matching_radius_m"`
	SLAMKeyframeDist    float64 `yaml:"slam_keyframe_distance_m"`
	SLAMKeyframeAngle   float64 `yaml:"slam_keyframe_angle_deg"`
}

// ControlConfig represents control module configuration
type ControlConfig struct {
	PathTopic           string  `yaml:"path_topic"`
	PoseTopic           string  `yaml:"pose_topic"`
	OdometryTopic       string  `yaml:"odometry_topic"`
	TwistTopic          string  `yaml:"twist_topic"`
	PathOutTopic        string  `yaml:"path_out_topic"`
	CmdVelTopic         string  `yaml:"cmd_vel_topic"`
	ProcessingRate      int     `yaml:"processing_rate_hz"`
	MaxSpeed            float64 `yaml:"max_speed_mps"`
	MaxAcceleration     float64 `yaml:"max_acceleration_mps2"`
	MaxDeceleration     float64 `yaml:"max_deceleration_mps2"`
	MaxAngularVelocity  float64 `yaml:"max_angular_velocity_rad_per_s"`
	LookaheadDistance   float64 `yaml:"lookahead_distance_m"`
	ControlGain         float64 `yaml:"control_gain"`
	EmergencyStopTime   float64 `yaml:"emergency_stop_time_s"`
}

// SafetyConfig represents safety monitor configuration
type SafetyConfig struct {
	PoseTopic           string  `yaml:"pose_topic"`
	OdometryTopic       string  `yaml:"odometry_topic"`
	PerceptionTopic     string  `yaml:"perception_topic"`
	AlertTopic          string  `yaml:"alert_topic"`
	EmergencyTopic      string  `yaml:"emergency_topic"`
	ProcessingRate      int     `yaml:"processing_rate_hz"`
	MaxSpeed            float64 `yaml:"max_speed_mps"`
	MinObstacleDistance float64 `yaml:"min_obstacle_distance_m"`
	MaxAcceleration     float64 `yaml:"max_acceleration_mps2"`
	MaxDeceleration     float64 `yaml:"max_deceleration_mps2"`
	SafetyTimeout       float64 `yaml:"safety_timeout_s"`
	AlertThreshold      int     `yaml:"alert_threshold"`
	CriticalThreshold   int     `yaml:"critical_threshold"`
}

// CloudConfig represents cloud bridge configuration
type CloudConfig struct {
	CloudURL        string `yaml:"cloud_url"`
	VehicleID       string `yaml:"vehicle_id"`
	APIKey          string `yaml:"api_key"`
	HeartbeatInterval int  `yaml:"heartbeat_interval_s"`
	TelemetryInterval int  `yaml:"telemetry_interval_s"`
	CommandTimeout  int    `yaml:"command_timeout_s"`
	RetryAttempts   int    `yaml:"retry_attempts"`
	RetryDelay      int    `yaml:"retry_delay_s"`
}

// Load loads configuration from environment variables and defaults
func Load() (*Config, error) {
	config := &Config{
		Server: ServerConfig{
			Port: getEnvInt("SERVER_PORT", 8080),
		},
		Database: DatabaseConfig{
			Host:     getEnv("DATABASE_HOST", "localhost"),
			Port:     getEnvInt("DATABASE_PORT", 5432),
			Database: getEnv("DATABASE_NAME", "atlasmesh"),
			Username: getEnv("DATABASE_USER", "atlasmesh"),
			Password: getEnv("DATABASE_PASSWORD", "atlasmesh"),
			SSLMode:  getEnv("DATABASE_SSL_MODE", "disable"),
		},
		Metrics: MetricsConfig{
			Enabled: getEnvBool("METRICS_ENABLED", true),
			Port:    getEnvInt("METRICS_PORT", 9090),
			Path:    getEnv("METRICS_PATH", "/metrics"),
		},
		LogLevel:  getEnv("LOG_LEVEL", "info"),
		VehicleID: getEnv("VEHICLE_ID", "vehicle-001"),
		ROS2: ROS2Config{
			NodeName:        getEnv("ROS2_NODE_NAME", "vehicle_agent"),
			DomainID:        getEnvInt("ROS2_DOMAIN_ID", 0),
			QoSProfile:      getEnv("ROS2_QOS_PROFILE", "reliable"),
			ExecutorType:    getEnv("ROS2_EXECUTOR_TYPE", "single_threaded"),
			SpinRate:        getEnvInt("ROS2_SPIN_RATE", 100),
			LogLevel:        getEnv("ROS2_LOG_LEVEL", "info"),
			EnableIntraProcess: getEnvBool("ROS2_ENABLE_INTRAPROCESS", false),
		},
		Perception: PerceptionConfig{
			LidarTopic:          getEnv("PERCEPTION_LIDAR_TOPIC", "/sensors/lidar/points"),
			CameraTopic:         getEnv("PERCEPTION_CAMERA_TOPIC", "/sensors/camera/image"),
			RadarTopic:          getEnv("PERCEPTION_RADAR_TOPIC", "/sensors/radar/scan"),
			ObjectTopic:         getEnv("PERCEPTION_OBJECT_TOPIC", "/perception/objects"),
			ObstacleTopic:       getEnv("PERCEPTION_OBSTACLE_TOPIC", "/perception/obstacles"),
			LanesTopic:          getEnv("PERCEPTION_LANES_TOPIC", "/perception/lanes"),
			TrafficLightTopic:   getEnv("PERCEPTION_TRAFFIC_LIGHT_TOPIC", "/perception/traffic_lights"),
			ProcessingRate:      getEnvInt("PERCEPTION_PROCESSING_RATE", 30),
			MaxDetectionRange:   getEnvFloat("PERCEPTION_MAX_DETECTION_RANGE", 100.0),
			MinObjectSize:       getEnvFloat("PERCEPTION_MIN_OBJECT_SIZE", 0.5),
			ConfidenceThreshold: getEnvFloat("PERCEPTION_CONFIDENCE_THRESHOLD", 0.7),
			EnableTracking:      getEnvBool("PERCEPTION_ENABLE_TRACKING", true),
			TrackingTimeout:     getEnvFloat("PERCEPTION_TRACKING_TIMEOUT", 2.0),
		},
		Localization: LocalizationConfig{
			GPSTopic:            getEnv("LOCALIZATION_GPS_TOPIC", "/sensors/gps/fix"),
			IMUTopic:            getEnv("LOCALIZATION_IMU_TOPIC", "/sensors/imu/data"),
			OdometryTopic:       getEnv("LOCALIZATION_ODOMETRY_TOPIC", "/odometry/wheel"),
			MapTopic:            getEnv("LOCALIZATION_MAP_TOPIC", "/map"),
			PoseTopic:           getEnv("LOCALIZATION_POSE_TOPIC", "/localization/pose"),
			OdometryOutTopic:    getEnv("LOCALIZATION_ODOMETRY_OUT_TOPIC", "/localization/odometry"),
			PathTopic:           getEnv("LOCALIZATION_PATH_TOPIC", "/localization/path"),
			ProcessingRate:      getEnvInt("LOCALIZATION_PROCESSING_RATE", 100),
			MinGPSAccuracy:      getEnvFloat("LOCALIZATION_MIN_GPS_ACCURACY", 2.0),
			MinSatellites:       getEnvInt("LOCALIZATION_MIN_SATELLITES", 6),
			EnableSLAM:          getEnvBool("LOCALIZATION_ENABLE_SLAM", true),
			EnableMapMatching:   getEnvBool("LOCALIZATION_ENABLE_MAP_MATCHING", true),
			MapMatchingRadius:   getEnvFloat("LOCALIZATION_MAP_MATCHING_RADIUS", 5.0),
			SLAMKeyframeDist:    getEnvFloat("LOCALIZATION_SLAM_KEYFRAME_DIST", 1.0),
			SLAMKeyframeAngle:   getEnvFloat("LOCALIZATION_SLAM_KEYFRAME_ANGLE", 15.0),
		},
		Control: ControlConfig{
			PathTopic:           getEnv("CONTROL_PATH_TOPIC", "/planning/path"),
			PoseTopic:           getEnv("CONTROL_POSE_TOPIC", "/localization/pose"),
			OdometryTopic:       getEnv("CONTROL_ODOMETRY_TOPIC", "/localization/odometry"),
			TwistTopic:          getEnv("CONTROL_TWIST_TOPIC", "/control/twist"),
			PathOutTopic:        getEnv("CONTROL_PATH_OUT_TOPIC", "/control/path"),
			CmdVelTopic:         getEnv("CONTROL_CMD_VEL_TOPIC", "/cmd_vel"),
			ProcessingRate:      getEnvInt("CONTROL_PROCESSING_RATE", 100),
			MaxSpeed:            getEnvFloat("CONTROL_MAX_SPEED", 5.0),
			MaxAcceleration:     getEnvFloat("CONTROL_MAX_ACCELERATION", 2.0),
			MaxDeceleration:     getEnvFloat("CONTROL_MAX_DECELERATION", 4.0),
			MaxAngularVelocity:  getEnvFloat("CONTROL_MAX_ANGULAR_VELOCITY", 1.0),
			LookaheadDistance:   getEnvFloat("CONTROL_LOOKAHEAD_DISTANCE", 3.0),
			ControlGain:         getEnvFloat("CONTROL_GAIN", 1.0),
			EmergencyStopTime:   getEnvFloat("CONTROL_EMERGENCY_STOP_TIME", 5.0),
		},
		Safety: SafetyConfig{
			PoseTopic:           getEnv("SAFETY_POSE_TOPIC", "/localization/pose"),
			OdometryTopic:       getEnv("SAFETY_ODOMETRY_TOPIC", "/localization/odometry"),
			PerceptionTopic:     getEnv("SAFETY_PERCEPTION_TOPIC", "/perception/objects"),
			AlertTopic:          getEnv("SAFETY_ALERT_TOPIC", "/safety/alerts"),
			EmergencyTopic:      getEnv("SAFETY_EMERGENCY_TOPIC", "/safety/emergency"),
			ProcessingRate:      getEnvInt("SAFETY_PROCESSING_RATE", 50),
			MaxSpeed:            getEnvFloat("SAFETY_MAX_SPEED", 5.0),
			MinObstacleDistance: getEnvFloat("SAFETY_MIN_OBSTACLE_DISTANCE", 1.0),
			MaxAcceleration:     getEnvFloat("SAFETY_MAX_ACCELERATION", 2.0),
			MaxDeceleration:     getEnvFloat("SAFETY_MAX_DECELERATION", 4.0),
			SafetyTimeout:       getEnvFloat("SAFETY_TIMEOUT", 10.0),
			AlertThreshold:      getEnvInt("SAFETY_ALERT_THRESHOLD", 3),
			CriticalThreshold:   getEnvInt("SAFETY_CRITICAL_THRESHOLD", 1),
		},
		Cloud: CloudConfig{
			CloudURL:        getEnv("CLOUD_URL", "https://api.atlasmesh.com"),
			VehicleID:       getEnv("VEHICLE_ID", "vehicle-001"),
			APIKey:          getEnv("CLOUD_API_KEY", ""),
			HeartbeatInterval: getEnvInt("CLOUD_HEARTBEAT_INTERVAL", 30),
			TelemetryInterval: getEnvInt("CLOUD_TELEMETRY_INTERVAL", 1),
			CommandTimeout:  getEnvInt("CLOUD_COMMAND_TIMEOUT", 10),
			RetryAttempts:   getEnvInt("CLOUD_RETRY_ATTEMPTS", 3),
			RetryDelay:      getEnvInt("CLOUD_RETRY_DELAY", 5),
		},
	}

	// Validate configuration
	if err := config.Validate(); err != nil {
		return nil, fmt.Errorf("configuration validation failed: %w", err)
	}

	return config, nil
}

// Validate validates the configuration
func (c *Config) Validate() error {
	if c.VehicleID == "" {
		return fmt.Errorf("vehicle_id is required")
	}

	if c.Cloud.APIKey == "" {
		return fmt.Errorf("cloud API key is required")
	}

	if c.Cloud.CloudURL == "" {
		return fmt.Errorf("cloud URL is required")
	}

	return nil
}

// Helper functions

func getEnv(key, defaultValue string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return defaultValue
}

func getEnvInt(key string, defaultValue int) int {
	if value := os.Getenv(key); value != "" {
		if intValue, err := strconv.Atoi(value); err == nil {
			return intValue
		}
	}
	return defaultValue
}

func getEnvFloat(key string, defaultValue float64) float64 {
	if value := os.Getenv(key); value != "" {
		if floatValue, err := strconv.ParseFloat(value, 64); err == nil {
			return floatValue
		}
	}
	return defaultValue
}

func getEnvBool(key string, defaultValue bool) bool {
	if value := os.Getenv(key); value != "" {
		if boolValue, err := strconv.ParseBool(value); err == nil {
			return boolValue
		}
	}
	return defaultValue
}
