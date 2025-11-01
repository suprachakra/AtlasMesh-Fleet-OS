package config

import (
	"fmt"
	"os"
	"strconv"
	"time"
)

// Config holds all configuration for the passenger safety adapter
type Config struct {
	Environment string         `json:"environment"`
	Server      ServerConfig   `json:"server"`
	Database    DatabaseConfig `json:"database"`
	Safety      SafetyConfig   `json:"safety"`
	Monitoring  MonitoringConfig `json:"monitoring"`
	Audit       AuditConfig    `json:"audit"`
}

// ServerConfig holds server configuration
type ServerConfig struct {
	HTTPPort int `json:"http_port"`
	GRPCPort int `json:"grpc_port"`
}

// DatabaseConfig holds database configuration
type DatabaseConfig struct {
	URL             string        `json:"url"`
	MaxConnections  int           `json:"max_connections"`
	ConnMaxLifetime time.Duration `json:"conn_max_lifetime"`
	ConnMaxIdleTime time.Duration `json:"conn_max_idle_time"`
}

// SafetyConfig holds safety-specific configuration
type SafetyConfig struct {
	EmergencyResponseEnabled bool    `json:"emergency_response_enabled"`
	PanicButtonEnabled       bool    `json:"panic_button_enabled"`
	VideoMonitoringEnabled   bool    `json:"video_monitoring_enabled"`
	AudioMonitoringEnabled   bool    `json:"audio_monitoring_enabled"`
	MaxAcceleration          float64 `json:"max_acceleration"`
	MaxDeceleration          float64 `json:"max_deceleration"`
	MaxSpeed                 float64 `json:"max_speed"`
	ComfortThreshold         float64 `json:"comfort_threshold"`
	SafetyMargin             float64 `json:"safety_margin_meters"`
}

// MonitoringConfig holds monitoring configuration
type MonitoringConfig struct {
	Enabled           bool    `json:"enabled"`
	CheckIntervalMs   int     `json:"check_interval_ms"`
	AlertThreshold    float64 `json:"alert_threshold"`
	CriticalThreshold float64 `json:"critical_threshold"`
	RetentionHours    int     `json:"retention_hours"`
}

// AuditConfig holds audit configuration
type AuditConfig struct {
	Enabled         bool   `json:"enabled"`
	SigningEnabled  bool   `json:"signing_enabled"`
	SigningKeyPath  string `json:"signing_key_path"`
	RetentionDays   int    `json:"retention_days"`
	BatchSize       int    `json:"batch_size"`
	FlushIntervalMs int    `json:"flush_interval_ms"`
}

// Load loads configuration from environment variables
func Load() (*Config, error) {
	cfg := &Config{
		Environment: getEnvString("ENVIRONMENT", "development"),
		Server: ServerConfig{
			HTTPPort: getEnvInt("HTTP_PORT", 8080),
			GRPCPort: getEnvInt("GRPC_PORT", 9090),
		},
		Database: DatabaseConfig{
			URL:             getEnvString("DATABASE_URL", "postgres://postgres:password@localhost:5432/atlasmesh_safety?sslmode=disable"),
			MaxConnections:  getEnvInt("DB_MAX_CONNECTIONS", 25),
			ConnMaxLifetime: time.Duration(getEnvInt("DB_CONN_MAX_LIFETIME_MINUTES", 5)) * time.Minute,
			ConnMaxIdleTime: time.Duration(getEnvInt("DB_CONN_MAX_IDLE_MINUTES", 5)) * time.Minute,
		},
		Safety: SafetyConfig{
			EmergencyResponseEnabled: getEnvBool("SAFETY_EMERGENCY_RESPONSE_ENABLED", true),
			PanicButtonEnabled:       getEnvBool("SAFETY_PANIC_BUTTON_ENABLED", true),
			VideoMonitoringEnabled:   getEnvBool("SAFETY_VIDEO_MONITORING_ENABLED", true),
			AudioMonitoringEnabled:   getEnvBool("SAFETY_AUDIO_MONITORING_ENABLED", true),
			MaxAcceleration:          getEnvFloat("SAFETY_MAX_ACCELERATION", 2.0),
			MaxDeceleration:          getEnvFloat("SAFETY_MAX_DECELERATION", 3.0),
			MaxSpeed:                 getEnvFloat("SAFETY_MAX_SPEED", 15.0),
			ComfortThreshold:         getEnvFloat("SAFETY_COMFORT_THRESHOLD", 0.5),
			SafetyMargin:             getEnvFloat("SAFETY_MARGIN_METERS", 1.0),
		},
		Monitoring: MonitoringConfig{
			Enabled:           getEnvBool("MONITORING_ENABLED", true),
			CheckIntervalMs:   getEnvInt("MONITORING_CHECK_INTERVAL_MS", 1000),
			AlertThreshold:    getEnvFloat("MONITORING_ALERT_THRESHOLD", 0.7),
			CriticalThreshold: getEnvFloat("MONITORING_CRITICAL_THRESHOLD", 0.9),
			RetentionHours:    getEnvInt("MONITORING_RETENTION_HOURS", 24),
		},
		Audit: AuditConfig{
			Enabled:         getEnvBool("AUDIT_ENABLED", true),
			SigningEnabled:  getEnvBool("AUDIT_SIGNING_ENABLED", true),
			SigningKeyPath:  getEnvString("AUDIT_SIGNING_KEY_PATH", "/app/keys/audit-signing.key"),
			RetentionDays:   getEnvInt("AUDIT_RETENTION_DAYS", 2555), // 7 years
			BatchSize:       getEnvInt("AUDIT_BATCH_SIZE", 100),
			FlushIntervalMs: getEnvInt("AUDIT_FLUSH_INTERVAL_MS", 1000),
		},
	}

	// Validate configuration
	if err := cfg.validate(); err != nil {
		return nil, fmt.Errorf("configuration validation failed: %w", err)
	}

	return cfg, nil
}

// validate validates the configuration
func (c *Config) validate() error {
	if c.Server.HTTPPort <= 0 || c.Server.HTTPPort > 65535 {
		return fmt.Errorf("invalid HTTP port: %d", c.Server.HTTPPort)
	}

	if c.Server.GRPCPort <= 0 || c.Server.GRPCPort > 65535 {
		return fmt.Errorf("invalid gRPC port: %d", c.Server.GRPCPort)
	}

	if c.Database.URL == "" {
		return fmt.Errorf("database URL is required")
	}

	if c.Safety.MaxAcceleration <= 0 {
		return fmt.Errorf("max acceleration must be positive")
	}

	if c.Safety.MaxDeceleration <= 0 {
		return fmt.Errorf("max deceleration must be positive")
	}

	if c.Safety.MaxSpeed <= 0 {
		return fmt.Errorf("max speed must be positive")
	}

	if c.Safety.ComfortThreshold < 0 || c.Safety.ComfortThreshold > 1 {
		return fmt.Errorf("comfort threshold must be between 0 and 1")
	}

	if c.Safety.SafetyMargin <= 0 {
		return fmt.Errorf("safety margin must be positive")
	}

	return nil
}

// Helper functions for environment variable parsing
func getEnvString(key, defaultValue string) string {
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
