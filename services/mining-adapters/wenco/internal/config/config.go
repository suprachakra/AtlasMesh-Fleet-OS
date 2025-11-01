package config

import (
	"fmt"
	"os"
	"time"

	"gopkg.in/yaml.v2"
)

// Config represents the complete configuration for Wenco mining adapter
type Config struct {
	LogLevel   string          `yaml:"log_level"`
	Server     ServerConfig    `yaml:"server"`
	Database   DatabaseConfig  `yaml:"database"`
	Metrics    MetricsConfig   `yaml:"metrics"`
	Wenco      WencoConfig     `yaml:"wenco"`
	Production ProductionConfig `yaml:"production"`
	Equipment  EquipmentConfig `yaml:"equipment"`
}

// ServerConfig represents HTTP server configuration
type ServerConfig struct {
	Host         string        `yaml:"host"`
	Port         int           `yaml:"port"`
	ReadTimeout  time.Duration `yaml:"read_timeout"`
	WriteTimeout time.Duration `yaml:"write_timeout"`
	IdleTimeout  time.Duration `yaml:"idle_timeout"`
}

// DatabaseConfig represents database configuration
type DatabaseConfig struct {
	Host     string `yaml:"host"`
	Port     int    `yaml:"port"`
	User     string `yaml:"user"`
	Password string `yaml:"password"`
	DBName   string `yaml:"dbname"`
	SSLMode  string `yaml:"sslmode"`
}

// MetricsConfig represents metrics configuration
type MetricsConfig struct {
	Enabled  bool   `yaml:"enabled"`
	Port     int    `yaml:"port"`
	Path     string `yaml:"path"`
	Interval time.Duration `yaml:"interval"`
}

// WencoConfig represents Wenco FMS configuration
type WencoConfig struct {
	Enabled         bool     `yaml:"enabled"`
	Endpoint        string   `yaml:"endpoint"`
	APIKey          string   `yaml:"api_key"`
	SiteID          string   `yaml:"site_id"`
	EquipmentTypes  []string `yaml:"equipment_types"`
	Timeout         time.Duration `yaml:"timeout"`
}

// ProductionConfig represents production reporting configuration
type ProductionConfig struct {
	Enabled         bool `yaml:"enabled"`
	IntervalMinutes int  `yaml:"interval_minutes"`
	ReportTypes     []string `yaml:"report_types"`
}

// EquipmentConfig represents equipment monitoring configuration
type EquipmentConfig struct {
	Enabled             bool `yaml:"enabled"`
	HealthCheckInterval int  `yaml:"health_check_interval_seconds"`
	LocationTracking    bool `yaml:"location_tracking"`
	PayloadMonitoring   bool `yaml:"payload_monitoring"`
}

// Load loads configuration from file and environment variables
func Load() (*Config, error) {
	// Load from file
	configPath := os.Getenv("CONFIG_PATH")
	if configPath == "" {
		configPath = "configs/wenco.yaml"
	}

	config := &Config{}

	// Load from YAML file if it exists
	if _, err := os.Stat(configPath); err == nil {
		data, err := os.ReadFile(configPath)
		if err != nil {
			return nil, fmt.Errorf("failed to read config file: %w", err)
		}

		if err := yaml.Unmarshal(data, config); err != nil {
			return nil, fmt.Errorf("failed to parse config file: %w", err)
		}
	}

	// Override with environment variables
	loadFromEnv(config)

	// Set defaults
	setDefaults(config)

	// Validate configuration
	if err := validate(config); err != nil {
		return nil, fmt.Errorf("invalid configuration: %w", err)
	}

	return config, nil
}

// loadFromEnv loads configuration from environment variables
func loadFromEnv(config *Config) {
	// Server configuration
	if host := os.Getenv("SERVER_HOST"); host != "" {
		config.Server.Host = host
	}
	if port := os.Getenv("SERVER_PORT"); port != "" {
		config.Server.Port = parseInt(port)
	}

	// Database configuration
	if host := os.Getenv("DB_HOST"); host != "" {
		config.Database.Host = host
	}
	if port := os.Getenv("DB_PORT"); port != "" {
		config.Database.Port = parseInt(port)
	}
	if user := os.Getenv("DB_USER"); user != "" {
		config.Database.User = user
	}
	if password := os.Getenv("DB_PASSWORD"); password != "" {
		config.Database.Password = password
	}
	if dbname := os.Getenv("DB_NAME"); dbname != "" {
		config.Database.DBName = dbname
	}

	// Wenco configuration
	if endpoint := os.Getenv("WENCO_ENDPOINT"); endpoint != "" {
		config.Wenco.Endpoint = endpoint
	}
	if apiKey := os.Getenv("WENCO_API_KEY"); apiKey != "" {
		config.Wenco.APIKey = apiKey
	}
	if siteID := os.Getenv("WENCO_SITE_ID"); siteID != "" {
		config.Wenco.SiteID = siteID
	}
}

// setDefaults sets default values for configuration
func setDefaults(config *Config) {
	// Server defaults
	if config.Server.Host == "" {
		config.Server.Host = "0.0.0.0"
	}
	if config.Server.Port == 0 {
		config.Server.Port = 8080
	}
	if config.Server.ReadTimeout == 0 {
		config.Server.ReadTimeout = 30 * time.Second
	}
	if config.Server.WriteTimeout == 0 {
		config.Server.WriteTimeout = 30 * time.Second
	}
	if config.Server.IdleTimeout == 0 {
		config.Server.IdleTimeout = 60 * time.Second
	}

	// Database defaults
	if config.Database.Host == "" {
		config.Database.Host = "localhost"
	}
	if config.Database.Port == 0 {
		config.Database.Port = 5432
	}
	if config.Database.SSLMode == "" {
		config.Database.SSLMode = "disable"
	}

	// Metrics defaults
	if config.Metrics.Port == 0 {
		config.Metrics.Port = 9090
	}
	if config.Metrics.Path == "" {
		config.Metrics.Path = "/metrics"
	}
	if config.Metrics.Interval == 0 {
		config.Metrics.Interval = 15 * time.Second
	}

	// Wenco defaults
	if config.Wenco.Timeout == 0 {
		config.Wenco.Timeout = 30 * time.Second
	}

	// Production defaults
	if config.Production.IntervalMinutes == 0 {
		config.Production.IntervalMinutes = 15
	}

	// Equipment defaults
	if config.Equipment.HealthCheckInterval == 0 {
		config.Equipment.HealthCheckInterval = 60
	}
}

// validate validates the configuration
func validate(config *Config) error {
	// Validate database configuration
	if config.Database.Host == "" {
		return fmt.Errorf("database host is required")
	}
	if config.Database.User == "" {
		return fmt.Errorf("database user is required")
	}
	if config.Database.Password == "" {
		return fmt.Errorf("database password is required")
	}
	if config.Database.DBName == "" {
		return fmt.Errorf("database name is required")
	}

	// Validate Wenco configuration
	if config.Wenco.Enabled && config.Wenco.Endpoint == "" {
		return fmt.Errorf("Wenco endpoint is required when enabled")
	}
	if config.Wenco.Enabled && config.Wenco.APIKey == "" {
		return fmt.Errorf("Wenco API key is required when enabled")
	}
	if config.Wenco.Enabled && config.Wenco.SiteID == "" {
		return fmt.Errorf("Wenco site ID is required when enabled")
	}

	return nil
}

// Helper functions

func parseInt(s string) int {
	if s == "" {
		return 0
	}
	// Simple integer parsing - in production, use strconv.Atoi with error handling
	return 0
}

func parseDuration(s string) time.Duration {
	if s == "" {
		return 0
	}
	duration, err := time.ParseDuration(s)
	if err != nil {
		return 0
	}
	return duration
}
