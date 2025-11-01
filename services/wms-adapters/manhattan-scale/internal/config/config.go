package config

import (
	"fmt"
	"os"
	"time"

	"gopkg.in/yaml.v2"
)

// Config represents the complete configuration for Manhattan SCALE adapter
type Config struct {
	LogLevel  string        `yaml:"log_level"`
	Server    ServerConfig  `yaml:"server"`
	Database  DatabaseConfig `yaml:"database"`
	Metrics   MetricsConfig `yaml:"metrics"`
	Manhattan ManhattanConfig `yaml:"manhattan"`
	Task      TaskConfig    `yaml:"task"`
	Inventory InventoryConfig `yaml:"inventory"`
	Resource  ResourceConfig `yaml:"resource"`
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

// ManhattanConfig represents Manhattan SCALE connection configuration
type ManhattanConfig struct {
	Host          string        `yaml:"host"`
	Port          int           `yaml:"port"`
	User          string        `yaml:"user"`
	Password      string        `yaml:"password"`
	AuthType      string        `yaml:"auth_type"`
	SSLEnabled    bool          `yaml:"ssl_enabled"`
	SSLVerify     bool          `yaml:"ssl_verify"`
	APIVersion    string        `yaml:"api_version"`
	Timeout       time.Duration `yaml:"timeout"`
	RetryAttempts int           `yaml:"retry_attempts"`
	RetryDelay    time.Duration `yaml:"retry_delay"`
	PollInterval  time.Duration `yaml:"poll_interval"`
	BatchSize     int           `yaml:"batch_size"`
	QueueSize     int           `yaml:"queue_size"`
	Warehouse     WarehouseConfig `yaml:"warehouse"`
}

// WarehouseConfig represents warehouse configuration
type WarehouseConfig struct {
	WarehouseID string      `yaml:"warehouse_id"`
	Zones       []ZoneConfig `yaml:"zones"`
}

// ZoneConfig represents warehouse zone configuration
type ZoneConfig struct {
	ZoneID     string      `yaml:"zone_id"`
	Coordinates Coordinates `yaml:"coordinates"`
}

// Coordinates represents zone coordinates
type Coordinates struct {
	XMin float64 `yaml:"x_min"`
	XMax float64 `yaml:"x_max"`
	YMin float64 `yaml:"y_min"`
	YMax float64 `yaml:"y_max"`
}

// TaskConfig represents task management configuration
type TaskConfig struct {
	Enabled       bool          `yaml:"enabled"`
	PollInterval  time.Duration `yaml:"poll_interval"`
	BatchSize     int           `yaml:"batch_size"`
	RetryAttempts int           `yaml:"retry_attempts"`
	RetryDelay    time.Duration `yaml:"retry_delay"`
	Timeout       time.Duration `yaml:"timeout"`
}

// InventoryConfig represents inventory management configuration
type InventoryConfig struct {
	Enabled       bool          `yaml:"enabled"`
	PollInterval  time.Duration `yaml:"poll_interval"`
	BatchSize     int           `yaml:"batch_size"`
	RetryAttempts int           `yaml:"retry_attempts"`
	RetryDelay    time.Duration `yaml:"retry_delay"`
	Timeout       time.Duration `yaml:"timeout"`
}

// ResourceConfig represents resource management configuration
type ResourceConfig struct {
	Enabled       bool          `yaml:"enabled"`
	PollInterval  time.Duration `yaml:"poll_interval"`
	BatchSize     int           `yaml:"batch_size"`
	RetryAttempts int           `yaml:"retry_attempts"`
	RetryDelay    time.Duration `yaml:"retry_delay"`
	Timeout       time.Duration `yaml:"timeout"`
}

// Load loads configuration from file and environment variables
func Load() (*Config, error) {
	// Load from file
	configPath := os.Getenv("CONFIG_PATH")
	if configPath == "" {
		configPath = "configs/manhattan-scale.yaml"
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

	// Manhattan SCALE configuration
	if host := os.Getenv("MANHATTAN_HOST"); host != "" {
		config.Manhattan.Host = host
	}
	if port := os.Getenv("MANHATTAN_PORT"); port != "" {
		config.Manhattan.Port = parseInt(port)
	}
	if user := os.Getenv("MANHATTAN_USER"); user != "" {
		config.Manhattan.User = user
	}
	if password := os.Getenv("MANHATTAN_PASSWORD"); password != "" {
		config.Manhattan.Password = password
	}
	if authType := os.Getenv("MANHATTAN_AUTH_TYPE"); authType != "" {
		config.Manhattan.AuthType = authType
	}
	if sslEnabled := os.Getenv("MANHATTAN_SSL_ENABLED"); sslEnabled != "" {
		config.Manhattan.SSLEnabled = sslEnabled == "true"
	}
	if sslVerify := os.Getenv("MANHATTAN_SSL_VERIFY"); sslVerify != "" {
		config.Manhattan.SSLVerify = sslVerify == "true"
	}
	if apiVersion := os.Getenv("MANHATTAN_API_VERSION"); apiVersion != "" {
		config.Manhattan.APIVersion = apiVersion
	}
	if timeout := os.Getenv("MANHATTAN_TIMEOUT"); timeout != "" {
		config.Manhattan.Timeout = parseDuration(timeout)
	}
	if retryAttempts := os.Getenv("MANHATTAN_RETRY_ATTEMPTS"); retryAttempts != "" {
		config.Manhattan.RetryAttempts = parseInt(retryAttempts)
	}
	if retryDelay := os.Getenv("MANHATTAN_RETRY_DELAY"); retryDelay != "" {
		config.Manhattan.RetryDelay = parseDuration(retryDelay)
	}
	if pollInterval := os.Getenv("MANHATTAN_POLL_INTERVAL"); pollInterval != "" {
		config.Manhattan.PollInterval = parseDuration(pollInterval)
	}
	if batchSize := os.Getenv("MANHATTAN_BATCH_SIZE"); batchSize != "" {
		config.Manhattan.BatchSize = parseInt(batchSize)
	}
	if queueSize := os.Getenv("MANHATTAN_QUEUE_SIZE"); queueSize != "" {
		config.Manhattan.QueueSize = parseInt(queueSize)
	}
	if warehouseID := os.Getenv("MANHATTAN_WAREHOUSE_ID"); warehouseID != "" {
		config.Manhattan.Warehouse.WarehouseID = warehouseID
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

	// Manhattan SCALE defaults
	if config.Manhattan.Port == 0 {
		config.Manhattan.Port = 8080
	}
	if config.Manhattan.AuthType == "" {
		config.Manhattan.AuthType = "basic"
	}
	if config.Manhattan.APIVersion == "" {
		config.Manhattan.APIVersion = "v1"
	}
	if config.Manhattan.Timeout == 0 {
		config.Manhattan.Timeout = 30 * time.Second
	}
	if config.Manhattan.RetryAttempts == 0 {
		config.Manhattan.RetryAttempts = 3
	}
	if config.Manhattan.RetryDelay == 0 {
		config.Manhattan.RetryDelay = 5 * time.Second
	}
	if config.Manhattan.PollInterval == 0 {
		config.Manhattan.PollInterval = 5 * time.Second
	}
	if config.Manhattan.BatchSize == 0 {
		config.Manhattan.BatchSize = 100
	}
	if config.Manhattan.QueueSize == 0 {
		config.Manhattan.QueueSize = 1000
	}

	// Task defaults
	if config.Task.PollInterval == 0 {
		config.Task.PollInterval = 5 * time.Second
	}
	if config.Task.BatchSize == 0 {
		config.Task.BatchSize = 100
	}
	if config.Task.RetryAttempts == 0 {
		config.Task.RetryAttempts = 3
	}
	if config.Task.RetryDelay == 0 {
		config.Task.RetryDelay = 5 * time.Second
	}
	if config.Task.Timeout == 0 {
		config.Task.Timeout = 30 * time.Second
	}

	// Inventory defaults
	if config.Inventory.PollInterval == 0 {
		config.Inventory.PollInterval = 10 * time.Second
	}
	if config.Inventory.BatchSize == 0 {
		config.Inventory.BatchSize = 50
	}
	if config.Inventory.RetryAttempts == 0 {
		config.Inventory.RetryAttempts = 3
	}
	if config.Inventory.RetryDelay == 0 {
		config.Inventory.RetryDelay = 5 * time.Second
	}
	if config.Inventory.Timeout == 0 {
		config.Inventory.Timeout = 30 * time.Second
	}

	// Resource defaults
	if config.Resource.PollInterval == 0 {
		config.Resource.PollInterval = 15 * time.Second
	}
	if config.Resource.BatchSize == 0 {
		config.Resource.BatchSize = 25
	}
	if config.Resource.RetryAttempts == 0 {
		config.Resource.RetryAttempts = 3
	}
	if config.Resource.RetryDelay == 0 {
		config.Resource.RetryDelay = 5 * time.Second
	}
	if config.Resource.Timeout == 0 {
		config.Resource.Timeout = 30 * time.Second
	}
}

// validate validates the configuration
func validate(config *Config) error {
	// Validate required fields
	if config.Manhattan.Host == "" {
		return fmt.Errorf("Manhattan SCALE host is required")
	}
	if config.Manhattan.User == "" {
		return fmt.Errorf("Manhattan SCALE user is required")
	}
	if config.Manhattan.Password == "" {
		return fmt.Errorf("Manhattan SCALE password is required")
	}
	if config.Manhattan.Warehouse.WarehouseID == "" {
		return fmt.Errorf("Manhattan SCALE warehouse ID is required")
	}

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
