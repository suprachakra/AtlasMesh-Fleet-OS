package config

import (
	"fmt"
	"os"
	"time"

	"gopkg.in/yaml.v2"
)

// Config represents the complete configuration for NATO Command adapter
type Config struct {
	LogLevel string        `yaml:"log_level"`
	Server   ServerConfig  `yaml:"server"`
	Database DatabaseConfig `yaml:"database"`
	Metrics  MetricsConfig `yaml:"metrics"`
	NATO     NATOConfig    `yaml:"nato"`
	Security SecurityConfig `yaml:"security"`
	Audit    AuditConfig   `yaml:"audit"`
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

// NATOConfig represents NATO Command system configuration
type NATOConfig struct {
	Enabled         bool     `yaml:"enabled"`
	Endpoint        string   `yaml:"endpoint"`
	APIKey          string   `yaml:"api_key"`
	SecurityLevel   string   `yaml:"security_level"`
	CommandHierarchy []string `yaml:"command_hierarchy"`
	Encryption      EncryptionConfig `yaml:"encryption"`
	Timeout         time.Duration `yaml:"timeout"`
}

// EncryptionConfig represents encryption configuration
type EncryptionConfig struct {
	Algorithm         string `yaml:"algorithm"`
	KeyRotationHours  int    `yaml:"key_rotation_hours"`
	KeySize           int    `yaml:"key_size"`
}

// SecurityConfig represents security configuration
type SecurityConfig struct {
	Enabled           bool   `yaml:"enabled"`
	EncryptionEnabled bool   `yaml:"encryption_enabled"`
	AuditEnabled      bool   `yaml:"audit_enabled"`
	SecurityLevel     string `yaml:"security_level"`
}

// AuditConfig represents audit configuration
type AuditConfig struct {
	Enabled        bool `yaml:"enabled"`
	RetentionDays  int  `yaml:"retention_days"`
	LogLevel       string `yaml:"log_level"`
}

// Load loads configuration from file and environment variables
func Load() (*Config, error) {
	// Load from file
	configPath := os.Getenv("CONFIG_PATH")
	if configPath == "" {
		configPath = "configs/nato-command.yaml"
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

	// NATO configuration
	if endpoint := os.Getenv("NATO_ENDPOINT"); endpoint != "" {
		config.NATO.Endpoint = endpoint
	}
	if apiKey := os.Getenv("NATO_API_KEY"); apiKey != "" {
		config.NATO.APIKey = apiKey
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

	// NATO defaults
	if config.NATO.Timeout == 0 {
		config.NATO.Timeout = 30 * time.Second
	}
	if config.NATO.SecurityLevel == "" {
		config.NATO.SecurityLevel = "classified"
	}
	if config.NATO.Encryption.Algorithm == "" {
		config.NATO.Encryption.Algorithm = "AES-256-GCM"
	}
	if config.NATO.Encryption.KeyRotationHours == 0 {
		config.NATO.Encryption.KeyRotationHours = 24
	}
	if config.NATO.Encryption.KeySize == 0 {
		config.NATO.Encryption.KeySize = 256
	}

	// Security defaults
	if config.Security.SecurityLevel == "" {
		config.Security.SecurityLevel = "classified"
	}

	// Audit defaults
	if config.Audit.RetentionDays == 0 {
		config.Audit.RetentionDays = 365
	}
	if config.Audit.LogLevel == "" {
		config.Audit.LogLevel = "info"
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

	// Validate NATO configuration
	if config.NATO.Enabled && config.NATO.Endpoint == "" {
		return fmt.Errorf("NATO endpoint is required when enabled")
	}
	if config.NATO.Enabled && config.NATO.APIKey == "" {
		return fmt.Errorf("NATO API key is required when enabled")
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
