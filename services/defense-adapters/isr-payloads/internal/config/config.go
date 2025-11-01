package config

import (
	"fmt"
	"os"
	"strconv"
	"time"
)

// Config holds all configuration for the ISR payloads adapter
type Config struct {
	Environment string         `json:"environment"`
	Server      ServerConfig   `json:"server"`
	Database    DatabaseConfig `json:"database"`
	ISR         ISRConfig      `json:"isr"`
	Security    SecurityConfig `json:"security"`
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

// ISRConfig holds ISR-specific configuration
type ISRConfig struct {
	PayloadTypes        []string `json:"payload_types"`
	DataRetentionHours  int      `json:"data_retention_hours"`
	EncryptionEnabled   bool     `json:"encryption_enabled"`
	ClassificationLevel string   `json:"classification_level"`
	RealTimeProcessing  bool     `json:"real_time_processing"`
	MaxPayloadSize      int      `json:"max_payload_size_mb"`
}

// SecurityConfig holds security configuration
type SecurityConfig struct {
	EncryptionKeyPath string `json:"encryption_key_path"`
	CertPath          string `json:"cert_path"`
	KeyPath           string `json:"key_path"`
	CAPath            string `json:"ca_path"`
	MTLSEnabled       bool   `json:"mtls_enabled"`
}

// AuditConfig holds audit configuration
type AuditConfig struct {
	Enabled             bool   `json:"enabled"`
	SigningEnabled      bool   `json:"signing_enabled"`
	SigningKeyPath      string `json:"signing_key_path"`
	RetentionDays       int    `json:"retention_days"`
	ClassificationLevel string `json:"classification_level"`
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
			URL:             getEnvString("DATABASE_URL", "postgres://postgres:password@localhost:5432/atlasmesh_isr?sslmode=disable"),
			MaxConnections:  getEnvInt("DB_MAX_CONNECTIONS", 25),
			ConnMaxLifetime: time.Duration(getEnvInt("DB_CONN_MAX_LIFETIME_MINUTES", 5)) * time.Minute,
			ConnMaxIdleTime: time.Duration(getEnvInt("DB_CONN_MAX_IDLE_MINUTES", 5)) * time.Minute,
		},
		ISR: ISRConfig{
			PayloadTypes:        []string{"video", "imagery", "sensor_data", "telemetry"},
			DataRetentionHours:  getEnvInt("ISR_DATA_RETENTION_HOURS", 168), // 7 days
			EncryptionEnabled:   getEnvBool("ISR_ENCRYPTION_ENABLED", true),
			ClassificationLevel: getEnvString("ISR_CLASSIFICATION_LEVEL", "CONFIDENTIAL"),
			RealTimeProcessing:  getEnvBool("ISR_REAL_TIME_PROCESSING", true),
			MaxPayloadSize:      getEnvInt("ISR_MAX_PAYLOAD_SIZE_MB", 100),
		},
		Security: SecurityConfig{
			EncryptionKeyPath: getEnvString("ENCRYPTION_KEY_PATH", "/app/keys/encryption.key"),
			CertPath:          getEnvString("CERT_PATH", "/app/certs/server.crt"),
			KeyPath:           getEnvString("KEY_PATH", "/app/certs/server.key"),
			CAPath:            getEnvString("CA_PATH", "/app/certs/ca.crt"),
			MTLSEnabled:       getEnvBool("MTLS_ENABLED", true),
		},
		Audit: AuditConfig{
			Enabled:             getEnvBool("AUDIT_ENABLED", true),
			SigningEnabled:      getEnvBool("AUDIT_SIGNING_ENABLED", true),
			SigningKeyPath:      getEnvString("AUDIT_SIGNING_KEY_PATH", "/app/keys/audit-signing.key"),
			RetentionDays:       getEnvInt("AUDIT_RETENTION_DAYS", 2555), // 7 years
			ClassificationLevel: getEnvString("AUDIT_CLASSIFICATION_LEVEL", "CONFIDENTIAL"),
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

	if len(c.ISR.PayloadTypes) == 0 {
		return fmt.Errorf("at least one payload type is required")
	}

	validClassifications := []string{"UNCLASSIFIED", "CONFIDENTIAL", "SECRET", "TOP_SECRET"}
	isValidClassification := false
	for _, valid := range validClassifications {
		if c.ISR.ClassificationLevel == valid {
			isValidClassification = true
			break
		}
	}
	if !isValidClassification {
		return fmt.Errorf("invalid classification level: %s", c.ISR.ClassificationLevel)
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

func getEnvBool(key string, defaultValue bool) bool {
	if value := os.Getenv(key); value != "" {
		if boolValue, err := strconv.ParseBool(value); err == nil {
			return boolValue
		}
	}
	return defaultValue
}
