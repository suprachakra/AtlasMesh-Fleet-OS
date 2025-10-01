package config

import (
	"fmt"
	"os"
	"strconv"
	"time"
)

// Config holds all configuration for the policy engine service
type Config struct {
	Environment string         `json:"environment"`
	Server      ServerConfig   `json:"server"`
	Database    DatabaseConfig `json:"database"`
	Policy      PolicyConfig   `json:"policy"`
	Audit       AuditConfig    `json:"audit"`
	Metrics     MetricsConfig  `json:"metrics"`
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

// PolicyConfig holds policy engine specific configuration
type PolicyConfig struct {
	EvaluationTimeoutMs int    `json:"evaluation_timeout_ms"`
	CacheEnabled        bool   `json:"cache_enabled"`
	CacheTTLSeconds     int    `json:"cache_ttl_seconds"`
	MaxCacheSize        int    `json:"max_cache_size"`
	PolicyStorePath     string `json:"policy_store_path"`
	RegoEngine          string `json:"rego_engine"` // "opa" or "cedar"
}

// AuditConfig holds audit configuration
type AuditConfig struct {
	Enabled             bool   `json:"enabled"`
	SigningEnabled      bool   `json:"signing_enabled"`
	SigningKeyPath      string `json:"signing_key_path"`
	RetentionDays       int    `json:"retention_days"`
	BatchSize           int    `json:"batch_size"`
	FlushIntervalMs     int    `json:"flush_interval_ms"`
	CompressionEnabled  bool   `json:"compression_enabled"`
}

// MetricsConfig holds metrics configuration
type MetricsConfig struct {
	Enabled           bool   `json:"enabled"`
	Namespace         string `json:"namespace"`
	Subsystem         string `json:"subsystem"`
	HistogramBuckets  []float64 `json:"histogram_buckets"`
}

// Load loads configuration from environment variables with sensible defaults
func Load() (*Config, error) {
	cfg := &Config{
		Environment: getEnvString("ENVIRONMENT", "development"),
		Server: ServerConfig{
			HTTPPort: getEnvInt("HTTP_PORT", 8080),
			GRPCPort: getEnvInt("GRPC_PORT", 9090),
		},
		Database: DatabaseConfig{
			URL:             getEnvString("DATABASE_URL", "postgres://postgres:password@localhost:5432/atlasmesh_policy?sslmode=disable"),
			MaxConnections:  getEnvInt("DB_MAX_CONNECTIONS", 25),
			ConnMaxLifetime: time.Duration(getEnvInt("DB_CONN_MAX_LIFETIME_MINUTES", 5)) * time.Minute,
			ConnMaxIdleTime: time.Duration(getEnvInt("DB_CONN_MAX_IDLE_MINUTES", 5)) * time.Minute,
		},
		Policy: PolicyConfig{
			EvaluationTimeoutMs: getEnvInt("POLICY_EVALUATION_TIMEOUT_MS", 10),
			CacheEnabled:        getEnvBool("POLICY_CACHE_ENABLED", true),
			CacheTTLSeconds:     getEnvInt("POLICY_CACHE_TTL_SECONDS", 300),
			MaxCacheSize:        getEnvInt("POLICY_MAX_CACHE_SIZE", 10000),
			PolicyStorePath:     getEnvString("POLICY_STORE_PATH", "/app/policies"),
			RegoEngine:          getEnvString("REGO_ENGINE", "opa"),
		},
		Audit: AuditConfig{
			Enabled:            getEnvBool("AUDIT_ENABLED", true),
			SigningEnabled:     getEnvBool("AUDIT_SIGNING_ENABLED", true),
			SigningKeyPath:     getEnvString("AUDIT_SIGNING_KEY_PATH", "/app/keys/audit-signing.key"),
			RetentionDays:      getEnvInt("AUDIT_RETENTION_DAYS", 2555), // 7 years
			BatchSize:          getEnvInt("AUDIT_BATCH_SIZE", 100),
			FlushIntervalMs:    getEnvInt("AUDIT_FLUSH_INTERVAL_MS", 1000),
			CompressionEnabled: getEnvBool("AUDIT_COMPRESSION_ENABLED", true),
		},
		Metrics: MetricsConfig{
			Enabled:   getEnvBool("METRICS_ENABLED", true),
			Namespace: getEnvString("METRICS_NAMESPACE", "atlasmesh"),
			Subsystem: getEnvString("METRICS_SUBSYSTEM", "policy_engine"),
			HistogramBuckets: []float64{0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1, 2.5, 5, 10},
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

	if c.Policy.EvaluationTimeoutMs <= 0 {
		return fmt.Errorf("policy evaluation timeout must be positive")
	}

	if c.Policy.EvaluationTimeoutMs > 1000 {
		return fmt.Errorf("policy evaluation timeout too high: %dms (max 1000ms)", c.Policy.EvaluationTimeoutMs)
	}

	if c.Policy.RegoEngine != "opa" && c.Policy.RegoEngine != "cedar" {
		return fmt.Errorf("unsupported rego engine: %s (supported: opa, cedar)", c.Policy.RegoEngine)
	}

	if c.Audit.RetentionDays <= 0 {
		return fmt.Errorf("audit retention days must be positive")
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
