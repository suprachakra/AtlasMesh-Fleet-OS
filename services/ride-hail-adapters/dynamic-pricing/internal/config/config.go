package config

import (
	"fmt"
	"os"
	"strconv"
	"time"
)

// Config holds all configuration for the dynamic pricing adapter
type Config struct {
	Environment string         `json:"environment"`
	Server      ServerConfig   `json:"server"`
	Database    DatabaseConfig `json:"database"`
	Pricing     PricingConfig  `json:"pricing"`
	Analytics   AnalyticsConfig `json:"analytics"`
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

// PricingConfig holds pricing-specific configuration
type PricingConfig struct {
	BaseFare              float64 `json:"base_fare"`
	PerKmRate             float64 `json:"per_km_rate"`
	PerMinuteRate         float64 `json:"per_minute_rate"`
	SurgeMultiplierMax    float64 `json:"surge_multiplier_max"`
	SurgeMultiplierMin    float64 `json:"surge_multiplier_min"`
	DemandThreshold       float64 `json:"demand_threshold"`
	SupplyThreshold       float64 `json:"supply_threshold"`
	WeatherMultiplier     float64 `json:"weather_multiplier"`
	TimeOfDayMultiplier   float64 `json:"time_of_day_multiplier"`
	UpdateIntervalSeconds int     `json:"update_interval_seconds"`
}

// AnalyticsConfig holds analytics configuration
type AnalyticsConfig struct {
	Enabled           bool    `json:"enabled"`
	PredictionWindow  int     `json:"prediction_window_minutes"`
	HistoricalDays    int     `json:"historical_days"`
	MLModelPath       string  `json:"ml_model_path"`
	ConfidenceThreshold float64 `json:"confidence_threshold"`
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
			URL:             getEnvString("DATABASE_URL", "postgres://postgres:password@localhost:5432/atlasmesh_pricing?sslmode=disable"),
			MaxConnections:  getEnvInt("DB_MAX_CONNECTIONS", 25),
			ConnMaxLifetime: time.Duration(getEnvInt("DB_CONN_MAX_LIFETIME_MINUTES", 5)) * time.Minute,
			ConnMaxIdleTime: time.Duration(getEnvInt("DB_CONN_MAX_IDLE_MINUTES", 5)) * time.Minute,
		},
		Pricing: PricingConfig{
			BaseFare:              getEnvFloat("BASE_FARE", 2.50),
			PerKmRate:             getEnvFloat("PER_KM_RATE", 1.20),
			PerMinuteRate:         getEnvFloat("PER_MINUTE_RATE", 0.30),
			SurgeMultiplierMax:    getEnvFloat("SURGE_MULTIPLIER_MAX", 3.0),
			SurgeMultiplierMin:    getEnvFloat("SURGE_MULTIPLIER_MIN", 1.0),
			DemandThreshold:       getEnvFloat("DEMAND_THRESHOLD", 0.7),
			SupplyThreshold:       getEnvFloat("SUPPLY_THRESHOLD", 0.3),
			WeatherMultiplier:     getEnvFloat("WEATHER_MULTIPLIER", 1.5),
			TimeOfDayMultiplier:   getEnvFloat("TIME_OF_DAY_MULTIPLIER", 1.2),
			UpdateIntervalSeconds: getEnvInt("PRICING_UPDATE_INTERVAL_SECONDS", 60),
		},
		Analytics: AnalyticsConfig{
			Enabled:            getEnvBool("ANALYTICS_ENABLED", true),
			PredictionWindow:   getEnvInt("PREDICTION_WINDOW_MINUTES", 30),
			HistoricalDays:     getEnvInt("HISTORICAL_DAYS", 30),
			MLModelPath:        getEnvString("ML_MODEL_PATH", "/app/models/pricing_model.pkl"),
			ConfidenceThreshold: getEnvFloat("CONFIDENCE_THRESHOLD", 0.8),
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

	if c.Pricing.BaseFare <= 0 {
		return fmt.Errorf("base fare must be positive")
	}

	if c.Pricing.PerKmRate <= 0 {
		return fmt.Errorf("per km rate must be positive")
	}

	if c.Pricing.SurgeMultiplierMax < c.Pricing.SurgeMultiplierMin {
		return fmt.Errorf("surge multiplier max must be >= min")
	}

	if c.Pricing.UpdateIntervalSeconds <= 0 {
		return fmt.Errorf("pricing update interval must be positive")
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
