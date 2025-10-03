package config

import (
	"fmt"
	"os"
	"strconv"
	"time"
)

// Config holds the configuration for the feature flags service
type Config struct {
	// Server configuration
	HTTP HTTPConfig `json:"http"`
	GRPC GRPCConfig `json:"grpc"`
	
	// Storage configuration
	Redis RedisConfig `json:"redis"`
	
	// Feature flags configuration
	FeatureFlags FeatureFlagsConfig `json:"feature_flags"`
	
	// Metrics and monitoring
	Metrics MetricsConfig `json:"metrics"`
	
	// Security
	Security SecurityConfig `json:"security"`
	
	// Environment
	Environment string `json:"environment"`
	LogLevel    string `json:"log_level"`
}

type HTTPConfig struct {
	Port            int           `json:"port"`
	ReadTimeout     time.Duration `json:"read_timeout"`
	WriteTimeout    time.Duration `json:"write_timeout"`
	IdleTimeout     time.Duration `json:"idle_timeout"`
	ShutdownTimeout time.Duration `json:"shutdown_timeout"`
}

type GRPCConfig struct {
	Port               int           `json:"port"`
	MaxConnectionIdle  time.Duration `json:"max_connection_idle"`
	MaxConnectionAge   time.Duration `json:"max_connection_age"`
	MaxConnectionAgeGrace time.Duration `json:"max_connection_age_grace"`
	Time               time.Duration `json:"time"`
	Timeout            time.Duration `json:"timeout"`
}

type RedisConfig struct {
	Address     string        `json:"address"`
	Password    string        `json:"password"`
	DB          int           `json:"db"`
	MaxRetries  int           `json:"max_retries"`
	PoolSize    int           `json:"pool_size"`
	PoolTimeout time.Duration `json:"pool_timeout"`
	IdleTimeout time.Duration `json:"idle_timeout"`
}

type FeatureFlagsConfig struct {
	// Cache settings
	CacheTTL           time.Duration `json:"cache_ttl"`
	CacheRefreshJitter time.Duration `json:"cache_refresh_jitter"`
	
	// Evaluation settings
	DefaultTimeout     time.Duration `json:"default_timeout"`
	MaxBatchSize       int           `json:"max_batch_size"`
	
	// Rollout settings
	DefaultRolloutRate float64       `json:"default_rollout_rate"`
	MaxRolloutRate     float64       `json:"max_rollout_rate"`
	RolloutIncrement   float64       `json:"rollout_increment"`
	
	// Kill switch settings
	KillSwitchTimeout  time.Duration `json:"kill_switch_timeout"`
	AutoKillThreshold  float64       `json:"auto_kill_threshold"`
	
	// Analytics settings
	AnalyticsEnabled   bool          `json:"analytics_enabled"`
	AnalyticsRetention time.Duration `json:"analytics_retention"`
	
	// Cohort settings
	MaxCohortSize      int           `json:"max_cohort_size"`
	CohortTTL          time.Duration `json:"cohort_ttl"`
}

type MetricsConfig struct {
	Enabled         bool          `json:"enabled"`
	Port            int           `json:"port"`
	Path            string        `json:"path"`
	CollectInterval time.Duration `json:"collect_interval"`
}

type SecurityConfig struct {
	// API authentication
	APIKeyRequired bool   `json:"api_key_required"`
	APIKeyHeader   string `json:"api_key_header"`
	
	// JWT authentication
	JWTEnabled    bool   `json:"jwt_enabled"`
	JWTSecret     string `json:"jwt_secret"`
	JWTIssuer     string `json:"jwt_issuer"`
	JWTAudience   string `json:"jwt_audience"`
	
	// Rate limiting
	RateLimitEnabled bool `json:"rate_limit_enabled"`
	RateLimitRPS     int  `json:"rate_limit_rps"`
	RateLimitBurst   int  `json:"rate_limit_burst"`
	
	// CORS
	CORSEnabled bool     `json:"cors_enabled"`
	CORSOrigins []string `json:"cors_origins"`
}

// Load loads configuration from environment variables
func Load() (*Config, error) {
	cfg := &Config{
		HTTP: HTTPConfig{
			Port:            getEnvInt("HTTP_PORT", 8080),
			ReadTimeout:     getEnvDuration("HTTP_READ_TIMEOUT", 30*time.Second),
			WriteTimeout:    getEnvDuration("HTTP_WRITE_TIMEOUT", 30*time.Second),
			IdleTimeout:     getEnvDuration("HTTP_IDLE_TIMEOUT", 120*time.Second),
			ShutdownTimeout: getEnvDuration("HTTP_SHUTDOWN_TIMEOUT", 30*time.Second),
		},
		GRPC: GRPCConfig{
			Port:                  getEnvInt("GRPC_PORT", 9090),
			MaxConnectionIdle:     getEnvDuration("GRPC_MAX_CONNECTION_IDLE", 15*time.Minute),
			MaxConnectionAge:      getEnvDuration("GRPC_MAX_CONNECTION_AGE", 30*time.Minute),
			MaxConnectionAgeGrace: getEnvDuration("GRPC_MAX_CONNECTION_AGE_GRACE", 5*time.Minute),
			Time:                  getEnvDuration("GRPC_KEEPALIVE_TIME", 5*time.Minute),
			Timeout:               getEnvDuration("GRPC_KEEPALIVE_TIMEOUT", 1*time.Minute),
		},
		Redis: RedisConfig{
			Address:     getEnvString("REDIS_ADDRESS", "localhost:6379"),
			Password:    getEnvString("REDIS_PASSWORD", ""),
			DB:          getEnvInt("REDIS_DB", 0),
			MaxRetries:  getEnvInt("REDIS_MAX_RETRIES", 3),
			PoolSize:    getEnvInt("REDIS_POOL_SIZE", 10),
			PoolTimeout: getEnvDuration("REDIS_POOL_TIMEOUT", 30*time.Second),
			IdleTimeout: getEnvDuration("REDIS_IDLE_TIMEOUT", 5*time.Minute),
		},
		FeatureFlags: FeatureFlagsConfig{
			CacheTTL:           getEnvDuration("CACHE_TTL", 5*time.Minute),
			CacheRefreshJitter: getEnvDuration("CACHE_REFRESH_JITTER", 30*time.Second),
			DefaultTimeout:     getEnvDuration("DEFAULT_TIMEOUT", 5*time.Second),
			MaxBatchSize:       getEnvInt("MAX_BATCH_SIZE", 100),
			DefaultRolloutRate: getEnvFloat("DEFAULT_ROLLOUT_RATE", 0.0),
			MaxRolloutRate:     getEnvFloat("MAX_ROLLOUT_RATE", 1.0),
			RolloutIncrement:   getEnvFloat("ROLLOUT_INCREMENT", 0.1),
			KillSwitchTimeout:  getEnvDuration("KILL_SWITCH_TIMEOUT", 1*time.Minute),
			AutoKillThreshold:  getEnvFloat("AUTO_KILL_THRESHOLD", 0.05), // 5% error rate
			AnalyticsEnabled:   getEnvBool("ANALYTICS_ENABLED", true),
			AnalyticsRetention: getEnvDuration("ANALYTICS_RETENTION", 30*24*time.Hour), // 30 days
			MaxCohortSize:      getEnvInt("MAX_COHORT_SIZE", 10000),
			CohortTTL:          getEnvDuration("COHORT_TTL", 24*time.Hour),
		},
		Metrics: MetricsConfig{
			Enabled:         getEnvBool("METRICS_ENABLED", true),
			Port:            getEnvInt("METRICS_PORT", 8081),
			Path:            getEnvString("METRICS_PATH", "/metrics"),
			CollectInterval: getEnvDuration("METRICS_COLLECT_INTERVAL", 15*time.Second),
		},
		Security: SecurityConfig{
			APIKeyRequired:   getEnvBool("API_KEY_REQUIRED", false),
			APIKeyHeader:     getEnvString("API_KEY_HEADER", "X-API-Key"),
			JWTEnabled:       getEnvBool("JWT_ENABLED", true),
			JWTSecret:        getEnvString("JWT_SECRET", ""),
			JWTIssuer:        getEnvString("JWT_ISSUER", "atlasmesh-feature-flags"),
			JWTAudience:      getEnvString("JWT_AUDIENCE", "atlasmesh-fleet-os"),
			RateLimitEnabled: getEnvBool("RATE_LIMIT_ENABLED", true),
			RateLimitRPS:     getEnvInt("RATE_LIMIT_RPS", 1000),
			RateLimitBurst:   getEnvInt("RATE_LIMIT_BURST", 2000),
			CORSEnabled:      getEnvBool("CORS_ENABLED", true),
			CORSOrigins:      getEnvStringSlice("CORS_ORIGINS", []string{"*"}),
		},
		Environment: getEnvString("ENVIRONMENT", "development"),
		LogLevel:    getEnvString("LOG_LEVEL", "info"),
	}

	// Validate configuration
	if err := cfg.Validate(); err != nil {
		return nil, fmt.Errorf("invalid configuration: %w", err)
	}

	return cfg, nil
}

// Validate validates the configuration
func (c *Config) Validate() error {
	if c.HTTP.Port <= 0 || c.HTTP.Port > 65535 {
		return fmt.Errorf("invalid HTTP port: %d", c.HTTP.Port)
	}

	if c.GRPC.Port <= 0 || c.GRPC.Port > 65535 {
		return fmt.Errorf("invalid gRPC port: %d", c.GRPC.Port)
	}

	if c.Redis.Address == "" {
		return fmt.Errorf("Redis address is required")
	}

	if c.FeatureFlags.MaxBatchSize <= 0 {
		return fmt.Errorf("max batch size must be positive")
	}

	if c.FeatureFlags.DefaultRolloutRate < 0 || c.FeatureFlags.DefaultRolloutRate > 1 {
		return fmt.Errorf("default rollout rate must be between 0 and 1")
	}

	if c.FeatureFlags.MaxRolloutRate < 0 || c.FeatureFlags.MaxRolloutRate > 1 {
		return fmt.Errorf("max rollout rate must be between 0 and 1")
	}

	if c.Security.JWTEnabled && c.Security.JWTSecret == "" {
		return fmt.Errorf("JWT secret is required when JWT is enabled")
	}

	return nil
}

// Helper functions to get environment variables with defaults
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

func getEnvDuration(key string, defaultValue time.Duration) time.Duration {
	if value := os.Getenv(key); value != "" {
		if duration, err := time.ParseDuration(value); err == nil {
			return duration
		}
	}
	return defaultValue
}

func getEnvStringSlice(key string, defaultValue []string) []string {
	if value := os.Getenv(key); value != "" {
		// Simple comma-separated parsing
		// In production, you might want more sophisticated parsing
		return []string{value}
	}
	return defaultValue
}
