package config

import (
	"fmt"
	"os"
	"strconv"
	"strings"
)

// Config represents the HD map service configuration
type Config struct {
	Server       ServerConfig       `yaml:"server"`
	Database     DatabaseConfig     `yaml:"database"`
	Storage      StorageConfig      `yaml:"storage"`
	Metrics      MetricsConfig      `yaml:"metrics"`
	LogLevel     string             `yaml:"log_level"`
	MapBuilder   MapBuilderConfig   `yaml:"map_builder"`
	MapServer    MapServerConfig    `yaml:"map_server"`
	Validation   ValidationConfig   `yaml:"validation"`
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

// StorageConfig represents storage configuration
type StorageConfig struct {
	Type     string `yaml:"type"` // S3, MinIO, Local
	Endpoint string `yaml:"endpoint"`
	Bucket   string `yaml:"bucket"`
	AccessKey string `yaml:"access_key"`
	SecretKey string `yaml:"secret_key"`
	Region   string `yaml:"region"`
	Path     string `yaml:"path"`
}

// MetricsConfig represents metrics configuration
type MetricsConfig struct {
	Enabled bool   `yaml:"enabled"`
	Port    int    `yaml:"port"`
	Path    string `yaml:"path"`
}

// MapBuilderConfig represents map builder configuration
type MapBuilderConfig struct {
	ProcessingRate      int     `yaml:"processing_rate_hz"`
	BatchSize           int     `yaml:"batch_size"`
	MaxConcurrency      int     `yaml:"max_concurrency"`
	ValidationEnabled   bool    `yaml:"validation_enabled"`
	QualityThreshold    float64 `yaml:"quality_threshold"`
	UpdateInterval      int     `yaml:"update_interval_s"`
	StoragePath         string  `yaml:"storage_path"`
	TempPath            string  `yaml:"temp_path"`
}

// MapServerConfig represents map server configuration
type MapServerConfig struct {
	CacheEnabled        bool    `yaml:"cache_enabled"`
	CacheSize           int     `yaml:"cache_size_mb"`
	CacheTTL            int     `yaml:"cache_ttl_s"`
	MaxTileSize         int     `yaml:"max_tile_size_mb"`
	CompressionEnabled  bool    `yaml:"compression_enabled"`
	CompressionLevel    int     `yaml:"compression_level"`
	QueryTimeout        int     `yaml:"query_timeout_s"`
	MaxConcurrency      int     `yaml:"max_concurrency"`
	PreloadEnabled      bool    `yaml:"preload_enabled"`
	PreloadRadius       float64 `yaml:"preload_radius_km"`
}

// ValidationConfig represents validation configuration
type ValidationConfig struct {
	Enabled           bool    `yaml:"enabled"`
	QualityThreshold  float64 `yaml:"quality_threshold"`
	GeometryCheck     bool    `yaml:"geometry_check"`
	TopologyCheck     bool    `yaml:"topology_check"`
	ConsistencyCheck  bool    `yaml:"consistency_check"`
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
		Storage: StorageConfig{
			Type:      getEnv("STORAGE_TYPE", "local"),
			Endpoint:  getEnv("STORAGE_ENDPOINT", ""),
			Bucket:    getEnv("STORAGE_BUCKET", "hd-maps"),
			AccessKey: getEnv("STORAGE_ACCESS_KEY", ""),
			SecretKey: getEnv("STORAGE_SECRET_KEY", ""),
			Region:    getEnv("STORAGE_REGION", "us-east-1"),
			Path:      getEnv("STORAGE_PATH", "/data/maps"),
		},
		Metrics: MetricsConfig{
			Enabled: getEnvBool("METRICS_ENABLED", true),
			Port:    getEnvInt("METRICS_PORT", 9090),
			Path:    getEnv("METRICS_PATH", "/metrics"),
		},
		LogLevel: getEnv("LOG_LEVEL", "info"),
		MapBuilder: MapBuilderConfig{
			ProcessingRate:    getEnvInt("MAP_BUILDER_PROCESSING_RATE", 10),
			BatchSize:         getEnvInt("MAP_BUILDER_BATCH_SIZE", 100),
			MaxConcurrency:    getEnvInt("MAP_BUILDER_MAX_CONCURRENCY", 10),
			ValidationEnabled: getEnvBool("MAP_BUILDER_VALIDATION_ENABLED", true),
			QualityThreshold:  getEnvFloat("MAP_BUILDER_QUALITY_THRESHOLD", 0.8),
			UpdateInterval:    getEnvInt("MAP_BUILDER_UPDATE_INTERVAL", 60),
			StoragePath:       getEnv("MAP_BUILDER_STORAGE_PATH", "/data/maps"),
			TempPath:          getEnv("MAP_BUILDER_TEMP_PATH", "/tmp/maps"),
		},
		MapServer: MapServerConfig{
			CacheEnabled:       getEnvBool("MAP_SERVER_CACHE_ENABLED", true),
			CacheSize:          getEnvInt("MAP_SERVER_CACHE_SIZE", 1024),
			CacheTTL:           getEnvInt("MAP_SERVER_CACHE_TTL", 3600),
			MaxTileSize:        getEnvInt("MAP_SERVER_MAX_TILE_SIZE", 10),
			CompressionEnabled: getEnvBool("MAP_SERVER_COMPRESSION_ENABLED", true),
			CompressionLevel:   getEnvInt("MAP_SERVER_COMPRESSION_LEVEL", 6),
			QueryTimeout:       getEnvInt("MAP_SERVER_QUERY_TIMEOUT", 30),
			MaxConcurrency:     getEnvInt("MAP_SERVER_MAX_CONCURRENCY", 100),
			PreloadEnabled:     getEnvBool("MAP_SERVER_PRELOAD_ENABLED", true),
			PreloadRadius:      getEnvFloat("MAP_SERVER_PRELOAD_RADIUS", 5.0),
		},
		Validation: ValidationConfig{
			Enabled:           getEnvBool("VALIDATION_ENABLED", true),
			QualityThreshold:  getEnvFloat("VALIDATION_QUALITY_THRESHOLD", 0.8),
			GeometryCheck:     getEnvBool("VALIDATION_GEOMETRY_CHECK", true),
			TopologyCheck:     getEnvBool("VALIDATION_TOPOLOGY_CHECK", true),
			ConsistencyCheck:  getEnvBool("VALIDATION_CONSISTENCY_CHECK", true),
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
	if c.Storage.Type == "S3" || c.Storage.Type == "MinIO" {
		if c.Storage.Endpoint == "" {
			return fmt.Errorf("storage endpoint is required for %s", c.Storage.Type)
		}
		if c.Storage.AccessKey == "" {
			return fmt.Errorf("storage access key is required for %s", c.Storage.Type)
		}
		if c.Storage.SecretKey == "" {
			return fmt.Errorf("storage secret key is required for %s", c.Storage.Type)
		}
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
