package config

import (
	"fmt"
	"os"
	"strconv"
	"time"
)

// Config holds all configuration for the telemetry ingestion service
type Config struct {
	Server         ServerConfig
	Kafka          KafkaConfig
	SchemaRegistry SchemaRegistryConfig
	ClickHouse     ClickHouseConfig
	MinIO          MinIOConfig
	Ingestion      IngestionConfig
	Metrics        MetricsConfig
}

// ServerConfig holds HTTP server configuration
type ServerConfig struct {
	Port         int
	ReadTimeout  time.Duration
	WriteTimeout time.Duration
	IdleTimeout  time.Duration
}

// KafkaConfig holds Kafka configuration
type KafkaConfig struct {
	Brokers                []string
	TelemetryTopic         string
	CommandTopic           string
	DeadLetterTopic        string
	ProducerRetries        int
	ProducerRetryBackoff   time.Duration
	ProducerFlushFrequency time.Duration
	ProducerBatchSize      int
	CompressionType        string
	SecurityProtocol       string
	SASLMechanism          string
	SASLUsername           string
	SASLPassword           string
}

// SchemaRegistryConfig holds schema registry configuration
type SchemaRegistryConfig struct {
	URL      string
	Username string
	Password string
	Timeout  time.Duration
}

// ClickHouseConfig holds ClickHouse configuration
type ClickHouseConfig struct {
	Host            string
	Port            int
	Database        string
	Username        string
	Password        string
	MaxOpenConns    int
	MaxIdleConns    int
	ConnMaxLifetime time.Duration
	Timeout         time.Duration
}

// MinIOConfig holds MinIO/S3 configuration
type MinIOConfig struct {
	Endpoint        string
	AccessKeyID     string
	SecretAccessKey string
	BucketName      string
	Region          string
	UseSSL          bool
	Timeout         time.Duration
}

// IngestionConfig holds ingestion-specific configuration
type IngestionConfig struct {
	BatchSize           int
	BatchTimeout        time.Duration
	MaxConcurrency      int
	RetentionHot        time.Duration
	RetentionCold       time.Duration
	CompressionEnabled  bool
	ValidationEnabled   bool
	DeduplicationWindow time.Duration
	RateLimitRPS        int
}

// MetricsConfig holds metrics configuration
type MetricsConfig struct {
	Enabled           bool
	CollectionInterval time.Duration
	RetentionPeriod    time.Duration
}

// Load loads configuration from environment variables with defaults
func Load() (*Config, error) {
	config := &Config{
		Server: ServerConfig{
			Port:         getEnvAsInt("SERVER_PORT", 8080),
			ReadTimeout:  getEnvAsDuration("SERVER_READ_TIMEOUT", 30*time.Second),
			WriteTimeout: getEnvAsDuration("SERVER_WRITE_TIMEOUT", 30*time.Second),
			IdleTimeout:  getEnvAsDuration("SERVER_IDLE_TIMEOUT", 60*time.Second),
		},
		Kafka: KafkaConfig{
			Brokers:                getEnvAsSlice("KAFKA_BROKERS", []string{"localhost:9092"}),
			TelemetryTopic:         getEnv("KAFKA_TELEMETRY_TOPIC", "vehicle-telemetry"),
			CommandTopic:           getEnv("KAFKA_COMMAND_TOPIC", "vehicle-commands"),
			DeadLetterTopic:        getEnv("KAFKA_DLQ_TOPIC", "dead-letter-queue"),
			ProducerRetries:        getEnvAsInt("KAFKA_PRODUCER_RETRIES", 3),
			ProducerRetryBackoff:   getEnvAsDuration("KAFKA_PRODUCER_RETRY_BACKOFF", 100*time.Millisecond),
			ProducerFlushFrequency: getEnvAsDuration("KAFKA_PRODUCER_FLUSH_FREQUENCY", 100*time.Millisecond),
			ProducerBatchSize:      getEnvAsInt("KAFKA_PRODUCER_BATCH_SIZE", 16384),
			CompressionType:        getEnv("KAFKA_COMPRESSION_TYPE", "snappy"),
			SecurityProtocol:       getEnv("KAFKA_SECURITY_PROTOCOL", "PLAINTEXT"),
			SASLMechanism:          getEnv("KAFKA_SASL_MECHANISM", ""),
			SASLUsername:           getEnv("KAFKA_SASL_USERNAME", ""),
			SASLPassword:           getEnv("KAFKA_SASL_PASSWORD", ""),
		},
		SchemaRegistry: SchemaRegistryConfig{
			URL:      getEnv("SCHEMA_REGISTRY_URL", "http://localhost:8081"),
			Username: getEnv("SCHEMA_REGISTRY_USERNAME", ""),
			Password: getEnv("SCHEMA_REGISTRY_PASSWORD", ""),
			Timeout:  getEnvAsDuration("SCHEMA_REGISTRY_TIMEOUT", 10*time.Second),
		},
		ClickHouse: ClickHouseConfig{
			Host:            getEnv("CLICKHOUSE_HOST", "localhost"),
			Port:            getEnvAsInt("CLICKHOUSE_PORT", 9000),
			Database:        getEnv("CLICKHOUSE_DATABASE", "atlasmesh_telemetry"),
			Username:        getEnv("CLICKHOUSE_USERNAME", "default"),
			Password:        getEnv("CLICKHOUSE_PASSWORD", ""),
			MaxOpenConns:    getEnvAsInt("CLICKHOUSE_MAX_OPEN_CONNS", 10),
			MaxIdleConns:    getEnvAsInt("CLICKHOUSE_MAX_IDLE_CONNS", 5),
			ConnMaxLifetime: getEnvAsDuration("CLICKHOUSE_CONN_MAX_LIFETIME", 5*time.Minute),
			Timeout:         getEnvAsDuration("CLICKHOUSE_TIMEOUT", 10*time.Second),
		},
		MinIO: MinIOConfig{
			Endpoint:        getEnv("MINIO_ENDPOINT", "localhost:9000"),
			AccessKeyID:     getEnv("MINIO_ACCESS_KEY_ID", "minioadmin"),
			SecretAccessKey: getEnv("MINIO_SECRET_ACCESS_KEY", "minioadmin"),
			BucketName:      getEnv("MINIO_BUCKET_NAME", "atlasmesh-telemetry"),
			Region:          getEnv("MINIO_REGION", "us-east-1"),
			UseSSL:          getEnvAsBool("MINIO_USE_SSL", false),
			Timeout:         getEnvAsDuration("MINIO_TIMEOUT", 30*time.Second),
		},
		Ingestion: IngestionConfig{
			BatchSize:           getEnvAsInt("INGESTION_BATCH_SIZE", 1000),
			BatchTimeout:        getEnvAsDuration("INGESTION_BATCH_TIMEOUT", 5*time.Second),
			MaxConcurrency:      getEnvAsInt("INGESTION_MAX_CONCURRENCY", 10),
			RetentionHot:        getEnvAsDuration("INGESTION_RETENTION_HOT", 7*24*time.Hour),   // 7 days
			RetentionCold:       getEnvAsDuration("INGESTION_RETENTION_COLD", 365*24*time.Hour), // 1 year
			CompressionEnabled:  getEnvAsBool("INGESTION_COMPRESSION_ENABLED", true),
			ValidationEnabled:   getEnvAsBool("INGESTION_VALIDATION_ENABLED", true),
			DeduplicationWindow: getEnvAsDuration("INGESTION_DEDUPLICATION_WINDOW", 1*time.Minute),
			RateLimitRPS:        getEnvAsInt("INGESTION_RATE_LIMIT_RPS", 1000),
		},
		Metrics: MetricsConfig{
			Enabled:            getEnvAsBool("METRICS_ENABLED", true),
			CollectionInterval: getEnvAsDuration("METRICS_COLLECTION_INTERVAL", 30*time.Second),
			RetentionPeriod:    getEnvAsDuration("METRICS_RETENTION_PERIOD", 30*24*time.Hour), // 30 days
		},
	}

	// Validate configuration
	if err := config.validate(); err != nil {
		return nil, fmt.Errorf("configuration validation failed: %w", err)
	}

	return config, nil
}

// validate performs basic configuration validation
func (c *Config) validate() error {
	if c.Server.Port <= 0 || c.Server.Port > 65535 {
		return fmt.Errorf("invalid server port: %d", c.Server.Port)
	}

	if len(c.Kafka.Brokers) == 0 {
		return fmt.Errorf("kafka brokers cannot be empty")
	}

	if c.Kafka.TelemetryTopic == "" {
		return fmt.Errorf("kafka telemetry topic cannot be empty")
	}

	if c.SchemaRegistry.URL == "" {
		return fmt.Errorf("schema registry URL cannot be empty")
	}

	if c.ClickHouse.Host == "" {
		return fmt.Errorf("clickhouse host cannot be empty")
	}

	if c.MinIO.Endpoint == "" {
		return fmt.Errorf("minio endpoint cannot be empty")
	}

	if c.Ingestion.BatchSize <= 0 {
		return fmt.Errorf("ingestion batch size must be positive")
	}

	if c.Ingestion.MaxConcurrency <= 0 {
		return fmt.Errorf("ingestion max concurrency must be positive")
	}

	return nil
}

// Helper functions for environment variable parsing

func getEnv(key, defaultValue string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return defaultValue
}

func getEnvAsInt(key string, defaultValue int) int {
	if value := os.Getenv(key); value != "" {
		if intValue, err := strconv.Atoi(value); err == nil {
			return intValue
		}
	}
	return defaultValue
}

func getEnvAsBool(key string, defaultValue bool) bool {
	if value := os.Getenv(key); value != "" {
		if boolValue, err := strconv.ParseBool(value); err == nil {
			return boolValue
		}
	}
	return defaultValue
}

func getEnvAsDuration(key string, defaultValue time.Duration) time.Duration {
	if value := os.Getenv(key); value != "" {
		if duration, err := time.ParseDuration(value); err == nil {
			return duration
		}
	}
	return defaultValue
}

func getEnvAsSlice(key string, defaultValue []string) []string {
	if value := os.Getenv(key); value != "" {
		// Simple comma-separated parsing
		// In production, you might want more sophisticated parsing
		return []string{value} // Simplified for this example
	}
	return defaultValue
}
