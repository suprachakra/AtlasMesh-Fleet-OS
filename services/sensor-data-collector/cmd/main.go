package main

import (
	"context"
	"database/sql"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net/http"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/confluentinc/confluent-kafka-go/kafka"
	"github.com/gorilla/mux"
	"github.com/lib/pq"
	_ "github.com/lib/pq"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"github.com/redis/go-redis/v9"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// AtlasMesh Sensor Data Collector Service
// Comprehensive sensor data ingestion, validation, and storage optimization
// Abu Dhabi Autonomous Vehicle Operations

type Config struct {
	Port                    int                    `json:"port"`
	DatabaseURL            string                 `json:"database_url"`
	RedisURL               string                 `json:"redis_url"`
	KafkaBrokers           string                 `json:"kafka_brokers"`
	ClickHouseURL          string                 `json:"clickhouse_url"`
	MinIOURL               string                 `json:"minio_url"`
	AbuDhabiMode           bool                   `json:"abu_dhabi_mode"`
	ValidationRules        ValidationConfig       `json:"validation_rules"`
	StorageOptimization    StorageConfig          `json:"storage_optimization"`
	RetentionPolicies      RetentionConfig        `json:"retention_policies"`
	DataQualityThresholds  DataQualityConfig      `json:"data_quality_thresholds"`
}

type ValidationConfig struct {
	EnableStrictValidation bool                   `json:"enable_strict_validation"`
	AllowedSensorTypes     []string               `json:"allowed_sensor_types"`
	ValueRanges           map[string]ValueRange   `json:"value_ranges"`
	RequiredFields        []string               `json:"required_fields"`
	AbuDhabiSpecific      AbuDhabiValidation     `json:"abu_dhabi_specific"`
}

type ValueRange struct {
	Min         float64 `json:"min"`
	Max         float64 `json:"max"`
	Unit        string  `json:"unit"`
	Description string  `json:"description"`
}

type AbuDhabiValidation struct {
	GeofenceValidation    bool    `json:"geofence_validation"`
	TemperatureRange     ValueRange `json:"temperature_range"`
	HumidityRange        ValueRange `json:"humidity_range"`
	DustLevelMax         float64 `json:"dust_level_max"`
	AllowedTimeZone      string  `json:"allowed_timezone"`
}

type StorageConfig struct {
	HotStorageDuration    time.Duration          `json:"hot_storage_duration"`
	ColdStorageDuration   time.Duration          `json:"cold_storage_duration"`
	CompressionEnabled    bool                   `json:"compression_enabled"`
	PartitioningStrategy  string                 `json:"partitioning_strategy"`
	IndexingStrategy      []string               `json:"indexing_strategy"`
	BatchSize            int                    `json:"batch_size"`
	FlushInterval        time.Duration          `json:"flush_interval"`
}

type RetentionConfig struct {
	RealTimeData         time.Duration          `json:"real_time_data"`
	AggregatedData       time.Duration          `json:"aggregated_data"`
	HistoricalData       time.Duration          `json:"historical_data"`
	ComplianceData       time.Duration          `json:"compliance_data"`
	DebugData           time.Duration          `json:"debug_data"`
	AuditLogs           time.Duration          `json:"audit_logs"`
}

type DataQualityConfig struct {
	MinQualityScore      float64                `json:"min_quality_score"`
	MaxLatency          time.Duration          `json:"max_latency"`
	RequiredAccuracy    float64                `json:"required_accuracy"`
	OutlierDetection    bool                   `json:"outlier_detection"`
	AnomalyThreshold    float64                `json:"anomaly_threshold"`
}

type SensorDataCollectorService struct {
	config              Config
	db                  *sql.DB
	redis               *redis.Client
	kafkaProducer       *kafka.Producer
	kafkaConsumer       *kafka.Consumer
	clickHouseClient    *sql.DB
	metrics             *SensorMetrics
	tracer              trace.Tracer
	validator           *DataValidator
	storageManager      *StorageManager
	retentionManager    *RetentionManager
	qualityMonitor      *DataQualityMonitor
	ingestionPipelines  map[string]*IngestionPipeline
	pipelinesMutex      sync.RWMutex
}

type SensorMetrics struct {
	SensorDataReceived    *prometheus.CounterVec
	SensorDataProcessed   *prometheus.CounterVec
	ValidationErrors      *prometheus.CounterVec
	StorageOperations     *prometheus.CounterVec
	DataQualityScore      *prometheus.GaugeVec
	IngestionLatency      *prometheus.HistogramVec
	StorageUtilization    *prometheus.GaugeVec
	RetentionOperations   *prometheus.CounterVec
}

// Comprehensive sensor data structures
type SensorDataPacket struct {
	VehicleID       string                 `json:"vehicle_id"`
	Timestamp       time.Time              `json:"timestamp"`
	SensorType      string                 `json:"sensor_type"`
	SensorID        string                 `json:"sensor_id"`
	Location        Location               `json:"location"`
	RawData         map[string]interface{} `json:"raw_data"`
	ProcessedData   map[string]interface{} `json:"processed_data"`
	Metadata        SensorMetadata         `json:"metadata"`
	QualityMetrics  QualityMetrics         `json:"quality_metrics"`
	ValidationResult ValidationResult       `json:"validation_result"`
}

type Location struct {
	Latitude    float64   `json:"latitude"`
	Longitude   float64   `json:"longitude"`
	Altitude    float64   `json:"altitude"`
	Accuracy    float64   `json:"accuracy"`
	Heading     float64   `json:"heading"`
	Speed       float64   `json:"speed"`
	Timestamp   time.Time `json:"timestamp"`
}

type SensorMetadata struct {
	SensorModel         string                 `json:"sensor_model"`
	SensorVersion       string                 `json:"sensor_version"`
	CalibrationDate     time.Time              `json:"calibration_date"`
	SamplingRate        float64                `json:"sampling_rate"`
	Resolution          float64                `json:"resolution"`
	OperatingConditions map[string]interface{} `json:"operating_conditions"`
	AbuDhabiAdaptations map[string]interface{} `json:"abu_dhabi_adaptations,omitempty"`
}

type QualityMetrics struct {
	Accuracy            float64   `json:"accuracy"`
	Precision           float64   `json:"precision"`
	Completeness        float64   `json:"completeness"`
	Timeliness          float64   `json:"timeliness"`
	Consistency         float64   `json:"consistency"`
	OverallScore        float64   `json:"overall_score"`
	CalculatedAt        time.Time `json:"calculated_at"`
}

type ValidationResult struct {
	IsValid             bool                   `json:"is_valid"`
	Errors              []string               `json:"errors"`
	Warnings            []string               `json:"warnings"`
	QualityScore        float64                `json:"quality_score"`
	ProcessingTime      time.Duration          `json:"processing_time"`
	ValidatedFields     []string               `json:"validated_fields"`
	AbuDhabiCompliance  bool                   `json:"abu_dhabi_compliance"`
}

type DataValidator struct {
	service     *SensorDataCollectorService
	rules       []ValidationRule
	rulesMutex  sync.RWMutex
}

type ValidationRule struct {
	ID              string                 `json:"id"`
	Name            string                 `json:"name"`
	SensorType      string                 `json:"sensor_type"`
	Field           string                 `json:"field"`
	RuleType        string                 `json:"rule_type"` // range, enum, regex, custom
	Parameters      map[string]interface{} `json:"parameters"`
	Severity        string                 `json:"severity"` // error, warning, info
	Enabled         bool                   `json:"enabled"`
	AbuDhabiSpecific bool                  `json:"abu_dhabi_specific"`
}

type StorageManager struct {
	service         *SensorDataCollectorService
	hotStorage      *HotStorageHandler
	coldStorage     *ColdStorageHandler
	batchProcessor  *BatchProcessor
	compressionMgr  *CompressionManager
}

type HotStorageHandler struct {
	redis           *redis.Client
	clickHouse      *sql.DB
	batchSize       int
	flushInterval   time.Duration
	buffer          []SensorDataPacket
	bufferMutex     sync.Mutex
}

type ColdStorageHandler struct {
	minIOClient     interface{} // MinIO client interface
	compressionType string
	partitionStrategy string
}

type BatchProcessor struct {
	batchSize       int
	flushInterval   time.Duration
	processingQueue chan SensorDataPacket
	workers         []*BatchWorker
}

type BatchWorker struct {
	id              int
	processor       *BatchProcessor
	running         bool
	stopChan        chan bool
}

type CompressionManager struct {
	algorithm       string
	compressionRatio float64
	enabled         bool
}

type RetentionManager struct {
	service         *SensorDataCollectorService
	policies        map[string]RetentionPolicy
	scheduler       *RetentionScheduler
}

type RetentionPolicy struct {
	DataType        string        `json:"data_type"`
	RetentionPeriod time.Duration `json:"retention_period"`
	ArchiveAfter    time.Duration `json:"archive_after"`
	DeleteAfter     time.Duration `json:"delete_after"`
	CompressionType string        `json:"compression_type"`
}

type RetentionScheduler struct {
	retentionMgr    *RetentionManager
	running         bool
	stopChan        chan bool
}

type DataQualityMonitor struct {
	service         *SensorDataCollectorService
	qualityRules    []QualityRule
	anomalyDetector *AnomalyDetector
	reportGenerator *QualityReportGenerator
}

type QualityRule struct {
	ID              string                 `json:"id"`
	Name            string                 `json:"name"`
	MetricType      string                 `json:"metric_type"`
	Threshold       float64                `json:"threshold"`
	Operator        string                 `json:"operator"`
	Parameters      map[string]interface{} `json:"parameters"`
	Enabled         bool                   `json:"enabled"`
}

type AnomalyDetector struct {
	models          map[string]*AnomalyModel
	modelsMutex     sync.RWMutex
	detectionWindow time.Duration
}

type AnomalyModel struct {
	SensorType      string    `json:"sensor_type"`
	Baseline        float64   `json:"baseline"`
	Variance        float64   `json:"variance"`
	SampleCount     int       `json:"sample_count"`
	LastUpdate      time.Time `json:"last_update"`
}

type QualityReportGenerator struct {
	monitor         *DataQualityMonitor
	reportInterval  time.Duration
	running         bool
	stopChan        chan bool
}

type IngestionPipeline struct {
	ID              string                 `json:"id"`
	SensorType      string                 `json:"sensor_type"`
	VehicleID       string                 `json:"vehicle_id"`
	Config          PipelineConfig         `json:"config"`
	Status          string                 `json:"status"`
	Metrics         PipelineMetrics        `json:"metrics"`
	LastActivity    time.Time              `json:"last_activity"`
}

type PipelineConfig struct {
	BufferSize      int           `json:"buffer_size"`
	FlushInterval   time.Duration `json:"flush_interval"`
	ValidationLevel string        `json:"validation_level"`
	StorageTier     string        `json:"storage_tier"`
	RetentionPolicy string        `json:"retention_policy"`
}

type PipelineMetrics struct {
	MessagesProcessed   int64     `json:"messages_processed"`
	ErrorCount         int64     `json:"error_count"`
	AverageLatency     float64   `json:"average_latency"`
	ThroughputPerSec   float64   `json:"throughput_per_sec"`
	LastProcessedAt    time.Time `json:"last_processed_at"`
}

func main() {
	config := loadConfig()
	
	service, err := NewSensorDataCollectorService(config)
	if err != nil {
		log.Fatalf("Failed to create sensor data collector service: %v", err)
	}
	
	// Start the service
	if err := service.Start(); err != nil {
		log.Fatalf("Failed to start service: %v", err)
	}
	
	// Wait for shutdown signal
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c
	
	log.Println("Shutting down sensor data collector service...")
	service.Shutdown()
}

func NewSensorDataCollectorService(config Config) (*SensorDataCollectorService, error) {
	// Initialize PostgreSQL connection
	db, err := sql.Open("postgres", config.DatabaseURL)
	if err != nil {
		return nil, fmt.Errorf("failed to connect to PostgreSQL: %v", err)
	}
	
	// Initialize Redis client
	redisClient := redis.NewClient(&redis.Options{
		Addr: config.RedisURL,
	})
	
	// Initialize ClickHouse connection for time-series data
	clickHouseDB, err := sql.Open("clickhouse", config.ClickHouseURL)
	if err != nil {
		log.Printf("⚠️ ClickHouse connection failed (will use PostgreSQL): %v", err)
		clickHouseDB = db // Fallback to PostgreSQL
	}
	
	// Initialize Kafka producer
	kafkaProducer, err := kafka.NewProducer(&kafka.ConfigMap{
		"bootstrap.servers": config.KafkaBrokers,
		"client.id":        "sensor-data-collector-producer",
		"acks":            "all",
		"retries":         3,
		"batch.size":      16384,
		"linger.ms":       5,
		"compression.type": "snappy",
	})
	if err != nil {
		return nil, fmt.Errorf("failed to create Kafka producer: %v", err)
	}
	
	// Initialize Kafka consumer
	kafkaConsumer, err := kafka.NewConsumer(&kafka.ConfigMap{
		"bootstrap.servers": config.KafkaBrokers,
		"group.id":         "sensor-data-collector-consumer",
		"auto.offset.reset": "latest",
		"enable.auto.commit": false,
	})
	if err != nil {
		return nil, fmt.Errorf("failed to create Kafka consumer: %v", err)
	}
	
	// Initialize metrics
	metrics := &SensorMetrics{
		SensorDataReceived: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "sensor_data_received_total",
				Help: "Total number of sensor data packets received",
			},
			[]string{"vehicle_id", "sensor_type", "status"},
		),
		SensorDataProcessed: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "sensor_data_processed_total",
				Help: "Total number of sensor data packets processed",
			},
			[]string{"vehicle_id", "sensor_type", "processing_stage", "status"},
		),
		ValidationErrors: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "sensor_validation_errors_total",
				Help: "Total number of sensor data validation errors",
			},
			[]string{"vehicle_id", "sensor_type", "error_type"},
		),
		StorageOperations: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "sensor_storage_operations_total",
				Help: "Total number of sensor data storage operations",
			},
			[]string{"storage_type", "operation", "status"},
		),
		DataQualityScore: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "sensor_data_quality_score",
				Help: "Data quality score for sensor streams",
			},
			[]string{"vehicle_id", "sensor_type", "metric_type"},
		),
		IngestionLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "sensor_ingestion_latency_seconds",
				Help: "Sensor data ingestion latency",
				Buckets: []float64{0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0},
			},
			[]string{"vehicle_id", "sensor_type", "processing_stage"},
		),
		StorageUtilization: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "sensor_storage_utilization_percent",
				Help: "Storage utilization percentage",
			},
			[]string{"storage_type", "tier"},
		),
		RetentionOperations: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "sensor_retention_operations_total",
				Help: "Total number of retention operations",
			},
			[]string{"operation_type", "data_type", "status"},
		),
	}
	
	// Register metrics
	prometheus.MustRegister(
		metrics.SensorDataReceived,
		metrics.SensorDataProcessed,
		metrics.ValidationErrors,
		metrics.StorageOperations,
		metrics.DataQualityScore,
		metrics.IngestionLatency,
		metrics.StorageUtilization,
		metrics.RetentionOperations,
	)
	
	service := &SensorDataCollectorService{
		config:             config,
		db:                db,
		redis:             redisClient,
		kafkaProducer:     kafkaProducer,
		kafkaConsumer:     kafkaConsumer,
		clickHouseClient:  clickHouseDB,
		metrics:           metrics,
		tracer:            otel.Tracer("sensor-data-collector"),
		ingestionPipelines: make(map[string]*IngestionPipeline),
	}
	
	// Initialize components
	service.validator = NewDataValidator(service)
	service.storageManager = NewStorageManager(service)
	service.retentionManager = NewRetentionManager(service)
	service.qualityMonitor = NewDataQualityMonitor(service)
	
	return service, nil
}

func (s *SensorDataCollectorService) Start() error {
	log.Println("🚀 Starting AtlasMesh Sensor Data Collector Service...")
	
	// Create necessary database tables
	if err := s.createDatabaseTables(); err != nil {
		return fmt.Errorf("failed to create database tables: %v", err)
	}
	
	// Start Kafka consumer
	go s.startKafkaConsumer()
	
	// Start storage manager
	go s.storageManager.Start()
	
	// Start retention manager
	go s.retentionManager.Start()
	
	// Start data quality monitor
	go s.qualityMonitor.Start()
	
	// Start HTTP server
	go s.startHTTPServer()
	
	log.Printf("✅ Sensor Data Collector Service started on port %d", s.config.Port)
	
	return nil
}

func (s *SensorDataCollectorService) createDatabaseTables() error {
	// Create sensor data tables with optimized schema
	queries := []string{
		`CREATE TABLE IF NOT EXISTS sensor_data_hot (
			id BIGSERIAL PRIMARY KEY,
			vehicle_id UUID NOT NULL,
			sensor_type VARCHAR(50) NOT NULL,
			sensor_id VARCHAR(100) NOT NULL,
			timestamp TIMESTAMP WITH TIME ZONE NOT NULL,
			location_lat DOUBLE PRECISION,
			location_lon DOUBLE PRECISION,
			location_alt DOUBLE PRECISION,
			raw_data JSONB,
			processed_data JSONB,
			metadata JSONB,
			quality_score DOUBLE PRECISION,
			created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
		)`,
		`CREATE INDEX IF NOT EXISTS idx_sensor_data_hot_vehicle_timestamp 
		 ON sensor_data_hot (vehicle_id, timestamp DESC)`,
		`CREATE INDEX IF NOT EXISTS idx_sensor_data_hot_sensor_type_timestamp 
		 ON sensor_data_hot (sensor_type, timestamp DESC)`,
		`CREATE INDEX IF NOT EXISTS idx_sensor_data_hot_location 
		 ON sensor_data_hot (location_lat, location_lon)`,
		`CREATE INDEX IF NOT EXISTS idx_sensor_data_hot_quality 
		 ON sensor_data_hot (quality_score DESC) WHERE quality_score < 0.8`,
		
		// Partitioned table for time-series data
		`CREATE TABLE IF NOT EXISTS sensor_data_timeseries (
			vehicle_id UUID NOT NULL,
			sensor_type VARCHAR(50) NOT NULL,
			timestamp TIMESTAMP WITH TIME ZONE NOT NULL,
			sensor_values DOUBLE PRECISION[],
			location POINT,
			quality_metrics JSONB,
			PRIMARY KEY (vehicle_id, sensor_type, timestamp)
		) PARTITION BY RANGE (timestamp)`,
		
		// Quality metrics table
		`CREATE TABLE IF NOT EXISTS sensor_quality_metrics (
			id BIGSERIAL PRIMARY KEY,
			vehicle_id UUID NOT NULL,
			sensor_type VARCHAR(50) NOT NULL,
			measurement_window INTERVAL NOT NULL,
			accuracy DOUBLE PRECISION,
			precision_val DOUBLE PRECISION,
			completeness DOUBLE PRECISION,
			timeliness DOUBLE PRECISION,
			consistency DOUBLE PRECISION,
			overall_score DOUBLE PRECISION,
			calculated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
		)`,
		
		// Validation results table
		`CREATE TABLE IF NOT EXISTS sensor_validation_results (
			id BIGSERIAL PRIMARY KEY,
			vehicle_id UUID NOT NULL,
			sensor_type VARCHAR(50) NOT NULL,
			validation_timestamp TIMESTAMP WITH TIME ZONE NOT NULL,
			is_valid BOOLEAN NOT NULL,
			errors TEXT[],
			warnings TEXT[],
			quality_score DOUBLE PRECISION,
			processing_time INTERVAL,
			abu_dhabi_compliance BOOLEAN
		)`,
		
		// Retention policies table
		`CREATE TABLE IF NOT EXISTS sensor_retention_policies (
			id SERIAL PRIMARY KEY,
			data_type VARCHAR(100) NOT NULL,
			retention_period INTERVAL NOT NULL,
			archive_after INTERVAL,
			compression_type VARCHAR(50),
			created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
			updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
		)`,
	}
	
	for _, query := range queries {
		if _, err := s.db.Exec(query); err != nil {
			return fmt.Errorf("failed to execute query: %v", err)
		}
	}
	
	// Create time-series partitions for the current and next month
	now := time.Now()
	for i := 0; i < 3; i++ {
		partitionDate := now.AddDate(0, i, 0)
		partitionName := fmt.Sprintf("sensor_data_timeseries_%s", partitionDate.Format("2006_01"))
		startDate := time.Date(partitionDate.Year(), partitionDate.Month(), 1, 0, 0, 0, 0, time.UTC)
		endDate := startDate.AddDate(0, 1, 0)
		
		partitionQuery := fmt.Sprintf(`
			CREATE TABLE IF NOT EXISTS %s PARTITION OF sensor_data_timeseries
			FOR VALUES FROM ('%s') TO ('%s')`,
			partitionName, startDate.Format("2006-01-02"), endDate.Format("2006-01-02"))
		
		if _, err := s.db.Exec(partitionQuery); err != nil {
			log.Printf("⚠️ Failed to create partition %s: %v", partitionName, err)
		}
	}
	
	log.Println("✅ Database tables created successfully")
	return nil
}

func (s *SensorDataCollectorService) startKafkaConsumer() {
	topics := []string{
		"sensor.data.raw",
		"sensor.data.lidar",
		"sensor.data.camera",
		"sensor.data.radar",
		"sensor.data.gps",
		"sensor.data.imu",
		"sensor.data.environmental",
		"sensor.data.vehicle_dynamics",
	}
	
	err := s.kafkaConsumer.SubscribeTopics(topics, nil)
	if err != nil {
		log.Fatalf("Failed to subscribe to Kafka topics: %v", err)
	}
	
	log.Printf("✅ Kafka consumer subscribed to sensor data topics: %v", topics)
	
	for {
		msg, err := s.kafkaConsumer.ReadMessage(-1)
		if err != nil {
			log.Printf("❌ Kafka consumer error: %v", err)
			continue
		}
		
		// Process message asynchronously
		go s.processSensorDataMessage(msg)
	}
}

func (s *SensorDataCollectorService) processSensorDataMessage(msg *kafka.Message) {
	ctx := context.Background()
	span := s.tracer.Start(ctx, "process_sensor_data_message")
	defer span.End()
	
	start := time.Now()
	topic := *msg.TopicPartition.Topic
	
	var sensorData SensorDataPacket
	if err := json.Unmarshal(msg.Value, &sensorData); err != nil {
		log.Printf("❌ Failed to unmarshal sensor data: %v", err)
		s.metrics.SensorDataReceived.WithLabelValues("unknown", "unknown", "unmarshal_error").Inc()
		return
	}
	
	// Record ingestion latency
	defer func() {
		s.metrics.IngestionLatency.WithLabelValues(
			sensorData.VehicleID, 
			sensorData.SensorType, 
			"kafka_processing",
		).Observe(time.Since(start).Seconds())
	}()
	
	s.metrics.SensorDataReceived.WithLabelValues(
		sensorData.VehicleID, 
		sensorData.SensorType, 
		"received",
	).Inc()
	
	// Validate sensor data
	validationResult := s.validator.ValidateData(sensorData)
	sensorData.ValidationResult = validationResult
	
	if !validationResult.IsValid {
		s.metrics.ValidationErrors.WithLabelValues(
			sensorData.VehicleID,
			sensorData.SensorType,
			"validation_failed",
		).Inc()
		
		if s.config.ValidationRules.EnableStrictValidation {
			log.Printf("❌ Sensor data validation failed for %s/%s: %v", 
				sensorData.VehicleID, sensorData.SensorType, validationResult.Errors)
			return
		}
	}
	
	// Calculate quality metrics
	qualityMetrics := s.qualityMonitor.CalculateQualityMetrics(sensorData)
	sensorData.QualityMetrics = qualityMetrics
	
	// Update quality score metric
	s.metrics.DataQualityScore.WithLabelValues(
		sensorData.VehicleID,
		sensorData.SensorType,
		"overall",
	).Set(qualityMetrics.OverallScore)
	
	// Abu Dhabi specific processing
	if s.config.AbuDhabiMode {
		s.enrichSensorDataForAbuDhabi(&sensorData)
	}
	
	// Store sensor data using storage manager
	if err := s.storageManager.StoreSensorData(sensorData); err != nil {
		log.Printf("❌ Failed to store sensor data: %v", err)
		s.metrics.StorageOperations.WithLabelValues("hot", "store", "error").Inc()
		return
	}
	
	// Update ingestion pipeline metrics
	s.updatePipelineMetrics(sensorData)
	
	s.metrics.SensorDataProcessed.WithLabelValues(
		sensorData.VehicleID,
		sensorData.SensorType,
		"complete",
		"success",
	).Inc()
	
	// Commit message
	s.kafkaConsumer.Commit()
	
	log.Printf("✅ Processed sensor data: %s/%s (quality: %.1f%%)", 
		sensorData.VehicleID, sensorData.SensorType, qualityMetrics.OverallScore*100)
}

func (s *SensorDataCollectorService) enrichSensorDataForAbuDhabi(sensorData *SensorDataPacket) {
	// Add Abu Dhabi timezone
	abuDhabiTime := sensorData.Timestamp.In(time.FixedZone("GST", 4*3600))
	
	if sensorData.Metadata.AbuDhabiAdaptations == nil {
		sensorData.Metadata.AbuDhabiAdaptations = make(map[string]interface{})
	}
	
	sensorData.Metadata.AbuDhabiAdaptations["local_time"] = abuDhabiTime.Format("2006-01-02 15:04:05 GST")
	sensorData.Metadata.AbuDhabiAdaptations["region"] = "abu_dhabi"
	
	// Environmental adaptations
	if sensorData.SensorType == "environmental" {
		// Check for extreme heat conditions
		if temp, exists := sensorData.RawData["temperature"]; exists {
			if tempFloat, ok := temp.(float64); ok && tempFloat > 45.0 {
				sensorData.Metadata.AbuDhabiAdaptations["extreme_heat_detected"] = true
				sensorData.Metadata.AbuDhabiAdaptations["cooling_recommendation"] = "immediate"
			}
		}
		
		// Check for dust storm conditions
		if visibility, exists := sensorData.RawData["visibility"]; exists {
			if visFloat, ok := visibility.(float64); ok && visFloat < 1000.0 {
				sensorData.Metadata.AbuDhabiAdaptations["dust_conditions"] = "poor"
				sensorData.Metadata.AbuDhabiAdaptations["filtration_recommendation"] = "enhanced"
			}
		}
	}
	
	// Location-based adaptations
	if s.isWithinAbuDhabi(sensorData.Location) {
		sensorData.Metadata.AbuDhabiAdaptations["within_emirate"] = true
		
		// Add district information (simplified)
		district := s.determineDistrict(sensorData.Location)
		if district != "" {
			sensorData.Metadata.AbuDhabiAdaptations["district"] = district
		}
	}
	
	// Prayer time considerations
	hour := abuDhabiTime.Hour()
	prayerTimes := []int{5, 12, 15, 18, 19} // Approximate prayer times
	for _, prayerHour := range prayerTimes {
		if hour == prayerHour {
			sensorData.Metadata.AbuDhabiAdaptations["prayer_time_proximity"] = true
			break
		}
	}
}

func (s *SensorDataCollectorService) updatePipelineMetrics(sensorData SensorDataPacket) {
	pipelineID := fmt.Sprintf("%s_%s", sensorData.VehicleID, sensorData.SensorType)
	
	s.pipelinesMutex.Lock()
	defer s.pipelinesMutex.Unlock()
	
	pipeline, exists := s.ingestionPipelines[pipelineID]
	if !exists {
		pipeline = &IngestionPipeline{
			ID:         pipelineID,
			SensorType: sensorData.SensorType,
			VehicleID:  sensorData.VehicleID,
			Status:     "active",
			Config: PipelineConfig{
				BufferSize:      1000,
				FlushInterval:   30 * time.Second,
				ValidationLevel: "standard",
				StorageTier:     "hot",
				RetentionPolicy: "default",
			},
			Metrics: PipelineMetrics{},
		}
		s.ingestionPipelines[pipelineID] = pipeline
	}
	
	// Update metrics
	pipeline.Metrics.MessagesProcessed++
	if !sensorData.ValidationResult.IsValid {
		pipeline.Metrics.ErrorCount++
	}
	
	// Update latency (simplified)
	processingLatency := time.Since(sensorData.Timestamp).Seconds()
	if pipeline.Metrics.MessagesProcessed == 1 {
		pipeline.Metrics.AverageLatency = processingLatency
	} else {
		pipeline.Metrics.AverageLatency = (pipeline.Metrics.AverageLatency*float64(pipeline.Metrics.MessagesProcessed-1) + processingLatency) / float64(pipeline.Metrics.MessagesProcessed)
	}
	
	pipeline.Metrics.LastProcessedAt = time.Now()
	pipeline.LastActivity = time.Now()
	
	// Calculate throughput
	if pipeline.Metrics.MessagesProcessed > 1 {
		timeDiff := time.Since(pipeline.Metrics.LastProcessedAt).Seconds()
		if timeDiff > 0 {
			pipeline.Metrics.ThroughputPerSec = 1.0 / timeDiff
		}
	}
}

func (s *SensorDataCollectorService) startHTTPServer() {
	router := mux.NewRouter()
	
	// Health check
	router.HandleFunc("/health", s.healthCheck).Methods("GET")
	
	// Metrics endpoint
	router.Handle("/metrics", promhttp.Handler())
	
	// Sensor data endpoints
	router.HandleFunc("/api/v1/sensors/data/{vehicleId}/{sensorType}", s.getSensorData).Methods("GET")
	router.HandleFunc("/api/v1/sensors/quality/{vehicleId}", s.getDataQuality).Methods("GET")
	router.HandleFunc("/api/v1/sensors/pipelines", s.getIngestionPipelines).Methods("GET")
	router.HandleFunc("/api/v1/sensors/validation/rules", s.getValidationRules).Methods("GET")
	router.HandleFunc("/api/v1/sensors/retention/policies", s.getRetentionPolicies).Methods("GET")
	
	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", s.config.Port),
		Handler: router,
	}
	
	log.Fatal(server.ListenAndServe())
}

func (s *SensorDataCollectorService) healthCheck(w http.ResponseWriter, r *http.Request) {
	status := map[string]interface{}{
		"status":    "healthy",
		"timestamp": time.Now(),
		"version":   "1.0.0",
		"services": map[string]bool{
			"postgresql":     s.db.Ping() == nil,
			"clickhouse":     s.clickHouseClient.Ping() == nil,
			"redis":         s.redis.Ping(context.Background()).Err() == nil,
			"kafka_producer": s.kafkaProducer != nil,
			"kafka_consumer": s.kafkaConsumer != nil,
		},
		"components": map[string]bool{
			"validator":        s.validator != nil,
			"storage_manager":  s.storageManager != nil,
			"retention_manager": s.retentionManager != nil,
			"quality_monitor":  s.qualityMonitor != nil,
		},
		"pipelines": map[string]int{
			"active_pipelines": len(s.ingestionPipelines),
		},
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(status)
}

func (s *SensorDataCollectorService) getSensorData(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	sensorType := vars["sensorType"]
	
	// Get query parameters
	hoursStr := r.URL.Query().Get("hours")
	hours := 24 // Default 24 hours
	
	if hoursStr != "" {
		if h, err := strconv.Atoi(hoursStr); err == nil && h > 0 && h <= 168 {
			hours = h
		}
	}
	
	// Query sensor data from database
	query := `
		SELECT timestamp, raw_data, processed_data, quality_score
		FROM sensor_data_hot 
		WHERE vehicle_id = $1 AND sensor_type = $2 
		AND timestamp > NOW() - INTERVAL '%d hours'
		ORDER BY timestamp DESC
		LIMIT 1000`
	
	rows, err := s.db.Query(fmt.Sprintf(query, hours), vehicleID, sensorType)
	if err != nil {
		http.Error(w, "Failed to query sensor data", http.StatusInternalServerError)
		return
	}
	defer rows.Close()
	
	var sensorDataPoints []map[string]interface{}
	for rows.Next() {
		var timestamp time.Time
		var rawData, processedData []byte
		var qualityScore float64
		
		err := rows.Scan(&timestamp, &rawData, &processedData, &qualityScore)
		if err != nil {
			continue
		}
		
		var rawDataMap, processedDataMap map[string]interface{}
		json.Unmarshal(rawData, &rawDataMap)
		json.Unmarshal(processedData, &processedDataMap)
		
		dataPoint := map[string]interface{}{
			"timestamp":      timestamp,
			"raw_data":       rawDataMap,
			"processed_data": processedDataMap,
			"quality_score":  qualityScore,
		}
		
		sensorDataPoints = append(sensorDataPoints, dataPoint)
	}
	
	response := map[string]interface{}{
		"vehicle_id":   vehicleID,
		"sensor_type":  sensorType,
		"hours":        hours,
		"data_points":  sensorDataPoints,
		"count":        len(sensorDataPoints),
		"timestamp":    time.Now(),
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func (s *SensorDataCollectorService) getDataQuality(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	
	// Get quality metrics for the vehicle
	qualityReport := s.qualityMonitor.GenerateQualityReport(vehicleID)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(qualityReport)
}

func (s *SensorDataCollectorService) getIngestionPipelines(w http.ResponseWriter, r *http.Request) {
	s.pipelinesMutex.RLock()
	defer s.pipelinesMutex.RUnlock()
	
	pipelines := make([]IngestionPipeline, 0, len(s.ingestionPipelines))
	for _, pipeline := range s.ingestionPipelines {
		pipelines = append(pipelines, *pipeline)
	}
	
	response := map[string]interface{}{
		"pipelines": pipelines,
		"count":     len(pipelines),
		"timestamp": time.Now(),
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func (s *SensorDataCollectorService) getValidationRules(w http.ResponseWriter, r *http.Request) {
	rules := s.validator.GetRules()
	
	response := map[string]interface{}{
		"rules":     rules,
		"count":     len(rules),
		"timestamp": time.Now(),
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func (s *SensorDataCollectorService) getRetentionPolicies(w http.ResponseWriter, r *http.Request) {
	policies := s.retentionManager.GetPolicies()
	
	response := map[string]interface{}{
		"policies":  policies,
		"count":     len(policies),
		"timestamp": time.Now(),
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func (s *SensorDataCollectorService) Shutdown() {
	log.Println("🛑 Shutting down sensor data collector service...")
	
	// Close Kafka connections
	if s.kafkaProducer != nil {
		s.kafkaProducer.Close()
	}
	if s.kafkaConsumer != nil {
		s.kafkaConsumer.Close()
	}
	
	// Close database connections
	if s.db != nil {
		s.db.Close()
	}
	if s.clickHouseClient != nil {
		s.clickHouseClient.Close()
	}
	
	// Close Redis connection
	if s.redis != nil {
		s.redis.Close()
	}
	
	// Stop components
	if s.storageManager != nil {
		s.storageManager.Stop()
	}
	if s.retentionManager != nil {
		s.retentionManager.Stop()
	}
	if s.qualityMonitor != nil {
		s.qualityMonitor.Stop()
	}
	
	log.Println("✅ Sensor data collector service shutdown complete")
}

// Utility functions
func (s *SensorDataCollectorService) isWithinAbuDhabi(location Location) bool {
	// Simplified Abu Dhabi boundary check
	return location.Latitude >= 24.0 && location.Latitude <= 25.0 &&
		   location.Longitude >= 54.0 && location.Longitude <= 55.5
}

func (s *SensorDataCollectorService) determineDistrict(location Location) string {
	// Simplified district determination
	if location.Latitude >= 24.45 && location.Latitude <= 24.47 &&
	   location.Longitude >= 54.35 && location.Longitude <= 54.39 {
		return "downtown_abu_dhabi"
	}
	if location.Latitude >= 24.40 && location.Latitude <= 24.44 &&
	   location.Longitude >= 54.60 && location.Longitude <= 54.70 {
		return "al_reem_island"
	}
	if location.Latitude >= 24.50 && location.Latitude <= 24.55 &&
	   location.Longitude >= 54.35 && location.Longitude <= 54.45 {
		return "al_zahiyah"
	}
	return ""
}

func loadConfig() Config {
	return Config{
		Port:           8080,
		DatabaseURL:    getEnv("DATABASE_URL", "postgres://user:pass@localhost/fleet_db?sslmode=disable"),
		RedisURL:       getEnv("REDIS_URL", "localhost:6379"),
		KafkaBrokers:   getEnv("KAFKA_BROKERS", "localhost:9092"),
		ClickHouseURL:  getEnv("CLICKHOUSE_URL", "tcp://localhost:9000/sensor_data"),
		MinIOURL:       getEnv("MINIO_URL", "localhost:9000"),
		AbuDhabiMode:   getEnv("ABU_DHABI_MODE", "true") == "true",
		ValidationRules: ValidationConfig{
			EnableStrictValidation: true,
			AllowedSensorTypes: []string{
				"lidar", "camera", "radar", "gps", "imu", 
				"environmental", "vehicle_dynamics", "ultrasonic",
			},
			ValueRanges: map[string]ValueRange{
				"temperature": {Min: -40, Max: 85, Unit: "celsius", Description: "Operating temperature range"},
				"humidity":    {Min: 0, Max: 100, Unit: "percent", Description: "Relative humidity"},
				"speed":       {Min: 0, Max: 200, Unit: "km/h", Description: "Vehicle speed"},
				"latitude":    {Min: -90, Max: 90, Unit: "degrees", Description: "GPS latitude"},
				"longitude":   {Min: -180, Max: 180, Unit: "degrees", Description: "GPS longitude"},
			},
			RequiredFields: []string{"vehicle_id", "sensor_type", "timestamp"},
			AbuDhabiSpecific: AbuDhabiValidation{
				GeofenceValidation: true,
				TemperatureRange:   ValueRange{Min: -10, Max: 60, Unit: "celsius"},
				HumidityRange:      ValueRange{Min: 10, Max: 90, Unit: "percent"},
				DustLevelMax:       0.8,
				AllowedTimeZone:    "Asia/Dubai",
			},
		},
		StorageOptimization: StorageConfig{
			HotStorageDuration:   24 * time.Hour,
			ColdStorageDuration:  30 * 24 * time.Hour,
			CompressionEnabled:   true,
			PartitioningStrategy: "time_based",
			IndexingStrategy:     []string{"vehicle_id", "sensor_type", "timestamp"},
			BatchSize:           1000,
			FlushInterval:       30 * time.Second,
		},
		RetentionPolicies: RetentionConfig{
			RealTimeData:   7 * 24 * time.Hour,
			AggregatedData: 90 * 24 * time.Hour,
			HistoricalData: 2 * 365 * 24 * time.Hour,
			ComplianceData: 7 * 365 * 24 * time.Hour,
			DebugData:      24 * time.Hour,
			AuditLogs:      5 * 365 * 24 * time.Hour,
		},
		DataQualityThresholds: DataQualityConfig{
			MinQualityScore:   0.7,
			MaxLatency:        30 * time.Second,
			RequiredAccuracy:  0.95,
			OutlierDetection:  true,
			AnomalyThreshold:  0.8,
		},
	}
}

func getEnv(key, defaultValue string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return defaultValue
}

// Additional components will be implemented in separate files:
// - data_validator.go: Data validation logic and rules
// - storage_manager.go: Storage optimization and management
// - retention_manager.go: Data retention and lifecycle management
// - quality_monitor.go: Data quality monitoring and reporting
