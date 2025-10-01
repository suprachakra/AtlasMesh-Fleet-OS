package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/confluentinc/confluent-kafka-go/kafka"
	"github.com/gorilla/mux"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// EventBusService manages Kafka event streaming and DLQ patterns
type EventBusService struct {
	producer       *kafka.Producer
	adminClient    *kafka.AdminClient
	schemaRegistry *SchemaRegistry
	dlqHandler     *DLQHandler
	metrics        *EventBusMetrics
	tracer         trace.Tracer
	config         *Config
}

// Config holds the service configuration
type Config struct {
	KafkaBrokers     string `json:"kafka_brokers"`
	SchemaRegistry   string `json:"schema_registry_url"`
	Port             int    `json:"port"`
	MetricsPort      int    `json:"metrics_port"`
	DLQRetentionDays int    `json:"dlq_retention_days"`
	MaxRetries       int    `json:"max_retries"`
	RetryDelayMs     int    `json:"retry_delay_ms"`
}

// EventBusMetrics contains Prometheus metrics
type EventBusMetrics struct {
	MessagesProduced   *prometheus.CounterVec
	MessagesConsumed   *prometheus.CounterVec
	MessagesFailed     *prometheus.CounterVec
	DLQMessages        *prometheus.CounterVec
	ProcessingDuration *prometheus.HistogramVec
	TopicLag           *prometheus.GaugeVec
}

// SchemaRegistry handles Avro schema management
type SchemaRegistry struct {
	baseURL string
	client  *http.Client
}

// DLQHandler manages dead letter queue operations
type DLQHandler struct {
	producer *kafka.Producer
	metrics  *EventBusMetrics
	tracer   trace.Tracer
}

// DLQMessage represents a message in the dead letter queue
type DLQMessage struct {
	ErrorID          string            `json:"error_id"`
	OriginalTopic    string            `json:"original_topic"`
	OriginalKey      string            `json:"original_key,omitempty"`
	OriginalValue    []byte            `json:"original_value"`
	OriginalHeaders  map[string]string `json:"original_headers"`
	ErrorTimestamp   time.Time         `json:"error_timestamp"`
	ErrorType        string            `json:"error_type"`
	ErrorMessage     string            `json:"error_message"`
	ServiceName      string            `json:"service_name"`
	ServiceVersion   string            `json:"service_version"`
	ProcessingAttempt int              `json:"processing_attempt"`
	Priority         string            `json:"priority"`
	CorrelationID    string            `json:"correlation_id,omitempty"`
	TenantID         string            `json:"tenant_id,omitempty"`
	VehicleID        string            `json:"vehicle_id,omitempty"`
	TripID           string            `json:"trip_id,omitempty"`
}

func main() {
	// Load configuration
	config := loadConfig()
	
	// Initialize metrics
	metrics := initializeMetrics()
	
	// Initialize tracer
	tracer := otel.Tracer("event-bus-service")
	
	// Create Kafka producer
	producer, err := kafka.NewProducer(&kafka.ConfigMap{
		"bootstrap.servers": config.KafkaBrokers,
		"client.id":         "atlasmesh-event-bus",
		"acks":              "all",
		"retries":           config.MaxRetries,
		"retry.backoff.ms":  config.RetryDelayMs,
		"compression.type":  "snappy",
		"batch.size":        16384,
		"linger.ms":         5,
		"buffer.memory":     33554432,
	})
	if err != nil {
		log.Fatalf("Failed to create Kafka producer: %v", err)
	}
	defer producer.Close()
	
	// Create Kafka admin client
	adminClient, err := kafka.NewAdminClient(&kafka.ConfigMap{
		"bootstrap.servers": config.KafkaBrokers,
	})
	if err != nil {
		log.Fatalf("Failed to create Kafka admin client: %v", err)
	}
	defer adminClient.Close()
	
	// Initialize schema registry
	schemaRegistry := &SchemaRegistry{
		baseURL: config.SchemaRegistry,
		client:  &http.Client{Timeout: 30 * time.Second},
	}
	
	// Initialize DLQ handler
	dlqHandler := &DLQHandler{
		producer: producer,
		metrics:  metrics,
		tracer:   tracer,
	}
	
	// Create service instance
	service := &EventBusService{
		producer:       producer,
		adminClient:    adminClient,
		schemaRegistry: schemaRegistry,
		dlqHandler:     dlqHandler,
		metrics:        metrics,
		tracer:         tracer,
		config:         config,
	}
	
	// Start HTTP server
	router := mux.NewRouter()
	service.setupRoutes(router)
	
	server := &http.Server{
		Addr:         fmt.Sprintf(":%d", config.Port),
		Handler:      router,
		ReadTimeout:  15 * time.Second,
		WriteTimeout: 15 * time.Second,
		IdleTimeout:  60 * time.Second,
	}
	
	// Start metrics server
	metricsRouter := mux.NewRouter()
	metricsRouter.Handle("/metrics", promhttp.Handler())
	metricsRouter.HandleFunc("/health", service.healthCheck)
	metricsRouter.HandleFunc("/ready", service.readinessCheck)
	
	metricsServer := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.MetricsPort),
		Handler: metricsRouter,
	}
	
	// Start servers
	go func() {
		log.Printf("Starting Event Bus service on port %d", config.Port)
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Server failed to start: %v", err)
		}
	}()
	
	go func() {
		log.Printf("Starting metrics server on port %d", config.MetricsPort)
		if err := metricsServer.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Metrics server failed to start: %v", err)
		}
	}()
	
	// Start background tasks
	go service.startTopicManagement()
	go service.startDLQMonitoring()
	
	// Wait for interrupt signal
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c
	
	log.Println("Shutting down Event Bus service...")
	
	// Graceful shutdown
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	
	server.Shutdown(ctx)
	metricsServer.Shutdown(ctx)
	
	log.Println("Event Bus service stopped")
}

func (s *EventBusService) setupRoutes(router *mux.Router) {
	// API routes
	api := router.PathPrefix("/api/v1").Subrouter()
	
	// Event publishing
	api.HandleFunc("/events/publish", s.publishEvent).Methods("POST")
	api.HandleFunc("/events/batch", s.publishBatch).Methods("POST")
	
	// Topic management
	api.HandleFunc("/topics", s.listTopics).Methods("GET")
	api.HandleFunc("/topics", s.createTopic).Methods("POST")
	api.HandleFunc("/topics/{topic}", s.getTopic).Methods("GET")
	api.HandleFunc("/topics/{topic}", s.deleteTopic).Methods("DELETE")
	
	// Schema management
	api.HandleFunc("/schemas", s.listSchemas).Methods("GET")
	api.HandleFunc("/schemas/{subject}", s.getSchema).Methods("GET")
	api.HandleFunc("/schemas/{subject}", s.registerSchema).Methods("POST")
	
	// DLQ management
	api.HandleFunc("/dlq/topics", s.listDLQTopics).Methods("GET")
	api.HandleFunc("/dlq/{topic}/messages", s.getDLQMessages).Methods("GET")
	api.HandleFunc("/dlq/{topic}/retry", s.retryDLQMessages).Methods("POST")
	api.HandleFunc("/dlq/{topic}/discard", s.discardDLQMessages).Methods("POST")
	
	// Health and status
	api.HandleFunc("/health", s.healthCheck).Methods("GET")
	api.HandleFunc("/ready", s.readinessCheck).Methods("GET")
	api.HandleFunc("/status", s.getStatus).Methods("GET")
}

func (s *EventBusService) publishEvent(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "publish_event")
	defer span.End()
	
	var event struct {
		Topic   string            `json:"topic"`
		Key     string            `json:"key,omitempty"`
		Value   json.RawMessage   `json:"value"`
		Headers map[string]string `json:"headers,omitempty"`
	}
	
	if err := json.NewDecoder(r.Body).Decode(&event); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}
	
	// Validate schema if available
	if err := s.schemaRegistry.ValidateMessage(event.Topic, event.Value); err != nil {
		s.dlqHandler.SendToDLQ(ctx, &DLQMessage{
			ErrorID:        generateErrorID(),
			OriginalTopic:  event.Topic,
			OriginalKey:    event.Key,
			OriginalValue:  event.Value,
			ErrorTimestamp: time.Now(),
			ErrorType:      "SCHEMA_VALIDATION_ERROR",
			ErrorMessage:   err.Error(),
			ServiceName:    "event-bus-service",
			ServiceVersion: "1.0.0",
			Priority:       "MEDIUM",
		})
		http.Error(w, "Schema validation failed", http.StatusBadRequest)
		return
	}
	
	// Produce message
	deliveryChan := make(chan kafka.Event)
	defer close(deliveryChan)
	
	headers := make([]kafka.Header, 0, len(event.Headers))
	for k, v := range event.Headers {
		headers = append(headers, kafka.Header{Key: k, Value: []byte(v)})
	}
	
	err := s.producer.Produce(&kafka.Message{
		TopicPartition: kafka.TopicPartition{Topic: &event.Topic, Partition: kafka.PartitionAny},
		Key:            []byte(event.Key),
		Value:          event.Value,
		Headers:        headers,
	}, deliveryChan)
	
	if err != nil {
		s.metrics.MessagesFailed.WithLabelValues(event.Topic, "produce_error").Inc()
		http.Error(w, "Failed to produce message", http.StatusInternalServerError)
		return
	}
	
	// Wait for delivery confirmation
	select {
	case e := <-deliveryChan:
		if msg, ok := e.(*kafka.Message); ok {
			if msg.TopicPartition.Error != nil {
				s.metrics.MessagesFailed.WithLabelValues(event.Topic, "delivery_error").Inc()
				http.Error(w, "Message delivery failed", http.StatusInternalServerError)
				return
			}
			s.metrics.MessagesProduced.WithLabelValues(event.Topic).Inc()
		}
	case <-time.After(30 * time.Second):
		s.metrics.MessagesFailed.WithLabelValues(event.Topic, "timeout").Inc()
		http.Error(w, "Message delivery timeout", http.StatusRequestTimeout)
		return
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "success"})
}

func (s *EventBusService) startTopicManagement() {
	ticker := time.NewTicker(5 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.updateTopicMetrics()
		}
	}
}

func (s *EventBusService) startDLQMonitoring() {
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.monitorDLQTopics()
		}
	}
}

func (s *EventBusService) updateTopicMetrics() {
	// Get topic metadata
	metadata, err := s.adminClient.GetMetadata(nil, false, 5000)
	if err != nil {
		log.Printf("Failed to get topic metadata: %v", err)
		return
	}
	
	for topicName := range metadata.Topics {
		// Update topic lag metrics
		// This would typically involve querying consumer group offsets
		s.metrics.TopicLag.WithLabelValues(topicName).Set(0) // Placeholder
	}
}

func (s *EventBusService) monitorDLQTopics() {
	// Monitor DLQ topics for high message counts
	// Implement alerting logic here
	log.Println("Monitoring DLQ topics...")
}

func (s *DLQHandler) SendToDLQ(ctx context.Context, dlqMsg *DLQMessage) error {
	dlqTopic := fmt.Sprintf("dlq.%s", getDLQTopicSuffix(dlqMsg.OriginalTopic))
	
	dlqBytes, err := json.Marshal(dlqMsg)
	if err != nil {
		return fmt.Errorf("failed to marshal DLQ message: %w", err)
	}
	
	err = s.producer.Produce(&kafka.Message{
		TopicPartition: kafka.TopicPartition{Topic: &dlqTopic, Partition: kafka.PartitionAny},
		Key:            []byte(dlqMsg.ErrorID),
		Value:          dlqBytes,
		Headers: []kafka.Header{
			{Key: "error_type", Value: []byte(dlqMsg.ErrorType)},
			{Key: "original_topic", Value: []byte(dlqMsg.OriginalTopic)},
			{Key: "priority", Value: []byte(dlqMsg.Priority)},
		},
	}, nil)
	
	if err != nil {
		return fmt.Errorf("failed to send message to DLQ: %w", err)
	}
	
	s.metrics.DLQMessages.WithLabelValues(dlqTopic, dlqMsg.ErrorType).Inc()
	return nil
}

func (s *SchemaRegistry) ValidateMessage(topic string, message json.RawMessage) error {
	// Implement schema validation logic
	// This would typically involve fetching the schema from the registry
	// and validating the message against it
	return nil // Placeholder
}

func (s *EventBusService) healthCheck(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "healthy"})
}

func (s *EventBusService) readinessCheck(w http.ResponseWriter, r *http.Request) {
	// Check Kafka connectivity
	metadata, err := s.adminClient.GetMetadata(nil, false, 5000)
	if err != nil {
		http.Error(w, "Not ready", http.StatusServiceUnavailable)
		return
	}
	
	if len(metadata.Brokers) == 0 {
		http.Error(w, "No Kafka brokers available", http.StatusServiceUnavailable)
		return
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "ready"})
}

func (s *EventBusService) getStatus(w http.ResponseWriter, r *http.Request) {
	metadata, err := s.adminClient.GetMetadata(nil, false, 5000)
	if err != nil {
		http.Error(w, "Failed to get status", http.StatusInternalServerError)
		return
	}
	
	status := map[string]interface{}{
		"brokers": len(metadata.Brokers),
		"topics":  len(metadata.Topics),
		"service": "event-bus",
		"version": "1.0.0",
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(status)
}

// Helper functions
func loadConfig() *Config {
	return &Config{
		KafkaBrokers:     getEnv("KAFKA_BROKERS", "localhost:9092"),
		SchemaRegistry:   getEnv("SCHEMA_REGISTRY_URL", "http://localhost:8081"),
		Port:             8080,
		MetricsPort:      9090,
		DLQRetentionDays: 7,
		MaxRetries:       3,
		RetryDelayMs:     1000,
	}
}

func initializeMetrics() *EventBusMetrics {
	metrics := &EventBusMetrics{
		MessagesProduced: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_events_produced_total",
				Help: "Total number of events produced",
			},
			[]string{"topic"},
		),
		MessagesConsumed: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_events_consumed_total",
				Help: "Total number of events consumed",
			},
			[]string{"topic", "consumer_group"},
		),
		MessagesFailed: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_events_failed_total",
				Help: "Total number of failed events",
			},
			[]string{"topic", "error_type"},
		),
		DLQMessages: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_dlq_messages_total",
				Help: "Total number of messages sent to DLQ",
			},
			[]string{"dlq_topic", "error_type"},
		),
		ProcessingDuration: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_event_processing_duration_seconds",
				Help: "Event processing duration",
			},
			[]string{"topic", "operation"},
		),
		TopicLag: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_topic_lag",
				Help: "Topic consumer lag",
			},
			[]string{"topic"},
		),
	}
	
	prometheus.MustRegister(
		metrics.MessagesProduced,
		metrics.MessagesConsumed,
		metrics.MessagesFailed,
		metrics.DLQMessages,
		metrics.ProcessingDuration,
		metrics.TopicLag,
	)
	
	return metrics
}

func getEnv(key, defaultValue string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return defaultValue
}

func generateErrorID() string {
	return fmt.Sprintf("err_%d", time.Now().UnixNano())
}

func getDLQTopicSuffix(originalTopic string) string {
	// Map original topics to DLQ topic suffixes
	topicMappings := map[string]string{
		"vehicle.telemetry.raw":      "vehicle.telemetry",
		"vehicle.telemetry.processed": "vehicle.telemetry",
		"vehicle.commands.outbound":   "vehicle.telemetry",
		"fleet.trips.lifecycle":       "fleet.operations",
		"fleet.dispatch.events":       "fleet.operations",
		"fleet.routing.updates":       "fleet.operations",
		"safety.alerts.critical":      "safety.alerts",
		"safety.policy.violations":    "safety.alerts",
		"safety.incidents.reports":    "safety.alerts",
	}
	
	if suffix, exists := topicMappings[originalTopic]; exists {
		return suffix
	}
	
	// Default mapping
	return "general"
}

// Placeholder implementations for remaining handlers
func (s *EventBusService) publishBatch(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) listTopics(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) createTopic(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) getTopic(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) deleteTopic(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) listSchemas(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) getSchema(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) registerSchema(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) listDLQTopics(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) getDLQMessages(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) retryDLQMessages(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}

func (s *EventBusService) discardDLQMessages(w http.ResponseWriter, r *http.Request) {
	http.Error(w, "Not implemented", http.StatusNotImplemented)
}
