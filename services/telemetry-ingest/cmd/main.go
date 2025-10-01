package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/confluentinc/confluent-kafka-go/kafka"
	"github.com/gorilla/mux"
	"github.com/gorilla/websocket"
	"github.com/lib/pq"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"github.com/redis/go-redis/v9"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// AtlasMesh Telemetry Ingest Service
// Real-time data processing for Abu Dhabi autonomous vehicle operations
// Handles Kafka events, WebSocket connections, and stream processing

type Config struct {
	Port                int    `json:"port"`
	DatabaseURL         string `json:"database_url"`
	RedisURL           string `json:"redis_url"`
	KafkaBrokers       string `json:"kafka_brokers"`
	WebSocketPort      int    `json:"websocket_port"`
	AbuDhabiMode       bool   `json:"abu_dhabi_mode"`
	MaxConnections     int    `json:"max_connections"`
	ProcessingWorkers  int    `json:"processing_workers"`
	AlertThresholds    map[string]float64 `json:"alert_thresholds"`
}

type TelemetryIngestService struct {
	config          Config
	redis          *redis.Client
	kafkaProducer  *kafka.Producer
	kafkaConsumer  *kafka.Consumer
	wsUpgrader     websocket.Upgrader
	wsConnections  map[string]*websocket.Conn
	wsConnectionsMux sync.RWMutex
	metrics        *TelemetryMetrics
	tracer         trace.Tracer
	alertManager   *AlertManager
	streamProcessor *StreamProcessor
}

type TelemetryMetrics struct {
	MessagesReceived   *prometheus.CounterVec
	MessagesProcessed  *prometheus.CounterVec
	ProcessingLatency  *prometheus.HistogramVec
	ActiveConnections  *prometheus.GaugeVec
	AlertsGenerated    *prometheus.CounterVec
	DataQualityScore   *prometheus.GaugeVec
}

// Real-time telemetry data structures
type VehicleTelemetry struct {
	VehicleID     string                 `json:"vehicle_id"`
	Timestamp     time.Time              `json:"timestamp"`
	Location      Location               `json:"location"`
	Speed         float64                `json:"speed"`
	Heading       float64                `json:"heading"`
	BatteryLevel  float64                `json:"battery_level"`
	FuelLevel     float64                `json:"fuel_level"`
	SystemStatus  map[string]interface{} `json:"system_status"`
	SensorData    map[string]interface{} `json:"sensor_data"`
	Alerts        []Alert                `json:"alerts,omitempty"`
	QualityScore  float64                `json:"quality_score"`
}

type Location struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Altitude  float64 `json:"altitude,omitempty"`
	Accuracy  float64 `json:"accuracy,omitempty"`
}

type Alert struct {
	ID          string                 `json:"id"`
	Type        string                 `json:"type"`
	Severity    string                 `json:"severity"`
	Message     string                 `json:"message"`
	Timestamp   time.Time              `json:"timestamp"`
	VehicleID   string                 `json:"vehicle_id"`
	Location    Location               `json:"location"`
	Metadata    map[string]interface{} `json:"metadata"`
	Acknowledged bool                   `json:"acknowledged"`
}

type StreamProcessor struct {
	service         *TelemetryIngestService
	processingQueue chan VehicleTelemetry
	workers         []*StreamWorker
	windowSize      time.Duration
	dataWindow      map[string][]VehicleTelemetry
	windowMutex     sync.RWMutex
}

type StreamWorker struct {
	id      int
	service *TelemetryIngestService
	queue   chan VehicleTelemetry
}

type AlertManager struct {
	service    *TelemetryIngestService
	rules      []AlertRule
	rulesMutex sync.RWMutex
}

type AlertRule struct {
	ID          string                 `json:"id"`
	Name        string                 `json:"name"`
	Condition   string                 `json:"condition"`
	Threshold   float64                `json:"threshold"`
	Severity    string                 `json:"severity"`
	Enabled     bool                   `json:"enabled"`
	Metadata    map[string]interface{} `json:"metadata"`
}

func main() {
	config := loadConfig()
	
	service, err := NewTelemetryIngestService(config)
	if err != nil {
		log.Fatalf("Failed to create telemetry ingest service: %v", err)
	}
	
	// Start the service
	if err := service.Start(); err != nil {
		log.Fatalf("Failed to start service: %v", err)
	}
	
	// Wait for shutdown signal
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c
	
	log.Println("Shutting down telemetry ingest service...")
	service.Shutdown()
}

func NewTelemetryIngestService(config Config) (*TelemetryIngestService, error) {
	// Initialize Redis client
	redisClient := redis.NewClient(&redis.Options{
		Addr: config.RedisURL,
	})
	
	// Initialize Kafka producer
	kafkaProducer, err := kafka.NewProducer(&kafka.ConfigMap{
		"bootstrap.servers": config.KafkaBrokers,
		"client.id":        "telemetry-ingest-producer",
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
		"group.id":         "telemetry-ingest-consumer",
		"auto.offset.reset": "latest",
		"enable.auto.commit": false,
	})
	if err != nil {
		return nil, fmt.Errorf("failed to create Kafka consumer: %v", err)
	}
	
	// Initialize metrics
	metrics := &TelemetryMetrics{
		MessagesReceived: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "telemetry_messages_received_total",
				Help: "Total number of telemetry messages received",
			},
			[]string{"vehicle_id", "message_type", "status"},
		),
		MessagesProcessed: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "telemetry_messages_processed_total",
				Help: "Total number of telemetry messages processed",
			},
			[]string{"vehicle_id", "processing_stage", "status"},
		),
		ProcessingLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "telemetry_processing_latency_seconds",
				Help: "Telemetry message processing latency",
				Buckets: []float64{0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0},
			},
			[]string{"vehicle_id", "processing_stage"},
		),
		ActiveConnections: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "telemetry_active_connections",
				Help: "Number of active WebSocket connections",
			},
			[]string{"connection_type"},
		),
		AlertsGenerated: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "telemetry_alerts_generated_total",
				Help: "Total number of alerts generated",
			},
			[]string{"alert_type", "severity", "vehicle_id"},
		),
		DataQualityScore: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "telemetry_data_quality_score",
				Help: "Data quality score for telemetry streams",
			},
			[]string{"vehicle_id", "metric_type"},
		),
	}
	
	// Register metrics
	prometheus.MustRegister(
		metrics.MessagesReceived,
		metrics.MessagesProcessed,
		metrics.ProcessingLatency,
		metrics.ActiveConnections,
		metrics.AlertsGenerated,
		metrics.DataQualityScore,
	)
	
	service := &TelemetryIngestService{
		config:        config,
		redis:         redisClient,
		kafkaProducer: kafkaProducer,
		kafkaConsumer: kafkaConsumer,
		wsUpgrader: websocket.Upgrader{
			CheckOrigin: func(r *http.Request) bool {
				return true // Configure properly for production
			},
		},
		wsConnections: make(map[string]*websocket.Conn),
		metrics:       metrics,
		tracer:        otel.Tracer("telemetry-ingest"),
	}
	
	// Initialize alert manager
	service.alertManager = NewAlertManager(service)
	
	// Initialize stream processor
	service.streamProcessor = NewStreamProcessor(service)
	
	return service, nil
}

func (s *TelemetryIngestService) Start() error {
	log.Println("🚀 Starting AtlasMesh Telemetry Ingest Service...")
	
	// Start Kafka consumer
	go s.startKafkaConsumer()
	
	// Start stream processor
	go s.streamProcessor.Start()
	
	// Start alert manager
	go s.alertManager.Start()
	
	// Start WebSocket server
	go s.startWebSocketServer()
	
	// Start HTTP server
	go s.startHTTPServer()
	
	log.Printf("✅ Telemetry Ingest Service started on port %d", s.config.Port)
	log.Printf("✅ WebSocket server started on port %d", s.config.WebSocketPort)
	
	return nil
}

func (s *TelemetryIngestService) startKafkaConsumer() {
	topics := []string{
		"vehicle.telemetry",
		"vehicle.alerts",
		"vehicle.commands",
		"system.events",
	}
	
	err := s.kafkaConsumer.SubscribeTopics(topics, nil)
	if err != nil {
		log.Fatalf("Failed to subscribe to Kafka topics: %v", err)
	}
	
	log.Printf("✅ Kafka consumer subscribed to topics: %v", topics)
	
	for {
		msg, err := s.kafkaConsumer.ReadMessage(-1)
		if err != nil {
			log.Printf("❌ Kafka consumer error: %v", err)
			continue
		}
		
		// Process message asynchronously
		go s.processKafkaMessage(msg)
	}
}

func (s *TelemetryIngestService) processKafkaMessage(msg *kafka.Message) {
	ctx := context.Background()
	span := s.tracer.Start(ctx, "process_kafka_message")
	defer span.End()
	
	start := time.Now()
	topic := *msg.TopicPartition.Topic
	
	defer func() {
		s.metrics.ProcessingLatency.WithLabelValues("", "kafka_processing").Observe(time.Since(start).Seconds())
	}()
	
	switch topic {
	case "vehicle.telemetry":
		s.processVehicleTelemetry(msg.Value)
	case "vehicle.alerts":
		s.processVehicleAlert(msg.Value)
	case "vehicle.commands":
		s.processVehicleCommand(msg.Value)
	case "system.events":
		s.processSystemEvent(msg.Value)
	default:
		log.Printf("⚠️ Unknown topic: %s", topic)
	}
	
	// Commit message
	s.kafkaConsumer.Commit()
}

func (s *TelemetryIngestService) processVehicleTelemetry(data []byte) {
	var telemetry VehicleTelemetry
	if err := json.Unmarshal(data, &telemetry); err != nil {
		log.Printf("❌ Failed to unmarshal telemetry data: %v", err)
		s.metrics.MessagesReceived.WithLabelValues("unknown", "telemetry", "error").Inc()
		return
	}
	
	// Validate and enrich telemetry data
	if err := s.validateTelemetry(&telemetry); err != nil {
		log.Printf("❌ Invalid telemetry data: %v", err)
		s.metrics.MessagesReceived.WithLabelValues(telemetry.VehicleID, "telemetry", "invalid").Inc()
		return
	}
	
	// Abu Dhabi specific processing
	if s.config.AbuDhabiMode {
		s.enrichTelemetryForAbuDhabi(&telemetry)
	}
	
	// Calculate data quality score
	telemetry.QualityScore = s.calculateDataQualityScore(&telemetry)
	s.metrics.DataQualityScore.WithLabelValues(telemetry.VehicleID, "overall").Set(telemetry.QualityScore)
	
	// Store in Redis for real-time access
	telemetryJSON, _ := json.Marshal(telemetry)
	s.redis.Set(context.Background(), fmt.Sprintf("telemetry:current:%s", telemetry.VehicleID), telemetryJSON, 30*time.Second)
	
	// Add to stream processing queue
	s.streamProcessor.processingQueue <- telemetry
	
	// Broadcast to WebSocket clients
	s.broadcastToWebSocketClients("telemetry", telemetry)
	
	// Check for alerts
	alerts := s.alertManager.EvaluateAlerts(telemetry)
	if len(alerts) > 0 {
		for _, alert := range alerts {
			s.handleAlert(alert)
		}
	}
	
	s.metrics.MessagesReceived.WithLabelValues(telemetry.VehicleID, "telemetry", "success").Inc()
	log.Printf("✅ Processed telemetry for vehicle %s (quality: %.1f%%)", telemetry.VehicleID, telemetry.QualityScore*100)
}

func (s *TelemetryIngestService) validateTelemetry(telemetry *VehicleTelemetry) error {
	if telemetry.VehicleID == "" {
		return fmt.Errorf("vehicle_id is required")
	}
	
	if telemetry.Timestamp.IsZero() {
		telemetry.Timestamp = time.Now()
	}
	
	// Validate location
	if telemetry.Location.Latitude < -90 || telemetry.Location.Latitude > 90 {
		return fmt.Errorf("invalid latitude: %f", telemetry.Location.Latitude)
	}
	
	if telemetry.Location.Longitude < -180 || telemetry.Location.Longitude > 180 {
		return fmt.Errorf("invalid longitude: %f", telemetry.Location.Longitude)
	}
	
	// Validate speed (reasonable range)
	if telemetry.Speed < 0 || telemetry.Speed > 200 {
		return fmt.Errorf("invalid speed: %f km/h", telemetry.Speed)
	}
	
	// Validate battery/fuel levels
	if telemetry.BatteryLevel < 0 || telemetry.BatteryLevel > 100 {
		telemetry.BatteryLevel = 0 // Reset invalid values
	}
	
	if telemetry.FuelLevel < 0 || telemetry.FuelLevel > 100 {
		telemetry.FuelLevel = 0 // Reset invalid values
	}
	
	return nil
}

func (s *TelemetryIngestService) enrichTelemetryForAbuDhabi(telemetry *VehicleTelemetry) {
	// Add Abu Dhabi timezone
	abuDhabiTime := telemetry.Timestamp.In(time.FixedZone("GST", 4*3600))
	telemetry.SystemStatus["local_time"] = abuDhabiTime.Format("2006-01-02 15:04:05 GST")
	
	// Add environmental context
	telemetry.SystemStatus["region"] = "abu_dhabi"
	
	// Check if in Abu Dhabi boundaries (simplified)
	if telemetry.Location.Latitude >= 24.0 && telemetry.Location.Latitude <= 25.0 &&
	   telemetry.Location.Longitude >= 54.0 && telemetry.Location.Longitude <= 55.5 {
		telemetry.SystemStatus["within_abu_dhabi"] = true
		
		// Add district information (simplified)
		if telemetry.Location.Latitude >= 24.45 && telemetry.Location.Latitude <= 24.47 &&
		   telemetry.Location.Longitude >= 54.35 && telemetry.Location.Longitude <= 54.39 {
			telemetry.SystemStatus["district"] = "downtown_abu_dhabi"
		}
	}
	
	// Environmental factors
	hour := abuDhabiTime.Hour()
	if hour >= 6 && hour <= 18 {
		telemetry.SystemStatus["daylight"] = true
		if hour >= 11 && hour <= 16 {
			telemetry.SystemStatus["peak_heat_hours"] = true
		}
	}
	
	// Prayer time considerations (simplified)
	prayerTimes := []int{5, 12, 15, 18, 19} // Approximate prayer times
	for _, prayerHour := range prayerTimes {
		if hour == prayerHour {
			telemetry.SystemStatus["prayer_time_proximity"] = true
			break
		}
	}
}

func (s *TelemetryIngestService) calculateDataQualityScore(telemetry *VehicleTelemetry) float64 {
	score := 1.0
	
	// Location accuracy
	if telemetry.Location.Accuracy > 0 {
		if telemetry.Location.Accuracy > 10 {
			score -= 0.1 // Reduce score for poor GPS accuracy
		}
	} else {
		score -= 0.05 // No accuracy data
	}
	
	// Data freshness
	age := time.Since(telemetry.Timestamp)
	if age > 30*time.Second {
		score -= 0.2
	} else if age > 10*time.Second {
		score -= 0.1
	}
	
	// Sensor data completeness
	requiredSensors := []string{"speed", "battery_level", "system_status"}
	missingSensors := 0
	
	if telemetry.Speed == 0 && telemetry.SystemStatus["engine_status"] == "running" {
		missingSensors++
	}
	
	if telemetry.BatteryLevel == 0 && telemetry.SystemStatus["power_source"] == "battery" {
		missingSensors++
	}
	
	if len(telemetry.SystemStatus) == 0 {
		missingSensors++
	}
	
	score -= float64(missingSensors) * 0.1
	
	// Ensure score is between 0 and 1
	if score < 0 {
		score = 0
	}
	
	return score
}

func (s *TelemetryIngestService) processVehicleAlert(data []byte) {
	var alert Alert
	if err := json.Unmarshal(data, &alert); err != nil {
		log.Printf("❌ Failed to unmarshal alert data: %v", err)
		return
	}
	
	s.handleAlert(alert)
}

func (s *TelemetryIngestService) handleAlert(alert Alert) {
	// Store alert in Redis
	alertJSON, _ := json.Marshal(alert)
	s.redis.Set(context.Background(), fmt.Sprintf("alert:%s", alert.ID), alertJSON, 24*time.Hour)
	
	// Add to vehicle's alert list
	s.redis.LPush(context.Background(), fmt.Sprintf("alerts:vehicle:%s", alert.VehicleID), alertJSON)
	s.redis.LTrim(context.Background(), fmt.Sprintf("alerts:vehicle:%s", alert.VehicleID), 0, 99) // Keep last 100 alerts
	
	// Broadcast to WebSocket clients
	s.broadcastToWebSocketClients("alert", alert)
	
	// Send to external systems if critical
	if alert.Severity == "critical" || alert.Severity == "emergency" {
		s.sendToExternalSystems(alert)
	}
	
	s.metrics.AlertsGenerated.WithLabelValues(alert.Type, alert.Severity, alert.VehicleID).Inc()
	log.Printf("🚨 Alert generated: %s - %s (%s)", alert.Type, alert.Message, alert.Severity)
}

func (s *TelemetryIngestService) sendToExternalSystems(alert Alert) {
	// Abu Dhabi specific integrations
	if s.config.AbuDhabiMode {
		switch alert.Severity {
		case "emergency":
			// Send to emergency services (999)
			s.sendToEmergencyServices(alert)
		case "critical":
			// Send to ADTA (Abu Dhabi Transport Authority)
			s.sendToADTA(alert)
		}
	}
}

func (s *TelemetryIngestService) sendToEmergencyServices(alert Alert) {
	// Implementation would integrate with UAE emergency services API
	log.Printf("🚨 EMERGENCY: Notifying emergency services for alert: %s", alert.ID)
	
	// Create emergency notification
	notification := map[string]interface{}{
		"alert_id":    alert.ID,
		"vehicle_id":  alert.VehicleID,
		"location":    alert.Location,
		"severity":    alert.Severity,
		"message":     alert.Message,
		"timestamp":   alert.Timestamp,
		"contact_info": "AtlasMesh Fleet Operations",
	}
	
	// Store for audit
	notificationJSON, _ := json.Marshal(notification)
	s.redis.Set(context.Background(), fmt.Sprintf("emergency:notification:%s", alert.ID), notificationJSON, 7*24*time.Hour)
}

func (s *TelemetryIngestService) sendToADTA(alert Alert) {
	// Implementation would integrate with ADTA systems
	log.Printf("📡 ADTA: Reporting critical alert: %s", alert.ID)
	
	// Create ADTA report
	report := map[string]interface{}{
		"alert_id":     alert.ID,
		"vehicle_id":   alert.VehicleID,
		"location":     alert.Location,
		"alert_type":   alert.Type,
		"description":  alert.Message,
		"timestamp":    alert.Timestamp,
		"operator":     "AtlasMesh",
		"compliance":   "UAE_AV_Regulations_2024",
	}
	
	// Store for compliance
	reportJSON, _ := json.Marshal(report)
	s.redis.Set(context.Background(), fmt.Sprintf("adta:report:%s", alert.ID), reportJSON, 30*24*time.Hour)
}

func (s *TelemetryIngestService) processVehicleCommand(data []byte) {
	// Process vehicle commands (stop, recall, update route, etc.)
	log.Printf("📝 Processing vehicle command: %s", string(data))
}

func (s *TelemetryIngestService) processSystemEvent(data []byte) {
	// Process system-wide events (maintenance windows, weather alerts, etc.)
	log.Printf("🔔 Processing system event: %s", string(data))
}

func (s *TelemetryIngestService) startWebSocketServer() {
	router := mux.NewRouter()
	router.HandleFunc("/ws/telemetry", s.handleWebSocketConnection)
	
	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", s.config.WebSocketPort),
		Handler: router,
	}
	
	log.Fatal(server.ListenAndServe())
}

func (s *TelemetryIngestService) handleWebSocketConnection(w http.ResponseWriter, r *http.Request) {
	conn, err := s.wsUpgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("❌ WebSocket upgrade failed: %v", err)
		return
	}
	defer conn.Close()
	
	clientID := r.Header.Get("X-Client-ID")
	if clientID == "" {
		clientID = fmt.Sprintf("client_%d", time.Now().UnixNano())
	}
	
	// Register connection
	s.wsConnectionsMux.Lock()
	s.wsConnections[clientID] = conn
	s.wsConnectionsMux.Unlock()
	
	s.metrics.ActiveConnections.WithLabelValues("websocket").Inc()
	log.Printf("✅ WebSocket client connected: %s", clientID)
	
	// Handle connection
	defer func() {
		s.wsConnectionsMux.Lock()
		delete(s.wsConnections, clientID)
		s.wsConnectionsMux.Unlock()
		s.metrics.ActiveConnections.WithLabelValues("websocket").Dec()
		log.Printf("🔌 WebSocket client disconnected: %s", clientID)
	}()
	
	// Send initial data
	s.sendInitialDataToClient(conn, clientID)
	
	// Keep connection alive and handle incoming messages
	for {
		_, message, err := conn.ReadMessage()
		if err != nil {
			log.Printf("❌ WebSocket read error: %v", err)
			break
		}
		
		// Process client message (subscriptions, filters, etc.)
		s.handleWebSocketMessage(clientID, message)
	}
}

func (s *TelemetryIngestService) sendInitialDataToClient(conn *websocket.Conn, clientID string) {
	// Send current vehicle states
	ctx := context.Background()
	keys, err := s.redis.Keys(ctx, "telemetry:current:*").Result()
	if err != nil {
		return
	}
	
	for _, key := range keys {
		data, err := s.redis.Get(ctx, key).Result()
		if err != nil {
			continue
		}
		
		message := map[string]interface{}{
			"type": "initial_telemetry",
			"data": json.RawMessage(data),
		}
		
		if err := conn.WriteJSON(message); err != nil {
			log.Printf("❌ Failed to send initial data to client %s: %v", clientID, err)
			break
		}
	}
}

func (s *TelemetryIngestService) handleWebSocketMessage(clientID string, message []byte) {
	var msg map[string]interface{}
	if err := json.Unmarshal(message, &msg); err != nil {
		log.Printf("❌ Invalid WebSocket message from client %s: %v", clientID, err)
		return
	}
	
	msgType, ok := msg["type"].(string)
	if !ok {
		return
	}
	
	switch msgType {
	case "subscribe":
		// Handle subscription requests
		s.handleSubscription(clientID, msg)
	case "unsubscribe":
		// Handle unsubscription requests
		s.handleUnsubscription(clientID, msg)
	case "filter":
		// Handle filter updates
		s.handleFilterUpdate(clientID, msg)
	}
}

func (s *TelemetryIngestService) handleSubscription(clientID string, msg map[string]interface{}) {
	// Implementation for handling client subscriptions
	log.Printf("📡 Client %s subscription: %v", clientID, msg)
}

func (s *TelemetryIngestService) handleUnsubscription(clientID string, msg map[string]interface{}) {
	// Implementation for handling client unsubscriptions
	log.Printf("📡 Client %s unsubscription: %v", clientID, msg)
}

func (s *TelemetryIngestService) handleFilterUpdate(clientID string, msg map[string]interface{}) {
	// Implementation for handling client filter updates
	log.Printf("🔍 Client %s filter update: %v", clientID, msg)
}

func (s *TelemetryIngestService) broadcastToWebSocketClients(messageType string, data interface{}) {
	message := map[string]interface{}{
		"type":      messageType,
		"data":      data,
		"timestamp": time.Now(),
	}
	
	s.wsConnectionsMux.RLock()
	defer s.wsConnectionsMux.RUnlock()
	
	for clientID, conn := range s.wsConnections {
		if err := conn.WriteJSON(message); err != nil {
			log.Printf("❌ Failed to send message to client %s: %v", clientID, err)
			// Connection will be cleaned up by the connection handler
		}
	}
}

func (s *TelemetryIngestService) startHTTPServer() {
	router := mux.NewRouter()
	
	// Health check
	router.HandleFunc("/health", s.healthCheck).Methods("GET")
	
	// Metrics endpoint
	router.Handle("/metrics", promhttp.Handler())
	
	// Telemetry endpoints
	router.HandleFunc("/api/v1/telemetry/current/{vehicleId}", s.getCurrentTelemetry).Methods("GET")
	router.HandleFunc("/api/v1/telemetry/history/{vehicleId}", s.getTelemetryHistory).Methods("GET")
	router.HandleFunc("/api/v1/alerts/active", s.getActiveAlerts).Methods("GET")
	router.HandleFunc("/api/v1/alerts/{vehicleId}", s.getVehicleAlerts).Methods("GET")
	
	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", s.config.Port),
		Handler: router,
	}
	
	log.Fatal(server.ListenAndServe())
}

func (s *TelemetryIngestService) healthCheck(w http.ResponseWriter, r *http.Request) {
	status := map[string]interface{}{
		"status":    "healthy",
		"timestamp": time.Now(),
		"version":   "1.0.0",
		"services": map[string]bool{
			"kafka_producer": s.kafkaProducer != nil,
			"kafka_consumer": s.kafkaConsumer != nil,
			"redis":         s.redis.Ping(context.Background()).Err() == nil,
			"stream_processor": s.streamProcessor != nil,
			"alert_manager": s.alertManager != nil,
		},
		"connections": map[string]int{
			"websocket_clients": len(s.wsConnections),
		},
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(status)
}

func (s *TelemetryIngestService) getCurrentTelemetry(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	
	data, err := s.redis.Get(context.Background(), fmt.Sprintf("telemetry:current:%s", vehicleID)).Result()
	if err != nil {
		http.Error(w, "Telemetry not found", http.StatusNotFound)
		return
	}
	
	w.Header().Set("Content-Type", "application/json")
	w.Write([]byte(data))
}

func (s *TelemetryIngestService) getTelemetryHistory(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	
	// Get query parameters
	hoursStr := r.URL.Query().Get("hours")
	hours := 24 // Default 24 hours
	
	if hoursStr != "" {
		if h, err := strconv.Atoi(hoursStr); err == nil && h > 0 && h <= 168 {
			hours = h
		}
	}
	
	// In production, this would query the time-series database
	response := map[string]interface{}{
		"vehicle_id": vehicleID,
		"hours":     hours,
		"data":      []interface{}{}, // Would contain historical telemetry
		"message":   "Historical telemetry data would be retrieved from time-series database",
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func (s *TelemetryIngestService) getActiveAlerts(w http.ResponseWriter, r *http.Request) {
	// Get active alerts from Redis
	ctx := context.Background()
	keys, err := s.redis.Keys(ctx, "alert:*").Result()
	if err != nil {
		http.Error(w, "Failed to retrieve alerts", http.StatusInternalServerError)
		return
	}
	
	alerts := []Alert{}
	for _, key := range keys {
		data, err := s.redis.Get(ctx, key).Result()
		if err != nil {
			continue
		}
		
		var alert Alert
		if err := json.Unmarshal([]byte(data), &alert); err != nil {
			continue
		}
		
		// Only include unacknowledged alerts
		if !alert.Acknowledged {
			alerts = append(alerts, alert)
		}
	}
	
	response := map[string]interface{}{
		"alerts": alerts,
		"count":  len(alerts),
		"timestamp": time.Now(),
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func (s *TelemetryIngestService) getVehicleAlerts(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	
	// Get vehicle alerts from Redis
	ctx := context.Background()
	alertsData, err := s.redis.LRange(ctx, fmt.Sprintf("alerts:vehicle:%s", vehicleID), 0, 49).Result() // Last 50 alerts
	if err != nil {
		http.Error(w, "Failed to retrieve vehicle alerts", http.StatusInternalServerError)
		return
	}
	
	alerts := []Alert{}
	for _, data := range alertsData {
		var alert Alert
		if err := json.Unmarshal([]byte(data), &alert); err != nil {
			continue
		}
		alerts = append(alerts, alert)
	}
	
	response := map[string]interface{}{
		"vehicle_id": vehicleID,
		"alerts":     alerts,
		"count":      len(alerts),
		"timestamp":  time.Now(),
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

func (s *TelemetryIngestService) Shutdown() {
	log.Println("🛑 Shutting down telemetry ingest service...")
	
	// Close Kafka connections
	if s.kafkaProducer != nil {
		s.kafkaProducer.Close()
	}
	if s.kafkaConsumer != nil {
		s.kafkaConsumer.Close()
	}
	
	// Close Redis connection
	if s.redis != nil {
		s.redis.Close()
	}
	
	// Close WebSocket connections
	s.wsConnectionsMux.Lock()
	for clientID, conn := range s.wsConnections {
		conn.Close()
		log.Printf("🔌 Closed WebSocket connection: %s", clientID)
	}
	s.wsConnectionsMux.Unlock()
	
	// Stop stream processor
	if s.streamProcessor != nil {
		s.streamProcessor.Stop()
	}
	
	// Stop alert manager
	if s.alertManager != nil {
		s.alertManager.Stop()
	}
	
	log.Println("✅ Telemetry ingest service shutdown complete")
}

func loadConfig() Config {
	return Config{
		Port:              8080,
		DatabaseURL:       getEnv("DATABASE_URL", "postgres://user:pass@localhost/fleet_db?sslmode=disable"),
		RedisURL:          getEnv("REDIS_URL", "localhost:6379"),
		KafkaBrokers:      getEnv("KAFKA_BROKERS", "localhost:9092"),
		WebSocketPort:     8081,
		AbuDhabiMode:      getEnv("ABU_DHABI_MODE", "true") == "true",
		MaxConnections:    1000,
		ProcessingWorkers: 10,
		AlertThresholds: map[string]float64{
			"battery_low":     20.0,
			"fuel_low":        15.0,
			"speed_limit":     120.0,
			"temperature_high": 85.0,
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
// - stream_processor.go: Stream processing logic
// - alert_manager.go: Alert rule evaluation and management
// - data_quality.go: Data quality assessment and monitoring
