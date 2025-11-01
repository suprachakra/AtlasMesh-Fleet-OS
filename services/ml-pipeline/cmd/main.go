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
	"sync"
	"syscall"
	"time"

	"github.com/gorilla/mux"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"github.com/segmentio/kafka-go"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// AtlasMesh ML Pipeline Service
//
// PUBLIC API: Machine Learning pipeline service providing predictive maintenance,
// route optimization, demand forecasting, and anomaly detection
// Handles model training, inference, and real-time predictions

// Service Configuration
type MLPipelineConfig struct {
	Port                int    `json:"port"`
	KafkaBrokers        string `json:"kafka_brokers"`
	KafkaTopic          string `json:"kafka_topic"`
	ModelStoragePath    string `json:"model_storage_path"`
	TrainingDataPath    string `json:"training_data_path"`
	InferenceTimeout    int    `json:"inference_timeout"`
	BatchSize           int    `json:"batch_size"`
	ModelUpdateInterval int    `json:"model_update_interval"`
	EnableGPU           bool   `json:"enable_gpu"`
	MaxConcurrentJobs   int    `json:"max_concurrent_jobs"`
	EnableTracing       bool   `json:"enable_tracing"`
	EnableMetrics       bool   `json:"enable_metrics"`
}

// Service Metrics
type MLPipelineMetrics struct {
	ModelInferences     *prometheus.CounterVec
	InferenceLatency    *prometheus.HistogramVec
	TrainingJobs        *prometheus.CounterVec
	TrainingLatency      *prometheus.HistogramVec
	ModelAccuracy       *prometheus.GaugeVec
	DataProcessingErrors *prometheus.CounterVec
	ModelUpdates        *prometheus.CounterVec
	PredictionAccuracy   *prometheus.GaugeVec
}

// ML Pipeline Service
type MLPipelineService struct {
	config     *MLPipelineConfig
	metrics    *MLPipelineMetrics
	tracer     trace.Tracer
	kafkaWriter *kafka.Writer
	kafkaReader *kafka.Reader
	models     map[string]MLModel
	modelsMutex sync.RWMutex
	jobQueue   chan MLJob
	workers    []MLWorker
	ctx        context.Context
	cancel     context.CancelFunc
}

// ML Models
type MLModel struct {
	ID           string    `json:"id"`
	Name         string    `json:"name"`
	Type         string    `json:"type"`
	Version      string    `json:"version"`
	Accuracy     float64   `json:"accuracy"`
	Status       string    `json:"status"`
	LastTrained  time.Time `json:"last_trained"`
	LastUpdated  time.Time `json:"last_updated"`
	ModelPath    string    `json:"model_path"`
	Config       ModelConfig `json:"config"`
}

type ModelConfig struct {
	Features     []string `json:"features"`
	TargetColumn string   `json:"target_column"`
	Algorithm    string   `json:"algorithm"`
	Parameters   map[string]interface{} `json:"parameters"`
}

// ML Jobs
type MLJob struct {
	ID          string    `json:"id"`
	Type        string    `json:"type"`
	ModelID     string    `json:"model_id"`
	Data        []byte    `json:"data"`
	Priority    int       `json:"priority"`
	CreatedAt   time.Time `json:"created_at"`
	Status      string    `json:"status"`
	Result      interface{} `json:"result,omitempty"`
	Error       string    `json:"error,omitempty"`
}

type MLWorker struct {
	ID       string
	JobQueue chan MLJob
	Quit     chan bool
	Service  *MLPipelineService
}

// Predictions
type MaintenancePrediction struct {
	VehicleID       string    `json:"vehicle_id"`
	Component       string    `json:"component"`
	FailureProbability float64 `json:"failure_probability"`
	TimeToFailure   int       `json:"time_to_failure_hours"`
	Confidence      float64   `json:"confidence"`
	Recommendations []string  `json:"recommendations"`
	Timestamp       time.Time `json:"timestamp"`
}

type RouteOptimization struct {
	RouteID         string    `json:"route_id"`
	Origin          Location  `json:"origin"`
	Destination     Location  `json:"destination"`
	OptimizedRoute  []Location `json:"optimized_route"`
	Distance        float64   `json:"distance"`
	Duration        float64   `json:"duration"`
	FuelEfficiency  float64   `json:"fuel_efficiency"`
	TrafficFactor   float64   `json:"traffic_factor"`
	WeatherFactor   float64   `json:"weather_factor"`
	Confidence      float64   `json:"confidence"`
	Timestamp       time.Time `json:"timestamp"`
}

type Location struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Address   string  `json:"address,omitempty"`
}

type DemandForecast struct {
	LocationID      string    `json:"location_id"`
	Location        Location  `json:"location"`
	TimeWindow      string    `json:"time_window"`
	PredictedDemand float64   `json:"predicted_demand"`
	Confidence      float64   `json:"confidence"`
	Factors         map[string]float64 `json:"factors"`
	Timestamp       time.Time `json:"timestamp"`
}

type AnomalyDetection struct {
	VehicleID       string    `json:"vehicle_id"`
	AnomalyType    string    `json:"anomaly_type"`
	Severity        string    `json:"severity"`
	Score           float64   `json:"score"`
	Description     string    `json:"description"`
	DetectedAt      time.Time `json:"detected_at"`
	Recommendations []string  `json:"recommendations"`
}

func main() {
	log.Println("🤖 Starting AtlasMesh ML Pipeline Service...")

	// Load configuration
	config := loadConfig()
	
	// Initialize service
	ctx, cancel := context.WithCancel(context.Background())
	service := &MLPipelineService{
		config:   config,
		metrics:  initMetrics(),
		tracer:   otel.Tracer("ml-pipeline-service"),
		models:   make(map[string]MLModel),
		jobQueue: make(chan MLJob, config.MaxConcurrentJobs*2),
		ctx:      ctx,
		cancel:   cancel,
	}

	// Initialize Kafka
	if err := service.initKafka(); err != nil {
		log.Fatalf("❌ Failed to initialize Kafka: %v", err)
	}

	// Initialize ML models
	if err := service.initModels(); err != nil {
		log.Fatalf("❌ Failed to initialize ML models: %v", err)
	}

	// Start worker pool
	service.startWorkers()

	// Setup HTTP server
	router := mux.NewRouter()
	service.setupRoutes(router)

	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.Port),
		Handler: router,
	}

	// Start background services
	go service.startModelUpdater(ctx)
	go service.startDataProcessor(ctx)
	go service.startMetricsCollector(ctx)

	// Start server
	go func() {
		log.Printf("🌐 ML Pipeline Service listening on port %d", config.Port)
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("❌ Server failed to start: %v", err)
		}
	}()

	// Wait for shutdown signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("🛑 Shutting down ML Pipeline Service...")

	// Graceful shutdown
	shutdownCtx, shutdownCancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer shutdownCancel()

	if err := server.Shutdown(shutdownCtx); err != nil {
		log.Printf("Server shutdown error: %v", err)
	}

	// Close Kafka connections
	if err := service.kafkaWriter.Close(); err != nil {
		log.Printf("Kafka writer close error: %v", err)
	}
	if err := service.kafkaReader.Close(); err != nil {
		log.Printf("Kafka reader close error: %v", err)
	}

	// Stop workers
	service.stopWorkers()

	log.Println("✅ ML Pipeline Service shutdown complete")
}

func loadConfig() *MLPipelineConfig {
	config := &MLPipelineConfig{
		Port:                8080,
		KafkaBrokers:        "localhost:9092",
		KafkaTopic:          "ml-pipeline",
		ModelStoragePath:    "/models",
		TrainingDataPath:    "/data",
		InferenceTimeout:    30,
		BatchSize:           100,
		ModelUpdateInterval: 3600, // 1 hour
		EnableGPU:           false,
		MaxConcurrentJobs:   10,
		EnableTracing:       true,
		EnableMetrics:       true,
	}

	// Override with environment variables
	if port := os.Getenv("PORT"); port != "" {
		if p, err := strconv.Atoi(port); err == nil {
			config.Port = p
		}
	}

	return config
}

func initMetrics() *MLPipelineMetrics {
	return &MLPipelineMetrics{
		ModelInferences: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "ml_pipeline_inferences_total",
				Help: "Total number of ML model inferences",
			},
			[]string{"model_id", "model_type", "status"},
		),
		InferenceLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name:    "ml_pipeline_inference_duration_seconds",
				Help:    "ML model inference latency",
				Buckets: prometheus.DefBuckets,
			},
			[]string{"model_id", "model_type"},
		),
		TrainingJobs: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "ml_pipeline_training_jobs_total",
				Help: "Total number of ML training jobs",
			},
			[]string{"model_id", "status"},
		),
		TrainingLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name:    "ml_pipeline_training_duration_seconds",
				Help:    "ML model training latency",
				Buckets: prometheus.DefBuckets,
			},
			[]string{"model_id"},
		),
		ModelAccuracy: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "ml_pipeline_model_accuracy",
				Help: "ML model accuracy score",
			},
			[]string{"model_id", "model_type"},
		),
		DataProcessingErrors: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "ml_pipeline_data_processing_errors_total",
				Help: "Total number of data processing errors",
			},
			[]string{"error_type"},
		),
		ModelUpdates: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "ml_pipeline_model_updates_total",
				Help: "Total number of model updates",
			},
			[]string{"model_id", "status"},
		),
		PredictionAccuracy: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "ml_pipeline_prediction_accuracy",
				Help: "ML prediction accuracy",
			},
			[]string{"prediction_type"},
		),
	}
}

func (s *MLPipelineService) initKafka() error {
	// Initialize Kafka writer
	s.kafkaWriter = &kafka.Writer{
		Addr:     kafka.TCP(s.config.KafkaBrokers),
		Topic:    s.config.KafkaTopic,
		Balancer: &kafka.LeastBytes{},
	}

	// Initialize Kafka reader
	s.kafkaReader = kafka.NewReader(kafka.ReaderConfig{
		Brokers: []string{s.config.KafkaBrokers},
		Topic:   s.config.KafkaTopic,
		GroupID: "ml-pipeline-service",
	})

	return nil
}

func (s *MLPipelineService) initModels() error {
	// Initialize ML models
	models := []MLModel{
		{
			ID:          "maintenance-predictor",
			Name:        "Predictive Maintenance Model",
			Type:        "classification",
			Version:     "1.0.0",
			Accuracy:    0.92,
			Status:      "active",
			LastTrained: time.Now().AddDate(0, 0, -7),
			LastUpdated: time.Now().AddDate(0, 0, -1),
			ModelPath:   "/models/maintenance-predictor-v1.0.0",
			Config: ModelConfig{
				Features:     []string{"battery_level", "engine_temp", "mileage", "age", "usage_pattern"},
				TargetColumn: "failure_probability",
				Algorithm:    "random_forest",
				Parameters: map[string]interface{}{
					"n_estimators": 100,
					"max_depth":    10,
					"random_state": 42,
				},
			},
		},
		{
			ID:          "route-optimizer",
			Name:        "Route Optimization Model",
			Type:        "optimization",
			Version:     "1.0.0",
			Accuracy:    0.88,
			Status:      "active",
			LastTrained: time.Now().AddDate(0, 0, -3),
			LastUpdated: time.Now().AddDate(0, 0, -1),
			ModelPath:   "/models/route-optimizer-v1.0.0",
			Config: ModelConfig{
				Features:     []string{"distance", "traffic", "weather", "time_of_day", "vehicle_type"},
				TargetColumn: "optimal_route",
				Algorithm:    "genetic_algorithm",
				Parameters: map[string]interface{}{
					"population_size": 100,
					"generations":     50,
					"mutation_rate":   0.1,
				},
			},
		},
		{
			ID:          "demand-forecaster",
			Name:        "Demand Forecasting Model",
			Type:        "regression",
			Version:     "1.0.0",
			Accuracy:    0.85,
			Status:      "active",
			LastTrained: time.Now().AddDate(0, 0, -1),
			LastUpdated: time.Now(),
			ModelPath:   "/models/demand-forecaster-v1.0.0",
			Config: ModelConfig{
				Features:     []string{"historical_demand", "weather", "events", "time_of_day", "day_of_week"},
				TargetColumn: "predicted_demand",
				Algorithm:    "lstm",
				Parameters: map[string]interface{}{
					"sequence_length": 24,
					"hidden_units":    64,
					"dropout":         0.2,
				},
			},
		},
		{
			ID:          "anomaly-detector",
			Name:        "Anomaly Detection Model",
			Type:        "anomaly_detection",
			Version:     "1.0.0",
			Accuracy:    0.90,
			Status:      "active",
			LastTrained: time.Now().AddDate(0, 0, -2),
			LastUpdated: time.Now().AddDate(0, 0, -1),
			ModelPath:   "/models/anomaly-detector-v1.0.0",
			Config: ModelConfig{
				Features:     []string{"sensor_data", "behavior_patterns", "environmental_factors"},
				TargetColumn: "anomaly_score",
				Algorithm:    "isolation_forest",
				Parameters: map[string]interface{}{
					"n_estimators": 100,
					"contamination": 0.1,
					"random_state": 42,
				},
			},
		},
	}

	s.modelsMutex.Lock()
	for _, model := range models {
		s.models[model.ID] = model
		s.metrics.ModelAccuracy.WithLabelValues(model.ID, model.Type).Set(model.Accuracy)
	}
	s.modelsMutex.Unlock()

	return nil
}

func (s *MLPipelineService) startWorkers() {
	for i := 0; i < s.config.MaxConcurrentJobs; i++ {
		worker := MLWorker{
			ID:       fmt.Sprintf("worker-%d", i),
			JobQueue: s.jobQueue,
			Quit:     make(chan bool),
			Service:  s,
		}
		s.workers = append(s.workers, worker)
		go worker.start()
	}
}

func (s *MLPipelineService) stopWorkers() {
	for _, worker := range s.workers {
		worker.Quit <- true
	}
}

func (w *MLWorker) start() {
	for {
		select {
		case job := <-w.JobQueue:
			w.processJob(job)
		case <-w.Quit:
			return
		}
	}
}

func (w *MLWorker) processJob(job MLJob) {
	start := time.Now()
	defer func() {
		w.Service.metrics.InferenceLatency.WithLabelValues(job.ModelID, "inference").Observe(time.Since(start).Seconds())
	}()

	// Get model
	w.Service.modelsMutex.RLock()
	model, exists := w.Service.models[job.ModelID]
	w.Service.modelsMutex.RUnlock()

	if !exists {
		job.Status = "failed"
		job.Error = "Model not found"
		w.Service.metrics.ModelInferences.WithLabelValues(job.ModelID, model.Type, "failed").Inc()
		return
	}

	// Process job based on type
	switch job.Type {
	case "maintenance_prediction":
		w.processMaintenancePrediction(job, model)
	case "route_optimization":
		w.processRouteOptimization(job, model)
	case "demand_forecast":
		w.processDemandForecast(job, model)
	case "anomaly_detection":
		w.processAnomalyDetection(job, model)
	default:
		job.Status = "failed"
		job.Error = "Unknown job type"
		w.Service.metrics.ModelInferences.WithLabelValues(job.ModelID, model.Type, "failed").Inc()
	}
}

func (w *MLWorker) processMaintenancePrediction(job MLJob, model MLModel) {
	// Mock maintenance prediction - in production, this would use actual ML model
	prediction := MaintenancePrediction{
		VehicleID:           "vehicle-001",
		Component:           "battery",
		FailureProbability:  0.15,
		TimeToFailure:       72,
		Confidence:          0.92,
		Recommendations:     []string{"Schedule battery inspection", "Monitor charging patterns"},
		Timestamp:           time.Now(),
	}

	job.Status = "completed"
	job.Result = prediction
	w.Service.metrics.ModelInferences.WithLabelValues(job.ModelID, model.Type, "success").Inc()
}

func (w *MLWorker) processRouteOptimization(job MLJob, model MLModel) {
	// Mock route optimization - in production, this would use actual ML model
	optimization := RouteOptimization{
		RouteID:        "route-001",
		Origin:         Location{Latitude: 25.2048, Longitude: 55.2708, Address: "Dubai Marina"},
		Destination:    Location{Latitude: 25.2582, Longitude: 55.3047, Address: "Dubai Mall"},
		OptimizedRoute: []Location{
			{Latitude: 25.2048, Longitude: 55.2708},
			{Latitude: 25.2200, Longitude: 55.2800},
			{Latitude: 25.2400, Longitude: 55.2900},
			{Latitude: 25.2582, Longitude: 55.3047},
		},
		Distance:       15.5,
		Duration:       25.0,
		FuelEfficiency: 0.85,
		TrafficFactor:  0.7,
		WeatherFactor:  0.9,
		Confidence:     0.88,
		Timestamp:      time.Now(),
	}

	job.Status = "completed"
	job.Result = optimization
	w.Service.metrics.ModelInferences.WithLabelValues(job.ModelID, model.Type, "success").Inc()
}

func (w *MLWorker) processDemandForecast(job MLJob, model MLModel) {
	// Mock demand forecast - in production, this would use actual ML model
	forecast := DemandForecast{
		LocationID:      "location-001",
		Location:        Location{Latitude: 25.2048, Longitude: 55.2708, Address: "Dubai Marina"},
		TimeWindow:      "next_hour",
		PredictedDemand: 45.5,
		Confidence:      0.85,
		Factors: map[string]float64{
			"weather":      0.8,
			"time_of_day": 0.9,
			"events":       1.2,
		},
		Timestamp: time.Now(),
	}

	job.Status = "completed"
	job.Result = forecast
	w.Service.metrics.ModelInferences.WithLabelValues(job.ModelID, model.Type, "success").Inc()
}

func (w *MLWorker) processAnomalyDetection(job MLJob, model MLModel) {
	// Mock anomaly detection - in production, this would use actual ML model
	anomaly := AnomalyDetection{
		VehicleID:    "vehicle-001",
		AnomalyType:  "sensor_malfunction",
		Severity:     "medium",
		Score:        0.75,
		Description:  "Unusual sensor readings detected",
		DetectedAt:   time.Now(),
		Recommendations: []string{"Check sensor connections", "Schedule diagnostic test"},
	}

	job.Status = "completed"
	job.Result = anomaly
	w.Service.metrics.ModelInferences.WithLabelValues(job.ModelID, model.Type, "success").Inc()
}

// API Routes and Handlers

func (s *MLPipelineService) setupRoutes(router *mux.Router) {
	// Health endpoints
	router.HandleFunc("/health", s.healthCheck).Methods("GET")
	router.HandleFunc("/ready", s.readinessCheck).Methods("GET")
	router.HandleFunc("/metrics", promhttp.Handler().ServeHTTP).Methods("GET")

	// ML Pipeline API
	api := router.PathPrefix("/api/v1").Subrouter()

	// Model management
	api.HandleFunc("/models", s.getModels).Methods("GET")
	api.HandleFunc("/models/{modelId}", s.getModel).Methods("GET")
	api.HandleFunc("/models/{modelId}/train", s.trainModel).Methods("POST")
	api.HandleFunc("/models/{modelId}/update", s.updateModel).Methods("POST")
	
	// Predictions
	api.HandleFunc("/predict/maintenance", s.predictMaintenance).Methods("POST")
	api.HandleFunc("/predict/route", s.optimizeRoute).Methods("POST")
	api.HandleFunc("/predict/demand", s.forecastDemand).Methods("POST")
	api.HandleFunc("/predict/anomaly", s.detectAnomaly).Methods("POST")
	
	// Batch processing
	api.HandleFunc("/batch/predict", s.batchPredict).Methods("POST")
	api.HandleFunc("/batch/train", s.batchTrain).Methods("POST")
	
	// Model performance
	api.HandleFunc("/models/{modelId}/performance", s.getModelPerformance).Methods("GET")
	api.HandleFunc("/models/{modelId}/metrics", s.getModelMetrics).Methods("GET")
}

func (s *MLPipelineService) healthCheck(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":    "healthy",
		"service":   "ml-pipeline-service",
		"timestamp": time.Now(),
	})
}

func (s *MLPipelineService) readinessCheck(w http.ResponseWriter, r *http.Request) {
	// Check if models are loaded
	s.modelsMutex.RLock()
	modelCount := len(s.models)
	s.modelsMutex.RUnlock()

	if modelCount == 0 {
		w.WriteHeader(http.StatusServiceUnavailable)
		json.NewEncoder(w).Encode(map[string]interface{}{
			"status": "not_ready",
			"error":  "No models loaded",
		})
		return
	}

	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":      "ready",
		"model_count": modelCount,
	})
}

// Model Management

func (s *MLPipelineService) getModels(w http.ResponseWriter, r *http.Request) {
	s.modelsMutex.RLock()
	models := make([]MLModel, 0, len(s.models))
	for _, model := range s.models {
		models = append(models, model)
	}
	s.modelsMutex.RUnlock()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(models)
}

func (s *MLPipelineService) getModel(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	modelID := vars["modelId"]

	s.modelsMutex.RLock()
	model, exists := s.models[modelID]
	s.modelsMutex.RUnlock()

	if !exists {
		http.Error(w, "Model not found", http.StatusNotFound)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(model)
}

func (s *MLPipelineService) trainModel(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	modelID := vars["modelId"]

	var req struct {
		TrainingData string `json:"training_data"`
		Parameters   map[string]interface{} `json:"parameters"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request payload", http.StatusBadRequest)
		return
	}

	// Create training job
	job := MLJob{
		ID:        fmt.Sprintf("train-%d", time.Now().UnixNano()),
		Type:      "training",
		ModelID:   modelID,
		Data:      []byte(req.TrainingData),
		Priority:  1,
		CreatedAt: time.Now(),
		Status:    "queued",
	}

	// Add to job queue
	select {
	case s.jobQueue <- job:
		s.metrics.TrainingJobs.WithLabelValues(modelID, "queued").Inc()
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(map[string]interface{}{
			"job_id": job.ID,
			"status": "queued",
			"message": "Training job queued successfully",
		})
	default:
		http.Error(w, "Job queue is full", http.StatusServiceUnavailable)
	}
}

func (s *MLPipelineService) updateModel(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	modelID := vars["modelId"]

	// Update model
	s.modelsMutex.Lock()
	if model, exists := s.models[modelID]; exists {
		model.LastUpdated = time.Now()
		model.Version = fmt.Sprintf("1.%d", time.Now().Unix())
		s.models[modelID] = model
		s.metrics.ModelUpdates.WithLabelValues(modelID, "success").Inc()
	}
	s.modelsMutex.Unlock()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":  "success",
		"message": "Model updated successfully",
	})
}

// Predictions

func (s *MLPipelineService) predictMaintenance(w http.ResponseWriter, r *http.Request) {
	var req struct {
		VehicleID string                 `json:"vehicle_id"`
		Data      map[string]interface{} `json:"data"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request payload", http.StatusBadRequest)
		return
	}

	// Create prediction job
	job := MLJob{
		ID:        fmt.Sprintf("pred-%d", time.Now().UnixNano()),
		Type:      "maintenance_prediction",
		ModelID:   "maintenance-predictor",
		Data:      []byte(fmt.Sprintf(`{"vehicle_id":"%s","data":%s}`, req.VehicleID, req.Data)),
		Priority:  2,
		CreatedAt: time.Now(),
		Status:    "queued",
	}

	// Add to job queue
	select {
	case s.jobQueue <- job:
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(map[string]interface{}{
			"job_id": job.ID,
			"status": "queued",
			"message": "Maintenance prediction queued successfully",
		})
	default:
		http.Error(w, "Job queue is full", http.StatusServiceUnavailable)
	}
}

func (s *MLPipelineService) optimizeRoute(w http.ResponseWriter, r *http.Request) {
	var req struct {
		Origin      Location `json:"origin"`
		Destination Location `json:"destination"`
		Constraints map[string]interface{} `json:"constraints"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request payload", http.StatusBadRequest)
		return
	}

	// Create optimization job
	job := MLJob{
		ID:        fmt.Sprintf("route-%d", time.Now().UnixNano()),
		Type:      "route_optimization",
		ModelID:   "route-optimizer",
		Data:      []byte(fmt.Sprintf(`{"origin":%s,"destination":%s,"constraints":%s}`, req.Origin, req.Destination, req.Constraints)),
		Priority:  2,
		CreatedAt: time.Now(),
		Status:    "queued",
	}

	// Add to job queue
	select {
	case s.jobQueue <- job:
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(map[string]interface{}{
			"job_id": job.ID,
			"status": "queued",
			"message": "Route optimization queued successfully",
		})
	default:
		http.Error(w, "Job queue is full", http.StatusServiceUnavailable)
	}
}

func (s *MLPipelineService) forecastDemand(w http.ResponseWriter, r *http.Request) {
	var req struct {
		LocationID string                 `json:"location_id"`
		TimeWindow string                 `json:"time_window"`
		Factors    map[string]interface{} `json:"factors"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request payload", http.StatusBadRequest)
		return
	}

	// Create forecast job
	job := MLJob{
		ID:        fmt.Sprintf("forecast-%d", time.Now().UnixNano()),
		Type:      "demand_forecast",
		ModelID:   "demand-forecaster",
		Data:      []byte(fmt.Sprintf(`{"location_id":"%s","time_window":"%s","factors":%s}`, req.LocationID, req.TimeWindow, req.Factors)),
		Priority:  2,
		CreatedAt: time.Now(),
		Status:    "queued",
	}

	// Add to job queue
	select {
	case s.jobQueue <- job:
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(map[string]interface{}{
			"job_id": job.ID,
			"status": "queued",
			"message": "Demand forecast queued successfully",
		})
	default:
		http.Error(w, "Job queue is full", http.StatusServiceUnavailable)
	}
}

func (s *MLPipelineService) detectAnomaly(w http.ResponseWriter, r *http.Request) {
	var req struct {
		VehicleID string                 `json:"vehicle_id"`
		Data      map[string]interface{} `json:"data"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request payload", http.StatusBadRequest)
		return
	}

	// Create anomaly detection job
	job := MLJob{
		ID:        fmt.Sprintf("anomaly-%d", time.Now().UnixNano()),
		Type:      "anomaly_detection",
		ModelID:   "anomaly-detector",
		Data:      []byte(fmt.Sprintf(`{"vehicle_id":"%s","data":%s}`, req.VehicleID, req.Data)),
		Priority:  1, // High priority for anomaly detection
		CreatedAt: time.Now(),
		Status:    "queued",
	}

	// Add to job queue
	select {
	case s.jobQueue <- job:
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(map[string]interface{}{
			"job_id": job.ID,
			"status": "queued",
			"message": "Anomaly detection queued successfully",
		})
	default:
		http.Error(w, "Job queue is full", http.StatusServiceUnavailable)
	}
}

// Batch Processing

func (s *MLPipelineService) batchPredict(w http.ResponseWriter, r *http.Request) {
	var req struct {
		ModelID string                   `json:"model_id"`
		Data    []map[string]interface{} `json:"data"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request payload", http.StatusBadRequest)
		return
	}

	// Create batch prediction jobs
	jobIDs := make([]string, 0, len(req.Data))
	for i, data := range req.Data {
		job := MLJob{
			ID:        fmt.Sprintf("batch-%d-%d", time.Now().UnixNano(), i),
			Type:      "batch_prediction",
			ModelID:   req.ModelID,
			Data:      []byte(fmt.Sprintf(`{"data":%s}`, data)),
			Priority:  3,
			CreatedAt: time.Now(),
			Status:    "queued",
		}

		select {
		case s.jobQueue <- job:
			jobIDs = append(jobIDs, job.ID)
		default:
			http.Error(w, "Job queue is full", http.StatusServiceUnavailable)
			return
		}
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"job_ids": jobIDs,
		"status":  "queued",
		"message": "Batch prediction queued successfully",
	})
}

func (s *MLPipelineService) batchTrain(w http.ResponseWriter, r *http.Request) {
	var req struct {
		ModelID       string `json:"model_id"`
		TrainingData  string `json:"training_data"`
		Parameters    map[string]interface{} `json:"parameters"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request payload", http.StatusBadRequest)
		return
	}

	// Create batch training job
	job := MLJob{
		ID:        fmt.Sprintf("batch-train-%d", time.Now().UnixNano()),
		Type:      "batch_training",
		ModelID:   req.ModelID,
		Data:      []byte(req.TrainingData),
		Priority:  1,
		CreatedAt: time.Now(),
		Status:    "queued",
	}

	// Add to job queue
	select {
	case s.jobQueue <- job:
		s.metrics.TrainingJobs.WithLabelValues(req.ModelID, "queued").Inc()
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(map[string]interface{}{
			"job_id": job.ID,
			"status": "queued",
			"message": "Batch training queued successfully",
		})
	default:
		http.Error(w, "Job queue is full", http.StatusServiceUnavailable)
	}
}

// Model Performance

func (s *MLPipelineService) getModelPerformance(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	modelID := vars["modelId"]

	s.modelsMutex.RLock()
	model, exists := s.models[modelID]
	s.modelsMutex.RUnlock()

	if !exists {
		http.Error(w, "Model not found", http.StatusNotFound)
		return
	}

	performance := map[string]interface{}{
		"model_id":    model.ID,
		"accuracy":    model.Accuracy,
		"status":      model.Status,
		"last_trained": model.LastTrained,
		"last_updated": model.LastUpdated,
		"version":     model.Version,
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(performance)
}

func (s *MLPipelineService) getModelMetrics(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	modelID := vars["modelId"]

	// Mock metrics - in production, this would fetch from metrics store
	metrics := map[string]interface{}{
		"model_id":           modelID,
		"total_inferences":   1500,
		"successful_inferences": 1425,
		"failed_inferences":    75,
		"average_latency":  0.15,
		"accuracy":           0.92,
		"precision":          0.89,
		"recall":             0.91,
		"f1_score":           0.90,
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(metrics)
}

// Background Services

func (s *MLPipelineService) startModelUpdater(ctx context.Context) {
	go func() {
		ticker := time.NewTicker(time.Duration(s.config.ModelUpdateInterval) * time.Second)
		defer ticker.Stop()

		for {
			select {
			case <-ticker.C:
				s.updateModels()
			case <-ctx.Done():
				return
			}
		}
	}()
}

func (s *MLPipelineService) updateModels() {
	log.Printf("🔄 Updating ML models...")
	
	s.modelsMutex.Lock()
	for id, model := range s.models {
		model.LastUpdated = time.Now()
		model.Version = fmt.Sprintf("1.%d", time.Now().Unix())
		s.models[id] = model
		s.metrics.ModelUpdates.WithLabelValues(id, "success").Inc()
	}
	s.modelsMutex.Unlock()
	
	log.Printf("✅ ML models updated")
}

func (s *MLPipelineService) startDataProcessor(ctx context.Context) {
	go func() {
		for {
			select {
			case <-ctx.Done():
				return
			default:
				message, err := s.kafkaReader.ReadMessage(ctx)
				if err != nil {
					log.Printf("❌ Failed to read ML event from Kafka: %v", err)
					continue
				}

				s.processMLEvent(ctx, message)
			}
		}
	}()
}

func (s *MLPipelineService) processMLEvent(ctx context.Context, message kafka.Message) {
	// Process ML events from Kafka
	log.Printf("🤖 Processing ML event: %s", string(message.Key))
}

func (s *MLPipelineService) startMetricsCollector(ctx context.Context) {
	go func() {
		ticker := time.NewTicker(30 * time.Second)
		defer ticker.Stop()

		for {
			select {
			case <-ticker.C:
				s.collectMetrics()
			case <-ctx.Done():
				return
			}
		}
	}()
}

func (s *MLPipelineService) collectMetrics() {
	// Collect and update metrics
	s.modelsMutex.RLock()
	for _, model := range s.models {
		s.metrics.ModelAccuracy.WithLabelValues(model.ID, model.Type).Set(model.Accuracy)
	}
	s.modelsMutex.RUnlock()
}