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
	"github.com/prometheus/client_golang/prometheus/promauto"
	"github.com/redis/go-redis/v9"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// EnergyManagementService provides comprehensive energy management for autonomous fleets
type EnergyManagementService struct {
	config      Config
	tracer      trace.Tracer
	metrics     *Metrics
	redis       *redis.Client
	kafkaWriter *KafkaWriter
	clickhouse  *ClickHouseClient
	logger      *Logger
	router      *mux.Router
	
	// Energy management components
	chargingScheduler *ChargingScheduler
	v2gManager       *V2GManager
	energyOptimizer  *EnergyOptimizer
	gridManager      *GridManager
	batteryManager   *BatteryManager
}

// Config holds configuration for the Energy Management service
type Config struct {
	Port                int      `json:"port"`
	RedisURL            string   `json:"redis_url"`
	KafkaBrokers        []string `json:"kafka_brokers"`
	ClickHouseURL       string   `json:"clickhouse_url"`
	LogLevel            string   `json:"log_level"`
	BufferSize          int      `json:"buffer_size"`
	RedisTTL            time.Duration `json:"redis_ttl"`
	
	// Energy management specific
	GridIntegration     bool     `json:"grid_integration"`
	V2GEnabled         bool     `json:"v2g_enabled"`
	SmartCharging      bool     `json:"smart_charging"`
	EnergyOptimization bool     `json:"energy_optimization"`
	
	// Charging configuration
	MaxChargingPower   float64  `json:"max_charging_power"`
	MinChargingPower   float64  `json:"min_charging_power"`
	ChargingEfficiency float64  `json:"charging_efficiency"`
	
	// V2G configuration
	MaxV2GPower        float64  `json:"max_v2g_power"`
	V2GEfficiency      float64  `json:"v2g_efficiency"`
	V2GMinBatteryLevel float64  `json:"v2g_min_battery_level"`
	
	// Grid configuration
	GridAPIURL         string   `json:"grid_api_url"`
	GridAPIKey         string   `json:"grid_api_key"`
	GridUpdateInterval time.Duration `json:"grid_update_interval"`
}

// Metrics tracks energy management metrics
type Metrics struct {
	ChargingSessions    *prometheus.CounterVec
	V2GSessions         *prometheus.CounterVec
	EnergyTransferred   *prometheus.CounterVec
	BatteryLevels       *prometheus.GaugeVec
	ChargingPower       *prometheus.GaugeVec
	GridPower           *prometheus.GaugeVec
	EnergyEfficiency    *prometheus.GaugeVec
	CostSavings         *prometheus.CounterVec
	OptimizationRuns    *prometheus.CounterVec
	ErrorCount          *prometheus.CounterVec
}

// ChargingScheduler manages charging schedules
type ChargingScheduler struct {
	scheduler *Scheduler
	vehicles  map[string]*Vehicle
	mutex     sync.RWMutex
}

// V2GManager manages Vehicle-to-Grid operations
type V2GManager struct {
	vehicles    map[string]*Vehicle
	gridStatus  *GridStatus
	mutex       sync.RWMutex
}

// EnergyOptimizer optimizes energy usage across the fleet
type EnergyOptimizer struct {
	optimizer *Optimizer
	vehicles  map[string]*Vehicle
	mutex     sync.RWMutex
}

// GridManager manages grid integration
type GridManager struct {
	gridAPI    *GridAPI
	status     *GridStatus
	updateTime time.Time
	mutex      sync.RWMutex
}

// BatteryManager manages battery health and optimization
type BatteryManager struct {
	vehicles map[string]*Vehicle
	mutex    sync.RWMutex
}

// Vehicle represents a vehicle in the energy management system
type Vehicle struct {
	ID              string                 `json:"id"`
	FleetID         string                 `json:"fleet_id"`
	BatteryLevel    float64                `json:"battery_level"`
	BatteryCapacity float64                `json:"battery_capacity"`
	ChargingStatus  string                 `json:"charging_status"`
	Location        Location               `json:"location"`
	EnergyProfile   EnergyProfile          `json:"energy_profile"`
	V2GCapable      bool                   `json:"v2g_capable"`
	ChargingRate    float64                `json:"charging_rate"`
	LastUpdate      time.Time              `json:"last_update"`
	Metadata        map[string]interface{} `json:"metadata"`
}

// EnergyProfile represents a vehicle's energy consumption profile
type EnergyProfile struct {
	AverageConsumption float64 `json:"average_consumption"`
	PeakConsumption    float64 `json:"peak_consumption"`
	Efficiency         float64 `json:"efficiency"`
	Range              float64 `json:"range"`
	ChargingTime       float64 `json:"charging_time"`
}

// Location represents a geographical location
type Location struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Altitude  float64 `json:"altitude"`
	Accuracy  float64 `json:"accuracy"`
}

// GridStatus represents the current grid status
type GridStatus struct {
	Demand           float64                `json:"demand"`
	Supply           float64                `json:"supply"`
	Price            float64                `json:"price"`
	CarbonIntensity  float64                `json:"carbon_intensity"`
	RenewablePercent float64                `json:"renewable_percent"`
	Timestamp        time.Time              `json:"timestamp"`
	Metadata         map[string]interface{} `json:"metadata"`
}

// ChargingSession represents a charging session
type ChargingSession struct {
	ID              string    `json:"id"`
	VehicleID       string    `json:"vehicle_id"`
	FleetID         string    `json:"fleet_id"`
	StartTime       time.Time `json:"start_time"`
	EndTime         time.Time `json:"end_time"`
	StartLevel      float64   `json:"start_level"`
	EndLevel        float64   `json:"end_level"`
	EnergyCharged   float64   `json:"energy_charged"`
	PowerUsed       float64   `json:"power_used"`
	Cost            float64   `json:"cost"`
	ChargingStation string    `json:"charging_station"`
	Status          string    `json:"status"`
}

// V2GSession represents a Vehicle-to-Grid session
type V2GSession struct {
	ID              string    `json:"id"`
	VehicleID       string    `json:"vehicle_id"`
	FleetID         string    `json:"fleet_id"`
	StartTime       time.Time `json:"start_time"`
	EndTime         time.Time `json:"end_time"`
	EnergyDischarged float64   `json:"energy_discharged"`
	PowerProvided   float64   `json:"power_provided"`
	Revenue         float64   `json:"revenue"`
	GridService     string    `json:"grid_service"`
	Status          string    `json:"status"`
}

// EnergyOptimization represents an energy optimization result
type EnergyOptimization struct {
	ID              string                 `json:"id"`
	FleetID         string                 `json:"fleet_id"`
	Timestamp       time.Time              `json:"timestamp"`
	OptimizationType string                `json:"optimization_type"`
	Vehicles        []string               `json:"vehicles"`
	EnergySaved     float64                `json:"energy_saved"`
	CostReduction   float64                `json:"cost_reduction"`
	Recommendations []OptimizationRecommendation `json:"recommendations"`
	Status          string                 `json:"status"`
}

// OptimizationRecommendation represents an optimization recommendation
type OptimizationRecommendation struct {
	VehicleID       string  `json:"vehicle_id"`
	Recommendation  string  `json:"recommation"`
	Priority        string  `json:"priority"`
	ExpectedSavings float64 `json:"expected_savings"`
	Implementation  string  `json:"implementation"`
}

// Scheduler manages scheduling operations
type Scheduler struct {
	schedules map[string]*Schedule
	mutex     sync.RWMutex
}

// Schedule represents a charging schedule
type Schedule struct {
	ID              string    `json:"id"`
	VehicleID       string    `json:"vehicle_id"`
	FleetID         string    `json:"fleet_id"`
	StartTime       time.Time `json:"start_time"`
	EndTime         time.Time `json:"end_time"`
	TargetLevel     float64   `json:"target_level"`
	Priority        int       `json:"priority"`
	Status          string    `json:"status"`
	CreatedAt       time.Time `json:"created_at"`
	UpdatedAt       time.Time `json:"updated_at"`
}

// Optimizer manages optimization operations
type Optimizer struct {
	optimizations map[string]*EnergyOptimization
	mutex         sync.RWMutex
}

// GridAPI manages grid API interactions
type GridAPI struct {
	baseURL string
	apiKey  string
	client  *http.Client
}

// Logger provides structured logging
type Logger struct {
	level string
}

// KafkaWriter manages Kafka message publishing
type KafkaWriter struct {
	writer *kafka.Writer
}

// ClickHouseClient manages ClickHouse interactions
type ClickHouseClient struct {
	client *clickhouse.Client
}

// NewEnergyManagementService creates a new energy management service
func NewEnergyManagementService(config Config) *EnergyManagementService {
	// Initialize metrics
	metrics := &Metrics{
		ChargingSessions: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "energy_charging_sessions_total",
				Help: "Total number of charging sessions",
			},
			[]string{"fleet_id", "status"},
		),
		V2GSessions: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "energy_v2g_sessions_total",
				Help: "Total number of V2G sessions",
			},
			[]string{"fleet_id", "status"},
		),
		EnergyTransferred: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "energy_transferred_total",
				Help: "Total energy transferred in kWh",
			},
			[]string{"type", "fleet_id"},
		),
		BatteryLevels: promauto.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "energy_battery_level_percent",
				Help: "Battery level percentage",
			},
			[]string{"vehicle_id", "fleet_id"},
		),
		ChargingPower: promauto.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "energy_charging_power_kw",
				Help: "Charging power in kW",
			},
			[]string{"vehicle_id", "fleet_id"},
		),
		GridPower: promauto.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "energy_grid_power_kw",
				Help: "Grid power in kW",
			},
			[]string{"grid_id"},
		),
		EnergyEfficiency: promauto.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "energy_efficiency_percent",
				Help: "Energy efficiency percentage",
			},
			[]string{"fleet_id"},
		),
		CostSavings: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "energy_cost_savings_total",
				Help: "Total cost savings in currency units",
			},
			[]string{"fleet_id", "type"},
		),
		OptimizationRuns: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "energy_optimization_runs_total",
				Help: "Total number of optimization runs",
			},
			[]string{"fleet_id", "type", "status"},
		),
		ErrorCount: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "energy_errors_total",
				Help: "Total number of errors",
			},
			[]string{"error_type", "component"},
		),
	}
	
	// Initialize components
	chargingScheduler := &ChargingScheduler{
		scheduler: &Scheduler{
			schedules: make(map[string]*Schedule),
		},
		vehicles: make(map[string]*Vehicle),
	}
	
	v2gManager := &V2GManager{
		vehicles:   make(map[string]*Vehicle),
		gridStatus: &GridStatus{},
	}
	
	energyOptimizer := &EnergyOptimizer{
		optimizer: &Optimizer{
			optimizations: make(map[string]*EnergyOptimization),
		},
		vehicles: make(map[string]*Vehicle),
	}
	
	gridManager := &GridManager{
		gridAPI: &GridAPI{
			baseURL: config.GridAPIURL,
			apiKey:  config.GridAPIKey,
			client:  &http.Client{Timeout: 30 * time.Second},
		},
		status: &GridStatus{},
	}
	
	batteryManager := &BatteryManager{
		vehicles: make(map[string]*Vehicle),
	}
	
	return &EnergyManagementService{
		config:            config,
		tracer:            otel.Tracer("energy-management-service"),
		metrics:           metrics,
		chargingScheduler: chargingScheduler,
		v2gManager:        v2gManager,
		energyOptimizer:   energyOptimizer,
		gridManager:       gridManager,
		batteryManager:    batteryManager,
	}
}

// loadConfig loads configuration from environment variables
func loadConfig() Config {
	return Config{
		Port:                getEnvInt("PORT", 8080),
		RedisURL:            getEnv("REDIS_URL", "redis://localhost:6379/0"),
		KafkaBrokers:        getEnvStringSlice("KAFKA_BROKERS", []string{"localhost:9092"}),
		ClickHouseURL:       getEnv("CLICKHOUSE_URL", "http://localhost:8123"),
		LogLevel:            getEnv("LOG_LEVEL", "info"),
		BufferSize:          getEnvInt("BUFFER_SIZE", 1000),
		RedisTTL:            time.Duration(getEnvInt("REDIS_TTL_SECONDS", 300)) * time.Second,
		
		GridIntegration:     getEnvBool("GRID_INTEGRATION", true),
		V2GEnabled:         getEnvBool("V2G_ENABLED", true),
		SmartCharging:      getEnvBool("SMART_CHARGING", true),
		EnergyOptimization: getEnvBool("ENERGY_OPTIMIZATION", true),
		
		MaxChargingPower:    getEnvFloat("MAX_CHARGING_POWER", 150.0),
		MinChargingPower:    getEnvFloat("MIN_CHARGING_POWER", 3.7),
		ChargingEfficiency:  getEnvFloat("CHARGING_EFFICIENCY", 0.95),
		
		MaxV2GPower:         getEnvFloat("MAX_V2G_POWER", 22.0),
		V2GEfficiency:       getEnvFloat("V2G_EFFICIENCY", 0.90),
		V2GMinBatteryLevel:  getEnvFloat("V2G_MIN_BATTERY_LEVEL", 0.20),
		
		GridAPIURL:          getEnv("GRID_API_URL", "https://api.grid.example.com"),
		GridAPIKey:          getEnv("GRID_API_KEY", "your-grid-api-key"),
		GridUpdateInterval:  time.Duration(getEnvInt("GRID_UPDATE_INTERVAL_SECONDS", 300)) * time.Second,
	}
}

// setupRoutes sets up HTTP routes
func (ems *EnergyManagementService) setupRoutes() *mux.Router {
	router := mux.NewRouter()
	api := router.PathPrefix("/api/v1").Subrouter()
	
	// Charging management endpoints
	api.HandleFunc("/charging/sessions", ems.listChargingSessions).Methods("GET")
	api.HandleFunc("/charging/sessions", ems.createChargingSession).Methods("POST")
	api.HandleFunc("/charging/sessions/{id}", ems.getChargingSession).Methods("GET")
	api.HandleFunc("/charging/sessions/{id}", ems.updateChargingSession).Methods("PUT")
	api.HandleFunc("/charging/sessions/{id}", ems.deleteChargingSession).Methods("DELETE")
	api.HandleFunc("/charging/sessions/{id}/start", ems.startChargingSession).Methods("POST")
	api.HandleFunc("/charging/sessions/{id}/stop", ems.stopChargingSession).Methods("POST")
	
	// V2G management endpoints
	api.HandleFunc("/v2g/sessions", ems.listV2GSessions).Methods("GET")
	api.HandleFunc("/v2g/sessions", ems.createV2GSession).Methods("POST")
	api.HandleFunc("/v2g/sessions/{id}", ems.getV2GSession).Methods("GET")
	api.HandleFunc("/v2g/sessions/{id}", ems.updateV2GSession).Methods("PUT")
	api.HandleFunc("/v2g/sessions/{id}", ems.deleteV2GSession).Methods("DELETE")
	api.HandleFunc("/v2g/sessions/{id}/start", ems.startV2GSession).Methods("POST")
	api.HandleFunc("/v2g/sessions/{id}/stop", ems.stopV2GSession).Methods("POST")
	
	// Energy optimization endpoints
	api.HandleFunc("/optimization", ems.listOptimizations).Methods("GET")
	api.HandleFunc("/optimization", ems.createOptimization).Methods("POST")
	api.HandleFunc("/optimization/{id}", ems.getOptimization).Methods("GET")
	api.HandleFunc("/optimization/{id}/run", ems.runOptimization).Methods("POST")
	api.HandleFunc("/optimization/{id}/results", ems.getOptimizationResults).Methods("GET")
	
	// Grid integration endpoints
	api.HandleFunc("/grid/status", ems.getGridStatus).Methods("GET")
	api.HandleFunc("/grid/price", ems.getGridPrice).Methods("GET")
	api.HandleFunc("/grid/demand", ems.getGridDemand).Methods("GET")
	api.HandleFunc("/grid/supply", ems.getGridSupply).Methods("GET")
	
	// Battery management endpoints
	api.HandleFunc("/battery/health", ems.getBatteryHealth).Methods("GET")
	api.HandleFunc("/battery/optimization", ems.optimizeBatteryUsage).Methods("POST")
	api.HandleFunc("/battery/maintenance", ems.scheduleBatteryMaintenance).Methods("POST")
	
	// Fleet energy endpoints
	api.HandleFunc("/fleet/{fleet_id}/energy", ems.getFleetEnergyStatus).Methods("GET")
	api.HandleFunc("/fleet/{fleet_id}/charging", ems.getFleetChargingStatus).Methods("GET")
	api.HandleFunc("/fleet/{fleet_id}/v2g", ems.getFleetV2GStatus).Methods("GET")
	api.HandleFunc("/fleet/{fleet_id}/optimization", ems.optimizeFleetEnergy).Methods("POST")
	
	// Vehicle energy endpoints
	api.HandleFunc("/vehicles/{vehicle_id}/energy", ems.getVehicleEnergyStatus).Methods("GET")
	api.HandleFunc("/vehicles/{vehicle_id}/charging", ems.getVehicleChargingStatus).Methods("GET")
	api.HandleFunc("/vehicles/{vehicle_id}/v2g", ems.getVehicleV2GStatus).Methods("GET")
	api.HandleFunc("/vehicles/{vehicle_id}/schedule", ems.scheduleVehicleCharging).Methods("POST")
	
	// Analytics endpoints
	api.HandleFunc("/analytics/energy-usage", ems.getEnergyUsageAnalytics).Methods("GET")
	api.HandleFunc("/analytics/cost-analysis", ems.getCostAnalysis).Methods("GET")
	api.HandleFunc("/analytics/efficiency", ems.getEfficiencyMetrics).Methods("GET")
	api.HandleFunc("/analytics/savings", ems.getSavingsReport).Methods("GET")
	
	// Health and metrics
	api.HandleFunc("/health", ems.healthCheck).Methods("GET")
	router.Handle("/metrics", promhttp.Handler())
	
	return router
}

// Charging Session Management

func (ems *EnergyManagementService) listChargingSessions(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "listChargingSessions")
	defer span.End()
	
	// Get query parameters
	fleetID := r.URL.Query().Get("fleet_id")
	vehicleID := r.URL.Query().Get("vehicle_id")
	status := r.URL.Query().Get("status")
	startDate := r.URL.Query().Get("start_date")
	endDate := r.URL.Query().Get("end_date")
	
	// Build query
	query := "SELECT * FROM charging_sessions WHERE 1=1"
	args := []interface{}{}
	argIndex := 1
	
	if fleetID != "" {
		query += fmt.Sprintf(" AND fleet_id = $%d", argIndex)
		args = append(args, fleetID)
		argIndex++
	}
	
	if vehicleID != "" {
		query += fmt.Sprintf(" AND vehicle_id = $%d", argIndex)
		args = append(args, vehicleID)
		argIndex++
	}
	
	if status != "" {
		query += fmt.Sprintf(" AND status = $%d", argIndex)
		args = append(args, status)
		argIndex++
	}
	
	if startDate != "" {
		query += fmt.Sprintf(" AND start_time >= $%d", argIndex)
		args = append(args, startDate)
		argIndex++
	}
	
	if endDate != "" {
		query += fmt.Sprintf(" AND start_time <= $%d", argIndex)
		args = append(args, endDate)
		argIndex++
	}
	
	query += " ORDER BY start_time DESC LIMIT 100"
	
	// Execute query
	sessions := []ChargingSession{}
	// Implementation would execute query and populate sessions
	
	// Return response
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"sessions": sessions,
		"total":   len(sessions),
		"timestamp": time.Now(),
	})
}

func (ems *EnergyManagementService) createChargingSession(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "createChargingSession")
	defer span.End()
	
	var session ChargingSession
	if err := json.NewDecoder(r.Body).Decode(&session); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}
	
	// Validate input
	if err := ems.validateChargingSession(session); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	
	// Generate ID and timestamps
	session.ID = ems.generateID()
	session.StartTime = time.Now()
	session.Status = "scheduled"
	
	// Store in database
	// Implementation would store in database
	
	// Update metrics
	ems.metrics.ChargingSessions.WithLabelValues(session.FleetID, session.Status).Inc()
	
	// Return response
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusCreated)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"session":   session,
		"status":    "created",
		"timestamp": time.Now(),
	})
}

func (ems *EnergyManagementService) getChargingSession(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "getChargingSession")
	defer span.End()
	
	vars := mux.Vars(r)
	sessionID := vars["id"]
	
	// Get session from database
	session := &ChargingSession{}
	// Implementation would retrieve from database
	
	// Return response
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"session":   session,
		"timestamp": time.Now(),
	})
}

func (ems *EnergyManagementService) updateChargingSession(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "updateChargingSession")
	defer span.End()
	
	vars := mux.Vars(r)
	sessionID := vars["id"]
	
	var updates map[string]interface{}
	if err := json.NewDecoder(r.Body).Decode(&updates); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}
	
	// Update session in database
	// Implementation would update in database
	
	// Return response
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":    "updated",
		"timestamp": time.Now(),
	})
}

func (ems *EnergyManagementService) deleteChargingSession(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "deleteChargingSession")
	defer span.End()
	
	vars := mux.Vars(r)
	sessionID := vars["id"]
	
	// Delete session from database
	// Implementation would delete from database
	
	// Return response
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":    "deleted",
		"timestamp": time.Now(),
	})
}

func (ems *EnergyManagementService) startChargingSession(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "startChargingSession")
	defer span.End()
	
	vars := mux.Vars(r)
	sessionID := vars["id"]
	
	// Start charging session
	// Implementation would start charging
	
	// Update metrics
	ems.metrics.ChargingSessions.WithLabelValues("", "active").Inc()
	
	// Return response
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":    "started",
		"timestamp": time.Now(),
	})
}

func (ems *EnergyManagementService) stopChargingSession(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "stopChargingSession")
	defer span.End()
	
	vars := mux.Vars(r)
	sessionID := vars["id"]
	
	// Stop charging session
	// Implementation would stop charging
	
	// Update metrics
	ems.metrics.ChargingSessions.WithLabelValues("", "completed").Inc()
	
	// Return response
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":    "stopped",
		"timestamp": time.Now(),
	})
}

// V2G Session Management

func (ems *EnergyManagementService) listV2GSessions(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "listV2GSessions")
	defer span.End()
	
	// Implementation similar to charging sessions
	sessions := []V2GSession{}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"sessions":  sessions,
		"total":    len(sessions),
		"timestamp": time.Now(),
	})
}

func (ems *EnergyManagementService) createV2GSession(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "createV2GSession")
	defer span.End()
	
	var session V2GSession
	if err := json.NewDecoder(r.Body).Decode(&session); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}
	
	// Validate input
	if err := ems.validateV2GSession(session); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	
	// Generate ID and timestamps
	session.ID = ems.generateID()
	session.StartTime = time.Now()
	session.Status = "scheduled"
	
	// Store in database
	// Implementation would store in database
	
	// Update metrics
	ems.metrics.V2GSessions.WithLabelValues(session.FleetID, session.Status).Inc()
	
	// Return response
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusCreated)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"session":   session,
		"status":    "created",
		"timestamp": time.Now(),
	})
}

// Energy Optimization

func (ems *EnergyManagementService) listOptimizations(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "listOptimizations")
	defer span.End()
	
	// Implementation would list optimizations
	optimizations := []EnergyOptimization{}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"optimizations": optimizations,
		"total":        len(optimizations),
		"timestamp":    time.Now(),
	})
}

func (ems *EnergyManagementService) createOptimization(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "createOptimization")
	defer span.End()
	
	var optimization EnergyOptimization
	if err := json.NewDecoder(r.Body).Decode(&optimization); err != nil {
		http.Error(w, "Invalid JSON", http.StatusBadRequest)
		return
	}
	
	// Generate ID and timestamps
	optimization.ID = ems.generateID()
	optimization.Timestamp = time.Now()
	optimization.Status = "pending"
	
	// Store in database
	// Implementation would store in database
	
	// Update metrics
	ems.metrics.OptimizationRuns.WithLabelValues(optimization.FleetID, optimization.OptimizationType, optimization.Status).Inc()
	
	// Return response
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusCreated)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"optimization": optimization,
		"status":      "created",
		"timestamp":   time.Now(),
	})
}

// Grid Integration

func (ems *EnergyManagementService) getGridStatus(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "getGridStatus")
	defer span.End()
	
	// Get grid status
	status := ems.gridManager.GetStatus()
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":    status,
		"timestamp": time.Now(),
	})
}

// Battery Management

func (ems *EnergyManagementService) getBatteryHealth(w http.ResponseWriter, r *http.Request) {
	ctx, span := ems.tracer.Start(r.Context(), "getBatteryHealth")
	defer span.End()
	
	// Get battery health data
	health := ems.batteryManager.GetHealth()
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"health":    health,
		"timestamp": time.Now(),
	})
}

// Health Check

func (ems *EnergyManagementService) healthCheck(w http.ResponseWriter, r *http.Request) {
	health := map[string]interface{}{
		"status":    "healthy",
		"timestamp": time.Now(),
		"service":   "energy-management",
		"version":   "1.0.0",
	}
	
	// Check dependencies
	if ems.redis != nil {
		ctx := context.Background()
		if err := ems.redis.Ping(ctx).Err(); err != nil {
			health["status"] = "unhealthy"
			health["redis"] = "failed"
		} else {
			health["redis"] = "healthy"
		}
	}
	
	if health["status"] == "unhealthy" {
		w.WriteHeader(http.StatusServiceUnavailable)
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(health)
}

// Validation functions

func (ems *EnergyManagementService) validateChargingSession(session ChargingSession) error {
	if session.VehicleID == "" {
		return fmt.Errorf("vehicle_id is required")
	}
	if session.FleetID == "" {
		return fmt.Errorf("fleet_id is required")
	}
	if session.StartLevel < 0 || session.StartLevel > 100 {
		return fmt.Errorf("start_level must be between 0 and 100")
	}
	if session.EndLevel < 0 || session.EndLevel > 100 {
		return fmt.Errorf("end_level must be between 0 and 100")
	}
	if session.StartLevel >= session.EndLevel {
		return fmt.Errorf("end_level must be greater than start_level")
	}
	return nil
}

func (ems *EnergyManagementService) validateV2GSession(session V2GSession) error {
	if session.VehicleID == "" {
		return fmt.Errorf("vehicle_id is required")
	}
	if session.FleetID == "" {
		return fmt.Errorf("fleet_id is required")
	}
	if session.GridService == "" {
		return fmt.Errorf("grid_service is required")
	}
	return nil
}

// Utility functions

func (ems *EnergyManagementService) generateID() string {
	return fmt.Sprintf("em_%d_%d", time.Now().UnixNano(), time.Now().Unix())
}

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

func getEnvStringSlice(key string, defaultValue []string) []string {
	if value := os.Getenv(key); value != "" {
		return []string{value} // Simplified - in production, parse comma-separated values
	}
	return defaultValue
}

// Main function

func main() {
	config := loadConfig()
	service := NewEnergyManagementService(config)
	
	// Setup routes
	router := service.setupRoutes()
	
	// Start server
	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.Port),
		Handler: router,
	}
	
	// Start background services
	go service.startBackgroundServices()
	
	// Start server
	go func() {
		log.Printf("🚀 Energy Management Service starting on port %d", config.Port)
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Server failed: %v", err)
		}
	}()
	
	// Graceful shutdown
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c
	
	log.Println("🛑 Shutting down Energy Management Service...")
	server.Shutdown(context.Background())
}

// Background services

func (ems *EnergyManagementService) startBackgroundServices() {
	// Start grid status updates
	if ems.config.GridIntegration {
		go ems.updateGridStatus()
	}
	
	// Start energy optimization
	if ems.config.EnergyOptimization {
		go ems.runEnergyOptimization()
	}
	
	// Start battery health monitoring
	go ems.monitorBatteryHealth()
}

func (ems *EnergyManagementService) updateGridStatus() {
	ticker := time.NewTicker(ems.config.GridUpdateInterval)
	defer ticker.Stop()
	
	for range ticker.C {
		// Update grid status
		// Implementation would update grid status
	}
}

func (ems *EnergyManagementService) runEnergyOptimization() {
	ticker := time.NewTicker(1 * time.Hour)
	defer ticker.Stop()
	
	for range ticker.C {
		// Run energy optimization
		// Implementation would run optimization
	}
}

func (ems *EnergyManagementService) monitorBatteryHealth() {
	ticker := time.NewTicker(5 * time.Minute)
	defer ticker.Stop()
	
	for range ticker.C {
		// Monitor battery health
		// Implementation would monitor battery health
	}
}
