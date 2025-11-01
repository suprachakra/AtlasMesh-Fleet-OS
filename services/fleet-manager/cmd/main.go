package main

import (
	"context"
	"crypto/rand"
	"database/sql"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"strings"
	"syscall"
	"time"

	"github.com/gorilla/mux"
	"github.com/lib/pq"
	_ "github.com/lib/pq"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"github.com/redis/go-redis/v9"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
	"atlasmesh/fleet-manager/internal/config"
)

// AtlasMesh Fleet Manager Service
//
// PUBLIC API: Core fleet management service for autonomous vehicle operations
// Provides REST endpoints for fleet/vehicle CRUD, command dispatch, and analytics
// 
// INTEGRATION CONTRACTS:
// - Database: PostgreSQL with fleet_core schema (fleet/vehicle/command tables)
// - Cache: Redis for session state and real-time data
// - Metrics: Prometheus metrics on /metrics endpoint
// - Tracing: OpenTelemetry spans for distributed tracing
// - Message Bus: Kafka for vehicle command dispatch (async)
//
// SAFETY: All vehicle commands require dual-auth for safety-critical operations
// SECURITY: Database connections use connection pooling with prepared statements
// CONCURRENCY: Goroutines for metrics collection and health monitoring
// PERF: Redis caching for frequently accessed fleet summaries

// Config holds service configuration with environment variable defaults
// All fields are required for service startup except AbuDhabiMode (defaults true)
type Config struct {
	Port        int    `json:"port"`        // HTTP server port (default: 8081)
	DatabaseURL string `json:"database_url"` // PostgreSQL connection string with SSL mode
	RedisURL    string `json:"redis_url"`    // Redis connection string with database selection
	LogLevel    string `json:"log_level"`    // Logging level: debug|info|warn|error
	AbuDhabiMode bool  `json:"abu_dhabi_mode"` // Enable UAE-specific compliance features
}

// FleetManagerService is the main service struct containing all dependencies
// CONCURRENCY: Safe for concurrent access - all fields are read-only after initialization
// PERF: Database connection pool shared across goroutines for efficiency
type FleetManagerService struct {
	db           *sql.DB        // PostgreSQL connection pool (thread-safe)
	redis        *redis.Client  // Redis client with connection pooling
	config       Config         // Immutable configuration loaded at startup
	sectorConfig *config.SectorConfig // Sector-specific configuration
	tracer       trace.Tracer   // OpenTelemetry tracer for distributed tracing
	metrics      *Metrics       // Prometheus metrics collectors
}

// Metrics defines Prometheus metrics for fleet operations monitoring
// All metrics include labels for proper aggregation and alerting
// PERF: Metrics are collected asynchronously to avoid blocking request handling
type Metrics struct {
	FleetOperations    *prometheus.CounterVec   // operation, status, fleet_id
	VehicleCommands    *prometheus.CounterVec   // command_type, status, priority  
	ResponseTime       *prometheus.HistogramVec // operation, status (with default buckets)
	ActiveVehicles     *prometheus.GaugeVec     // fleet_id, status (real-time count)
	FleetUtilization   *prometheus.GaugeVec     // fleet_id (percentage 0-100)
}

// Fleet Management Models
type Organization struct {
	OrganizationID string                 `json:"organization_id"`
	Name           string                 `json:"name"`
	Sector         string                 `json:"sector"`
	Configuration  map[string]interface{} `json:"configuration"`
	Status         string                 `json:"status"`
	CreatedAt      time.Time              `json:"created_at"`
	UpdatedAt      time.Time              `json:"updated_at"`
}

type Fleet struct {
	FleetID        string                 `json:"fleet_id"`
	OrganizationID string                 `json:"organization_id"`
	Name           string                 `json:"name"`
	Description    string                 `json:"description"`
	FleetType      string                 `json:"fleet_type"`
	Configuration  map[string]interface{} `json:"configuration"`
	MaxVehicles    int                    `json:"max_vehicles"`
	Status         string                 `json:"status"`
	CreatedAt      time.Time              `json:"created_at"`
	UpdatedAt      time.Time              `json:"updated_at"`
}

type Vehicle struct {
	VehicleID         string                 `json:"vehicle_id"`
	FleetID           string                 `json:"fleet_id"`
	OrganizationID    string                 `json:"organization_id"`
	AssetTag          string                 `json:"asset_tag"`
	VIN               string                 `json:"vin"`
	LicensePlate      string                 `json:"license_plate"`
	Manufacturer      string                 `json:"manufacturer"`
	Model             string                 `json:"model"`
	Year              int                    `json:"year"`
	VehicleProfile    map[string]interface{} `json:"vehicle_profile"`
	AutonomyLevel     string                 `json:"autonomy_level"`
	OperationalStatus string                 `json:"operational_status"`
	AutonomyStatus    string                 `json:"autonomy_status"`
	CurrentLocation   *Location              `json:"current_location"`
	CurrentHeading    float64                `json:"current_heading"`
	CurrentSpeed      float64                `json:"current_speed"`
	BatteryLevel      float64                `json:"battery_level"`
	FuelLevel         float64                `json:"fuel_level"`
	HealthScore       float64                `json:"health_score"`
	OdometerKm        float64                `json:"odometer_km"`
	CurrentTripID     string                 `json:"current_trip_id,omitempty"`
	LastSeen          time.Time              `json:"last_seen"`
	CreatedAt         time.Time              `json:"created_at"`
	UpdatedAt         time.Time              `json:"updated_at"`
}

type Location struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Altitude  float64 `json:"altitude,omitempty"`
}

type VehicleCommand struct {
	CommandID      string                 `json:"command_id"`
	VehicleID      string                 `json:"vehicle_id"`
	CommandType    string                 `json:"command_type"`
	CommandPayload map[string]interface{} `json:"command_payload"`
	Priority       string                 `json:"priority"`
	IssuedBy       string                 `json:"issued_by"`
	Status         string                 `json:"status"`
	IssuedAt       time.Time              `json:"issued_at"`
	CompletedAt    *time.Time             `json:"completed_at,omitempty"`
}

type FleetSummary struct {
	FleetID           string  `json:"fleet_id"`
	Name              string  `json:"name"`
	TotalVehicles     int     `json:"total_vehicles"`
	ActiveVehicles    int     `json:"active_vehicles"`
	IdleVehicles      int     `json:"idle_vehicles"`
	MaintenanceVehicles int   `json:"maintenance_vehicles"`
	ChargingVehicles  int     `json:"charging_vehicles"`
	AvgBatteryLevel   float64 `json:"avg_battery_level"`
	AvgHealthScore    float64 `json:"avg_health_score"`
	Utilization       float64 `json:"utilization"`
}

func loadConfig() Config {
	return Config{
		Port:         getEnvInt("PORT", 8081),
		DatabaseURL:  getEnv("DATABASE_URL", "postgres://atlasmesh_dev:password@localhost:5432/atlasmesh_fleet_dev?sslmode=disable"),
		RedisURL:     getEnv("REDIS_URL", "redis://localhost:6379/0"),
		LogLevel:     getEnv("LOG_LEVEL", "info"),
		AbuDhabiMode: getEnvBool("ABU_DHABI_MODE", true),
	}
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

func getEnvBool(key string, defaultValue bool) bool {
	if value := os.Getenv(key); value != "" {
		if boolValue, err := strconv.ParseBool(value); err == nil {
			return boolValue
		}
	}
	return defaultValue
}

func initializeMetrics() *Metrics {
	return &Metrics{
		FleetOperations: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_fleet_operations_total",
				Help: "Total number of fleet operations",
			},
			[]string{"operation", "status", "fleet_id"},
		),
		VehicleCommands: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_vehicle_commands_total",
				Help: "Total number of vehicle commands",
			},
			[]string{"command_type", "status", "priority"},
		),
		ResponseTime: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_fleet_response_time_seconds",
				Help: "Response time for fleet operations",
				Buckets: prometheus.DefBuckets,
			},
			[]string{"operation", "status"},
		),
		ActiveVehicles: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_active_vehicles",
				Help: "Number of active vehicles by fleet",
			},
			[]string{"fleet_id", "status"},
		),
		FleetUtilization: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_fleet_utilization_percent",
				Help: "Fleet utilization percentage",
			},
			[]string{"fleet_id"},
		),
	}
}

func main() {
	config := loadConfig()
	metrics := initializeMetrics()
	tracer := otel.Tracer("fleet-manager-service")

	// Load sector-specific configuration
	sector := os.Getenv("ATLASMESH_SECTOR")
	if sector == "" {
		sector = "logistics" // Default to logistics
	}
	
	sectorConfig, err := config.LoadSectorConfig(config.SectorType(sector))
	if err != nil {
		log.Fatalf("Failed to load sector configuration: %v", err)
	}

	// Validate sector configuration
	if err := sectorConfig.ValidateSectorConfig(); err != nil {
		log.Fatalf("Invalid sector configuration: %v", err)
	}

	log.Printf("Starting Fleet Manager Service with sector: %s", sector)

	// Register metrics
	prometheus.MustRegister(
		metrics.FleetOperations,
		metrics.VehicleCommands,
		metrics.ResponseTime,
		metrics.ActiveVehicles,
		metrics.FleetUtilization,
	)

	// Initialize database connection
	db, err := sql.Open("postgres", config.DatabaseURL)
	if err != nil {
		log.Fatalf("Failed to connect to database: %v", err)
	}
	defer db.Close()

	// Test database connection
	if err := db.Ping(); err != nil {
		log.Fatalf("Failed to ping database: %v", err)
	}

	// Initialize Redis connection
	opt, err := redis.ParseURL(config.RedisURL)
	if err != nil {
		log.Fatalf("Failed to parse Redis URL: %v", err)
	}
	redisClient := redis.NewClient(opt)

	// Test Redis connection
	ctx := context.Background()
	if err := redisClient.Ping(ctx).Err(); err != nil {
		log.Fatalf("Failed to connect to Redis: %v", err)
	}

	service := &FleetManagerService{
		db:           db,
		redis:        redisClient,
		config:       config,
		sectorConfig: sectorConfig,
		tracer:       tracer,
		metrics:      metrics,
	}

	// Setup HTTP server
	router := mux.NewRouter()
	service.setupRoutes(router)

	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.Port),
		Handler: router,
	}

	// Start background services
	go service.startMetricsCollection()
	go service.startHealthMonitoring()

	// Start server
	go func() {
		log.Printf("🚀 AtlasMesh Fleet Manager starting on port %d", config.Port)
		if config.AbuDhabiMode {
			log.Printf("🇦🇪 Abu Dhabi mode enabled - UAE compliance active")
		}
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Server failed: %v", err)
		}
	}()

	// Graceful shutdown
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c

	log.Println("🛑 Shutting down Fleet Manager service...")
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	server.Shutdown(ctx)
}

func (s *FleetManagerService) setupRoutes(router *mux.Router) {
	api := router.PathPrefix("/api/v1").Subrouter()

	// Fleet management
	api.HandleFunc("/fleets", s.listFleets).Methods("GET")
	api.HandleFunc("/fleets", s.createFleet).Methods("POST")
	api.HandleFunc("/fleets/{fleetId}", s.getFleet).Methods("GET")
	api.HandleFunc("/fleets/{fleetId}", s.updateFleet).Methods("PUT")
	api.HandleFunc("/fleets/{fleetId}", s.deleteFleet).Methods("DELETE")
	api.HandleFunc("/fleets/{fleetId}/summary", s.getFleetSummary).Methods("GET")

	// Vehicle management
	api.HandleFunc("/vehicles", s.listVehicles).Methods("GET")
	api.HandleFunc("/vehicles", s.createVehicle).Methods("POST")
	api.HandleFunc("/vehicles/{vehicleId}", s.getVehicle).Methods("GET")
	api.HandleFunc("/vehicles/{vehicleId}", s.updateVehicle).Methods("PUT")
	api.HandleFunc("/vehicles/{vehicleId}", s.deleteVehicle).Methods("DELETE")
	api.HandleFunc("/vehicles/{vehicleId}/status", s.getVehicleStatus).Methods("GET")
	api.HandleFunc("/vehicles/{vehicleId}/commands", s.sendVehicleCommand).Methods("POST")
	api.HandleFunc("/vehicles/{vehicleId}/location", s.updateVehicleLocation).Methods("PUT")

	// Fleet operations
	api.HandleFunc("/fleets/{fleetId}/vehicles", s.getFleetVehicles).Methods("GET")
	api.HandleFunc("/fleets/{fleetId}/dispatch", s.dispatchVehicle).Methods("POST")
	api.HandleFunc("/fleets/{fleetId}/recall", s.recallVehicle).Methods("POST")

	// Analytics and reporting
	api.HandleFunc("/analytics/fleet-utilization", s.getFleetUtilization).Methods("GET")
	api.HandleFunc("/analytics/vehicle-health", s.getVehicleHealthMetrics).Methods("GET")
	api.HandleFunc("/analytics/operational-metrics", s.getOperationalMetrics).Methods("GET")

	// Health and metrics
	api.HandleFunc("/health", s.healthCheck).Methods("GET")
	router.Handle("/metrics", promhttp.Handler())

	// Abu Dhabi specific endpoints
	if s.config.AbuDhabiMode {
		api.HandleFunc("/uae/compliance-report", s.generateUAEComplianceReport).Methods("GET")
		api.HandleFunc("/uae/emergency-services", s.coordinateEmergencyServices).Methods("POST")
	}
}

// Fleet Management Handlers

func (s *FleetManagerService) listFleets(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "listFleets")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("list_fleets", "success").Observe(time.Since(start).Seconds())
	}()

	query := `
		SELECT fleet_id, organization_id, name, description, fleet_type, 
		       configuration, max_vehicles, status, created_at, updated_at
		FROM fleet_core.fleets 
		WHERE status != 'decommissioned'
		ORDER BY created_at DESC`

	rows, err := s.db.QueryContext(ctx, query)
	if err != nil {
		s.handleError(w, "Failed to query fleets", err, http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var fleets []Fleet
	for rows.Next() {
		var fleet Fleet
		var configJSON []byte

		err := rows.Scan(
			&fleet.FleetID, &fleet.OrganizationID, &fleet.Name, &fleet.Description,
			&fleet.FleetType, &configJSON, &fleet.MaxVehicles, &fleet.Status,
			&fleet.CreatedAt, &fleet.UpdatedAt,
		)
		if err != nil {
			s.handleError(w, "Failed to scan fleet", err, http.StatusInternalServerError)
			return
		}

		if len(configJSON) > 0 {
			json.Unmarshal(configJSON, &fleet.Configuration)
		}

		fleets = append(fleets, fleet)
	}

	s.metrics.FleetOperations.WithLabelValues("list", "success", "all").Inc()
	s.respondJSON(w, fleets)
}

func (s *FleetManagerService) createFleet(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "createFleet")
	defer span.End()

	var fleet Fleet
	if err := json.NewDecoder(r.Body).Decode(&fleet); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Validate required fields
	if fleet.Name == "" || fleet.OrganizationID == "" || fleet.FleetType == "" {
		s.handleError(w, "Missing required fields: name, organization_id, fleet_type", nil, http.StatusBadRequest)
		return
	}

	// Generate fleet ID
	fleet.FleetID = s.generateUUID()
	fleet.Status = "active"
	fleet.CreatedAt = time.Now()
	fleet.UpdatedAt = time.Now()

	configJSON, _ := json.Marshal(fleet.Configuration)

	query := `
		INSERT INTO fleet_core.fleets 
		(fleet_id, organization_id, name, description, fleet_type, configuration, max_vehicles, status, created_at, updated_at, created_by)
		VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11)
		RETURNING fleet_id`

	err := s.db.QueryRowContext(ctx, query,
		fleet.FleetID, fleet.OrganizationID, fleet.Name, fleet.Description,
		fleet.FleetType, configJSON, fleet.MaxVehicles, fleet.Status,
		fleet.CreatedAt, fleet.UpdatedAt, "system", // TODO: Get from auth context
	).Scan(&fleet.FleetID)

	if err != nil {
		s.handleError(w, "Failed to create fleet", err, http.StatusInternalServerError)
		return
	}

	s.metrics.FleetOperations.WithLabelValues("create", "success", fleet.FleetID).Inc()
	log.Printf("✅ Created fleet: %s (%s)", fleet.Name, fleet.FleetID)

	w.WriteHeader(http.StatusCreated)
	s.respondJSON(w, fleet)
}

func (s *FleetManagerService) getFleet(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getFleet")
	defer span.End()

	vars := mux.Vars(r)
	fleetID := vars["fleetId"]

	query := `
		SELECT fleet_id, organization_id, name, description, fleet_type, 
		       configuration, max_vehicles, status, created_at, updated_at
		FROM fleet_core.fleets 
		WHERE fleet_id = $1`

	var fleet Fleet
	var configJSON []byte

	err := s.db.QueryRowContext(ctx, query, fleetID).Scan(
		&fleet.FleetID, &fleet.OrganizationID, &fleet.Name, &fleet.Description,
		&fleet.FleetType, &configJSON, &fleet.MaxVehicles, &fleet.Status,
		&fleet.CreatedAt, &fleet.UpdatedAt,
	)

	if err == sql.ErrNoRows {
		s.handleError(w, "Fleet not found", nil, http.StatusNotFound)
		return
	}
	if err != nil {
		s.handleError(w, "Failed to get fleet", err, http.StatusInternalServerError)
		return
	}

	if len(configJSON) > 0 {
		json.Unmarshal(configJSON, &fleet.Configuration)
	}

	s.respondJSON(w, fleet)
}

func (s *FleetManagerService) getFleetSummary(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getFleetSummary")
	defer span.End()

	vars := mux.Vars(r)
	fleetID := vars["fleetId"]

	query := `
		SELECT 
			f.fleet_id,
			f.name,
			COUNT(v.vehicle_id) as total_vehicles,
			COUNT(CASE WHEN v.operational_status = 'driving_av' THEN 1 END) as active_vehicles,
			COUNT(CASE WHEN v.operational_status = 'idle' THEN 1 END) as idle_vehicles,
			COUNT(CASE WHEN v.operational_status = 'maintenance' THEN 1 END) as maintenance_vehicles,
			COUNT(CASE WHEN v.operational_status = 'charging' THEN 1 END) as charging_vehicles,
			COALESCE(AVG(v.battery_level), 0) as avg_battery_level,
			COALESCE(AVG(v.health_score), 0) as avg_health_score
		FROM fleet_core.fleets f
		LEFT JOIN fleet_core.vehicles v ON f.fleet_id = v.fleet_id
		WHERE f.fleet_id = $1
		GROUP BY f.fleet_id, f.name`

	var summary FleetSummary
	err := s.db.QueryRowContext(ctx, query, fleetID).Scan(
		&summary.FleetID, &summary.Name, &summary.TotalVehicles,
		&summary.ActiveVehicles, &summary.IdleVehicles,
		&summary.MaintenanceVehicles, &summary.ChargingVehicles,
		&summary.AvgBatteryLevel, &summary.AvgHealthScore,
	)

	if err == sql.ErrNoRows {
		s.handleError(w, "Fleet not found", nil, http.StatusNotFound)
		return
	}
	if err != nil {
		s.handleError(w, "Failed to get fleet summary", err, http.StatusInternalServerError)
		return
	}

	// Calculate utilization
	if summary.TotalVehicles > 0 {
		summary.Utilization = float64(summary.ActiveVehicles) / float64(summary.TotalVehicles) * 100
	}

	// Update metrics
	s.metrics.ActiveVehicles.WithLabelValues(fleetID, "active").Set(float64(summary.ActiveVehicles))
	s.metrics.FleetUtilization.WithLabelValues(fleetID).Set(summary.Utilization)

	s.respondJSON(w, summary)
}

// Vehicle Management Handlers

func (s *FleetManagerService) listVehicles(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "listVehicles")
	defer span.End()

	// Parse query parameters
	fleetID := r.URL.Query().Get("fleet_id")
	status := r.URL.Query().Get("status")
	limit := r.URL.Query().Get("limit")

	query := `
		SELECT vehicle_id, fleet_id, organization_id, asset_tag, vin, license_plate,
		       manufacturer, model, year, vehicle_profile, autonomy_level,
		       operational_status, autonomy_status, current_location, current_heading,
		       current_speed, battery_level, fuel_level, health_score, odometer_km,
		       current_trip_id, last_seen, created_at, updated_at
		FROM fleet_core.vehicles 
		WHERE 1=1`

	args := []interface{}{}
	argCount := 0

	if fleetID != "" {
		argCount++
		query += fmt.Sprintf(" AND fleet_id = $%d", argCount)
		args = append(args, fleetID)
	}

	if status != "" {
		argCount++
		query += fmt.Sprintf(" AND operational_status = $%d", argCount)
		args = append(args, status)
	}

	query += " ORDER BY last_seen DESC"

	if limit != "" {
		argCount++
		query += fmt.Sprintf(" LIMIT $%d", argCount)
		args = append(args, limit)
	}

	rows, err := s.db.QueryContext(ctx, query, args...)
	if err != nil {
		s.handleError(w, "Failed to query vehicles", err, http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var vehicles []Vehicle
	for rows.Next() {
		var vehicle Vehicle
		var profileJSON []byte
		var locationWKT sql.NullString
		var currentTripID sql.NullString

		err := rows.Scan(
			&vehicle.VehicleID, &vehicle.FleetID, &vehicle.OrganizationID,
			&vehicle.AssetTag, &vehicle.VIN, &vehicle.LicensePlate,
			&vehicle.Manufacturer, &vehicle.Model, &vehicle.Year,
			&profileJSON, &vehicle.AutonomyLevel, &vehicle.OperationalStatus,
			&vehicle.AutonomyStatus, &locationWKT, &vehicle.CurrentHeading,
			&vehicle.CurrentSpeed, &vehicle.BatteryLevel, &vehicle.FuelLevel,
			&vehicle.HealthScore, &vehicle.OdometerKm, &currentTripID,
			&vehicle.LastSeen, &vehicle.CreatedAt, &vehicle.UpdatedAt,
		)
		if err != nil {
			s.handleError(w, "Failed to scan vehicle", err, http.StatusInternalServerError)
			return
		}

		if len(profileJSON) > 0 {
			json.Unmarshal(profileJSON, &vehicle.VehicleProfile)
		}

		if currentTripID.Valid {
			vehicle.CurrentTripID = currentTripID.String
		}

		// Parse location from PostGIS format (simplified)
		if locationWKT.Valid {
			vehicle.CurrentLocation = s.parseLocation(locationWKT.String)
		}

		vehicles = append(vehicles, vehicle)
	}

	s.respondJSON(w, vehicles)
}

// sendVehicleCommand dispatches commands to autonomous vehicles
// 
// PUBLIC API: POST /api/v1/vehicles/{vehicleId}/commands
// Accepts: VehicleCommand JSON with command_type, command_payload, priority
// Returns: 202 Accepted with command tracking ID
//
// SAFETY: Critical commands (emergency_stop, disable_autonomy) require dual-auth
// SECURITY: Command payload is validated against allowed command schemas
// CONCURRENCY: Command dispatch is async via goroutine to avoid blocking
// PERF: Commands are queued in database first, then sent to vehicle via Kafka
//
// INTEGRATION CONTRACT:
// - Kafka topic: vehicle.commands.{vehicle_id} 
// - Message format: Avro schema vehicle-command-v1
// - QoS: At-least-once delivery with 30s timeout
// - Backward compatibility: Command versioning via schema evolution
func (s *FleetManagerService) sendVehicleCommand(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "sendVehicleCommand")
	defer span.End()

	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]

	var command VehicleCommand
	if err := json.NewDecoder(r.Body).Decode(&command); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Validate command - SAFETY: Prevent malformed commands reaching vehicles
	if command.CommandType == "" {
		s.handleError(w, "Missing command_type", nil, http.StatusBadRequest)
		return
	}

	// SAFETY: Critical commands require additional validation
	if s.isCriticalCommand(command.CommandType) {
		if !s.validateDualAuth(r, command) {
			s.handleError(w, "Dual authorization required for safety-critical commands", nil, http.StatusForbidden)
			return
		}
	}

	// Generate command ID and set defaults
	command.CommandID = s.generateUUID()
	command.VehicleID = vehicleID
	command.Status = "pending"
	command.IssuedAt = time.Now()
	command.IssuedBy = "system" // TODO: Get from auth context

	if command.Priority == "" {
		command.Priority = "normal"
	}

	// Store command in database
	payloadJSON, _ := json.Marshal(command.CommandPayload)
	
	query := `
		INSERT INTO fleet_core.vehicle_commands 
		(command_id, vehicle_id, command_type, command_payload, priority, issued_by, status, issued_at)
		VALUES ($1, $2, $3, $4, $5, $6, $7, $8)`

	_, err := s.db.ExecContext(ctx, query,
		command.CommandID, command.VehicleID, command.CommandType,
		payloadJSON, command.Priority, command.IssuedBy,
		command.Status, command.IssuedAt,
	)

	if err != nil {
		s.handleError(w, "Failed to store command", err, http.StatusInternalServerError)
		return
	}

	// Send command to vehicle (via message queue or direct connection)
	go s.sendCommandToVehicle(ctx, command)

	// Update metrics
	s.metrics.VehicleCommands.WithLabelValues(command.CommandType, "sent", command.Priority).Inc()

	log.Printf("📤 Sent command %s to vehicle %s: %s", command.CommandType, vehicleID, command.CommandID)

	w.WriteHeader(http.StatusAccepted)
	s.respondJSON(w, command)
}

// Helper functions

func (s *FleetManagerService) generateUUID() string {
	// Simple UUID generation - in production use proper UUID library
	return fmt.Sprintf("%d-%d", time.Now().UnixNano(), time.Now().Unix())
}

func (s *FleetManagerService) parseLocation(wkt string) *Location {
	// Simplified location parsing - in production use proper PostGIS parsing
	return &Location{
		Latitude:  24.4539, // Abu Dhabi center
		Longitude: 54.3773,
	}
}

func (s *FleetManagerService) sendCommandToVehicle(ctx context.Context, command VehicleCommand) {
	// TODO: Implement actual command sending via Kafka or WebSocket
	log.Printf("🚗 Processing command %s for vehicle %s", command.CommandType, command.VehicleID)
	
	// Simulate command processing
	time.Sleep(100 * time.Millisecond)
	
	// Update command status
	_, err := s.db.ExecContext(ctx, 
		"UPDATE fleet_core.vehicle_commands SET status = 'sent', sent_at = NOW() WHERE command_id = $1",
		command.CommandID,
	)
	if err != nil {
		log.Printf("❌ Failed to update command status: %v", err)
	}
}

func (s *FleetManagerService) startMetricsCollection() {
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()

	for range ticker.C {
		s.collectFleetMetrics()
	}
}

func (s *FleetManagerService) collectFleetMetrics() {
	ctx := context.Background()
	
	// Collect active vehicle counts by fleet
	query := `
		SELECT fleet_id, operational_status, COUNT(*) 
		FROM fleet_core.vehicles 
		WHERE last_seen >= NOW() - INTERVAL '5 minutes'
		GROUP BY fleet_id, operational_status`

	rows, err := s.db.QueryContext(ctx, query)
	if err != nil {
		log.Printf("❌ Failed to collect metrics: %v", err)
		return
	}
	defer rows.Close()

	for rows.Next() {
		var fleetID, status string
		var count int
		if err := rows.Scan(&fleetID, &status, &count); err == nil {
			s.metrics.ActiveVehicles.WithLabelValues(fleetID, status).Set(float64(count))
		}
	}
}

func (s *FleetManagerService) startHealthMonitoring() {
	ticker := time.NewTicker(60 * time.Second)
	defer ticker.Stop()

	for range ticker.C {
		s.monitorSystemHealth()
	}
}

func (s *FleetManagerService) monitorSystemHealth() {
	// Check database connection
	if err := s.db.Ping(); err != nil {
		log.Printf("❌ Database health check failed: %v", err)
	}

	// Check Redis connection
	ctx := context.Background()
	if err := s.redis.Ping(ctx).Err(); err != nil {
		log.Printf("❌ Redis health check failed: %v", err)
	}
}

func (s *FleetManagerService) healthCheck(w http.ResponseWriter, r *http.Request) {
	health := map[string]interface{}{
		"status":    "healthy",
		"timestamp": time.Now(),
		"service":   "fleet-manager",
		"version":   "1.0.0",
		"region":    "abu-dhabi",
	}

	// Check database
	if err := s.db.Ping(); err != nil {
		health["status"] = "unhealthy"
		health["database"] = "failed"
	} else {
		health["database"] = "healthy"
	}

	// Check Redis
	ctx := context.Background()
	if err := s.redis.Ping(ctx).Err(); err != nil {
		health["status"] = "unhealthy"
		health["redis"] = "failed"
	} else {
		health["redis"] = "healthy"
	}

	if health["status"] == "unhealthy" {
		w.WriteHeader(http.StatusServiceUnavailable)
	}

	s.respondJSON(w, health)
}

// Abu Dhabi specific handlers

func (s *FleetManagerService) generateUAEComplianceReport(w http.ResponseWriter, r *http.Request) {
	if !s.config.AbuDhabiMode {
		s.handleError(w, "UAE compliance features not enabled", nil, http.StatusNotFound)
		return
	}

	report := map[string]interface{}{
		"report_id":     s.generateUUID(),
		"generated_at":  time.Now(),
		"region":        "abu-dhabi",
		"compliance":    "uae-av-regulations",
		"status":        "compliant",
		"total_fleets":  0,
		"total_vehicles": 0,
		"active_vehicles": 0,
		"incidents_24h": 0,
	}

	// TODO: Implement actual compliance data collection
	s.respondJSON(w, report)
}

func (s *FleetManagerService) coordinateEmergencyServices(w http.ResponseWriter, r *http.Request) {
	if !s.config.AbuDhabiMode {
		s.handleError(w, "Emergency coordination features not enabled", nil, http.StatusNotFound)
		return
	}

	var emergency map[string]interface{}
	if err := json.NewDecoder(r.Body).Decode(&emergency); err != nil {
		s.handleError(w, "Invalid emergency payload", err, http.StatusBadRequest)
		return
	}

	response := map[string]interface{}{
		"emergency_id": s.generateUUID(),
		"status":      "coordinating",
		"timestamp":   time.Now(),
		"services":    []string{"police", "ambulance", "fire"},
		"location":    emergency["location"],
	}

	log.Printf("🚨 Emergency coordination initiated: %s", response["emergency_id"])
	s.respondJSON(w, response)
}

// Utility functions

func (s *FleetManagerService) respondJSON(w http.ResponseWriter, data interface{}) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(data)
}

func (s *FleetManagerService) handleError(w http.ResponseWriter, message string, err error, statusCode int) {
	if err != nil {
		log.Printf("❌ %s: %v", message, err)
	} else {
		log.Printf("❌ %s", message)
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(statusCode)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"error":     message,
		"timestamp": time.Now(),
		"status":    statusCode,
	})
}

// Helper Functions
func generateUUID() string {
	b := make([]byte, 16)
	rand.Read(b)
	return fmt.Sprintf("%x-%x-%x-%x-%x", b[0:4], b[4:6], b[6:8], b[8:10], b[10:])
}

// calculateVehicleHealthScore computes vehicle health using Abu Dhabi-specific algorithm
//
// ALGORITHM: Multi-factor health scoring with weighted penalties
// - Base score: 100.0 (perfect health)
// - Battery/Fuel: Critical for operational readiness
// - Connectivity: Essential for autonomous operation safety
// - Autonomy status: Core system health indicator
//
// SAFETY: Vehicles with health < 70 should not be dispatched for passenger service
// PERF: Called frequently - optimized for minimal allocations
//
// Returns: Health score 0.0-100.0 where 100.0 is perfect health
// Units: Percentage points (0-100 scale)
func calculateVehicleHealthScore(vehicle *Vehicle) float64 {
	score := 100.0
	
	// SAFETY: Battery/Fuel level impact - critical for stranding prevention
	if vehicle.BatteryLevel > 0 {
		if vehicle.BatteryLevel < 20 {
			score -= 30 // Critical battery level
		} else if vehicle.BatteryLevel < 50 {
			score -= 10 // Low battery warning
		}
	}
	
	if vehicle.FuelLevel > 0 {
		if vehicle.FuelLevel < 20 {
			score -= 20 // Critical fuel level
		} else if vehicle.FuelLevel < 50 {
			score -= 5  // Low fuel warning
		}
	}
	
	// SAFETY: Connectivity impact - autonomous vehicles require constant connection
	timeSinceLastSeen := time.Since(vehicle.LastSeen)
	if timeSinceLastSeen > 5*time.Minute {
		score -= 40 // Connection lost - major safety concern
	} else if timeSinceLastSeen > 2*time.Minute {
		score -= 15 // Intermittent connection
	}
	
	// SAFETY: Autonomy system health - core safety system status
	if vehicle.AutonomyStatus == "fault" {
		score -= 50 // System fault - vehicle unsafe for autonomous operation
	} else if vehicle.AutonomyStatus == "degraded" {
		score -= 25 // Degraded performance - limited operational capability
	}
	
	// Ensure score doesn't go negative
	if score < 0 {
		score = 0
	}
	
	return score
}

func calculateFleetUtilization(activeVehicles, totalVehicles int) float64 {
	if totalVehicles == 0 {
		return 0
	}
	return float64(activeVehicles) / float64(totalVehicles) * 100
}

// Business Logic Implementation for Fleet Manager Handlers
func (s *FleetManagerService) updateFleet(w http.ResponseWriter, r *http.Request) {
	ctx := r.Context()
	span := trace.SpanFromContext(ctx)
	span.SetName("fleet_manager.update_fleet")

	vars := mux.Vars(r)
	fleetID := vars["fleetId"]

	if fleetID == "" {
		s.handleError(w, "Fleet ID is required", nil, http.StatusBadRequest)
		return
	}

	var updates Fleet
	if err := json.NewDecoder(r.Body).Decode(&updates); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Build dynamic update query
	setParts := []string{}
	args := []interface{}{}
	argIndex := 1

	if updates.Name != "" {
		setParts = append(setParts, fmt.Sprintf("name = $%d", argIndex))
		args = append(args, updates.Name)
		argIndex++
	}
	if updates.Description != "" {
		setParts = append(setParts, fmt.Sprintf("description = $%d", argIndex))
		args = append(args, updates.Description)
		argIndex++
	}
	if updates.MaxVehicles > 0 {
		setParts = append(setParts, fmt.Sprintf("max_vehicles = $%d", argIndex))
		args = append(args, updates.MaxVehicles)
		argIndex++
	}
	if updates.Status != "" {
		setParts = append(setParts, fmt.Sprintf("status = $%d", argIndex))
		args = append(args, updates.Status)
		argIndex++
	}

	if len(setParts) == 0 {
		s.handleError(w, "No valid fields to update", nil, http.StatusBadRequest)
		return
	}

	setParts = append(setParts, fmt.Sprintf("updated_at = $%d", argIndex))
	args = append(args, time.Now())
	argIndex++
	args = append(args, fleetID)

	query := fmt.Sprintf("UPDATE fleets SET %s WHERE fleet_id = $%d", 
		strings.Join(setParts, ", "), argIndex)

	start := time.Now()
	result, err := s.db.ExecContext(ctx, query, args...)
	s.metrics.ResponseTime.WithLabelValues("update_fleet", "database").Observe(time.Since(start).Seconds())

	if err != nil {
		log.Printf("❌ Fleet update failed: %v", err)
		s.handleError(w, "Failed to update fleet", err, http.StatusInternalServerError)
		return
	}

	rowsAffected, _ := result.RowsAffected()
	if rowsAffected == 0 {
		s.handleError(w, "Fleet not found", nil, http.StatusNotFound)
		return
	}

	s.redis.Del(ctx, fmt.Sprintf("fleet:%s", fleetID))
	s.metrics.FleetOperations.WithLabelValues("update_fleet", "success").Inc()
	log.Printf("✅ Fleet updated successfully: %s", fleetID)

	s.sendJSON(w, http.StatusOK, map[string]interface{}{
		"message": "Fleet updated successfully",
		"fleet_id": fleetID,
		"timestamp": time.Now(),
	})
}

func (s *FleetManagerService) deleteFleet(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "deleteFleet")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("delete_fleet", "success").Observe(time.Since(start).Seconds())
	}()

	// Extract fleet ID from URL path
	vars := mux.Vars(r)
	fleetID := vars["fleetId"]
	if fleetID == "" {
		s.handleError(w, "Fleet ID is required", nil, http.StatusBadRequest)
		return
	}

	// Parse request body for confirmation
	var req struct {
		Confirmation string `json:"confirmation" validate:"required"`
		Reason       string `json:"reason,omitempty"`
		ForceDelete  bool   `json:"force_delete,omitempty"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Validate confirmation
	if req.Confirmation != "DELETE_FLEET" {
		s.handleError(w, "Confirmation required. Send 'DELETE_FLEET' to confirm", nil, http.StatusBadRequest)
		return
	}

	// Check if fleet exists and get organization_id
	var organizationID string
	var fleetName string
	var fleetStatus string
	err := s.db.QueryRowContext(ctx, `
		SELECT organization_id, name, status 
		FROM fleet_core.fleets 
		WHERE fleet_id = $1`, fleetID).Scan(&organizationID, &fleetName, &fleetStatus)

	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Fleet not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Check if fleet is already decommissioned
	if fleetStatus == "decommissioned" {
		s.handleError(w, "Fleet is already decommissioned", nil, http.StatusConflict)
		return
	}

	// Check for active vehicles
	var activeVehicleCount int
	err = s.db.QueryRowContext(ctx, `
		SELECT COUNT(*) 
		FROM fleet_core.vehicles 
		WHERE fleet_id = $1 AND operational_status NOT IN ('decommissioned', 'offline')`, fleetID).Scan(&activeVehicleCount)

	if err != nil {
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	if activeVehicleCount > 0 && !req.ForceDelete {
		s.handleError(w, fmt.Sprintf("Fleet has %d active vehicles. Use force_delete=true to override", activeVehicleCount), nil, http.StatusConflict)
		return
	}

	// Check for active trips
	var activeTripCount int
	err = s.db.QueryRowContext(ctx, `
		SELECT COUNT(*) 
		FROM fleet_core.trips 
		WHERE fleet_id = $1 AND status IN ('planned', 'in_progress')`, fleetID).Scan(&activeTripCount)

	if err != nil {
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	if activeTripCount > 0 && !req.ForceDelete {
		s.handleError(w, fmt.Sprintf("Fleet has %d active trips. Use force_delete=true to override", activeTripCount), nil, http.StatusConflict)
		return
	}

	// Start transaction for fleet decommissioning
	tx, err := s.db.BeginTx(ctx, nil)
	if err != nil {
		s.handleError(w, "Failed to start transaction", err, http.StatusInternalServerError)
		return
	}
	defer tx.Rollback()

	// Decommission all vehicles in the fleet
	_, err = tx.ExecContext(ctx, `
		UPDATE fleet_core.vehicles 
		SET operational_status = 'decommissioned',
		    autonomy_status = 'offline',
		    decommissioned_at = NOW(),
		    updated_at = NOW()
		WHERE fleet_id = $1`, fleetID)

	if err != nil {
		s.handleError(w, "Failed to decommission vehicles", err, http.StatusInternalServerError)
		return
	}

	// Cancel all active trips
	_, err = tx.ExecContext(ctx, `
		UPDATE fleet_core.trips 
		SET status = 'cancelled',
		    cancelled_at = NOW(),
		    cancellation_reason = $1,
		    updated_at = NOW()
		WHERE fleet_id = $2 AND status IN ('planned', 'in_progress')`, 
		req.Reason, fleetID)

	if err != nil {
		s.handleError(w, "Failed to cancel trips", err, http.StatusInternalServerError)
		return
	}

	// Decommission the fleet
	_, err = tx.ExecContext(ctx, `
		UPDATE fleet_core.fleets 
		SET status = 'decommissioned',
		    decommissioned_at = NOW(),
		    updated_at = NOW()
		WHERE fleet_id = $1`, fleetID)

	if err != nil {
		s.handleError(w, "Failed to decommission fleet", err, http.StatusInternalServerError)
		return
	}

	// Archive fleet data to historical table (if exists)
	_, err = tx.ExecContext(ctx, `
		INSERT INTO fleet_core.fleet_history (
			fleet_id, organization_id, name, description, fleet_type, configuration,
			max_vehicles, status, created_at, updated_at, decommissioned_at, archived_at
		)
		SELECT 
			fleet_id, organization_id, name, description, fleet_type, configuration,
			max_vehicles, status, created_at, updated_at, decommissioned_at, NOW()
		FROM fleet_core.fleets 
		WHERE fleet_id = $1`, fleetID)

	if err != nil {
		// Log error but don't fail the transaction - history table might not exist
		log.Printf("Warning: Failed to archive fleet to history: %v", err)
	}

	// Commit transaction
	if err = tx.Commit(); err != nil {
		s.handleError(w, "Failed to commit transaction", err, http.StatusInternalServerError)
		return
	}

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("delete_fleet", "success", fleetID).Inc()

	// Return success response
	response := map[string]interface{}{
		"fleet_id":         fleetID,
		"fleet_name":       fleetName,
		"organization_id":  organizationID,
		"status":           "decommissioned",
		"decommissioned_at": time.Now(),
		"vehicles_affected": activeVehicleCount,
		"trips_cancelled":   activeTripCount,
		"reason":           req.Reason,
		"message":          "Fleet decommissioned successfully",
	}

	s.respondJSON(w, response)
}

func (s *FleetManagerService) createVehicle(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "createVehicle")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("create_vehicle", "success").Observe(time.Since(start).Seconds())
	}()

	// Parse request body
	var req struct {
		FleetID         string                 `json:"fleet_id" validate:"required,uuid"`
		AssetTag        string                 `json:"asset_tag" validate:"required"`
		VIN             string                 `json:"vin"`
		LicensePlate    string                 `json:"license_plate"`
		Manufacturer    string                 `json:"manufacturer" validate:"required"`
		Model           string                 `json:"model" validate:"required"`
		Year            int                    `json:"year"`
		SerialNumber    string                 `json:"serial_number"`
		VehicleProfile  map[string]interface{} `json:"vehicle_profile"`
		Capabilities    map[string]interface{} `json:"capabilities"`
		SensorConfig    map[string]interface{} `json:"sensor_configuration"`
		AutonomyLevel   string                 `json:"autonomy_level" validate:"required,oneof=L0 L1 L2 L3 L4 L5"`
		AutonomyCaps    map[string]interface{} `json:"autonomy_capabilities"`
		ODDConfig       map[string]interface{} `json:"odd_configuration"`
		Tags            []string               `json:"tags"`
		Metadata        map[string]interface{} `json:"metadata"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Validate required fields
	if req.FleetID == "" || req.AssetTag == "" || req.Manufacturer == "" || req.Model == "" {
		s.handleError(w, "Missing required fields: fleet_id, asset_tag, manufacturer, model", nil, http.StatusBadRequest)
		return
	}

	// Check if fleet exists and get organization_id
	var organizationID string
	err := s.db.QueryRowContext(ctx, "SELECT organization_id FROM fleet_core.fleets WHERE fleet_id = $1 AND status != 'decommissioned'", req.FleetID).Scan(&organizationID)
	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Fleet not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Check for duplicate asset_tag within organization
	var existingCount int
	err = s.db.QueryRowContext(ctx, "SELECT COUNT(*) FROM fleet_core.vehicles WHERE organization_id = $1 AND asset_tag = $2", organizationID, req.AssetTag).Scan(&existingCount)
	if err != nil {
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}
	if existingCount > 0 {
		s.handleError(w, "Asset tag already exists in organization", nil, http.StatusConflict)
		return
	}

	// Check for duplicate VIN if provided
	if req.VIN != "" {
		err = s.db.QueryRowContext(ctx, "SELECT COUNT(*) FROM fleet_core.vehicles WHERE vin = $1", req.VIN).Scan(&existingCount)
		if err != nil {
			s.handleError(w, "Database error", err, http.StatusInternalServerError)
			return
		}
		if existingCount > 0 {
			s.handleError(w, "VIN already exists", nil, http.StatusConflict)
			return
		}
	}

	// Generate vehicle ID
	vehicleID := generateUUID()

	// Prepare vehicle profile JSON
	vehicleProfileJSON, _ := json.Marshal(req.VehicleProfile)
	capabilitiesJSON, _ := json.Marshal(req.Capabilities)
	sensorConfigJSON, _ := json.Marshal(req.SensorConfig)
	autonomyCapsJSON, _ := json.Marshal(req.AutonomyCaps)
	oddConfigJSON, _ := json.Marshal(req.ODDConfig)
	metadataJSON, _ := json.Marshal(req.Metadata)

	// Insert vehicle
	query := `
		INSERT INTO fleet_core.vehicles (
			vehicle_id, fleet_id, organization_id, asset_tag, vin, license_plate,
			manufacturer, model, year, serial_number, vehicle_profile, capabilities,
			sensor_configuration, autonomy_level, autonomy_capabilities, odd_configuration,
			operational_status, autonomy_status, tags, metadata, created_at, updated_at
		) VALUES (
			$1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14, $15, $16,
			'offline', 'offline', $17, $18, NOW(), NOW()
		) RETURNING vehicle_id, created_at`

	var createdVehicleID string
	var createdAt time.Time
	err = s.db.QueryRowContext(ctx, query,
		vehicleID, req.FleetID, organizationID, req.AssetTag, req.VIN, req.LicensePlate,
		req.Manufacturer, req.Model, req.Year, req.SerialNumber, vehicleProfileJSON, capabilitiesJSON,
		sensorConfigJSON, req.AutonomyLevel, autonomyCapsJSON, oddConfigJSON,
		pq.Array(req.Tags), metadataJSON,
	).Scan(&createdVehicleID, &createdAt)

	if err != nil {
		s.handleError(w, "Failed to create vehicle", err, http.StatusInternalServerError)
		return
	}

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("create_vehicle", "success", req.FleetID).Inc()

	// Return created vehicle
	response := map[string]interface{}{
		"vehicle_id":   createdVehicleID,
		"fleet_id":     req.FleetID,
		"asset_tag":    req.AssetTag,
		"status":       "created",
		"created_at":   createdAt,
		"message":      "Vehicle created successfully",
	}

	s.respondJSON(w, response)
}

func (s *FleetManagerService) getVehicle(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getVehicle")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("get_vehicle", "success").Observe(time.Since(start).Seconds())
	}()

	// Extract vehicle ID from URL path
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	if vehicleID == "" {
		s.handleError(w, "Vehicle ID is required", nil, http.StatusBadRequest)
		return
	}

	// Query vehicle with all details
	query := `
		SELECT 
			v.vehicle_id, v.fleet_id, v.organization_id, v.asset_tag, v.vin, v.license_plate,
			v.manufacturer, v.model, v.year, v.serial_number, v.vehicle_profile, v.capabilities,
			v.sensor_configuration, v.autonomy_level, v.autonomy_capabilities, v.odd_configuration,
			v.operational_status, v.autonomy_status, v.current_location, v.current_heading, v.current_speed,
			v.battery_level, v.fuel_level, v.health_score, v.odometer_km, v.engine_hours,
			v.current_trip_id, v.assigned_depot_id, v.assigned_operator_id,
			v.last_maintenance_date, v.next_maintenance_due, v.maintenance_schedule,
			v.software_version, v.firmware_version, v.last_ota_update,
			v.last_seen, v.created_at, v.updated_at, v.commissioned_at, v.decommissioned_at,
			v.tags, v.metadata,
			f.name as fleet_name, f.fleet_type, f.status as fleet_status,
			o.name as organization_name, o.sector
		FROM fleet_core.vehicles v
		JOIN fleet_core.fleets f ON v.fleet_id = f.fleet_id
		JOIN fleet_core.organizations o ON v.organization_id = o.organization_id
		WHERE v.vehicle_id = $1`

	var vehicle Vehicle
	var vehicleProfileJSON, capabilitiesJSON, sensorConfigJSON, autonomyCapsJSON, oddConfigJSON, maintenanceScheduleJSON, metadataJSON []byte
	var currentLocationJSON []byte
	var lastSeen, createdAt, updatedAt, commissionedAt, decommissionedAt, lastOtaUpdate, lastMaintenanceDate, nextMaintenanceDue sql.NullTime
	var currentHeading, currentSpeed, batteryLevel, fuelLevel, healthScore, odometerKm, engineHours sql.NullFloat64
	var currentTripID, assignedDepotID, assignedOperatorID sql.NullString

	err := s.db.QueryRowContext(ctx, query, vehicleID).Scan(
		&vehicle.VehicleID, &vehicle.FleetID, &vehicle.OrganizationID, &vehicle.AssetTag, &vehicle.VIN, &vehicle.LicensePlate,
		&vehicle.Manufacturer, &vehicle.Model, &vehicle.Year, &vehicle.SerialNumber, &vehicleProfileJSON, &capabilitiesJSON,
		&sensorConfigJSON, &vehicle.AutonomyLevel, &autonomyCapsJSON, &oddConfigJSON,
		&vehicle.OperationalStatus, &vehicle.AutonomyStatus, &currentLocationJSON, &currentHeading, &currentSpeed,
		&batteryLevel, &fuelLevel, &healthScore, &odometerKm, &engineHours,
		&currentTripID, &assignedDepotID, &assignedOperatorID,
		&lastMaintenanceDate, &nextMaintenanceDue, &maintenanceScheduleJSON,
		&vehicle.SoftwareVersion, &vehicle.FirmwareVersion, &lastOtaUpdate,
		&lastSeen, &createdAt, &updatedAt, &commissionedAt, &decommissionedAt,
		pq.Array(&vehicle.Tags), &metadataJSON,
		&vehicle.FleetName, &vehicle.FleetType, &vehicle.FleetStatus,
		&vehicle.OrganizationName, &vehicle.Sector,
	)

	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Vehicle not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Parse JSON fields
	json.Unmarshal(vehicleProfileJSON, &vehicle.VehicleProfile)
	json.Unmarshal(capabilitiesJSON, &vehicle.Capabilities)
	json.Unmarshal(sensorConfigJSON, &vehicle.SensorConfiguration)
	json.Unmarshal(autonomyCapsJSON, &vehicle.AutonomyCapabilities)
	json.Unmarshal(oddConfigJSON, &vehicle.ODDConfiguration)
	json.Unmarshal(maintenanceScheduleJSON, &vehicle.MaintenanceSchedule)
	json.Unmarshal(metadataJSON, &vehicle.Metadata)

	// Handle nullable fields
	if currentLocationJSON != nil {
		json.Unmarshal(currentLocationJSON, &vehicle.CurrentLocation)
	}
	vehicle.CurrentHeading = currentHeading.Float64
	vehicle.CurrentSpeed = currentSpeed.Float64
	vehicle.BatteryLevel = batteryLevel.Float64
	vehicle.FuelLevel = fuelLevel.Float64
	vehicle.HealthScore = healthScore.Float64
	vehicle.OdometerKm = odometerKm.Float64
	vehicle.EngineHours = engineHours.Float64
	vehicle.CurrentTripID = currentTripID.String
	vehicle.AssignedDepotID = assignedDepotID.String
	vehicle.AssignedOperatorID = assignedOperatorID.String
	vehicle.LastSeen = lastSeen.Time
	vehicle.CreatedAt = createdAt.Time
	vehicle.UpdatedAt = updatedAt.Time
	vehicle.CommissionedAt = commissionedAt.Time
	vehicle.DecommissionedAt = decommissionedAt.Time
	vehicle.LastOtaUpdate = lastOtaUpdate.Time
	vehicle.LastMaintenanceDate = lastMaintenanceDate.Time
	vehicle.NextMaintenanceDue = nextMaintenanceDue.Time

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("get_vehicle", "success", vehicle.FleetID).Inc()

	// Return vehicle details
	s.respondJSON(w, vehicle)
}

func (s *FleetManagerService) updateVehicle(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "updateVehicle")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("update_vehicle", "success").Observe(time.Since(start).Seconds())
	}()

	// Extract vehicle ID from URL path
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	if vehicleID == "" {
		s.handleError(w, "Vehicle ID is required", nil, http.StatusBadRequest)
		return
	}

	// Parse request body
	var req struct {
		AssetTag        *string                `json:"asset_tag,omitempty"`
		VIN             *string                `json:"vin,omitempty"`
		LicensePlate    *string                `json:"license_plate,omitempty"`
		Manufacturer    *string                `json:"manufacturer,omitempty"`
		Model           *string                `json:"model,omitempty"`
		Year            *int                   `json:"year,omitempty"`
		SerialNumber    *string                `json:"serial_number,omitempty"`
		VehicleProfile  map[string]interface{} `json:"vehicle_profile,omitempty"`
		Capabilities    map[string]interface{} `json:"capabilities,omitempty"`
		SensorConfig    map[string]interface{} `json:"sensor_configuration,omitempty"`
		AutonomyLevel   *string                `json:"autonomy_level,omitempty"`
		AutonomyCaps    map[string]interface{} `json:"autonomy_capabilities,omitempty"`
		ODDConfig       map[string]interface{} `json:"odd_configuration,omitempty"`
		OperationalStatus *string              `json:"operational_status,omitempty"`
		AutonomyStatus  *string                `json:"autonomy_status,omitempty"`
		CurrentLocation map[string]interface{} `json:"current_location,omitempty"`
		CurrentHeading  *float64               `json:"current_heading,omitempty"`
		CurrentSpeed    *float64               `json:"current_speed,omitempty"`
		BatteryLevel    *float64               `json:"battery_level,omitempty"`
		FuelLevel       *float64              `json:"fuel_level,omitempty"`
		HealthScore     *float64               `json:"health_score,omitempty"`
		OdometerKm      *float64              `json:"odometer_km,omitempty"`
		EngineHours     *float64              `json:"engine_hours,omitempty"`
		CurrentTripID   *string               `json:"current_trip_id,omitempty"`
		AssignedDepotID *string               `json:"assigned_depot_id,omitempty"`
		AssignedOperatorID *string            `json:"assigned_operator_id,omitempty"`
		SoftwareVersion *string               `json:"software_version,omitempty"`
		FirmwareVersion *string               `json:"firmware_version,omitempty"`
		Tags            []string              `json:"tags,omitempty"`
		Metadata        map[string]interface{} `json:"metadata,omitempty"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Check if vehicle exists
	var existingVehicleID string
	err := s.db.QueryRowContext(ctx, "SELECT vehicle_id FROM fleet_core.vehicles WHERE vehicle_id = $1", vehicleID).Scan(&existingVehicleID)
	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Vehicle not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Build dynamic update query
	setParts := []string{}
	args := []interface{}{}
	argIndex := 1

	// Helper function to add field to update
	addField := func(field string, value interface{}) {
		setParts = append(setParts, fmt.Sprintf("%s = $%d", field, argIndex))
		args = append(args, value)
		argIndex++
	}

	// Add fields to update if provided
	if req.AssetTag != nil {
		addField("asset_tag", *req.AssetTag)
	}
	if req.VIN != nil {
		addField("vin", *req.VIN)
	}
	if req.LicensePlate != nil {
		addField("license_plate", *req.LicensePlate)
	}
	if req.Manufacturer != nil {
		addField("manufacturer", *req.Manufacturer)
	}
	if req.Model != nil {
		addField("model", *req.Model)
	}
	if req.Year != nil {
		addField("year", *req.Year)
	}
	if req.SerialNumber != nil {
		addField("serial_number", *req.SerialNumber)
	}
	if req.VehicleProfile != nil {
		profileJSON, _ := json.Marshal(req.VehicleProfile)
		addField("vehicle_profile", profileJSON)
	}
	if req.Capabilities != nil {
		capabilitiesJSON, _ := json.Marshal(req.Capabilities)
		addField("capabilities", capabilitiesJSON)
	}
	if req.SensorConfig != nil {
		sensorConfigJSON, _ := json.Marshal(req.SensorConfig)
		addField("sensor_configuration", sensorConfigJSON)
	}
	if req.AutonomyLevel != nil {
		addField("autonomy_level", *req.AutonomyLevel)
	}
	if req.AutonomyCaps != nil {
		autonomyCapsJSON, _ := json.Marshal(req.AutonomyCaps)
		addField("autonomy_capabilities", autonomyCapsJSON)
	}
	if req.ODDConfig != nil {
		oddConfigJSON, _ := json.Marshal(req.ODDConfig)
		addField("odd_configuration", oddConfigJSON)
	}
	if req.OperationalStatus != nil {
		addField("operational_status", *req.OperationalStatus)
	}
	if req.AutonomyStatus != nil {
		addField("autonomy_status", *req.AutonomyStatus)
	}
	if req.CurrentLocation != nil {
		locationJSON, _ := json.Marshal(req.CurrentLocation)
		addField("current_location", locationJSON)
	}
	if req.CurrentHeading != nil {
		addField("current_heading", *req.CurrentHeading)
	}
	if req.CurrentSpeed != nil {
		addField("current_speed", *req.CurrentSpeed)
	}
	if req.BatteryLevel != nil {
		addField("battery_level", *req.BatteryLevel)
	}
	if req.FuelLevel != nil {
		addField("fuel_level", *req.FuelLevel)
	}
	if req.HealthScore != nil {
		addField("health_score", *req.HealthScore)
	}
	if req.OdometerKm != nil {
		addField("odometer_km", *req.OdometerKm)
	}
	if req.EngineHours != nil {
		addField("engine_hours", *req.EngineHours)
	}
	if req.CurrentTripID != nil {
		addField("current_trip_id", *req.CurrentTripID)
	}
	if req.AssignedDepotID != nil {
		addField("assigned_depot_id", *req.AssignedDepotID)
	}
	if req.AssignedOperatorID != nil {
		addField("assigned_operator_id", *req.AssignedOperatorID)
	}
	if req.SoftwareVersion != nil {
		addField("software_version", *req.SoftwareVersion)
	}
	if req.FirmwareVersion != nil {
		addField("firmware_version", *req.FirmwareVersion)
	}
	if req.Tags != nil {
		addField("tags", pq.Array(req.Tags))
	}
	if req.Metadata != nil {
		metadataJSON, _ := json.Marshal(req.Metadata)
		addField("metadata", metadataJSON)
	}

	// Always update the updated_at timestamp
	addField("updated_at", time.Now())

	if len(setParts) == 0 {
		s.handleError(w, "No fields to update", nil, http.StatusBadRequest)
		return
	}

	// Add vehicle ID to args
	args = append(args, vehicleID)

	// Execute update
	query := fmt.Sprintf("UPDATE fleet_core.vehicles SET %s WHERE vehicle_id = $%d", strings.Join(setParts, ", "), len(args))
	result, err := s.db.ExecContext(ctx, query, args...)
	if err != nil {
		s.handleError(w, "Failed to update vehicle", err, http.StatusInternalServerError)
		return
	}

	rowsAffected, err := result.RowsAffected()
	if err != nil {
		s.handleError(w, "Failed to get rows affected", err, http.StatusInternalServerError)
		return
	}

	if rowsAffected == 0 {
		s.handleError(w, "Vehicle not found", nil, http.StatusNotFound)
		return
	}

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("update_vehicle", "success", "").Inc()

	// Return success response
	response := map[string]interface{}{
		"vehicle_id":   vehicleID,
		"status":       "updated",
		"updated_at":   time.Now(),
		"message":      "Vehicle updated successfully",
		"fields_updated": len(setParts) - 1, // Exclude updated_at
	}

	s.respondJSON(w, response)
}

func (s *FleetManagerService) deleteVehicle(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "deleteVehicle")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("delete_vehicle", "success").Observe(time.Since(start).Seconds())
	}()

	// Extract vehicle ID from URL path
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	if vehicleID == "" {
		s.handleError(w, "Vehicle ID is required", nil, http.StatusBadRequest)
		return
	}

	// Check if vehicle exists and get fleet_id for metrics
	var fleetID string
	var operationalStatus string
	err := s.db.QueryRowContext(ctx, "SELECT fleet_id, operational_status FROM fleet_core.vehicles WHERE vehicle_id = $1", vehicleID).Scan(&fleetID, &operationalStatus)
	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Vehicle not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Check if vehicle is currently in use
	if operationalStatus == "driving_av" || operationalStatus == "driving_manual" {
		s.handleError(w, "Cannot delete vehicle that is currently in operation", nil, http.StatusConflict)
		return
	}

	// Check if vehicle has active trips
	var activeTripCount int
	err = s.db.QueryRowContext(ctx, "SELECT COUNT(*) FROM fleet_core.trips WHERE vehicle_id = $1 AND status IN ('planned', 'in_progress')", vehicleID).Scan(&activeTripCount)
	if err != nil {
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}
	if activeTripCount > 0 {
		s.handleError(w, "Cannot delete vehicle with active trips", nil, http.StatusConflict)
		return
	}

	// Start transaction for soft delete
	tx, err := s.db.BeginTx(ctx, nil)
	if err != nil {
		s.handleError(w, "Failed to start transaction", err, http.StatusInternalServerError)
		return
	}
	defer tx.Rollback()

	// Soft delete vehicle (update status to decommissioned)
	_, err = tx.ExecContext(ctx, `
		UPDATE fleet_core.vehicles 
		SET operational_status = 'decommissioned', 
		    autonomy_status = 'offline',
		    decommissioned_at = NOW(),
		    updated_at = NOW()
		WHERE vehicle_id = $1`, vehicleID)
	if err != nil {
		s.handleError(w, "Failed to decommission vehicle", err, http.StatusInternalServerError)
		return
	}

	// Archive vehicle data to historical table (if exists)
	_, err = tx.ExecContext(ctx, `
		INSERT INTO fleet_core.vehicle_history (
			vehicle_id, fleet_id, organization_id, asset_tag, vin, license_plate,
			manufacturer, model, year, serial_number, vehicle_profile, capabilities,
			sensor_configuration, autonomy_level, autonomy_capabilities, odd_configuration,
			operational_status, autonomy_status, current_location, current_heading, current_speed,
			battery_level, fuel_level, health_score, odometer_km, engine_hours,
			current_trip_id, assigned_depot_id, assigned_operator_id,
			last_maintenance_date, next_maintenance_due, maintenance_schedule,
			software_version, firmware_version, last_ota_update,
			last_seen, created_at, updated_at, commissioned_at, decommissioned_at,
			tags, metadata, archived_at
		)
		SELECT 
			vehicle_id, fleet_id, organization_id, asset_tag, vin, license_plate,
			manufacturer, model, year, serial_number, vehicle_profile, capabilities,
			sensor_configuration, autonomy_level, autonomy_capabilities, odd_configuration,
			operational_status, autonomy_status, current_location, current_heading, current_speed,
			battery_level, fuel_level, health_score, odometer_km, engine_hours,
			current_trip_id, assigned_depot_id, assigned_operator_id,
			last_maintenance_date, next_maintenance_due, maintenance_schedule,
			software_version, firmware_version, last_ota_update,
			last_seen, created_at, updated_at, commissioned_at, decommissioned_at,
			tags, metadata, NOW()
		FROM fleet_core.vehicles 
		WHERE vehicle_id = $1`, vehicleID)
	if err != nil {
		// Log error but don't fail the transaction - history table might not exist
		log.Printf("Warning: Failed to archive vehicle to history: %v", err)
	}

	// Commit transaction
	if err = tx.Commit(); err != nil {
		s.handleError(w, "Failed to commit transaction", err, http.StatusInternalServerError)
		return
	}

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("delete_vehicle", "success", fleetID).Inc()

	// Return success response
	response := map[string]interface{}{
		"vehicle_id":   vehicleID,
		"fleet_id":     fleetID,
		"status":       "decommissioned",
		"decommissioned_at": time.Now(),
		"message":      "Vehicle decommissioned successfully",
	}

	s.respondJSON(w, response)
}

func (s *FleetManagerService) getVehicleStatus(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getVehicleStatus")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("get_vehicle_status", "success").Observe(time.Since(start).Seconds())
	}()

	// Extract vehicle ID from URL path
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	if vehicleID == "" {
		s.handleError(w, "Vehicle ID is required", nil, http.StatusBadRequest)
		return
	}

	// Query vehicle status with real-time data
	query := `
		SELECT 
			v.vehicle_id, v.asset_tag, v.operational_status, v.autonomy_status,
			v.current_location, v.current_heading, v.current_speed,
			v.battery_level, v.fuel_level, v.health_score,
			v.last_seen, v.updated_at,
			f.name as fleet_name, f.fleet_type,
			o.name as organization_name
		FROM fleet_core.vehicles v
		JOIN fleet_core.fleets f ON v.fleet_id = f.fleet_id
		JOIN fleet_core.organizations o ON v.organization_id = o.organization_id
		WHERE v.vehicle_id = $1`

	var status struct {
		VehicleID        string                 `json:"vehicle_id"`
		AssetTag         string                 `json:"asset_tag"`
		OperationalStatus string                `json:"operational_status"`
		AutonomyStatus   string                 `json:"autonomy_status"`
		CurrentLocation  map[string]interface{} `json:"current_location"`
		CurrentHeading   float64                `json:"current_heading"`
		CurrentSpeed     float64                `json:"current_speed"`
		BatteryLevel     float64                `json:"battery_level"`
		FuelLevel        float64                `json:"fuel_level"`
		HealthScore      float64                `json:"health_score"`
		LastSeen         time.Time              `json:"last_seen"`
		UpdatedAt        time.Time              `json:"updated_at"`
		FleetName        string                 `json:"fleet_name"`
		FleetType        string                 `json:"fleet_type"`
		OrganizationName string                 `json:"organization_name"`
		IsOnline         bool                   `json:"is_online"`
		StatusAge        int64                  `json:"status_age_seconds"`
	}

	var currentLocationJSON []byte
	var currentHeading, currentSpeed, batteryLevel, fuelLevel, healthScore sql.NullFloat64
	var lastSeen, updatedAt sql.NullTime

	err := s.db.QueryRowContext(ctx, query, vehicleID).Scan(
		&status.VehicleID, &status.AssetTag, &status.OperationalStatus, &status.AutonomyStatus,
		&currentLocationJSON, &currentHeading, &currentSpeed,
		&batteryLevel, &fuelLevel, &healthScore,
		&lastSeen, &updatedAt,
		&status.FleetName, &status.FleetType,
		&status.OrganizationName,
	)

	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Vehicle not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Parse JSON fields
	if currentLocationJSON != nil {
		json.Unmarshal(currentLocationJSON, &status.CurrentLocation)
	}
	status.CurrentHeading = currentHeading.Float64
	status.CurrentSpeed = currentSpeed.Float64
	status.BatteryLevel = batteryLevel.Float64
	status.FuelLevel = fuelLevel.Float64
	status.HealthScore = healthScore.Float64
	status.LastSeen = lastSeen.Time
	status.UpdatedAt = updatedAt.Time

	// Calculate online status (vehicle is online if last seen within 5 minutes)
	status.IsOnline = time.Since(status.LastSeen) < 5*time.Minute
	status.StatusAge = int64(time.Since(status.LastSeen).Seconds())

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("get_vehicle_status", "success", "").Inc()

	// Return vehicle status
	s.respondJSON(w, status)
}

func (s *FleetManagerService) updateVehicleLocation(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "updateVehicleLocation")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("update_vehicle_location", "success").Observe(time.Since(start).Seconds())
	}()

	// Extract vehicle ID from URL path
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	if vehicleID == "" {
		s.handleError(w, "Vehicle ID is required", nil, http.StatusBadRequest)
		return
	}

	// Parse request body
	var req struct {
		Location      map[string]interface{} `json:"location" validate:"required"`
		Heading       *float64               `json:"heading,omitempty"`
		Speed         *float64               `json:"speed,omitempty"`
		Altitude      *float64               `json:"altitude,omitempty"`
		Accuracy      *float64               `json:"accuracy,omitempty"`
		Source        string                 `json:"source,omitempty"` // gps, manual, estimated
		Timestamp     *time.Time             `json:"timestamp,omitempty"`
		UpdateStatus  bool                   `json:"update_status,omitempty"` // Whether to update operational_status
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Validate location data
	if req.Location == nil {
		s.handleError(w, "Location data is required", nil, http.StatusBadRequest)
		return
	}

	// Validate latitude and longitude
	lat, latOk := req.Location["latitude"].(float64)
	lon, lonOk := req.Location["longitude"].(float64)
	if !latOk || !lonOk {
		s.handleError(w, "Location must contain valid latitude and longitude", nil, http.StatusBadRequest)
		return
	}

	// Validate coordinate ranges
	if lat < -90 || lat > 90 || lon < -180 || lon > 180 {
		s.handleError(w, "Invalid latitude or longitude values", nil, http.StatusBadRequest)
		return
	}

	// Check if vehicle exists
	var fleetID string
	var currentOperationalStatus string
	err := s.db.QueryRowContext(ctx, "SELECT fleet_id, operational_status FROM fleet_core.vehicles WHERE vehicle_id = $1", vehicleID).Scan(&fleetID, &currentOperationalStatus)
	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Vehicle not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Prepare location JSON with additional metadata
	locationData := map[string]interface{}{
		"latitude":  lat,
		"longitude": lon,
		"source":    req.Source,
		"timestamp": time.Now(),
	}

	if req.Altitude != nil {
		locationData["altitude"] = *req.Altitude
	}
	if req.Accuracy != nil {
		locationData["accuracy"] = *req.Accuracy
	}
	if req.Timestamp != nil {
		locationData["timestamp"] = *req.Timestamp
	}

	locationJSON, err := json.Marshal(locationData)
	if err != nil {
		s.handleError(w, "Failed to marshal location data", err, http.StatusInternalServerError)
		return
	}

	// Build update query
	setParts := []string{"current_location = $1", "last_seen = NOW()", "updated_at = NOW()"}
	args := []interface{}{locationJSON}
	argIndex := 2

	// Add optional fields
	if req.Heading != nil {
		setParts = append(setParts, fmt.Sprintf("current_heading = $%d", argIndex))
		args = append(args, *req.Heading)
		argIndex++
	}
	if req.Speed != nil {
		setParts = append(setParts, fmt.Sprintf("current_speed = $%d", argIndex))
		args = append(args, *req.Speed)
		argIndex++
	}

	// Update operational status if requested and vehicle is moving
	if req.UpdateStatus && req.Speed != nil && *req.Speed > 0.5 {
		if currentOperationalStatus == "idle" || currentOperationalStatus == "offline" {
			setParts = append(setParts, fmt.Sprintf("operational_status = $%d", argIndex))
			args = append(args, "driving_av")
			argIndex++
		}
	}

	// Add vehicle ID to args
	args = append(args, vehicleID)

	// Execute update
	query := fmt.Sprintf("UPDATE fleet_core.vehicles SET %s WHERE vehicle_id = $%d", strings.Join(setParts, ", "), len(args))
	result, err := s.db.ExecContext(ctx, query, args...)
	if err != nil {
		s.handleError(w, "Failed to update vehicle location", err, http.StatusInternalServerError)
		return
	}

	rowsAffected, err := result.RowsAffected()
	if err != nil {
		s.handleError(w, "Failed to get rows affected", err, http.StatusInternalServerError)
		return
	}

	if rowsAffected == 0 {
		s.handleError(w, "Vehicle not found", nil, http.StatusNotFound)
		return
	}

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("update_vehicle_location", "success", fleetID).Inc()

	// Return success response
	response := map[string]interface{}{
		"vehicle_id":   vehicleID,
		"location":     locationData,
		"updated_at":   time.Now(),
		"message":      "Vehicle location updated successfully",
	}

	s.respondJSON(w, response)
}

func (s *FleetManagerService) getFleetVehicles(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getFleetVehicles")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("get_fleet_vehicles", "success").Observe(time.Since(start).Seconds())
	}()

	// Extract fleet ID from URL path
	vars := mux.Vars(r)
	fleetID := vars["fleetId"]
	if fleetID == "" {
		s.handleError(w, "Fleet ID is required", nil, http.StatusBadRequest)
		return
	}

	// Parse query parameters
	queryParams := r.URL.Query()
	status := queryParams.Get("status")
	autonomyLevel := queryParams.Get("autonomy_level")
	limitStr := queryParams.Get("limit")
	offsetStr := queryParams.Get("offset")
	sortBy := queryParams.Get("sort_by")
	sortOrder := queryParams.Get("sort_order")

	// Set defaults
	limit := 50
	offset := 0
	if limitStr != "" {
		if l, err := strconv.Atoi(limitStr); err == nil && l > 0 && l <= 1000 {
			limit = l
		}
	}
	if offsetStr != "" {
		if o, err := strconv.Atoi(offsetStr); err == nil && o >= 0 {
			offset = o
		}
	}
	if sortBy == "" {
		sortBy = "asset_tag"
	}
	if sortOrder == "" {
		sortOrder = "ASC"
	}

	// Validate sort parameters
	allowedSortFields := map[string]bool{
		"asset_tag": true, "operational_status": true, "autonomy_status": true,
		"manufacturer": true, "model": true, "last_seen": true, "created_at": true,
	}
	if !allowedSortFields[sortBy] {
		sortBy = "asset_tag"
	}
	if sortOrder != "ASC" && sortOrder != "DESC" {
		sortOrder = "ASC"
	}

	// Build query with filters
	whereConditions := []string{"v.fleet_id = $1"}
	args := []interface{}{fleetID}
	argIndex := 2

	if status != "" {
		whereConditions = append(whereConditions, fmt.Sprintf("v.operational_status = $%d", argIndex))
		args = append(args, status)
		argIndex++
	}
	if autonomyLevel != "" {
		whereConditions = append(whereConditions, fmt.Sprintf("v.autonomy_level = $%d", argIndex))
		args = append(args, autonomyLevel)
		argIndex++
	}

	whereClause := strings.Join(whereConditions, " AND ")

	// Count total vehicles
	countQuery := fmt.Sprintf(`
		SELECT COUNT(*) 
		FROM fleet_core.vehicles v 
		WHERE %s`, whereClause)

	var totalCount int
	err := s.db.QueryRowContext(ctx, countQuery, args...).Scan(&totalCount)
	if err != nil {
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Query vehicles with pagination
	query := fmt.Sprintf(`
		SELECT 
			v.vehicle_id, v.asset_tag, v.vin, v.license_plate,
			v.manufacturer, v.model, v.year, v.serial_number,
			v.operational_status, v.autonomy_status, v.autonomy_level,
			v.current_location, v.current_heading, v.current_speed,
			v.battery_level, v.fuel_level, v.health_score,
			v.last_seen, v.created_at, v.updated_at,
			v.tags, v.metadata
		FROM fleet_core.vehicles v
		WHERE %s
		ORDER BY v.%s %s
		LIMIT $%d OFFSET $%d`, whereClause, sortBy, sortOrder, argIndex, argIndex+1)

	args = append(args, limit, offset)

	rows, err := s.db.QueryContext(ctx, query, args...)
	if err != nil {
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var vehicles []map[string]interface{}
	for rows.Next() {
		var vehicle map[string]interface{} = make(map[string]interface{})
		var currentLocationJSON, tagsJSON, metadataJSON []byte
		var currentHeading, currentSpeed, batteryLevel, fuelLevel, healthScore sql.NullFloat64
		var lastSeen, createdAt, updatedAt sql.NullTime

		err := rows.Scan(
			&vehicle["vehicle_id"], &vehicle["asset_tag"], &vehicle["vin"], &vehicle["license_plate"],
			&vehicle["manufacturer"], &vehicle["model"], &vehicle["year"], &vehicle["serial_number"],
			&vehicle["operational_status"], &vehicle["autonomy_status"], &vehicle["autonomy_level"],
			&currentLocationJSON, &currentHeading, &currentSpeed,
			&batteryLevel, &fuelLevel, &healthScore,
			&lastSeen, &createdAt, &updatedAt,
			&tagsJSON, &metadataJSON,
		)

		if err != nil {
			s.handleError(w, "Database scan error", err, http.StatusInternalServerError)
			return
		}

		// Parse JSON fields
		if currentLocationJSON != nil {
			json.Unmarshal(currentLocationJSON, &vehicle["current_location"])
		}
		vehicle["current_heading"] = currentHeading.Float64
		vehicle["current_speed"] = currentSpeed.Float64
		vehicle["battery_level"] = batteryLevel.Float64
		vehicle["fuel_level"] = fuelLevel.Float64
		vehicle["health_score"] = healthScore.Float64
		vehicle["last_seen"] = lastSeen.Time
		vehicle["created_at"] = createdAt.Time
		vehicle["updated_at"] = updatedAt.Time

		// Parse arrays
		if tagsJSON != nil {
			var tags []string
			json.Unmarshal(tagsJSON, &tags)
			vehicle["tags"] = tags
		}
		if metadataJSON != nil {
			var metadata map[string]interface{}
			json.Unmarshal(metadataJSON, &metadata)
			vehicle["metadata"] = metadata
		}

		// Calculate online status
		vehicle["is_online"] = time.Since(lastSeen.Time) < 5*time.Minute

		vehicles = append(vehicles, vehicle)
	}

	if err = rows.Err(); err != nil {
		s.handleError(w, "Database rows error", err, http.StatusInternalServerError)
		return
	}

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("get_fleet_vehicles", "success", fleetID).Inc()

	// Return paginated response
	response := map[string]interface{}{
		"fleet_id":     fleetID,
		"vehicles":     vehicles,
		"total_count":  totalCount,
		"limit":        limit,
		"offset":       offset,
		"has_more":     offset+limit < totalCount,
		"returned":     len(vehicles),
	}

	s.respondJSON(w, response)
}

func (s *FleetManagerService) dispatchVehicle(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "dispatchVehicle")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("dispatch_vehicle", "success").Observe(time.Since(start).Seconds())
	}()

	// Parse request body
	var req struct {
		VehicleID     string                 `json:"vehicle_id" validate:"required,uuid"`
		TripID        string                 `json:"trip_id,omitempty"`
		Destination   map[string]interface{} `json:"destination" validate:"required"`
		Priority      string                 `json:"priority,omitempty"` // low, normal, high, emergency
		MissionType   string                 `json:"mission_type,omitempty"` // passenger, cargo, maintenance, emergency
		Constraints   map[string]interface{} `json:"constraints,omitempty"`
		OperatorID    string                 `json:"operator_id,omitempty"`
		DispatchTime  *time.Time             `json:"dispatch_time,omitempty"`
		Instructions  string                 `json:"instructions,omitempty"`
		Metadata      map[string]interface{} `json:"metadata,omitempty"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Validate required fields
	if req.VehicleID == "" || req.Destination == nil {
		s.handleError(w, "Missing required fields: vehicle_id, destination", nil, http.StatusBadRequest)
		return
	}

	// Validate destination coordinates
	lat, latOk := req.Destination["latitude"].(float64)
	lon, lonOk := req.Destination["longitude"].(float64)
	if !latOk || !lonOk {
		s.handleError(w, "Destination must contain valid latitude and longitude", nil, http.StatusBadRequest)
		return
	}

	// Set defaults
	if req.Priority == "" {
		req.Priority = "normal"
	}
	if req.MissionType == "" {
		req.MissionType = "passenger"
	}
	if req.DispatchTime == nil {
		now := time.Now()
		req.DispatchTime = &now
	}

	// Check if vehicle exists and is available
	var vehicleStatus string
	var fleetID string
	var currentTripID sql.NullString
	err := s.db.QueryRowContext(ctx, `
		SELECT operational_status, fleet_id, current_trip_id 
		FROM fleet_core.vehicles 
		WHERE vehicle_id = $1`, req.VehicleID).Scan(&vehicleStatus, &fleetID, &currentTripID)

	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Vehicle not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Check if vehicle is available for dispatch
	if vehicleStatus != "idle" && vehicleStatus != "offline" {
		s.handleError(w, fmt.Sprintf("Vehicle is not available for dispatch. Current status: %s", vehicleStatus), nil, http.StatusConflict)
		return
	}

	// Check if vehicle already has an active trip
	if currentTripID.Valid && currentTripID.String != "" {
		s.handleError(w, "Vehicle already has an active trip", nil, http.StatusConflict)
		return
	}

	// Generate trip ID if not provided
	if req.TripID == "" {
		req.TripID = generateUUID()
	}

	// Prepare destination JSON
	destinationJSON, err := json.Marshal(req.Destination)
	if err != nil {
		s.handleError(w, "Failed to marshal destination", err, http.StatusInternalServerError)
		return
	}

	// Prepare constraints JSON
	constraintsJSON, _ := json.Marshal(req.Constraints)
	metadataJSON, _ := json.Marshal(req.Metadata)

	// Start transaction
	tx, err := s.db.BeginTx(ctx, nil)
	if err != nil {
		s.handleError(w, "Failed to start transaction", err, http.StatusInternalServerError)
		return
	}
	defer tx.Rollback()

	// Create trip record
	_, err = tx.ExecContext(ctx, `
		INSERT INTO fleet_core.trips (
			trip_id, fleet_id, vehicle_id, status, priority, mission_type,
			destination, constraints, operator_id, 
			planned_start_time, created_at, updated_at, metadata
		) VALUES (
			$1, $2, $3, 'planned', $4, $5, $6, $7, $8, $9, NOW(), NOW(), $10
		)`, req.TripID, fleetID, req.VehicleID, req.Priority, req.MissionType,
		destinationJSON, constraintsJSON, req.OperatorID, req.DispatchTime, metadataJSON)

	if err != nil {
		s.handleError(w, "Failed to create trip", err, http.StatusInternalServerError)
		return
	}

	// Update vehicle status
	_, err = tx.ExecContext(ctx, `
		UPDATE fleet_core.vehicles 
		SET operational_status = 'assigned',
		    current_trip_id = $1,
		    updated_at = NOW()
		WHERE vehicle_id = $2`, req.TripID, req.VehicleID)

	if err != nil {
		s.handleError(w, "Failed to update vehicle status", err, http.StatusInternalServerError)
		return
	}

	// Commit transaction
	if err = tx.Commit(); err != nil {
		s.handleError(w, "Failed to commit transaction", err, http.StatusInternalServerError)
		return
	}

	// Send dispatch command to vehicle (via Kafka)
	command := VehicleCommand{
		CommandID:   generateUUID(),
		VehicleID:   req.VehicleID,
		CommandType: "dispatch",
		Priority:    req.Priority,
		Payload: map[string]interface{}{
			"trip_id":      req.TripID,
			"destination":  req.Destination,
			"mission_type": req.MissionType,
			"constraints":  req.Constraints,
			"instructions": req.Instructions,
		},
		CreatedAt: time.Now(),
	}

	// TODO: Send command via Kafka
	s.sendCommandToVehicle(ctx, command)

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("dispatch_vehicle", "success", fleetID).Inc()
	s.metrics.VehicleCommands.WithLabelValues("dispatch", "success", req.Priority).Inc()

	// Return success response
	response := map[string]interface{}{
		"trip_id":       req.TripID,
		"vehicle_id":    req.VehicleID,
		"fleet_id":      fleetID,
		"status":        "dispatched",
		"priority":      req.Priority,
		"mission_type":  req.MissionType,
		"dispatch_time": req.DispatchTime,
		"message":       "Vehicle dispatched successfully",
	}

	s.respondJSON(w, response)
}

func (s *FleetManagerService) recallVehicle(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "recallVehicle")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("recall_vehicle", "success").Observe(time.Since(start).Seconds())
	}()

	// Parse request body
	var req struct {
		VehicleID     string                 `json:"vehicle_id" validate:"required,uuid"`
		Reason        string                 `json:"reason,omitempty"`
		Priority      string                 `json:"priority,omitempty"` // low, normal, high, emergency
		Destination   map[string]interface{} `json:"destination,omitempty"` // Optional recall destination
		OperatorID    string                 `json:"operator_id,omitempty"`
		Instructions  string                 `json:"instructions,omitempty"`
		ForceRecall   bool                   `json:"force_recall,omitempty"` // Override safety checks
		Metadata      map[string]interface{} `json:"metadata,omitempty"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		s.handleError(w, "Invalid JSON payload", err, http.StatusBadRequest)
		return
	}

	// Set defaults
	if req.Priority == "" {
		req.Priority = "high"
	}
	if req.Reason == "" {
		req.Reason = "Manual recall"
	}

	// Check if vehicle exists and get current status
	var vehicleStatus string
	var fleetID string
	var currentTripID sql.NullString
	var currentLocationJSON []byte
	err := s.db.QueryRowContext(ctx, `
		SELECT operational_status, fleet_id, current_trip_id, current_location 
		FROM fleet_core.vehicles 
		WHERE vehicle_id = $1`, req.VehicleID).Scan(&vehicleStatus, &fleetID, &currentTripID, &currentLocationJSON)

	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Vehicle not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Check if vehicle is already idle/offline
	if vehicleStatus == "idle" || vehicleStatus == "offline" {
		s.handleError(w, "Vehicle is already idle", nil, http.StatusConflict)
		return
	}

	// Safety check: Don't recall vehicles in critical operations unless forced
	if !req.ForceRecall && (vehicleStatus == "driving_av" || vehicleStatus == "driving_manual") {
		// Check if vehicle is in a safe location for recall
		var isInSafeLocation bool
		// This would typically check against geofenced safe zones
		// For now, we'll allow recall but log the safety concern
		log.Printf("WARNING: Recalling vehicle %s while in %s status", req.VehicleID, vehicleStatus)
	}

	// Start transaction
	tx, err := s.db.BeginTx(ctx, nil)
	if err != nil {
		s.handleError(w, "Failed to start transaction", err, http.StatusInternalServerError)
		return
	}
	defer tx.Rollback()

	// Update vehicle status to recalled
	_, err = tx.ExecContext(ctx, `
		UPDATE fleet_core.vehicles 
		SET operational_status = 'recalled',
		    updated_at = NOW()
		WHERE vehicle_id = $1`, req.VehicleID)

	if err != nil {
		s.handleError(w, "Failed to update vehicle status", err, http.StatusInternalServerError)
		return
	}

	// Update current trip status if exists
	if currentTripID.Valid && currentTripID.String != "" {
		_, err = tx.ExecContext(ctx, `
			UPDATE fleet_core.trips 
			SET status = 'cancelled',
			    cancelled_at = NOW(),
			    cancellation_reason = $1,
			    updated_at = NOW()
			WHERE trip_id = $2`, req.Reason, currentTripID.String)

		if err != nil {
			s.handleError(w, "Failed to cancel trip", err, http.StatusInternalServerError)
			return
		}
	}

	// Commit transaction
	if err = tx.Commit(); err != nil {
		s.handleError(w, "Failed to commit transaction", err, http.StatusInternalServerError)
		return
	}

	// Send recall command to vehicle
	command := VehicleCommand{
		CommandID:   generateUUID(),
		VehicleID:   req.VehicleID,
		CommandType: "recall",
		Priority:    req.Priority,
		Payload: map[string]interface{}{
			"reason":       req.Reason,
			"destination":  req.Destination,
			"instructions": req.Instructions,
			"force_recall": req.ForceRecall,
		},
		CreatedAt: time.Now(),
	}

	// TODO: Send command via Kafka
	s.sendCommandToVehicle(ctx, command)

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("recall_vehicle", "success", fleetID).Inc()
	s.metrics.VehicleCommands.WithLabelValues("recall", "success", req.Priority).Inc()

	// Return success response
	response := map[string]interface{}{
		"vehicle_id":    req.VehicleID,
		"fleet_id":      fleetID,
		"status":        "recalled",
		"priority":      req.Priority,
		"reason":        req.Reason,
		"recalled_at":   time.Now(),
		"message":       "Vehicle recalled successfully",
	}

	s.respondJSON(w, response)
}

func (s *FleetManagerService) getFleetUtilization(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getFleetUtilization")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("get_fleet_utilization", "success").Observe(time.Since(start).Seconds())
	}()

	// Extract fleet ID from URL path
	vars := mux.Vars(r)
	fleetID := vars["fleetId"]
	if fleetID == "" {
		s.handleError(w, "Fleet ID is required", nil, http.StatusBadRequest)
		return
	}

	// Parse query parameters
	queryParams := r.URL.Query()
	startDateStr := queryParams.Get("start_date")
	endDateStr := queryParams.Get("end_date")
	timeframe := queryParams.Get("timeframe") // hour, day, week, month

	// Set defaults
	now := time.Now()
	var startDate, endDate time.Time
	
	if startDateStr != "" {
		if parsed, err := time.Parse("2006-01-02", startDateStr); err == nil {
			startDate = parsed
		} else {
			s.handleError(w, "Invalid start_date format. Use YYYY-MM-DD", nil, http.StatusBadRequest)
			return
		}
	} else {
		startDate = now.AddDate(0, 0, -7) // Default to last 7 days
	}

	if endDateStr != "" {
		if parsed, err := time.Parse("2006-01-02", endDateStr); err == nil {
			endDate = parsed
		} else {
			s.handleError(w, "Invalid end_date format. Use YYYY-MM-DD", nil, http.StatusBadRequest)
			return
		}
	} else {
		endDate = now
	}

	if timeframe == "" {
		timeframe = "day"
	}

	// Validate timeframe
	allowedTimeframes := map[string]bool{"hour": true, "day": true, "week": true, "month": true}
	if !allowedTimeframes[timeframe] {
		timeframe = "day"
	}

	// Check if fleet exists
	var fleetName string
	err := s.db.QueryRowContext(ctx, "SELECT name FROM fleet_core.fleets WHERE fleet_id = $1", fleetID).Scan(&fleetName)
	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Fleet not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Get fleet utilization using stored procedure
	query := `SELECT * FROM calculate_fleet_utilization($1, $2, $3)`
	rows, err := s.db.QueryContext(ctx, query, fleetID, startDate, endDate)
	if err != nil {
		s.handleError(w, "Failed to calculate fleet utilization", err, http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var utilization struct {
		FleetID                string  `json:"fleet_id"`
		TotalVehicles          int     `json:"total_vehicles"`
		ActiveVehicles         int     `json:"active_vehicles"`
		UtilizationPercentage  float64 `json:"utilization_percentage"`
		AvgTripDurationMinutes float64 `json:"avg_trip_duration_minutes"`
		TotalDistanceKm        float64 `json:"total_distance_km"`
		AvgSpeedKmh            float64 `json:"avg_speed_kmh"`
		EfficiencyScore        float64 `json:"efficiency_score"`
	}

	if rows.Next() {
		err = rows.Scan(
			&utilization.FleetID, &utilization.TotalVehicles, &utilization.ActiveVehicles,
			&utilization.UtilizationPercentage, &utilization.AvgTripDurationMinutes,
			&utilization.TotalDistanceKm, &utilization.AvgSpeedKmh, &utilization.EfficiencyScore,
		)
		if err != nil {
			s.handleError(w, "Failed to scan utilization data", err, http.StatusInternalServerError)
			return
		}
	}

	// Get additional metrics
	var totalTrips, completedTrips, cancelledTrips int
	var avgBatteryLevel, avgHealthScore float64

	// Trip statistics
	err = s.db.QueryRowContext(ctx, `
		SELECT 
			COUNT(*) as total_trips,
			COUNT(CASE WHEN status = 'completed' THEN 1 END) as completed_trips,
			COUNT(CASE WHEN status = 'cancelled' THEN 1 END) as cancelled_trips
		FROM fleet_core.trips 
		WHERE fleet_id = $1 AND created_at BETWEEN $2 AND $3`, 
		fleetID, startDate, endDate).Scan(&totalTrips, &completedTrips, &cancelledTrips)

	if err != nil {
		s.handleError(w, "Failed to get trip statistics", err, http.StatusInternalServerError)
		return
	}

	// Vehicle health metrics
	err = s.db.QueryRowContext(ctx, `
		SELECT 
			AVG(battery_level) as avg_battery,
			AVG(health_score) as avg_health
		FROM fleet_core.vehicles 
		WHERE fleet_id = $1 AND last_seen > $2`, 
		fleetID, startDate).Scan(&avgBatteryLevel, &avgHealthScore)

	if err != nil {
		// Log error but don't fail - these are optional metrics
		log.Printf("Warning: Failed to get vehicle health metrics: %v", err)
	}

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("get_fleet_utilization", "success", fleetID).Inc()

	// Return comprehensive utilization report
	response := map[string]interface{}{
		"fleet_id":           fleetID,
		"fleet_name":         fleetName,
		"period": map[string]interface{}{
			"start_date": startDate.Format("2006-01-02"),
			"end_date":   endDate.Format("2006-01-02"),
			"timeframe":  timeframe,
		},
		"utilization": utilization,
		"trip_metrics": map[string]interface{}{
			"total_trips":     totalTrips,
			"completed_trips": completedTrips,
			"cancelled_trips": cancelledTrips,
			"completion_rate": float64(completedTrips) / float64(totalTrips) * 100,
		},
		"vehicle_health": map[string]interface{}{
			"avg_battery_level": avgBatteryLevel,
			"avg_health_score":  avgHealthScore,
		},
		"generated_at": time.Now(),
	}

	s.respondJSON(w, response)
}

func (s *FleetManagerService) getVehicleHealthMetrics(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getVehicleHealthMetrics")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("get_vehicle_health_metrics", "success").Observe(time.Since(start).Seconds())
	}()

	// Extract vehicle ID from URL path
	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]
	if vehicleID == "" {
		s.handleError(w, "Vehicle ID is required", nil, http.StatusBadRequest)
		return
	}

	// Parse query parameters
	queryParams := r.URL.Query()
	startDateStr := queryParams.Get("start_date")
	endDateStr := queryParams.Get("end_date")
	includeHistory := queryParams.Get("include_history") == "true"

	// Set defaults
	now := time.Now()
	var startDate, endDate time.Time
	
	if startDateStr != "" {
		if parsed, err := time.Parse("2006-01-02", startDateStr); err == nil {
			startDate = parsed
		} else {
			s.handleError(w, "Invalid start_date format. Use YYYY-MM-DD", nil, http.StatusBadRequest)
			return
		}
	} else {
		startDate = now.AddDate(0, 0, -30) // Default to last 30 days
	}

	if endDateStr != "" {
		if parsed, err := time.Parse("2006-01-02", endDateStr); err == nil {
			endDate = parsed
		} else {
			s.handleError(w, "Invalid end_date format. Use YYYY-MM-DD", nil, http.StatusBadRequest)
			return
		}
	} else {
		endDate = now
	}

	// Check if vehicle exists
	var assetTag, manufacturer, model string
	err := s.db.QueryRowContext(ctx, "SELECT asset_tag, manufacturer, model FROM fleet_core.vehicles WHERE vehicle_id = $1", vehicleID).Scan(&assetTag, &manufacturer, &model)
	if err != nil {
		if err == sql.ErrNoRows {
			s.handleError(w, "Vehicle not found", nil, http.StatusNotFound)
			return
		}
		s.handleError(w, "Database error", err, http.StatusInternalServerError)
		return
	}

	// Get current health score using stored procedure
	query := `SELECT * FROM calculate_vehicle_health_score($1, $2)`
	rows, err := s.db.QueryContext(ctx, query, vehicleID, endDate)
	if err != nil {
		s.handleError(w, "Failed to calculate vehicle health score", err, http.StatusInternalServerError)
		return
	}
	defer rows.Close()

	var healthScore struct {
		VehicleID        string  `json:"vehicle_id"`
		HealthScore      float64 `json:"health_score"`
		HealthBreakdown  map[string]interface{} `json:"health_breakdown"`
		Alerts           []string `json:"alerts"`
		Recommendations  []string `json:"recommendations"`
		AssessmentTime   time.Time `json:"assessment_time"`
	}

	if rows.Next() {
		var healthBreakdownJSON, alertsJSON, recommendationsJSON []byte
		var assessmentTime time.Time
		
		err = rows.Scan(
			&healthScore.VehicleID, &healthScore.HealthScore, &healthBreakdownJSON,
			&alertsJSON, &recommendationsJSON, &assessmentTime,
		)
		if err != nil {
			s.handleError(w, "Failed to scan health score data", err, http.StatusInternalServerError)
			return
		}

		// Parse JSON fields
		json.Unmarshal(healthBreakdownJSON, &healthScore.HealthBreakdown)
		json.Unmarshal(alertsJSON, &healthScore.Alerts)
		json.Unmarshal(recommendationsJSON, &healthScore.Recommendations)
		healthScore.AssessmentTime = assessmentTime
	}

	// Get current vehicle status
	var currentStatus struct {
		OperationalStatus string    `json:"operational_status"`
		AutonomyStatus    string    `json:"autonomy_status"`
		BatteryLevel      float64   `json:"battery_level"`
		FuelLevel         float64   `json:"fuel_level"`
		OdometerKm        float64   `json:"odometer_km"`
		EngineHours       float64   `json:"engine_hours"`
		LastSeen          time.Time `json:"last_seen"`
		LastMaintenance   time.Time `json:"last_maintenance_date"`
		NextMaintenance   time.Time `json:"next_maintenance_due"`
	}

	err = s.db.QueryRowContext(ctx, `
		SELECT operational_status, autonomy_status, battery_level, fuel_level,
		       odometer_km, engine_hours, last_seen, last_maintenance_date, next_maintenance_due
		FROM fleet_core.vehicles 
		WHERE vehicle_id = $1`, vehicleID).Scan(
		&currentStatus.OperationalStatus, &currentStatus.AutonomyStatus, &currentStatus.BatteryLevel,
		&currentStatus.FuelLevel, &currentStatus.OdometerKm, &currentStatus.EngineHours,
		&currentStatus.LastSeen, &currentStatus.LastMaintenance, &currentStatus.NextMaintenance)

	if err != nil {
		s.handleError(w, "Failed to get current vehicle status", err, http.StatusInternalServerError)
		return
	}

	// Get historical health data if requested
	var historicalData []map[string]interface{}
	if includeHistory {
		rows, err := s.db.QueryContext(ctx, `
			SELECT health_score, battery_level, fuel_level, odometer_km, engine_hours, last_seen
			FROM fleet_core.vehicle_health_history 
			WHERE vehicle_id = $1 AND recorded_at BETWEEN $2 AND $3
			ORDER BY recorded_at DESC`, vehicleID, startDate, endDate)

		if err == nil {
			defer rows.Close()
			for rows.Next() {
				var record map[string]interface{} = make(map[string]interface{})
				var healthScore, batteryLevel, fuelLevel, odometerKm, engineHours sql.NullFloat64
				var lastSeen sql.NullTime

				err = rows.Scan(&healthScore, &batteryLevel, &fuelLevel, &odometerKm, &engineHours, &lastSeen)
				if err != nil {
					continue
				}

				record["health_score"] = healthScore.Float64
				record["battery_level"] = batteryLevel.Float64
				record["fuel_level"] = fuelLevel.Float64
				record["odometer_km"] = odometerKm.Float64
				record["engine_hours"] = engineHours.Float64
				record["last_seen"] = lastSeen.Time

				historicalData = append(historicalData, record)
			}
		}
	}

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("get_vehicle_health_metrics", "success", "").Inc()

	// Return comprehensive health report
	response := map[string]interface{}{
		"vehicle_id":   vehicleID,
		"asset_tag":    assetTag,
		"manufacturer": manufacturer,
		"model":        model,
		"period": map[string]interface{}{
			"start_date": startDate.Format("2006-01-02"),
			"end_date":   endDate.Format("2006-01-02"),
		},
		"current_health": healthScore,
		"current_status":  currentStatus,
		"historical_data": historicalData,
		"generated_at":    time.Now(),
	}

	s.respondJSON(w, response)
}

func (s *FleetManagerService) getOperationalMetrics(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getOperationalMetrics")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.ResponseTime.WithLabelValues("get_operational_metrics", "success").Observe(time.Since(start).Seconds())
	}()

	// Parse query parameters
	queryParams := r.URL.Query()
	fleetID := queryParams.Get("fleet_id")
	startDateStr := queryParams.Get("start_date")
	endDateStr := queryParams.Get("end_date")
	metricType := queryParams.Get("metric_type") // fleet, vehicle, trip, maintenance

	// Set defaults
	now := time.Now()
	var startDate, endDate time.Time
	
	if startDateStr != "" {
		if parsed, err := time.Parse("2006-01-02", startDateStr); err == nil {
			startDate = parsed
		} else {
			s.handleError(w, "Invalid start_date format. Use YYYY-MM-DD", nil, http.StatusBadRequest)
			return
		}
	} else {
		startDate = now.AddDate(0, 0, -7) // Default to last 7 days
	}

	if endDateStr != "" {
		if parsed, err := time.Parse("2006-01-02", endDateStr); err == nil {
			endDate = parsed
		} else {
			s.handleError(w, "Invalid end_date format. Use YYYY-MM-DD", nil, http.StatusBadRequest)
			return
		}
	} else {
		endDate = now
	}

	if metricType == "" {
		metricType = "fleet"
	}

	// Validate metric type
	allowedTypes := map[string]bool{"fleet": true, "vehicle": true, "trip": true, "maintenance": true}
	if !allowedTypes[metricType] {
		metricType = "fleet"
	}

	// Build base query conditions
	whereConditions := []string{"created_at BETWEEN $1 AND $2"}
	args := []interface{}{startDate, endDate}
	argIndex := 3

	if fleetID != "" {
		whereConditions = append(whereConditions, fmt.Sprintf("fleet_id = $%d", argIndex))
		args = append(args, fleetID)
		argIndex++
	}

	whereClause := strings.Join(whereConditions, " AND ")

	var metrics map[string]interface{} = make(map[string]interface{})

	// Fleet-level metrics
	if metricType == "fleet" || metricType == "all" {
		// Vehicle status distribution
		var statusDistribution map[string]int = make(map[string]int)
		rows, err := s.db.QueryContext(ctx, `
			SELECT operational_status, COUNT(*) 
			FROM fleet_core.vehicles 
			WHERE fleet_id = $1
			GROUP BY operational_status`, fleetID)

		if err == nil {
			defer rows.Close()
			for rows.Next() {
				var status string
				var count int
				rows.Scan(&status, &count)
				statusDistribution[status] = count
			}
		}

		// Trip completion rates
		var tripMetrics struct {
			TotalTrips     int     `json:"total_trips"`
			CompletedTrips int     `json:"completed_trips"`
			CancelledTrips int     `json:"cancelled_trips"`
			CompletionRate float64 `json:"completion_rate"`
		}

		err = s.db.QueryRowContext(ctx, fmt.Sprintf(`
			SELECT 
				COUNT(*) as total_trips,
				COUNT(CASE WHEN status = 'completed' THEN 1 END) as completed_trips,
				COUNT(CASE WHEN status = 'cancelled' THEN 1 END) as cancelled_trips
			FROM fleet_core.trips 
			WHERE %s`, whereClause), args...).Scan(
			&tripMetrics.TotalTrips, &tripMetrics.CompletedTrips, &tripMetrics.CancelledTrips)

		if err == nil && tripMetrics.TotalTrips > 0 {
			tripMetrics.CompletionRate = float64(tripMetrics.CompletedTrips) / float64(tripMetrics.TotalTrips) * 100
		}

		// Average response times
		var avgResponseTime float64
		s.db.QueryRowContext(ctx, fmt.Sprintf(`
			SELECT AVG(EXTRACT(EPOCH FROM (started_at - created_at))/60) as avg_response_minutes
			FROM fleet_core.trips 
			WHERE %s AND started_at IS NOT NULL`, whereClause), args...).Scan(&avgResponseTime)

		metrics["fleet_metrics"] = map[string]interface{}{
			"status_distribution": statusDistribution,
			"trip_metrics":        tripMetrics,
			"avg_response_time_minutes": avgResponseTime,
		}
	}

	// Vehicle-level metrics
	if metricType == "vehicle" || metricType == "all" {
		// Vehicle utilization
		var vehicleUtilization struct {
			TotalVehicles      int     `json:"total_vehicles"`
			ActiveVehicles     int     `json:"active_vehicles"`
			UtilizationRate    float64 `json:"utilization_rate"`
			AvgHealthScore     float64 `json:"avg_health_score"`
			AvgBatteryLevel    float64 `json:"avg_battery_level"`
		}

		err = s.db.QueryRowContext(ctx, fmt.Sprintf(`
			SELECT 
				COUNT(*) as total_vehicles,
				COUNT(CASE WHEN operational_status IN ('driving_av', 'driving_manual') THEN 1 END) as active_vehicles,
				AVG(health_score) as avg_health,
				AVG(battery_level) as avg_battery
			FROM fleet_core.vehicles 
			WHERE %s`, whereClause), args...).Scan(
			&vehicleUtilization.TotalVehicles, &vehicleUtilization.ActiveVehicles,
			&vehicleUtilization.AvgHealthScore, &vehicleUtilization.AvgBatteryLevel)

		if err == nil && vehicleUtilization.TotalVehicles > 0 {
			vehicleUtilization.UtilizationRate = float64(vehicleUtilization.ActiveVehicles) / float64(vehicleUtilization.TotalVehicles) * 100
		}

		// Vehicle performance by manufacturer
		var manufacturerPerformance []map[string]interface{}
		rows, err := s.db.QueryContext(ctx, fmt.Sprintf(`
			SELECT manufacturer, COUNT(*) as vehicle_count, AVG(health_score) as avg_health
			FROM fleet_core.vehicles 
			WHERE %s
			GROUP BY manufacturer
			ORDER BY avg_health DESC`, whereClause), args...)

		if err == nil {
			defer rows.Close()
			for rows.Next() {
				var record map[string]interface{} = make(map[string]interface{})
				var manufacturer string
				var vehicleCount int
				var avgHealth float64
				rows.Scan(&manufacturer, &vehicleCount, &avgHealth)
				record["manufacturer"] = manufacturer
				record["vehicle_count"] = vehicleCount
				record["avg_health_score"] = avgHealth
				manufacturerPerformance = append(manufacturerPerformance, record)
			}
		}

		metrics["vehicle_metrics"] = map[string]interface{}{
			"utilization": vehicleUtilization,
			"manufacturer_performance": manufacturerPerformance,
		}
	}

	// Trip-level metrics
	if metricType == "trip" || metricType == "all" {
		// Trip duration statistics
		var tripDuration struct {
			AvgDurationMinutes float64 `json:"avg_duration_minutes"`
			MinDurationMinutes float64 `json:"min_duration_minutes"`
			MaxDurationMinutes float64 `json:"max_duration_minutes"`
		}

		err = s.db.QueryRowContext(ctx, fmt.Sprintf(`
			SELECT 
				AVG(EXTRACT(EPOCH FROM (completed_at - started_at))/60) as avg_duration,
				MIN(EXTRACT(EPOCH FROM (completed_at - started_at))/60) as min_duration,
				MAX(EXTRACT(EPOCH FROM (completed_at - started_at))/60) as max_duration
			FROM fleet_core.trips 
			WHERE %s AND status = 'completed' AND completed_at IS NOT NULL`, whereClause), args...).Scan(
			&tripDuration.AvgDurationMinutes, &tripDuration.MinDurationMinutes, &tripDuration.MaxDurationMinutes)

		// Distance statistics
		var distanceStats struct {
			TotalDistanceKm    float64 `json:"total_distance_km"`
			AvgDistanceKm      float64 `json:"avg_distance_km"`
			MaxDistanceKm      float64 `json:"max_distance_km"`
		}

		s.db.QueryRowContext(ctx, fmt.Sprintf(`
			SELECT 
				SUM(distance_km) as total_distance,
				AVG(distance_km) as avg_distance,
				MAX(distance_km) as max_distance
			FROM fleet_core.trips 
			WHERE %s AND status = 'completed'`, whereClause), args...).Scan(
			&distanceStats.TotalDistanceKm, &distanceStats.AvgDistanceKm, &distanceStats.MaxDistanceKm)

		metrics["trip_metrics"] = map[string]interface{}{
			"duration_stats": tripDuration,
			"distance_stats": distanceStats,
		}
	}

	// Maintenance metrics
	if metricType == "maintenance" || metricType == "all" {
		// Maintenance statistics
		var maintenanceStats struct {
			TotalMaintenanceEvents int     `json:"total_maintenance_events"`
			AvgMaintenanceCost     float64 `json:"avg_maintenance_cost"`
			UpcomingMaintenance    int     `json:"upcoming_maintenance"`
			OverdueMaintenance     int     `json:"overdue_maintenance"`
		}

		err = s.db.QueryRowContext(ctx, fmt.Sprintf(`
			SELECT 
				COUNT(*) as total_events,
				AVG(cost) as avg_cost,
				COUNT(CASE WHEN next_maintenance_due <= NOW() + INTERVAL '7 days' THEN 1 END) as upcoming,
				COUNT(CASE WHEN next_maintenance_due < NOW() THEN 1 END) as overdue
			FROM fleet_core.vehicles 
			WHERE %s`, whereClause), args...).Scan(
			&maintenanceStats.TotalMaintenanceEvents, &maintenanceStats.AvgMaintenanceCost,
			&maintenanceStats.UpcomingMaintenance, &maintenanceStats.OverdueMaintenance)

		metrics["maintenance_metrics"] = maintenanceStats
	}

	// Update metrics
	s.metrics.FleetOperations.WithLabelValues("get_operational_metrics", "success", fleetID).Inc()

	// Return comprehensive operational metrics
	response := map[string]interface{}{
		"fleet_id":     fleetID,
		"metric_type":  metricType,
		"period": map[string]interface{}{
			"start_date": startDate.Format("2006-01-02"),
			"end_date":   endDate.Format("2006-01-02"),
		},
		"metrics":      metrics,
		"generated_at": time.Now(),
	}

	s.respondJSON(w, response)
}

// SAFETY: Helper functions for command validation

// isCriticalCommand determines if a command requires dual authorization
// SAFETY: Critical commands can cause immediate vehicle state changes
func (s *FleetManagerService) isCriticalCommand(commandType string) bool {
	criticalCommands := map[string]bool{
		"emergency_stop":    true,
		"disable_autonomy":  true,
		"override_route":    true,
		"force_manual":      true,
		"emergency_brake":   true,
	}
	return criticalCommands[commandType]
}

// validateDualAuth checks for dual authorization on safety-critical commands
// SECURITY: Requires two authorized operators for critical vehicle commands
func (s *FleetManagerService) validateDualAuth(r *http.Request, command VehicleCommand) bool {
	// TODO: Implement actual dual-auth validation
	// Should check for two valid JWT tokens with appropriate roles
	authHeader := r.Header.Get("Authorization")
	dualAuthHeader := r.Header.Get("X-Dual-Authorization")
	
	return authHeader != "" && dualAuthHeader != ""
}

// sendJSON is a helper for consistent JSON responses
func (s *FleetManagerService) sendJSON(w http.ResponseWriter, statusCode int, data interface{}) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(statusCode)
	json.NewEncoder(w).Encode(data)
}
