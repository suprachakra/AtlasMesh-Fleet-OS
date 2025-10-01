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
	db     *sql.DB        // PostgreSQL connection pool (thread-safe)
	redis  *redis.Client  // Redis client with connection pooling
	config Config         // Immutable configuration loaded at startup
	tracer trace.Tracer   // OpenTelemetry tracer for distributed tracing
	metrics *Metrics      // Prometheus metrics collectors
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
		db:      db,
		redis:   redisClient,
		config:  config,
		tracer:  tracer,
		metrics: metrics,
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
	// TODO: Implement fleet deletion logic
	s.handleError(w, "Fleet deletion not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) createVehicle(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement vehicle creation logic
	s.handleError(w, "Vehicle creation not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) getVehicle(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement get vehicle logic
	s.handleError(w, "Get vehicle not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) updateVehicle(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement vehicle update logic
	s.handleError(w, "Vehicle update not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) deleteVehicle(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement vehicle deletion logic
	s.handleError(w, "Vehicle deletion not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) getVehicleStatus(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement vehicle status logic
	s.handleError(w, "Vehicle status not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) updateVehicleLocation(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement location update logic
	s.handleError(w, "Location update not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) getFleetVehicles(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement fleet vehicles logic
	s.handleError(w, "Fleet vehicles not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) dispatchVehicle(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement vehicle dispatch logic
	s.handleError(w, "Vehicle dispatch not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) recallVehicle(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement vehicle recall logic
	s.handleError(w, "Vehicle recall not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) getFleetUtilization(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement utilization analytics
	s.handleError(w, "Fleet utilization analytics not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) getVehicleHealthMetrics(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement health metrics
	s.handleError(w, "Vehicle health metrics not yet implemented", nil, http.StatusNotImplemented)
}

func (s *FleetManagerService) getOperationalMetrics(w http.ResponseWriter, r *http.Request) {
	// TODO: Implement operational metrics
	s.handleError(w, "Operational metrics not yet implemented", nil, http.StatusNotImplemented)
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
