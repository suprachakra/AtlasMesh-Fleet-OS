package main

import (
	"context"
	"database/sql"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"syscall"
	"time"

	"github.com/gorilla/mux"
	_ "github.com/lib/pq"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"github.com/redis/go-redis/v9"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// AtlasMesh ERP Integration Service
// Enterprise Resource Planning and Warehouse Management System Integration
// SAP, Oracle WMS, and custom ERP adapters for Abu Dhabi operations

type Config struct {
	Port            int    `json:"port"`
	PostgresURL     string `json:"postgres_url"`
	RedisURL        string `json:"redis_url"`
	SAPEndpoint     string `json:"sap_endpoint"`
	SAPUsername     string `json:"sap_username"`
	SAPPassword     string `json:"sap_password"`
	OracleWMSURL    string `json:"oracle_wms_url"`
	OracleUsername  string `json:"oracle_username"`
	OraclePassword  string `json:"oracle_password"`
	CustomERPURL    string `json:"custom_erp_url"`
	CustomERPToken  string `json:"custom_erp_token"`
	LogLevel        string `json:"log_level"`
	AbuDhabiMode    bool   `json:"abu_dhabi_mode"`
	SyncInterval    int    `json:"sync_interval_minutes"`
}

type ERPIntegrationService struct {
	config   Config
	tracer   trace.Tracer
	metrics  *Metrics
	postgres *sql.DB
	redis    *redis.Client
	adapters map[string]ERPAdapter
}

type Metrics struct {
	SyncOperations    *prometheus.CounterVec
	SyncLatency       *prometheus.HistogramVec
	SyncErrors        *prometheus.CounterVec
	DataRecords       *prometheus.GaugeVec
	AdapterStatus     *prometheus.GaugeVec
}

// ERP Adapter Interface
type ERPAdapter interface {
	Connect() error
	Disconnect() error
	SyncVehicles() ([]VehicleRecord, error)
	SyncTrips() ([]TripRecord, error)
	SyncInventory() ([]InventoryRecord, error)
	SyncWorkOrders() ([]WorkOrderRecord, error)
	CreateWorkOrder(order WorkOrderRequest) (*WorkOrderResponse, error)
	UpdateVehicleStatus(vehicleID string, status VehicleStatusUpdate) error
	GetHealthStatus() AdapterHealth
}

// Data Models
type VehicleRecord struct {
	VehicleID       string                 `json:"vehicle_id"`
	AssetTag        string                 `json:"asset_tag"`
	ERPVehicleID    string                 `json:"erp_vehicle_id"`
	Status          string                 `json:"status"`
	Location        string                 `json:"location"`
	AssignedDriver  string                 `json:"assigned_driver"`
	MaintenanceInfo MaintenanceInfo        `json:"maintenance_info"`
	CostCenter      string                 `json:"cost_center"`
	Department      string                 `json:"department"`
	Metadata        map[string]interface{} `json:"metadata"`
	LastUpdated     time.Time              `json:"last_updated"`
}

type TripRecord struct {
	TripID          string                 `json:"trip_id"`
	ERPTripID       string                 `json:"erp_trip_id"`
	VehicleID       string                 `json:"vehicle_id"`
	DriverID        string                 `json:"driver_id"`
	Origin          Location               `json:"origin"`
	Destination     Location               `json:"destination"`
	Status          string                 `json:"status"`
	ScheduledStart  time.Time              `json:"scheduled_start"`
	ActualStart     *time.Time             `json:"actual_start,omitempty"`
	ScheduledEnd    time.Time              `json:"scheduled_end"`
	ActualEnd       *time.Time             `json:"actual_end,omitempty"`
	CargoInfo       CargoInfo              `json:"cargo_info"`
	CostCenter      string                 `json:"cost_center"`
	Priority        string                 `json:"priority"`
	Metadata        map[string]interface{} `json:"metadata"`
	LastUpdated     time.Time              `json:"last_updated"`
}

type InventoryRecord struct {
	ItemID          string                 `json:"item_id"`
	ERPItemID       string                 `json:"erp_item_id"`
	ItemName        string                 `json:"item_name"`
	Category        string                 `json:"category"`
	Quantity        int                    `json:"quantity"`
	Unit            string                 `json:"unit"`
	Location        string                 `json:"location"`
	CostPerUnit     float64                `json:"cost_per_unit"`
	ReorderLevel    int                    `json:"reorder_level"`
	Supplier        string                 `json:"supplier"`
	ExpiryDate      *time.Time             `json:"expiry_date,omitempty"`
	Metadata        map[string]interface{} `json:"metadata"`
	LastUpdated     time.Time              `json:"last_updated"`
}

type WorkOrderRecord struct {
	WorkOrderID     string                 `json:"work_order_id"`
	ERPWorkOrderID  string                 `json:"erp_work_order_id"`
	VehicleID       string                 `json:"vehicle_id"`
	WorkOrderType   string                 `json:"work_order_type"`
	Priority        string                 `json:"priority"`
	Status          string                 `json:"status"`
	Description     string                 `json:"description"`
	AssignedTo      string                 `json:"assigned_to"`
	ScheduledDate   time.Time              `json:"scheduled_date"`
	CompletedDate   *time.Time             `json:"completed_date,omitempty"`
	EstimatedCost   float64                `json:"estimated_cost"`
	ActualCost      float64                `json:"actual_cost"`
	PartsRequired   []PartRequirement      `json:"parts_required"`
	LaborHours      float64                `json:"labor_hours"`
	Metadata        map[string]interface{} `json:"metadata"`
	LastUpdated     time.Time              `json:"last_updated"`
}

type MaintenanceInfo struct {
	LastService     time.Time `json:"last_service"`
	NextService     time.Time `json:"next_service"`
	ServiceType     string    `json:"service_type"`
	Odometer        float64   `json:"odometer"`
	ServiceProvider string    `json:"service_provider"`
}

type Location struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Address   string  `json:"address"`
	Zone      string  `json:"zone"`
}

type CargoInfo struct {
	CargoType   string  `json:"cargo_type"`
	Weight      float64 `json:"weight"`
	Volume      float64 `json:"volume"`
	Value       float64 `json:"value"`
	Special     bool    `json:"special_handling"`
	Temperature string  `json:"temperature_requirements"`
}

type PartRequirement struct {
	PartID       string  `json:"part_id"`
	PartName     string  `json:"part_name"`
	Quantity     int     `json:"quantity"`
	UnitCost     float64 `json:"unit_cost"`
	Supplier     string  `json:"supplier"`
	LeadTime     int     `json:"lead_time_days"`
}

type WorkOrderRequest struct {
	VehicleID     string            `json:"vehicle_id"`
	WorkType      string            `json:"work_type"`
	Priority      string            `json:"priority"`
	Description   string            `json:"description"`
	ScheduledDate time.Time         `json:"scheduled_date"`
	Parts         []PartRequirement `json:"parts"`
	EstimatedCost float64           `json:"estimated_cost"`
}

type WorkOrderResponse struct {
	WorkOrderID    string    `json:"work_order_id"`
	ERPWorkOrderID string    `json:"erp_work_order_id"`
	Status         string    `json:"status"`
	CreatedAt      time.Time `json:"created_at"`
	EstimatedCompletion time.Time `json:"estimated_completion"`
}

type VehicleStatusUpdate struct {
	Status      string                 `json:"status"`
	Location    Location               `json:"location"`
	Odometer    float64                `json:"odometer"`
	FuelLevel   float64                `json:"fuel_level"`
	DriverID    string                 `json:"driver_id"`
	Metadata    map[string]interface{} `json:"metadata"`
	UpdatedAt   time.Time              `json:"updated_at"`
}

type AdapterHealth struct {
	AdapterName   string    `json:"adapter_name"`
	Status        string    `json:"status"`
	LastSync      time.Time `json:"last_sync"`
	ErrorCount    int       `json:"error_count"`
	RecordCount   int       `json:"record_count"`
	ResponseTime  float64   `json:"response_time_ms"`
}

// SAP Adapter Implementation
type SAPAdapter struct {
	config   Config
	client   *http.Client
	baseURL  string
	username string
	password string
	token    string
}

func NewSAPAdapter(config Config) *SAPAdapter {
	return &SAPAdapter{
		config:   config,
		client:   &http.Client{Timeout: 30 * time.Second},
		baseURL:  config.SAPEndpoint,
		username: config.SAPUsername,
		password: config.SAPPassword,
	}
}

func (s *SAPAdapter) Connect() error {
	// Implement SAP authentication
	log.Printf("🔗 Connecting to SAP system at %s", s.baseURL)
	
	// In production, implement proper SAP authentication
	s.token = "sap_auth_token_placeholder"
	
	return nil
}

func (s *SAPAdapter) Disconnect() error {
	log.Printf("🔌 Disconnecting from SAP system")
	s.token = ""
	return nil
}

func (s *SAPAdapter) SyncVehicles() ([]VehicleRecord, error) {
	// Implement SAP vehicle data synchronization
	log.Printf("📊 Syncing vehicles from SAP")
	
	// Mock data for demonstration
	vehicles := []VehicleRecord{
		{
			VehicleID:    "VEH_001",
			AssetTag:     "AUH_001",
			ERPVehicleID: "SAP_ASSET_001",
			Status:       "active",
			Location:     "Abu Dhabi Central",
			AssignedDriver: "DRIVER_001",
			MaintenanceInfo: MaintenanceInfo{
				LastService:     time.Now().Add(-30 * 24 * time.Hour),
				NextService:     time.Now().Add(60 * 24 * time.Hour),
				ServiceType:     "preventive",
				Odometer:        15000,
				ServiceProvider: "Abu Dhabi Motors",
			},
			CostCenter:  "CC_TRANSPORT_001",
			Department:  "Fleet Operations",
			LastUpdated: time.Now(),
		},
	}
	
	return vehicles, nil
}

func (s *SAPAdapter) SyncTrips() ([]TripRecord, error) {
	log.Printf("📊 Syncing trips from SAP")
	
	// Mock data
	trips := []TripRecord{
		{
			TripID:      "TRIP_001",
			ERPTripID:   "SAP_TRIP_001",
			VehicleID:   "VEH_001",
			DriverID:    "DRIVER_001",
			Origin:      Location{Latitude: 24.4539, Longitude: 54.3773, Address: "Abu Dhabi Central", Zone: "Zone_A"},
			Destination: Location{Latitude: 24.4648, Longitude: 54.3618, Address: "Abu Dhabi Mall", Zone: "Zone_B"},
			Status:      "scheduled",
			ScheduledStart: time.Now().Add(2 * time.Hour),
			ScheduledEnd:   time.Now().Add(4 * time.Hour),
			CargoInfo: CargoInfo{
				CargoType:   "general",
				Weight:      500,
				Volume:      10,
				Value:       5000,
				Special:     false,
				Temperature: "ambient",
			},
			CostCenter:  "CC_LOGISTICS_001",
			Priority:    "medium",
			LastUpdated: time.Now(),
		},
	}
	
	return trips, nil
}

func (s *SAPAdapter) SyncInventory() ([]InventoryRecord, error) {
	log.Printf("📊 Syncing inventory from SAP")
	
	// Mock data
	inventory := []InventoryRecord{
		{
			ItemID:       "ITEM_001",
			ERPItemID:    "SAP_MAT_001",
			ItemName:     "Brake Pads",
			Category:     "Vehicle Parts",
			Quantity:     50,
			Unit:         "pieces",
			Location:     "Abu Dhabi Warehouse",
			CostPerUnit:  150.0,
			ReorderLevel: 10,
			Supplier:     "Auto Parts UAE",
			LastUpdated:  time.Now(),
		},
	}
	
	return inventory, nil
}

func (s *SAPAdapter) SyncWorkOrders() ([]WorkOrderRecord, error) {
	log.Printf("📊 Syncing work orders from SAP")
	
	// Mock data
	workOrders := []WorkOrderRecord{
		{
			WorkOrderID:    "WO_001",
			ERPWorkOrderID: "SAP_WO_001",
			VehicleID:      "VEH_001",
			WorkOrderType:  "preventive_maintenance",
			Priority:       "medium",
			Status:         "scheduled",
			Description:    "Routine maintenance - brake inspection",
			AssignedTo:     "TECH_001",
			ScheduledDate:  time.Now().Add(7 * 24 * time.Hour),
			EstimatedCost:  800.0,
			ActualCost:     0.0,
			PartsRequired: []PartRequirement{
				{
					PartID:   "ITEM_001",
					PartName: "Brake Pads",
					Quantity: 4,
					UnitCost: 150.0,
					Supplier: "Auto Parts UAE",
					LeadTime: 3,
				},
			},
			LaborHours:  4.0,
			LastUpdated: time.Now(),
		},
	}
	
	return workOrders, nil
}

func (s *SAPAdapter) CreateWorkOrder(order WorkOrderRequest) (*WorkOrderResponse, error) {
	log.Printf("📝 Creating work order in SAP for vehicle %s", order.VehicleID)
	
	// Mock response
	response := &WorkOrderResponse{
		WorkOrderID:         fmt.Sprintf("WO_%d", time.Now().Unix()),
		ERPWorkOrderID:      fmt.Sprintf("SAP_WO_%d", time.Now().Unix()),
		Status:              "created",
		CreatedAt:           time.Now(),
		EstimatedCompletion: order.ScheduledDate.Add(4 * time.Hour),
	}
	
	return response, nil
}

func (s *SAPAdapter) UpdateVehicleStatus(vehicleID string, status VehicleStatusUpdate) error {
	log.Printf("🔄 Updating vehicle status in SAP: %s", vehicleID)
	return nil
}

func (s *SAPAdapter) GetHealthStatus() AdapterHealth {
	return AdapterHealth{
		AdapterName:  "SAP",
		Status:       "healthy",
		LastSync:     time.Now(),
		ErrorCount:   0,
		RecordCount:  100,
		ResponseTime: 250.0,
	}
}

// Oracle WMS Adapter Implementation
type OracleWMSAdapter struct {
	config   Config
	client   *http.Client
	baseURL  string
	username string
	password string
}

func NewOracleWMSAdapter(config Config) *OracleWMSAdapter {
	return &OracleWMSAdapter{
		config:   config,
		client:   &http.Client{Timeout: 30 * time.Second},
		baseURL:  config.OracleWMSURL,
		username: config.OracleUsername,
		password: config.OraclePassword,
	}
}

func (o *OracleWMSAdapter) Connect() error {
	log.Printf("🔗 Connecting to Oracle WMS at %s", o.baseURL)
	return nil
}

func (o *OracleWMSAdapter) Disconnect() error {
	log.Printf("🔌 Disconnecting from Oracle WMS")
	return nil
}

func (o *OracleWMSAdapter) SyncVehicles() ([]VehicleRecord, error) {
	log.Printf("📊 Syncing vehicles from Oracle WMS")
	return []VehicleRecord{}, nil
}

func (o *OracleWMSAdapter) SyncTrips() ([]TripRecord, error) {
	log.Printf("📊 Syncing trips from Oracle WMS")
	return []TripRecord{}, nil
}

func (o *OracleWMSAdapter) SyncInventory() ([]InventoryRecord, error) {
	log.Printf("📊 Syncing inventory from Oracle WMS")
	
	// Mock warehouse inventory data
	inventory := []InventoryRecord{
		{
			ItemID:       "WMS_ITEM_001",
			ERPItemID:    "ORA_MAT_001",
			ItemName:     "Engine Oil",
			Category:     "Fluids",
			Quantity:     200,
			Unit:         "liters",
			Location:     "Abu Dhabi Distribution Center",
			CostPerUnit:  25.0,
			ReorderLevel: 50,
			Supplier:     "Lubricants UAE",
			LastUpdated:  time.Now(),
		},
	}
	
	return inventory, nil
}

func (o *OracleWMSAdapter) SyncWorkOrders() ([]WorkOrderRecord, error) {
	log.Printf("📊 Syncing work orders from Oracle WMS")
	return []WorkOrderRecord{}, nil
}

func (o *OracleWMSAdapter) CreateWorkOrder(order WorkOrderRequest) (*WorkOrderResponse, error) {
	log.Printf("📝 Creating work order in Oracle WMS for vehicle %s", order.VehicleID)
	
	response := &WorkOrderResponse{
		WorkOrderID:         fmt.Sprintf("ORA_WO_%d", time.Now().Unix()),
		ERPWorkOrderID:      fmt.Sprintf("ORA_WMS_%d", time.Now().Unix()),
		Status:              "created",
		CreatedAt:           time.Now(),
		EstimatedCompletion: order.ScheduledDate.Add(6 * time.Hour),
	}
	
	return response, nil
}

func (o *OracleWMSAdapter) UpdateVehicleStatus(vehicleID string, status VehicleStatusUpdate) error {
	log.Printf("🔄 Updating vehicle status in Oracle WMS: %s", vehicleID)
	return nil
}

func (o *OracleWMSAdapter) GetHealthStatus() AdapterHealth {
	return AdapterHealth{
		AdapterName:  "Oracle WMS",
		Status:       "healthy",
		LastSync:     time.Now(),
		ErrorCount:   0,
		RecordCount:  75,
		ResponseTime: 180.0,
	}
}

func loadConfig() Config {
	return Config{
		Port:            getEnvInt("PORT", 8087),
		PostgresURL:     getEnv("POSTGRES_URL", "postgres://atlasmesh_dev:password@localhost:5432/atlasmesh_fleet_dev?sslmode=disable"),
		RedisURL:        getEnv("REDIS_URL", "redis://localhost:6379/0"),
		SAPEndpoint:     getEnv("SAP_ENDPOINT", "https://sap.company.ae/api"),
		SAPUsername:     getEnv("SAP_USERNAME", ""),
		SAPPassword:     getEnv("SAP_PASSWORD", ""),
		OracleWMSURL:    getEnv("ORACLE_WMS_URL", "https://wms.company.ae/api"),
		OracleUsername:  getEnv("ORACLE_USERNAME", ""),
		OraclePassword:  getEnv("ORACLE_PASSWORD", ""),
		CustomERPURL:    getEnv("CUSTOM_ERP_URL", "https://erp.company.ae/api"),
		CustomERPToken:  getEnv("CUSTOM_ERP_TOKEN", ""),
		LogLevel:        getEnv("LOG_LEVEL", "info"),
		AbuDhabiMode:    getEnvBool("ABU_DHABI_MODE", true),
		SyncInterval:    getEnvInt("SYNC_INTERVAL_MINUTES", 15),
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
		SyncOperations: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_erp_sync_operations_total",
				Help: "Total number of ERP sync operations",
			},
			[]string{"adapter", "operation", "status"},
		),
		SyncLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_erp_sync_latency_seconds",
				Help: "ERP sync operation latency",
				Buckets: []float64{0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0, 30.0},
			},
			[]string{"adapter", "operation"},
		),
		SyncErrors: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_erp_sync_errors_total",
				Help: "Total number of ERP sync errors",
			},
			[]string{"adapter", "error_type"},
		),
		DataRecords: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_erp_data_records",
				Help: "Number of records synchronized from ERP systems",
			},
			[]string{"adapter", "record_type"},
		),
		AdapterStatus: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_erp_adapter_status",
				Help: "ERP adapter health status (1=healthy, 0=unhealthy)",
			},
			[]string{"adapter"},
		),
	}
}

func main() {
	config := loadConfig()
	metrics := initializeMetrics()
	tracer := otel.Tracer("erp-integration-service")

	// Register metrics
	prometheus.MustRegister(
		metrics.SyncOperations,
		metrics.SyncLatency,
		metrics.SyncErrors,
		metrics.DataRecords,
		metrics.AdapterStatus,
	)

	// Initialize database connection
	postgres, err := sql.Open("postgres", config.PostgresURL)
	if err != nil {
		log.Fatalf("Failed to connect to PostgreSQL: %v", err)
	}
	defer postgres.Close()

	if err := postgres.Ping(); err != nil {
		log.Fatalf("Failed to ping PostgreSQL: %v", err)
	}

	// Initialize Redis connection
	opt, err := redis.ParseURL(config.RedisURL)
	if err != nil {
		log.Fatalf("Failed to parse Redis URL: %v", err)
	}
	redisClient := redis.NewClient(opt)

	ctx := context.Background()
	if err := redisClient.Ping(ctx).Err(); err != nil {
		log.Fatalf("Failed to connect to Redis: %v", err)
	}

	// Initialize ERP adapters
	adapters := make(map[string]ERPAdapter)
	
	if config.SAPEndpoint != "" {
		sapAdapter := NewSAPAdapter(config)
		adapters["sap"] = sapAdapter
	}
	
	if config.OracleWMSURL != "" {
		oracleAdapter := NewOracleWMSAdapter(config)
		adapters["oracle_wms"] = oracleAdapter
	}

	service := &ERPIntegrationService{
		config:   config,
		tracer:   tracer,
		metrics:  metrics,
		postgres: postgres,
		redis:    redisClient,
		adapters: adapters,
	}

	// Connect to all adapters
	for name, adapter := range service.adapters {
		if err := adapter.Connect(); err != nil {
			log.Printf("❌ Failed to connect to %s: %v", name, err)
		} else {
			log.Printf("✅ Connected to %s adapter", name)
		}
	}

	// Setup HTTP server
	router := mux.NewRouter()
	service.setupRoutes(router)

	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.Port),
		Handler: router,
	}

	// Start background services
	go service.startSyncScheduler()
	go service.startMetricsCollection()
	go service.startHealthMonitor()

	// Start server
	go func() {
		log.Printf("🚀 AtlasMesh ERP Integration starting on port %d", config.Port)
		if config.AbuDhabiMode {
			log.Printf("🇦🇪 Abu Dhabi mode enabled - UAE ERP integration active")
		}
		log.Printf("📊 Configured adapters: %v", getAdapterNames(adapters))
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Server failed: %v", err)
		}
	}()

	// Graceful shutdown
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c

	log.Println("🛑 Shutting down ERP Integration service...")
	
	// Disconnect from all adapters
	for name, adapter := range service.adapters {
		if err := adapter.Disconnect(); err != nil {
			log.Printf("❌ Failed to disconnect from %s: %v", name, err)
		}
	}
	
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	server.Shutdown(ctx)
}

func (s *ERPIntegrationService) setupRoutes(router *mux.Router) {
	api := router.PathPrefix("/api/v1").Subrouter()

	// Data synchronization endpoints
	api.HandleFunc("/sync/vehicles", s.syncVehicles).Methods("POST")
	api.HandleFunc("/sync/trips", s.syncTrips).Methods("POST")
	api.HandleFunc("/sync/inventory", s.syncInventory).Methods("POST")
	api.HandleFunc("/sync/work-orders", s.syncWorkOrders).Methods("POST")
	api.HandleFunc("/sync/all", s.syncAll).Methods("POST")

	// Work order management
	api.HandleFunc("/work-orders", s.createWorkOrder).Methods("POST")
	api.HandleFunc("/work-orders/{workOrderId}", s.getWorkOrder).Methods("GET")
	api.HandleFunc("/work-orders/{workOrderId}/status", s.updateWorkOrderStatus).Methods("PUT")

	// Vehicle status updates
	api.HandleFunc("/vehicles/{vehicleId}/status", s.updateVehicleStatus).Methods("PUT")
	api.HandleFunc("/vehicles/{vehicleId}/erp-data", s.getVehicleERPData).Methods("GET")

	// Inventory management
	api.HandleFunc("/inventory", s.getInventory).Methods("GET")
	api.HandleFunc("/inventory/{itemId}", s.getInventoryItem).Methods("GET")
	api.HandleFunc("/inventory/reorder", s.checkReorderLevels).Methods("GET")

	// Adapter management
	api.HandleFunc("/adapters", s.getAdapterStatus).Methods("GET")
	api.HandleFunc("/adapters/{adapterName}/health", s.getAdapterHealth).Methods("GET")
	api.HandleFunc("/adapters/{adapterName}/reconnect", s.reconnectAdapter).Methods("POST")

	// Health and metrics
	api.HandleFunc("/health", s.healthCheck).Methods("GET")
	router.Handle("/metrics", promhttp.Handler())
}

// Sync Handlers

func (s *ERPIntegrationService) syncVehicles(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "syncVehicles")
	defer span.End()

	start := time.Now()
	results := make(map[string]interface{})

	for name, adapter := range s.adapters {
		adapterStart := time.Now()
		
		vehicles, err := adapter.SyncVehicles()
		if err != nil {
			s.metrics.SyncErrors.WithLabelValues(name, "sync_error").Inc()
			results[name] = map[string]interface{}{
				"status": "error",
				"error":  err.Error(),
			}
			continue
		}

		// Store vehicles in database
		count, err := s.storeVehicles(ctx, vehicles, name)
		if err != nil {
			s.metrics.SyncErrors.WithLabelValues(name, "storage_error").Inc()
			results[name] = map[string]interface{}{
				"status": "error",
				"error":  err.Error(),
			}
			continue
		}

		s.metrics.SyncOperations.WithLabelValues(name, "vehicles", "success").Inc()
		s.metrics.SyncLatency.WithLabelValues(name, "vehicles").Observe(time.Since(adapterStart).Seconds())
		s.metrics.DataRecords.WithLabelValues(name, "vehicles").Set(float64(count))

		results[name] = map[string]interface{}{
			"status":        "success",
			"records_synced": count,
			"duration_ms":   time.Since(adapterStart).Milliseconds(),
		}

		log.Printf("✅ Synced %d vehicles from %s", count, name)
	}

	response := map[string]interface{}{
		"operation":    "sync_vehicles",
		"results":      results,
		"total_duration_ms": time.Since(start).Milliseconds(),
		"timestamp":    time.Now(),
	}

	s.respondJSON(w, response)
}

func (s *ERPIntegrationService) storeVehicles(ctx context.Context, vehicles []VehicleRecord, adapter string) (int, error) {
	// Store vehicles in PostgreSQL
	query := `
		INSERT INTO erp_vehicles (
			vehicle_id, asset_tag, erp_vehicle_id, status, location, assigned_driver,
			last_service, next_service, cost_center, department, adapter_name, 
			metadata, last_updated
		) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13)
		ON CONFLICT (vehicle_id, adapter_name) 
		DO UPDATE SET
			status = EXCLUDED.status,
			location = EXCLUDED.location,
			assigned_driver = EXCLUDED.assigned_driver,
			last_service = EXCLUDED.last_service,
			next_service = EXCLUDED.next_service,
			metadata = EXCLUDED.metadata,
			last_updated = EXCLUDED.last_updated`

	count := 0
	for _, vehicle := range vehicles {
		metadataJSON, _ := json.Marshal(vehicle.Metadata)
		
		_, err := s.postgres.ExecContext(ctx, query,
			vehicle.VehicleID, vehicle.AssetTag, vehicle.ERPVehicleID,
			vehicle.Status, vehicle.Location, vehicle.AssignedDriver,
			vehicle.MaintenanceInfo.LastService, vehicle.MaintenanceInfo.NextService,
			vehicle.CostCenter, vehicle.Department, adapter,
			metadataJSON, vehicle.LastUpdated,
		)
		
		if err != nil {
			return count, fmt.Errorf("failed to store vehicle %s: %w", vehicle.VehicleID, err)
		}
		count++
	}

	return count, nil
}

func (s *ERPIntegrationService) syncAll(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "syncAll")
	defer span.End()

	start := time.Now()
	results := make(map[string]interface{})

	// Sync all data types from all adapters
	syncOperations := []struct {
		name string
		fn   func(string, ERPAdapter) (int, error)
	}{
		{"vehicles", s.syncAdapterVehicles},
		{"trips", s.syncAdapterTrips},
		{"inventory", s.syncAdapterInventory},
		{"work_orders", s.syncAdapterWorkOrders},
	}

	for adapterName, adapter := range s.adapters {
		adapterResults := make(map[string]interface{})
		
		for _, op := range syncOperations {
			opStart := time.Now()
			count, err := op.fn(adapterName, adapter)
			
			if err != nil {
				s.metrics.SyncErrors.WithLabelValues(adapterName, "sync_error").Inc()
				adapterResults[op.name] = map[string]interface{}{
					"status": "error",
					"error":  err.Error(),
				}
			} else {
				s.metrics.SyncOperations.WithLabelValues(adapterName, op.name, "success").Inc()
				s.metrics.SyncLatency.WithLabelValues(adapterName, op.name).Observe(time.Since(opStart).Seconds())
				s.metrics.DataRecords.WithLabelValues(adapterName, op.name).Set(float64(count))
				
				adapterResults[op.name] = map[string]interface{}{
					"status":        "success",
					"records_synced": count,
					"duration_ms":   time.Since(opStart).Milliseconds(),
				}
			}
		}
		
		results[adapterName] = adapterResults
	}

	response := map[string]interface{}{
		"operation":         "sync_all",
		"results":           results,
		"total_duration_ms": time.Since(start).Milliseconds(),
		"timestamp":         time.Now(),
	}

	log.Printf("🔄 Completed full ERP sync in %dms", time.Since(start).Milliseconds())
	s.respondJSON(w, response)
}

func (s *ERPIntegrationService) syncAdapterVehicles(adapterName string, adapter ERPAdapter) (int, error) {
	vehicles, err := adapter.SyncVehicles()
	if err != nil {
		return 0, err
	}
	return s.storeVehicles(context.Background(), vehicles, adapterName)
}

func (s *ERPIntegrationService) syncAdapterTrips(adapterName string, adapter ERPAdapter) (int, error) {
	trips, err := adapter.SyncTrips()
	if err != nil {
		return 0, err
	}
	// Implement trip storage
	return len(trips), nil
}

func (s *ERPIntegrationService) syncAdapterInventory(adapterName string, adapter ERPAdapter) (int, error) {
	inventory, err := adapter.SyncInventory()
	if err != nil {
		return 0, err
	}
	// Implement inventory storage
	return len(inventory), nil
}

func (s *ERPIntegrationService) syncAdapterWorkOrders(adapterName string, adapter ERPAdapter) (int, error) {
	workOrders, err := adapter.SyncWorkOrders()
	if err != nil {
		return 0, err
	}
	// Implement work order storage
	return len(workOrders), nil
}

// Background Services

func (s *ERPIntegrationService) startSyncScheduler() {
	ticker := time.NewTicker(time.Duration(s.config.SyncInterval) * time.Minute)
	defer ticker.Stop()

	log.Printf("📅 Starting ERP sync scheduler (interval: %d minutes)", s.config.SyncInterval)

	for range ticker.C {
		s.performScheduledSync()
	}
}

func (s *ERPIntegrationService) performScheduledSync() {
	log.Printf("🔄 Starting scheduled ERP sync...")
	
	ctx := context.Background()
	
	for adapterName, adapter := range s.adapters {
		// Check adapter health before syncing
		health := adapter.GetHealthStatus()
		if health.Status != "healthy" {
			log.Printf("⚠️ Skipping sync for unhealthy adapter: %s", adapterName)
			continue
		}

		// Perform sync operations
		s.syncAdapterVehicles(adapterName, adapter)
		s.syncAdapterTrips(adapterName, adapter)
		s.syncAdapterInventory(adapterName, adapter)
		s.syncAdapterWorkOrders(adapterName, adapter)
	}
	
	log.Printf("✅ Scheduled ERP sync completed")
}

func (s *ERPIntegrationService) startHealthMonitor() {
	ticker := time.NewTicker(2 * time.Minute)
	defer ticker.Stop()

	for range ticker.C {
		s.monitorAdapterHealth()
	}
}

func (s *ERPIntegrationService) monitorAdapterHealth() {
	for name, adapter := range s.adapters {
		health := adapter.GetHealthStatus()
		
		if health.Status == "healthy" {
			s.metrics.AdapterStatus.WithLabelValues(name).Set(1)
		} else {
			s.metrics.AdapterStatus.WithLabelValues(name).Set(0)
			log.Printf("❌ Adapter %s is unhealthy: %d errors", name, health.ErrorCount)
		}
	}
}

func (s *ERPIntegrationService) startMetricsCollection() {
	ticker := time.NewTicker(60 * time.Second)
	defer ticker.Stop()

	for range ticker.C {
		s.collectSystemMetrics()
	}
}

func (s *ERPIntegrationService) collectSystemMetrics() {
	// Collect additional system metrics
	log.Printf("📊 Collecting ERP integration metrics...")
}

// Utility Functions

func getAdapterNames(adapters map[string]ERPAdapter) []string {
	names := make([]string, 0, len(adapters))
	for name := range adapters {
		names = append(names, name)
	}
	return names
}

func (s *ERPIntegrationService) healthCheck(w http.ResponseWriter, r *http.Request) {
	health := map[string]interface{}{
		"status":      "healthy",
		"timestamp":   time.Now(),
		"service":     "erp-integration",
		"version":     "1.0.0",
		"region":      "abu-dhabi",
		"adapters":    make(map[string]interface{}),
	}

	// Check database
	if err := s.postgres.Ping(); err != nil {
		health["status"] = "unhealthy"
		health["postgres"] = "failed"
	} else {
		health["postgres"] = "healthy"
	}

	// Check Redis
	ctx := context.Background()
	if err := s.redis.Ping(ctx).Err(); err != nil {
		health["status"] = "unhealthy"
		health["redis"] = "failed"
	} else {
		health["redis"] = "healthy"
	}

	// Check adapters
	adapterHealth := make(map[string]interface{})
	for name, adapter := range s.adapters {
		adapterStatus := adapter.GetHealthStatus()
		adapterHealth[name] = map[string]interface{}{
			"status":        adapterStatus.Status,
			"last_sync":     adapterStatus.LastSync,
			"error_count":   adapterStatus.ErrorCount,
			"record_count":  adapterStatus.RecordCount,
			"response_time": adapterStatus.ResponseTime,
		}
		
		if adapterStatus.Status != "healthy" {
			health["status"] = "degraded"
		}
	}
	health["adapters"] = adapterHealth

	if health["status"] == "unhealthy" {
		w.WriteHeader(http.StatusServiceUnavailable)
	} else if health["status"] == "degraded" {
		w.WriteHeader(http.StatusPartialContent)
	}

	s.respondJSON(w, health)
}

func (s *ERPIntegrationService) respondJSON(w http.ResponseWriter, data interface{}) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(data)
}

func (s *ERPIntegrationService) handleError(w http.ResponseWriter, message string, err error, statusCode int) {
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

// Placeholder implementations for remaining handlers
func (s *ERPIntegrationService) syncTrips(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Trip sync not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) syncInventory(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Inventory sync not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) syncWorkOrders(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Work order sync not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) createWorkOrder(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Create work order not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) getWorkOrder(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Get work order not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) updateWorkOrderStatus(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Update work order status not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) updateVehicleStatus(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Update vehicle status not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) getVehicleERPData(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Get vehicle ERP data not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) getInventory(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Get inventory not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) getInventoryItem(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Get inventory item not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) checkReorderLevels(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Check reorder levels not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) getAdapterStatus(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Get adapter status not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) getAdapterHealth(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Get adapter health not yet implemented", nil, http.StatusNotImplemented)
}

func (s *ERPIntegrationService) reconnectAdapter(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Reconnect adapter not yet implemented", nil, http.StatusNotImplemented)
}
