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

// AtlasMesh External Integrations Service
//
// PUBLIC API: External system integration service providing UAE government APIs,
// ERP/WMS adapters, weather/map services, and third-party integrations
// Handles data synchronization, compliance reporting, and external service orchestration

// Service Configuration
type ExternalIntegrationsConfig struct {
	Port                int    `json:"port"`
	KafkaBrokers        string `json:"kafka_brokers"`
	KafkaTopic          string `json:"kafka_topic"`
	UAEGovernmentAPI    string `json:"uae_government_api"`
	WeatherAPIKey        string `json:"weather_api_key"`
	MapAPIKey           string `json:"map_api_key"`
	ERPAdapterURL       string `json:"erp_adapter_url"`
	WMSAdapterURL       string `json:"wms_adapter_url"`
	RequestTimeout      int    `json:"request_timeout"`
	RetryAttempts        int    `json:"retry_attempts"`
	CacheTTL            int    `json:"cache_ttl"`
	RateLimitPerMinute  int    `json:"rate_limit_per_minute"`
	EnableTracing        bool   `json:"enable_tracing"`
	EnableMetrics        bool   `json:"enable_metrics"`
}

// Service Metrics
type ExternalIntegrationsMetrics struct {
	RequestCount        *prometheus.CounterVec
	RequestLatency      *prometheus.HistogramVec
	IntegrationErrors   *prometheus.CounterVec
	CacheHits           *prometheus.CounterVec
	RateLimitHits       *prometheus.CounterVec
	DataSyncOperations  *prometheus.CounterVec
	ComplianceReports   *prometheus.CounterVec
}

// External Integrations Service
type ExternalIntegrationsService struct {
	config     *ExternalIntegrationsConfig
	metrics    *ExternalIntegrationsMetrics
	tracer     trace.Tracer
	kafkaWriter *kafka.Writer
	kafkaReader *kafka.Reader
	httpClient *http.Client
	cache      map[string]interface{}
	cacheMutex sync.RWMutex
	rateLimiter map[string]time.Time
	rateMutex   sync.RWMutex
}

// Data Models
type UAEGovernmentData struct {
	VehicleRegistration string    `json:"vehicle_registration"`
	LicenseNumber      string    `json:"license_number"`
	RegistrationDate   time.Time `json:"registration_date"`
	ExpiryDate         time.Time `json:"expiry_date"`
	VehicleType        string    `json:"vehicle_type"`
	OwnerName          string    `json:"owner_name"`
	ComplianceStatus   string    `json:"compliance_status"`
	LastInspection     time.Time `json:"last_inspection"`
	NextInspection     time.Time `json:"next_inspection"`
}

type WeatherData struct {
	Location      string    `json:"location"`
	Temperature   float64   `json:"temperature"`
	Humidity      float64   `json:"humidity"`
	WindSpeed     float64   `json:"wind_speed"`
	WindDirection float64   `json:"wind_direction"`
	Visibility    float64   `json:"visibility"`
	Conditions    string    `json:"conditions"`
	Timestamp     time.Time `json:"timestamp"`
	Forecast      []WeatherForecast `json:"forecast"`
}

type WeatherForecast struct {
	DateTime    time.Time `json:"datetime"`
	Temperature float64   `json:"temperature"`
	Conditions  string    `json:"conditions"`
	Precipitation float64 `json:"precipitation"`
}

type MapData struct {
	Location     string    `json:"location"`
	Latitude     float64   `json:"latitude"`
	Longitude    float64   `json:"longitude"`
	Address      string    `json:"address"`
	TrafficLevel string    `json:"traffic_level"`
	RouteOptions []Route   `json:"route_options"`
	Timestamp    time.Time `json:"timestamp"`
}

type Route struct {
	Distance    float64 `json:"distance"`
	Duration    float64 `json:"duration"`
	Instructions []string `json:"instructions"`
	Waypoints   []Waypoint `json:"waypoints"`
}

type Waypoint struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Address   string  `json:"address"`
}

type ERPData struct {
	OrderID       string    `json:"order_id"`
	CustomerID    string    `json:"customer_id"`
	OrderDate     time.Time `json:"order_date"`
	DeliveryDate  time.Time `json:"delivery_date"`
	Priority      string    `json:"priority"`
	Status        string    `json:"status"`
	Items         []ERPItem `json:"items"`
	TotalValue    float64   `json:"total_value"`
	DeliveryAddress string  `json:"delivery_address"`
}

type ERPItem struct {
	ItemID       string  `json:"item_id"`
	Description  string  `json:"description"`
	Quantity     int     `json:"quantity"`
	UnitPrice    float64 `json:"unit_price"`
	TotalPrice   float64 `json:"total_price"`
	Weight       float64 `json:"weight"`
	Dimensions   string  `json:"dimensions"`
}

type WMSData struct {
	WarehouseID   string    `json:"warehouse_id"`
	Location      string    `json:"location"`
	Inventory     []InventoryItem `json:"inventory"`
	Capacity      int       `json:"capacity"`
	Utilization   float64   `json:"utilization"`
	LastUpdated   time.Time `json:"last_updated"`
}

type InventoryItem struct {
	ItemID        string  `json:"item_id"`
	Description   string  `json:"description"`
	Quantity     int     `json:"quantity"`
	Location     string  `json:"location"`
	LastMovement time.Time `json:"last_movement"`
	Status       string  `json:"status"`
}

type ComplianceReport struct {
	ReportID      string    `json:"report_id"`
	VehicleID     string    `json:"vehicle_id"`
	ReportType    string    `json:"report_type"`
	GeneratedAt   time.Time `json:"generated_at"`
	Status        string    `json:"status"`
	ComplianceScore float64 `json:"compliance_score"`
	Violations    []Violation `json:"violations"`
	Recommendations []string `json:"recommendations"`
}

type Violation struct {
	Type        string    `json:"type"`
	Description string    `json:"description"`
	Severity    string    `json:"severity"`
	DetectedAt  time.Time `json:"detected_at"`
	Resolved    bool      `json:"resolved"`
}

func main() {
	log.Println("🚀 Starting AtlasMesh External Integrations Service...")

	// Load configuration
	config := loadConfig()
	
	// Initialize service
	service := &ExternalIntegrationsService{
		config:      config,
		metrics:     initMetrics(),
		tracer:      otel.Tracer("external-integrations-service"),
		httpClient:  &http.Client{Timeout: time.Duration(config.RequestTimeout) * time.Second},
		cache:       make(map[string]interface{}),
		rateLimiter: make(map[string]time.Time),
	}

	// Initialize Kafka
	if err := service.initKafka(); err != nil {
		log.Fatalf("❌ Failed to initialize Kafka: %v", err)
	}

	// Setup HTTP server
	router := mux.NewRouter()
	service.setupRoutes(router)

	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.Port),
		Handler: router,
	}

	// Start background services
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	go service.startDataSyncProcessor(ctx)
	go service.startComplianceReporter(ctx)
	go service.startCacheCleanup(ctx)
	go service.startRateLimitCleanup(ctx)

	// Start server
	go func() {
		log.Printf("🌐 External Integrations Service listening on port %d", config.Port)
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("❌ Server failed to start: %v", err)
		}
	}()

	// Wait for shutdown signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("🛑 Shutting down External Integrations Service...")

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

	log.Println("✅ External Integrations Service shutdown complete")
}

func loadConfig() *ExternalIntegrationsConfig {
	config := &ExternalIntegrationsConfig{
		Port:               8080,
		KafkaBrokers:       "localhost:9092",
		KafkaTopic:         "external-integrations",
		UAEGovernmentAPI:   "https://api.uae.gov.ae",
		WeatherAPIKey:      os.Getenv("WEATHER_API_KEY"),
		MapAPIKey:          os.Getenv("MAP_API_KEY"),
		ERPAdapterURL:      "http://erp-adapter:8080",
		WMSAdapterURL:      "http://wms-adapter:8080",
		RequestTimeout:     30,
		RetryAttempts:      3,
		CacheTTL:           300,
		RateLimitPerMinute: 60,
		EnableTracing:       true,
		EnableMetrics:      true,
	}

	// Override with environment variables
	if port := os.Getenv("PORT"); port != "" {
		if p, err := strconv.Atoi(port); err == nil {
			config.Port = p
		}
	}

	return config
}

func initMetrics() *ExternalIntegrationsMetrics {
	return &ExternalIntegrationsMetrics{
		RequestCount: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "external_integrations_requests_total",
				Help: "Total number of external integration requests",
			},
			[]string{"service", "endpoint", "status"},
		),
		RequestLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name:    "external_integrations_request_duration_seconds",
				Help:    "External integration request latency",
				Buckets: prometheus.DefBuckets,
			},
			[]string{"service", "endpoint"},
		),
		IntegrationErrors: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "external_integrations_errors_total",
				Help: "Total number of integration errors",
			},
			[]string{"service", "error_type"},
		),
		CacheHits: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "external_integrations_cache_hits_total",
				Help: "Total number of cache hits",
			},
			[]string{"service"},
		),
		RateLimitHits: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "external_integrations_rate_limit_hits_total",
				Help: "Total number of rate limit hits",
			},
			[]string{"service"},
		),
		DataSyncOperations: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "external_integrations_data_sync_operations_total",
				Help: "Total number of data synchronization operations",
			},
			[]string{"service", "operation"},
		),
		ComplianceReports: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "external_integrations_compliance_reports_total",
				Help: "Total number of compliance reports generated",
			},
			[]string{"report_type", "status"},
		),
	}
}

func (s *ExternalIntegrationsService) initKafka() error {
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
		GroupID: "external-integrations-service",
	})

	return nil
}

// API Routes and Handlers

func (s *ExternalIntegrationsService) setupRoutes(router *mux.Router) {
	// Health endpoints
	router.HandleFunc("/health", s.healthCheck).Methods("GET")
	router.HandleFunc("/ready", s.readinessCheck).Methods("GET")
	router.HandleFunc("/metrics", promhttp.Handler().ServeHTTP).Methods("GET")

	// External Integrations API
	api := router.PathPrefix("/api/v1").Subrouter()
	
	// UAE Government API endpoints
	api.HandleFunc("/uae/vehicle-registration/{vehicleId}", s.getVehicleRegistration).Methods("GET")
	api.HandleFunc("/uae/compliance-check/{vehicleId}", s.checkCompliance).Methods("GET")
	api.HandleFunc("/uae/generate-report", s.generateComplianceReport).Methods("POST")
	
	// Weather API endpoints
	api.HandleFunc("/weather/current/{location}", s.getCurrentWeather).Methods("GET")
	api.HandleFunc("/weather/forecast/{location}", s.getWeatherForecast).Methods("GET")
	api.HandleFunc("/weather/alerts/{location}", s.getWeatherAlerts).Methods("GET")
	
	// Map API endpoints
	api.HandleFunc("/maps/geocode/{address}", s.geocodeAddress).Methods("GET")
	api.HandleFunc("/maps/route", s.getRoute).Methods("POST")
	api.HandleFunc("/maps/traffic/{location}", s.getTrafficInfo).Methods("GET")
	
	// ERP Integration endpoints
	api.HandleFunc("/erp/orders", s.getERPOrders).Methods("GET")
	api.HandleFunc("/erp/orders/{orderId}", s.getERPOrder).Methods("GET")
	api.HandleFunc("/erp/sync", s.syncERPData).Methods("POST")
	
	// WMS Integration endpoints
	api.HandleFunc("/wms/inventory/{warehouseId}", s.getWMSInventory).Methods("GET")
	api.HandleFunc("/wms/warehouses", s.getWMSWarehouses).Methods("GET")
	api.HandleFunc("/wms/sync", s.syncWMSData).Methods("POST")
	
	// Data synchronization endpoints
	api.HandleFunc("/sync/status", s.getSyncStatus).Methods("GET")
	api.HandleFunc("/sync/trigger", s.triggerSync).Methods("POST")
	api.HandleFunc("/sync/history", s.getSyncHistory).Methods("GET")
}

func (s *ExternalIntegrationsService) healthCheck(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":    "healthy",
		"service":   "external-integrations-service",
		"timestamp": time.Now(),
	})
}

func (s *ExternalIntegrationsService) readinessCheck(w http.ResponseWriter, r *http.Request) {
	// Check external service connections
	checks := map[string]bool{
		"uae_government_api": s.checkUAEGovernmentAPI(),
		"weather_api":        s.checkWeatherAPI(),
		"map_api":           s.checkMapAPI(),
		"erp_adapter":       s.checkERPAdapter(),
		"wms_adapter":       s.checkWMSAdapter(),
	}

	allHealthy := true
	for service, healthy := range checks {
		if !healthy {
			log.Printf("⚠️ Service %s is not healthy", service)
			allHealthy = false
		}
	}

	if !allHealthy {
		w.WriteHeader(http.StatusServiceUnavailable)
		json.NewEncoder(w).Encode(map[string]interface{}{
			"status": "not_ready",
			"checks": checks,
		})
		return
	}

	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status": "ready",
		"checks": checks,
	})
}

// UAE Government API Integration

func (s *ExternalIntegrationsService) getVehicleRegistration(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getVehicleRegistration")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.RequestLatency.WithLabelValues("uae_government", "vehicle_registration").Observe(time.Since(start).Seconds())
	}()

	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]

	// Check rate limit
	if !s.checkRateLimit("uae_government") {
		s.metrics.RateLimitHits.WithLabelValues("uae_government").Inc()
		http.Error(w, "Rate limit exceeded", http.StatusTooManyRequests)
		return
	}

	// Check cache first
	cacheKey := fmt.Sprintf("vehicle_registration_%s", vehicleID)
	if cached, found := s.getFromCache(cacheKey); found {
		s.metrics.CacheHits.WithLabelValues("uae_government").Inc()
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(cached)
		return
	}

	// Fetch from UAE Government API
	registration, err := s.fetchVehicleRegistration(ctx, vehicleID)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("uae_government", "api_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to fetch vehicle registration: %v", err), http.StatusInternalServerError)
		return
	}

	// Cache result
	s.setCache(cacheKey, registration, time.Duration(s.config.CacheTTL)*time.Second)

	s.metrics.RequestCount.WithLabelValues("uae_government", "vehicle_registration", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(registration)
}

func (s *ExternalIntegrationsService) fetchVehicleRegistration(ctx context.Context, vehicleID string) (*UAEGovernmentData, error) {
	// Mock implementation - in production, this would call the actual UAE Government API
	registration := &UAEGovernmentData{
		VehicleRegistration: vehicleID,
		LicenseNumber:      fmt.Sprintf("UAE-%s", vehicleID),
		RegistrationDate:   time.Now().AddDate(-2, 0, 0),
		ExpiryDate:         time.Now().AddDate(1, 0, 0),
		VehicleType:        "Autonomous Vehicle",
		OwnerName:          "AtlasMesh Fleet Management",
		ComplianceStatus:   "Compliant",
		LastInspection:      time.Now().AddDate(0, -3, 0),
		NextInspection:     time.Now().AddDate(0, 9, 0),
	}

	return registration, nil
}

func (s *ExternalIntegrationsService) checkCompliance(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "checkCompliance")
	defer span.End()

	vars := mux.Vars(r)
	vehicleID := vars["vehicleId"]

	// Check compliance status
	compliance, err := s.performComplianceCheck(ctx, vehicleID)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("uae_government", "compliance_check").Inc()
		http.Error(w, fmt.Sprintf("Failed to check compliance: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.RequestCount.WithLabelValues("uae_government", "compliance_check", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(compliance)
}

func (s *ExternalIntegrationsService) performComplianceCheck(ctx context.Context, vehicleID string) (map[string]interface{}, error) {
	// Mock compliance check
	compliance := map[string]interface{}{
		"vehicle_id":        vehicleID,
		"compliance_status": "Compliant",
		"score":            95.5,
		"last_check":       time.Now(),
		"violations":       []string{},
		"recommendations":  []string{"Schedule next inspection", "Update insurance"},
	}

	return compliance, nil
}

func (s *ExternalIntegrationsService) generateComplianceReport(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "generateComplianceReport")
	defer span.End()

	var req struct {
		VehicleIDs []string `json:"vehicle_ids"`
		ReportType string   `json:"report_type"`
		Format     string   `json:"format"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request payload", http.StatusBadRequest)
		return
	}

	// Generate compliance report
	report, err := s.createComplianceReport(ctx, req.VehicleIDs, req.ReportType, req.Format)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("uae_government", "report_generation").Inc()
		http.Error(w, fmt.Sprintf("Failed to generate report: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.ComplianceReports.WithLabelValues(req.ReportType, "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(report)
}

func (s *ExternalIntegrationsService) createComplianceReport(ctx context.Context, vehicleIDs []string, reportType, format string) (*ComplianceReport, error) {
	report := &ComplianceReport{
		ReportID:        fmt.Sprintf("COMP-%d", time.Now().Unix()),
		VehicleID:       vehicleIDs[0], // For simplicity, using first vehicle
		ReportType:      reportType,
		GeneratedAt:     time.Now(),
		Status:          "completed",
		ComplianceScore: 95.5,
		Violations:      []Violation{},
		Recommendations: []string{"Maintain current compliance standards"},
	}

	return report, nil
}

// Weather API Integration

func (s *ExternalIntegrationsService) getCurrentWeather(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getCurrentWeather")
	defer span.End()

	vars := mux.Vars(r)
	location := vars["location"]

	// Check rate limit
	if !s.checkRateLimit("weather") {
		s.metrics.RateLimitHits.WithLabelValues("weather").Inc()
		http.Error(w, "Rate limit exceeded", http.StatusTooManyRequests)
		return
	}

	// Check cache
	cacheKey := fmt.Sprintf("weather_current_%s", location)
	if cached, found := s.getFromCache(cacheKey); found {
		s.metrics.CacheHits.WithLabelValues("weather").Inc()
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(cached)
		return
	}

	// Fetch weather data
	weather, err := s.fetchCurrentWeather(ctx, location)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("weather", "api_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to fetch weather: %v", err), http.StatusInternalServerError)
		return
	}

	// Cache result
	s.setCache(cacheKey, weather, 5*time.Minute)

	s.metrics.RequestCount.WithLabelValues("weather", "current_weather", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(weather)
}

func (s *ExternalIntegrationsService) fetchCurrentWeather(ctx context.Context, location string) (*WeatherData, error) {
	// Mock weather data - in production, this would call a real weather API
	weather := &WeatherData{
		Location:      location,
		Temperature:   32.5,
		Humidity:      65.0,
		WindSpeed:     15.2,
		WindDirection: 180.0,
		Visibility:    10.0,
		Conditions:    "Clear",
		Timestamp:     time.Now(),
		Forecast: []WeatherForecast{
			{
				DateTime:      time.Now().Add(1 * time.Hour),
				Temperature:   33.0,
				Conditions:   "Partly Cloudy",
				Precipitation: 0.0,
			},
		},
	}

	return weather, nil
}

func (s *ExternalIntegrationsService) getWeatherForecast(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getWeatherForecast")
	defer span.End()

	vars := mux.Vars(r)
	location := vars["location"]

	// Fetch forecast
	forecast, err := s.fetchWeatherForecast(ctx, location)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("weather", "forecast_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to fetch forecast: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.RequestCount.WithLabelValues("weather", "forecast", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(forecast)
}

func (s *ExternalIntegrationsService) fetchWeatherForecast(ctx context.Context, location string) ([]WeatherForecast, error) {
	// Mock forecast data
	forecast := []WeatherForecast{
		{
			DateTime:      time.Now().Add(1 * time.Hour),
			Temperature:   33.0,
			Conditions:   "Partly Cloudy",
			Precipitation: 0.0,
		},
		{
			DateTime:      time.Now().Add(2 * time.Hour),
			Temperature:   32.0,
			Conditions:   "Clear",
			Precipitation: 0.0,
		},
	}

	return forecast, nil
}

func (s *ExternalIntegrationsService) getWeatherAlerts(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getWeatherAlerts")
	defer span.End()

	vars := mux.Vars(r)
	location := vars["location"]

	// Fetch weather alerts
	alerts, err := s.fetchWeatherAlerts(ctx, location)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("weather", "alerts_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to fetch alerts: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.RequestCount.WithLabelValues("weather", "alerts", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(alerts)
}

func (s *ExternalIntegrationsService) fetchWeatherAlerts(ctx context.Context, location string) ([]map[string]interface{}, error) {
	// Mock alerts data
	alerts := []map[string]interface{}{
		{
			"type":        "heat_warning",
			"severity":    "moderate",
			"description": "High temperature expected",
			"valid_until": time.Now().Add(24 * time.Hour),
		},
	}

	return alerts, nil
}

// Map API Integration

func (s *ExternalIntegrationsService) geocodeAddress(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "geocodeAddress")
	defer span.End()

	vars := mux.Vars(r)
	address := vars["address"]

	// Geocode address
	coordinates, err := s.performGeocoding(ctx, address)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("maps", "geocoding_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to geocode address: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.RequestCount.WithLabelValues("maps", "geocoding", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(coordinates)
}

func (s *ExternalIntegrationsService) performGeocoding(ctx context.Context, address string) (*MapData, error) {
	// Mock geocoding - in production, this would call a real map API
	coordinates := &MapData{
		Location:     address,
		Latitude:     25.2048,
		Longitude:    55.2708,
		Address:      address,
		TrafficLevel: "moderate",
		Timestamp:    time.Now(),
	}

	return coordinates, nil
}

func (s *ExternalIntegrationsService) getRoute(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getRoute")
	defer span.End()

	var req struct {
		Origin      string `json:"origin"`
		Destination string `json:"destination"`
		Mode        string `json:"mode"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request payload", http.StatusBadRequest)
		return
	}

	// Get route
	route, err := s.calculateRoute(ctx, req.Origin, req.Destination, req.Mode)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("maps", "routing_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to calculate route: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.RequestCount.WithLabelValues("maps", "routing", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(route)
}

func (s *ExternalIntegrationsService) calculateRoute(ctx context.Context, origin, destination, mode string) (*Route, error) {
	// Mock route calculation
	route := &Route{
		Distance: 15.5,
		Duration: 25.0,
		Instructions: []string{
			"Start at origin",
			"Turn right onto main street",
			"Continue for 10 km",
			"Turn left at destination",
		},
		Waypoints: []Waypoint{
			{Latitude: 25.2048, Longitude: 55.2708, Address: origin},
			{Latitude: 25.2148, Longitude: 55.2808, Address: destination},
		},
	}

	return route, nil
}

func (s *ExternalIntegrationsService) getTrafficInfo(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getTrafficInfo")
	defer span.End()

	vars := mux.Vars(r)
	location := vars["location"]

	// Get traffic information
	traffic, err := s.fetchTrafficInfo(ctx, location)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("maps", "traffic_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to fetch traffic info: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.RequestCount.WithLabelValues("maps", "traffic", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(traffic)
}

func (s *ExternalIntegrationsService) fetchTrafficInfo(ctx context.Context, location string) (map[string]interface{}, error) {
	// Mock traffic data
	traffic := map[string]interface{}{
		"location":      location,
		"traffic_level": "moderate",
		"congestion":    0.3,
		"incidents":     []string{},
		"timestamp":     time.Now(),
	}

	return traffic, nil
}

// ERP Integration

func (s *ExternalIntegrationsService) getERPOrders(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getERPOrders")
	defer span.End()

	// Fetch ERP orders
	orders, err := s.fetchERPOrders(ctx)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("erp", "orders_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to fetch ERP orders: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.RequestCount.WithLabelValues("erp", "orders", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(orders)
}

func (s *ExternalIntegrationsService) fetchERPOrders(ctx context.Context) ([]ERPData, error) {
	// Mock ERP orders
	orders := []ERPData{
		{
			OrderID:        "ORD-001",
			CustomerID:     "CUST-001",
			OrderDate:      time.Now().AddDate(0, 0, -1),
			DeliveryDate:   time.Now().AddDate(0, 0, 1),
			Priority:       "high",
			Status:         "pending",
			TotalValue:     1500.0,
			DeliveryAddress: "Dubai, UAE",
			Items: []ERPItem{
				{
					ItemID:      "ITEM-001",
					Description: "Electronics",
					Quantity:    5,
					UnitPrice:   300.0,
					TotalPrice:  1500.0,
					Weight:      2.5,
					Dimensions:  "30x20x15 cm",
				},
			},
		},
	}

	return orders, nil
}

func (s *ExternalIntegrationsService) getERPOrder(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getERPOrder")
	defer span.End()

	vars := mux.Vars(r)
	orderID := vars["orderId"]

	// Fetch specific ERP order
	order, err := s.fetchERPOrder(ctx, orderID)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("erp", "order_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to fetch ERP order: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.RequestCount.WithLabelValues("erp", "order", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(order)
}

func (s *ExternalIntegrationsService) fetchERPOrder(ctx context.Context, orderID string) (*ERPData, error) {
	// Mock ERP order
	order := &ERPData{
		OrderID:        orderID,
		CustomerID:     "CUST-001",
		OrderDate:      time.Now().AddDate(0, 0, -1),
		DeliveryDate:   time.Now().AddDate(0, 0, 1),
		Priority:       "high",
		Status:         "pending",
		TotalValue:     1500.0,
		DeliveryAddress: "Dubai, UAE",
		Items: []ERPItem{
			{
				ItemID:      "ITEM-001",
				Description: "Electronics",
				Quantity:    5,
				UnitPrice:   300.0,
				TotalPrice:  1500.0,
				Weight:      2.5,
				Dimensions:  "30x20x15 cm",
			},
		},
	}

	return order, nil
}

func (s *ExternalIntegrationsService) syncERPData(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "syncERPData")
	defer span.End()

	// Trigger ERP data synchronization
	err := s.performERPSync(ctx)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("erp", "sync_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to sync ERP data: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.DataSyncOperations.WithLabelValues("erp", "sync").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":  "success",
		"message": "ERP data synchronization completed",
	})
}

func (s *ExternalIntegrationsService) performERPSync(ctx context.Context) error {
	// Mock ERP sync - in production, this would sync with actual ERP system
	log.Printf("📊 Syncing ERP data...")
	time.Sleep(2 * time.Second) // Simulate sync time
	log.Printf("✅ ERP data sync completed")
	return nil
}

// WMS Integration

func (s *ExternalIntegrationsService) getWMSInventory(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getWMSInventory")
	defer span.End()

	vars := mux.Vars(r)
	warehouseID := vars["warehouseId"]

	// Fetch WMS inventory
	inventory, err := s.fetchWMSInventory(ctx, warehouseID)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("wms", "inventory_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to fetch WMS inventory: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.RequestCount.WithLabelValues("wms", "inventory", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(inventory)
}

func (s *ExternalIntegrationsService) fetchWMSInventory(ctx context.Context, warehouseID string) (*WMSData, error) {
	// Mock WMS inventory
	inventory := &WMSData{
		WarehouseID: warehouseID,
		Location:    "Dubai Warehouse",
		Capacity:    1000,
		Utilization: 75.5,
		LastUpdated: time.Now(),
		Inventory: []InventoryItem{
			{
				ItemID:        "ITEM-001",
				Description:  "Electronics",
				Quantity:     100,
				Location:     "A1-B2",
				LastMovement: time.Now().AddDate(0, 0, -1),
				Status:       "available",
			},
		},
	}

	return inventory, nil
}

func (s *ExternalIntegrationsService) getWMSWarehouses(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getWMSWarehouses")
	defer span.End()

	// Fetch WMS warehouses
	warehouses, err := s.fetchWMSWarehouses(ctx)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("wms", "warehouses_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to fetch WMS warehouses: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.RequestCount.WithLabelValues("wms", "warehouses", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(warehouses)
}

func (s *ExternalIntegrationsService) fetchWMSWarehouses(ctx context.Context) ([]WMSData, error) {
	// Mock WMS warehouses
	warehouses := []WMSData{
		{
			WarehouseID: "WH-001",
			Location:   "Dubai Warehouse",
			Capacity:   1000,
			Utilization: 75.5,
			LastUpdated: time.Now(),
		},
		{
			WarehouseID: "WH-002",
			Location:   "Abu Dhabi Warehouse",
			Capacity:   800,
			Utilization: 60.0,
			LastUpdated: time.Now(),
		},
	}

	return warehouses, nil
}

func (s *ExternalIntegrationsService) syncWMSData(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "syncWMSData")
	defer span.End()

	// Trigger WMS data synchronization
	err := s.performWMSSync(ctx)
	if err != nil {
		s.metrics.IntegrationErrors.WithLabelValues("wms", "sync_error").Inc()
		http.Error(w, fmt.Sprintf("Failed to sync WMS data: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.DataSyncOperations.WithLabelValues("wms", "sync").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":  "success",
		"message": "WMS data synchronization completed",
	})
}

func (s *ExternalIntegrationsService) performWMSSync(ctx context.Context) error {
	// Mock WMS sync - in production, this would sync with actual WMS system
	log.Printf("📦 Syncing WMS data...")
	time.Sleep(2 * time.Second) // Simulate sync time
	log.Printf("✅ WMS data sync completed")
	return nil
}

// Data Synchronization

func (s *ExternalIntegrationsService) getSyncStatus(w http.ResponseWriter, r *http.Request) {
	// Get synchronization status
	status := map[string]interface{}{
		"erp_sync": map[string]interface{}{
			"status":      "completed",
			"last_sync":   time.Now().Add(-1 * time.Hour),
			"next_sync":   time.Now().Add(23 * time.Hour),
			"records_synced": 150,
		},
		"wms_sync": map[string]interface{}{
			"status":      "completed",
			"last_sync":   time.Now().Add(-30 * time.Minute),
			"next_sync":   time.Now().Add(23.5 * time.Hour),
			"records_synced": 75,
		},
		"weather_sync": map[string]interface{}{
			"status":      "completed",
			"last_sync":   time.Now().Add(-5 * time.Minute),
			"next_sync":   time.Now().Add(55 * time.Minute),
			"records_synced": 1,
		},
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(status)
}

func (s *ExternalIntegrationsService) triggerSync(w http.ResponseWriter, r *http.Request) {
	var req struct {
		Services []string `json:"services"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request payload", http.StatusBadRequest)
		return
	}

	// Trigger synchronization for specified services
	for _, service := range req.Services {
		switch service {
		case "erp":
			go s.performERPSync(context.Background())
		case "wms":
			go s.performWMSSync(context.Background())
		case "weather":
			go s.performWeatherSync(context.Background())
		}
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":  "success",
		"message": "Synchronization triggered for specified services",
	})
}

func (s *ExternalIntegrationsService) performWeatherSync(ctx context.Context) error {
	// Mock weather sync
	log.Printf("🌤️ Syncing weather data...")
	time.Sleep(1 * time.Second)
	log.Printf("✅ Weather data sync completed")
	return nil
}

func (s *ExternalIntegrationsService) getSyncHistory(w http.ResponseWriter, r *http.Request) {
	// Get synchronization history
	history := []map[string]interface{}{
		{
			"service":      "erp",
			"timestamp":    time.Now().Add(-1 * time.Hour),
			"status":       "success",
			"records_synced": 150,
			"duration":     "2.5s",
		},
		{
			"service":      "wms",
			"timestamp":    time.Now().Add(-30 * time.Minute),
			"status":       "success",
			"records_synced": 75,
			"duration":     "1.8s",
		},
		{
			"service":      "weather",
			"timestamp":    time.Now().Add(-5 * time.Minute),
			"status":       "success",
			"records_synced": 1,
			"duration":     "0.5s",
		},
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(history)
}

// Background Services

func (s *ExternalIntegrationsService) startDataSyncProcessor(ctx context.Context) {
	go func() {
		ticker := time.NewTicker(1 * time.Hour)
		defer ticker.Stop()

		for {
			select {
			case <-ticker.C:
				s.performScheduledSync()
			case <-ctx.Done():
				return
			}
		}
	}()
}

func (s *ExternalIntegrationsService) performScheduledSync() {
	log.Printf("🔄 Performing scheduled data synchronization...")
	
	// Sync ERP data
	if err := s.performERPSync(context.Background()); err != nil {
		log.Printf("❌ ERP sync failed: %v", err)
	}
	
	// Sync WMS data
	if err := s.performWMSSync(context.Background()); err != nil {
		log.Printf("❌ WMS sync failed: %v", err)
	}
	
	// Sync weather data
	if err := s.performWeatherSync(context.Background()); err != nil {
		log.Printf("❌ Weather sync failed: %v", err)
	}
	
	log.Printf("✅ Scheduled synchronization completed")
}

func (s *ExternalIntegrationsService) startComplianceReporter(ctx context.Context) {
	go func() {
		ticker := time.NewTicker(24 * time.Hour)
		defer ticker.Stop()

		for {
			select {
			case <-ticker.C:
				s.generateDailyComplianceReport()
			case <-ctx.Done():
				return
			}
		}
	}()
}

func (s *ExternalIntegrationsService) generateDailyComplianceReport() {
	log.Printf("📋 Generating daily compliance report...")
	
	// Generate compliance report for all vehicles
	report, err := s.createComplianceReport(context.Background(), []string{"vehicle-001", "vehicle-002"}, "daily", "pdf")
	if err != nil {
		log.Printf("❌ Failed to generate compliance report: %v", err)
		return
	}
	
	log.Printf("✅ Daily compliance report generated: %s", report.ReportID)
}

func (s *ExternalIntegrationsService) startCacheCleanup(ctx context.Context) {
	go func() {
		ticker := time.NewTicker(5 * time.Minute)
		defer ticker.Stop()

		for {
			select {
			case <-ticker.C:
				s.cleanupCache()
			case <-ctx.Done():
				return
			}
		}
	}()
}

func (s *ExternalIntegrationsService) cleanupCache() {
	s.cacheMutex.Lock()
	defer s.cacheMutex.Unlock()
	
	// Remove expired cache entries
	// This is a simplified implementation
	// In production, you'd use a proper cache with TTL
}

func (s *ExternalIntegrationsService) startRateLimitCleanup(ctx context.Context) {
	go func() {
		ticker := time.NewTicker(1 * time.Minute)
		defer ticker.Stop()

		for {
			select {
			case <-ticker.C:
				s.cleanupRateLimiter()
			case <-ctx.Done():
				return
			}
		}
	}()
}

func (s *ExternalIntegrationsService) cleanupRateLimiter() {
	s.rateMutex.Lock()
	defer s.rateMutex.Unlock()
	
	// Remove expired rate limit entries
	now := time.Now()
	for service, lastRequest := range s.rateLimiter {
		if now.Sub(lastRequest) > time.Minute {
			delete(s.rateLimiter, service)
		}
	}
}

// Helper Functions

func (s *ExternalIntegrationsService) checkRateLimit(service string) bool {
	s.rateMutex.Lock()
	defer s.rateMutex.Unlock()
	
	now := time.Now()
	lastRequest, exists := s.rateLimiter[service]
	
	if !exists || now.Sub(lastRequest) > time.Minute {
		s.rateLimiter[service] = now
		return true
	}
	
	return false
}

func (s *ExternalIntegrationsService) getFromCache(key string) (interface{}, bool) {
	s.cacheMutex.RLock()
	defer s.cacheMutex.RUnlock()
	
	value, exists := s.cache[key]
	return value, exists
}

func (s *ExternalIntegrationsService) setCache(key string, value interface{}, ttl time.Duration) {
	s.cacheMutex.Lock()
	defer s.cacheMutex.Unlock()
	
	s.cache[key] = value
	// In production, implement proper TTL handling
}

// Health Check Functions

func (s *ExternalIntegrationsService) checkUAEGovernmentAPI() bool {
	// Mock health check - in production, this would ping the actual API
	return true
}

func (s *ExternalIntegrationsService) checkWeatherAPI() bool {
	// Mock health check - in production, this would ping the actual API
	return true
}

func (s *ExternalIntegrationsService) checkMapAPI() bool {
	// Mock health check - in production, this would ping the actual API
	return true
}

func (s *ExternalIntegrationsService) checkERPAdapter() bool {
	// Mock health check - in production, this would ping the actual adapter
	return true
}

func (s *ExternalIntegrationsService) checkWMSAdapter() bool {
	// Mock health check - in production, this would ping the actual adapter
	return true
}
