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

	"github.com/ClickHouse/clickhouse-go/v2"
	"github.com/gorilla/mux"
	_ "github.com/lib/pq"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"github.com/redis/go-redis/v9"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// AtlasMesh Analytics Service
// Executive dashboards, operational metrics, and compliance reports
// Abu Dhabi autonomous vehicle fleet analytics

type Config struct {
	Port           int    `json:"port"`
	PostgresURL    string `json:"postgres_url"`
	ClickHouseURL  string `json:"clickhouse_url"`
	RedisURL       string `json:"redis_url"`
	LogLevel       string `json:"log_level"`
	AbuDhabiMode   bool   `json:"abu_dhabi_mode"`
	CacheTimeout   int    `json:"cache_timeout_minutes"`
	ReportSchedule string `json:"report_schedule"`
}

type AnalyticsService struct {
	config     Config
	tracer     trace.Tracer
	metrics    *Metrics
	postgres   *sql.DB
	clickhouse clickhouse.Conn
	redis      *redis.Client
}

type Metrics struct {
	ReportsGenerated   *prometheus.CounterVec
	QueryLatency       *prometheus.HistogramVec
	CacheHitRate       *prometheus.GaugeVec
	DashboardRequests  *prometheus.CounterVec
	DataFreshness      *prometheus.GaugeVec
}

// Dashboard Models
type ExecutiveDashboard struct {
	Timestamp          time.Time                `json:"timestamp"`
	FleetOverview      FleetOverviewMetrics     `json:"fleet_overview"`
	OperationalMetrics OperationalMetrics       `json:"operational_metrics"`
	FinancialMetrics   FinancialMetrics         `json:"financial_metrics"`
	SafetyMetrics      SafetyMetrics            `json:"safety_metrics"`
	ComplianceMetrics  ComplianceMetrics        `json:"compliance_metrics"`
	EnvironmentalMetrics EnvironmentalMetrics   `json:"environmental_metrics"`
	Trends             map[string][]DataPoint   `json:"trends"`
	Alerts             []Alert                  `json:"alerts"`
	Recommendations    []string                 `json:"recommendations"`
}

type FleetOverviewMetrics struct {
	TotalVehicles      int     `json:"total_vehicles"`
	ActiveVehicles     int     `json:"active_vehicles"`
	UtilizationRate    float64 `json:"utilization_rate"`
	AvailabilityRate   float64 `json:"availability_rate"`
	MaintenanceVehicles int    `json:"maintenance_vehicles"`
	OfflineVehicles    int     `json:"offline_vehicles"`
	AvgHealthScore     float64 `json:"avg_health_score"`
}

type OperationalMetrics struct {
	TotalTrips         int     `json:"total_trips"`
	CompletedTrips     int     `json:"completed_trips"`
	CancelledTrips     int     `json:"cancelled_trips"`
	AvgTripDuration    float64 `json:"avg_trip_duration_minutes"`
	TotalDistance      float64 `json:"total_distance_km"`
	AvgSpeed           float64 `json:"avg_speed_kmh"`
	OnTimePerformance  float64 `json:"on_time_performance"`
	CustomerSatisfaction float64 `json:"customer_satisfaction"`
}

type FinancialMetrics struct {
	Revenue            float64 `json:"revenue"`
	OperatingCosts     float64 `json:"operating_costs"`
	FuelCosts          float64 `json:"fuel_costs"`
	MaintenanceCosts   float64 `json:"maintenance_costs"`
	CostPerKm          float64 `json:"cost_per_km"`
	CostPerTrip        float64 `json:"cost_per_trip"`
	Profitability      float64 `json:"profitability"`
	ROI                float64 `json:"roi"`
}

type SafetyMetrics struct {
	SafetyScore        float64 `json:"safety_score"`
	Incidents          int     `json:"incidents"`
	CriticalIncidents  int     `json:"critical_incidents"`
	NearMisses         int     `json:"near_misses"`
	HumanInterventions int     `json:"human_interventions"`
	MTBFHours          float64 `json:"mtbf_hours"`
	SafetyTraining     int     `json:"safety_training_completed"`
}

type ComplianceMetrics struct {
	ComplianceScore    float64 `json:"compliance_score"`
	ActiveViolations   int     `json:"active_violations"`
	ResolvedViolations int     `json:"resolved_violations"`
	AuditReadiness     float64 `json:"audit_readiness"`
	CertificationStatus string `json:"certification_status"`
	RegulatoryUpdates  int     `json:"regulatory_updates"`
}

type EnvironmentalMetrics struct {
	CO2Emissions       float64 `json:"co2_emissions_kg"`
	FuelConsumption    float64 `json:"fuel_consumption_liters"`
	EnergyEfficiency   float64 `json:"energy_efficiency"`
	CarbonFootprint    float64 `json:"carbon_footprint"`
	SustainabilityScore float64 `json:"sustainability_score"`
	WeatherImpact      string  `json:"weather_impact"`
}

type DataPoint struct {
	Timestamp time.Time `json:"timestamp"`
	Value     float64   `json:"value"`
}

type Alert struct {
	ID          string    `json:"id"`
	Type        string    `json:"type"`
	Severity    string    `json:"severity"`
	Title       string    `json:"title"`
	Description string    `json:"description"`
	VehicleID   string    `json:"vehicle_id,omitempty"`
	Timestamp   time.Time `json:"timestamp"`
	Status      string    `json:"status"`
}

type OperationalReport struct {
	ReportID       string                 `json:"report_id"`
	GeneratedAt    time.Time              `json:"generated_at"`
	Period         string                 `json:"period"`
	FleetMetrics   FleetOperationalData   `json:"fleet_metrics"`
	VehicleMetrics []VehicleMetrics       `json:"vehicle_metrics"`
	RouteAnalysis  []RouteAnalysis        `json:"route_analysis"`
	Efficiency     EfficiencyMetrics      `json:"efficiency"`
	Recommendations []string              `json:"recommendations"`
}

type FleetOperationalData struct {
	TotalOperatingHours float64 `json:"total_operating_hours"`
	TotalMileage        float64 `json:"total_mileage"`
	FuelEfficiency      float64 `json:"fuel_efficiency"`
	UtilizationByHour   map[string]float64 `json:"utilization_by_hour"`
	StatusDistribution  map[string]int `json:"status_distribution"`
}

type VehicleMetrics struct {
	VehicleID       string  `json:"vehicle_id"`
	AssetTag        string  `json:"asset_tag"`
	OperatingHours  float64 `json:"operating_hours"`
	Mileage         float64 `json:"mileage"`
	FuelConsumption float64 `json:"fuel_consumption"`
	HealthScore     float64 `json:"health_score"`
	Incidents       int     `json:"incidents"`
	Utilization     float64 `json:"utilization"`
}

type RouteAnalysis struct {
	RouteID         string  `json:"route_id"`
	TotalTrips      int     `json:"total_trips"`
	AvgDuration     float64 `json:"avg_duration"`
	AvgDistance     float64 `json:"avg_distance"`
	CompletionRate  float64 `json:"completion_rate"`
	TrafficImpact   string  `json:"traffic_impact"`
	WeatherImpact   string  `json:"weather_impact"`
}

type EfficiencyMetrics struct {
	OverallEfficiency   float64 `json:"overall_efficiency"`
	FuelEfficiency      float64 `json:"fuel_efficiency"`
	TimeEfficiency      float64 `json:"time_efficiency"`
	RouteOptimization   float64 `json:"route_optimization"`
	MaintenanceEfficiency float64 `json:"maintenance_efficiency"`
}

func loadConfig() Config {
	return Config{
		Port:           getEnvInt("PORT", 8085),
		PostgresURL:    getEnv("POSTGRES_URL", "postgres://atlasmesh_dev:password@localhost:5432/atlasmesh_fleet_dev?sslmode=disable"),
		ClickHouseURL:  getEnv("CLICKHOUSE_URL", "localhost:9000"),
		RedisURL:       getEnv("REDIS_URL", "redis://localhost:6379/0"),
		LogLevel:       getEnv("LOG_LEVEL", "info"),
		AbuDhabiMode:   getEnvBool("ABU_DHABI_MODE", true),
		CacheTimeout:   getEnvInt("CACHE_TIMEOUT_MINUTES", 15),
		ReportSchedule: getEnv("REPORT_SCHEDULE", "0 6 * * *"), // Daily at 6 AM
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
		ReportsGenerated: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_reports_generated_total",
				Help: "Total number of reports generated",
			},
			[]string{"report_type", "status"},
		),
		QueryLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_analytics_query_latency_seconds",
				Help: "Analytics query latency",
				Buckets: []float64{0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0},
			},
			[]string{"query_type", "data_source"},
		),
		CacheHitRate: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_cache_hit_rate",
				Help: "Cache hit rate for analytics queries",
			},
			[]string{"cache_type"},
		),
		DashboardRequests: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_dashboard_requests_total",
				Help: "Total number of dashboard requests",
			},
			[]string{"dashboard_type", "status"},
		),
		DataFreshness: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_data_freshness_minutes",
				Help: "Data freshness in minutes",
			},
			[]string{"data_type"},
		),
	}
}

func main() {
	config := loadConfig()
	metrics := initializeMetrics()
	tracer := otel.Tracer("analytics-service")

	// Register metrics
	prometheus.MustRegister(
		metrics.ReportsGenerated,
		metrics.QueryLatency,
		metrics.CacheHitRate,
		metrics.DashboardRequests,
		metrics.DataFreshness,
	)

	// Initialize PostgreSQL connection
	postgres, err := sql.Open("postgres", config.PostgresURL)
	if err != nil {
		log.Fatalf("Failed to connect to PostgreSQL: %v", err)
	}
	defer postgres.Close()

	if err := postgres.Ping(); err != nil {
		log.Fatalf("Failed to ping PostgreSQL: %v", err)
	}

	// Initialize ClickHouse connection
	clickhouseConn, err := clickhouse.Open(&clickhouse.Options{
		Addr: []string{config.ClickHouseURL},
		Auth: clickhouse.Auth{
			Database: "atlasmesh",
			Username: "default",
			Password: "",
		},
	})
	if err != nil {
		log.Fatalf("Failed to connect to ClickHouse: %v", err)
	}

	if err := clickhouseConn.Ping(context.Background()); err != nil {
		log.Fatalf("Failed to ping ClickHouse: %v", err)
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

	service := &AnalyticsService{
		config:     config,
		tracer:     tracer,
		metrics:    metrics,
		postgres:   postgres,
		clickhouse: clickhouseConn,
		redis:      redisClient,
	}

	// Setup HTTP server
	router := mux.NewRouter()
	service.setupRoutes(router)

	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.Port),
		Handler: router,
	}

	// Start background services
	go service.startScheduledReports()
	go service.startMetricsCollection()
	go service.startDataFreshnessMonitor()

	// Start server
	go func() {
		log.Printf("🚀 AtlasMesh Analytics Service starting on port %d", config.Port)
		if config.AbuDhabiMode {
			log.Printf("🇦🇪 Abu Dhabi mode enabled - UAE analytics active")
		}
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Server failed: %v", err)
		}
	}()

	// Graceful shutdown
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c

	log.Println("🛑 Shutting down Analytics service...")
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	server.Shutdown(ctx)
}

func (s *AnalyticsService) setupRoutes(router *mux.Router) {
	api := router.PathPrefix("/api/v1").Subrouter()

	// Executive dashboards
	api.HandleFunc("/dashboards/executive", s.getExecutiveDashboard).Methods("GET")
	api.HandleFunc("/dashboards/operational", s.getOperationalDashboard).Methods("GET")
	api.HandleFunc("/dashboards/fleet/{fleetId}", s.getFleetDashboard).Methods("GET")

	// Reports
	api.HandleFunc("/reports/operational", s.generateOperationalReport).Methods("GET")
	api.HandleFunc("/reports/compliance", s.generateComplianceReport).Methods("GET")
	api.HandleFunc("/reports/financial", s.generateFinancialReport).Methods("GET")
	api.HandleFunc("/reports/safety", s.generateSafetyReport).Methods("GET")

	// Analytics queries
	api.HandleFunc("/analytics/fleet-utilization", s.getFleetUtilization).Methods("GET")
	api.HandleFunc("/analytics/vehicle-performance", s.getVehiclePerformance).Methods("GET")
	api.HandleFunc("/analytics/route-efficiency", s.getRouteEfficiency).Methods("GET")
	api.HandleFunc("/analytics/cost-analysis", s.getCostAnalysis).Methods("GET")

	// Real-time metrics
	api.HandleFunc("/metrics/realtime", s.getRealtimeMetrics).Methods("GET")
	api.HandleFunc("/metrics/alerts", s.getActiveAlerts).Methods("GET")
	api.HandleFunc("/metrics/kpis", s.getKPIs).Methods("GET")

	// Health and system metrics
	api.HandleFunc("/health", s.healthCheck).Methods("GET")
	router.Handle("/metrics", promhttp.Handler())

	// Abu Dhabi specific endpoints
	if s.config.AbuDhabiMode {
		api.HandleFunc("/uae/environmental-impact", s.getEnvironmentalImpact).Methods("GET")
		api.HandleFunc("/uae/regulatory-compliance", s.getRegulatoryCompliance).Methods("GET")
		api.HandleFunc("/uae/weather-analytics", s.getWeatherAnalytics).Methods("GET")
	}
}

// Executive Dashboard Handler

func (s *AnalyticsService) getExecutiveDashboard(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getExecutiveDashboard")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.QueryLatency.WithLabelValues("executive_dashboard", "combined").Observe(time.Since(start).Seconds())
		s.metrics.DashboardRequests.WithLabelValues("executive", "success").Inc()
	}()

	// Check cache first
	cacheKey := "dashboard:executive:latest"
	cached, err := s.redis.Get(ctx, cacheKey).Result()
	if err == nil {
		var dashboard ExecutiveDashboard
		if json.Unmarshal([]byte(cached), &dashboard) == nil {
			s.metrics.CacheHitRate.WithLabelValues("executive_dashboard").Set(1.0)
			s.respondJSON(w, dashboard)
			return
		}
	}

	s.metrics.CacheHitRate.WithLabelValues("executive_dashboard").Set(0.0)

	// Generate fresh dashboard
	dashboard, err := s.generateExecutiveDashboard(ctx)
	if err != nil {
		s.handleError(w, "Failed to generate executive dashboard", err, http.StatusInternalServerError)
		return
	}

	// Cache the result
	dashboardJSON, _ := json.Marshal(dashboard)
	s.redis.Set(ctx, cacheKey, dashboardJSON, time.Duration(s.config.CacheTimeout)*time.Minute)

	s.respondJSON(w, dashboard)
}

func (s *AnalyticsService) generateExecutiveDashboard(ctx context.Context) (ExecutiveDashboard, error) {
	dashboard := ExecutiveDashboard{
		Timestamp: time.Now(),
		Trends:    make(map[string][]DataPoint),
		Alerts:    []Alert{},
		Recommendations: []string{},
	}

	// Get fleet overview metrics
	fleetOverview, err := s.getFleetOverviewMetrics(ctx)
	if err != nil {
		return dashboard, fmt.Errorf("failed to get fleet overview: %w", err)
	}
	dashboard.FleetOverview = fleetOverview

	// Get operational metrics
	operationalMetrics, err := s.getOperationalMetricsData(ctx)
	if err != nil {
		return dashboard, fmt.Errorf("failed to get operational metrics: %w", err)
	}
	dashboard.OperationalMetrics = operationalMetrics

	// Get financial metrics
	financialMetrics, err := s.getFinancialMetricsData(ctx)
	if err != nil {
		return dashboard, fmt.Errorf("failed to get financial metrics: %w", err)
	}
	dashboard.FinancialMetrics = financialMetrics

	// Get safety metrics
	safetyMetrics, err := s.getSafetyMetricsData(ctx)
	if err != nil {
		return dashboard, fmt.Errorf("failed to get safety metrics: %w", err)
	}
	dashboard.SafetyMetrics = safetyMetrics

	// Get compliance metrics
	complianceMetrics, err := s.getComplianceMetricsData(ctx)
	if err != nil {
		return dashboard, fmt.Errorf("failed to get compliance metrics: %w", err)
	}
	dashboard.ComplianceMetrics = complianceMetrics

	// Get environmental metrics (Abu Dhabi specific)
	if s.config.AbuDhabiMode {
		environmentalMetrics, err := s.getEnvironmentalMetricsData(ctx)
		if err != nil {
			return dashboard, fmt.Errorf("failed to get environmental metrics: %w", err)
		}
		dashboard.EnvironmentalMetrics = environmentalMetrics
	}

	// Get trends data
	dashboard.Trends = s.getTrendsData(ctx)

	// Get active alerts
	dashboard.Alerts = s.getActiveAlertsData(ctx)

	// Generate recommendations
	dashboard.Recommendations = s.generateRecommendations(dashboard)

	return dashboard, nil
}

func (s *AnalyticsService) getFleetOverviewMetrics(ctx context.Context) (FleetOverviewMetrics, error) {
	query := `
	SELECT 
		COUNT(*) as total_vehicles,
		COUNT(CASE WHEN operational_status IN ('driving_av', 'driving_manual', 'idle') THEN 1 END) as active_vehicles,
		COUNT(CASE WHEN operational_status = 'maintenance' THEN 1 END) as maintenance_vehicles,
		COUNT(CASE WHEN operational_status = 'offline' THEN 1 END) as offline_vehicles,
		AVG(health_score) as avg_health_score
	FROM fleet_core.vehicles 
	WHERE last_seen >= NOW() - INTERVAL '1 hour'`

	var metrics FleetOverviewMetrics
	err := s.postgres.QueryRowContext(ctx, query).Scan(
		&metrics.TotalVehicles,
		&metrics.ActiveVehicles,
		&metrics.MaintenanceVehicles,
		&metrics.OfflineVehicles,
		&metrics.AvgHealthScore,
	)

	if err != nil {
		return metrics, err
	}

	// Calculate rates
	if metrics.TotalVehicles > 0 {
		metrics.UtilizationRate = float64(metrics.ActiveVehicles) / float64(metrics.TotalVehicles) * 100
		metrics.AvailabilityRate = float64(metrics.TotalVehicles-metrics.OfflineVehicles) / float64(metrics.TotalVehicles) * 100
	}

	return metrics, nil
}

func (s *AnalyticsService) getOperationalMetricsData(ctx context.Context) (OperationalMetrics, error) {
	// Query from ClickHouse for better performance on large datasets
	query := `
	SELECT 
		count() as total_records,
		uniq(vehicle_id) as active_vehicles,
		avg(speed) as avg_speed,
		sum(speed * 0.016667) as total_distance_approx -- speed * minutes to get rough distance
	FROM telemetry 
	WHERE timestamp >= now() - INTERVAL 24 HOUR`

	var totalRecords, activeVehicles int
	var avgSpeed, totalDistance float64

	err := s.clickhouse.QueryRow(ctx, query).Scan(&totalRecords, &activeVehicles, &avgSpeed, &totalDistance)
	if err != nil {
		return OperationalMetrics{}, err
	}

	// Get trip data from PostgreSQL
	tripQuery := `
	SELECT 
		COUNT(*) as total_trips,
		COUNT(CASE WHEN status = 'completed' THEN 1 END) as completed_trips,
		COUNT(CASE WHEN status = 'cancelled' THEN 1 END) as cancelled_trips,
		AVG(EXTRACT(EPOCH FROM (completed_at - started_at))/60) as avg_duration
	FROM trips 
	WHERE started_at >= NOW() - INTERVAL '24 hours'`

	var totalTrips, completedTrips, cancelledTrips int
	var avgDuration float64

	err = s.postgres.QueryRowContext(ctx, tripQuery).Scan(&totalTrips, &completedTrips, &cancelledTrips, &avgDuration)
	if err != nil {
		return OperationalMetrics{}, err
	}

	onTimePerformance := 85.0 // Placeholder - would calculate from actual trip data
	customerSatisfaction := 4.2 // Placeholder - would come from feedback system

	return OperationalMetrics{
		TotalTrips:           totalTrips,
		CompletedTrips:       completedTrips,
		CancelledTrips:       cancelledTrips,
		AvgTripDuration:      avgDuration,
		TotalDistance:        totalDistance,
		AvgSpeed:             avgSpeed,
		OnTimePerformance:    onTimePerformance,
		CustomerSatisfaction: customerSatisfaction,
	}, nil
}

func (s *AnalyticsService) getFinancialMetricsData(ctx context.Context) (FinancialMetrics, error) {
	// Simplified financial calculations - in production would integrate with financial systems
	
	// Get operational data for cost calculations
	query := `
	SELECT 
		COUNT(DISTINCT vehicle_id) as active_vehicles,
		sum(speed * 0.016667) as total_distance
	FROM telemetry 
	WHERE timestamp >= now() - INTERVAL 24 HOUR`

	var activeVehicles int
	var totalDistance float64

	err := s.clickhouse.QueryRow(ctx, query).Scan(&activeVehicles, &totalDistance)
	if err != nil {
		return FinancialMetrics{}, err
	}

	// Calculate estimated costs (simplified)
	fuelCosts := totalDistance * 0.08 * 1.5 // 8L/100km * $1.5/L (Abu Dhabi fuel price)
	maintenanceCosts := float64(activeVehicles) * 50.0 // $50 per vehicle per day
	operatingCosts := fuelCosts + maintenanceCosts + (float64(activeVehicles) * 100.0) // Additional operational costs

	// Estimated revenue (simplified)
	revenue := totalDistance * 2.5 // $2.5 per km

	costPerKm := 0.0
	costPerTrip := 0.0
	if totalDistance > 0 {
		costPerKm = operatingCosts / totalDistance
	}

	profitability := revenue - operatingCosts
	roi := 0.0
	if operatingCosts > 0 {
		roi = (profitability / operatingCosts) * 100
	}

	return FinancialMetrics{
		Revenue:          revenue,
		OperatingCosts:   operatingCosts,
		FuelCosts:        fuelCosts,
		MaintenanceCosts: maintenanceCosts,
		CostPerKm:        costPerKm,
		CostPerTrip:      costPerTrip,
		Profitability:    profitability,
		ROI:              roi,
	}, nil
}

func (s *AnalyticsService) getSafetyMetricsData(ctx context.Context) (SafetyMetrics, error) {
	query := `
	SELECT 
		COUNT(*) as total_incidents,
		COUNT(CASE WHEN severity IN ('critical', 'high') THEN 1 END) as critical_incidents
	FROM fleet_core.vehicle_alerts 
	WHERE category = 'safety' 
	AND occurred_at >= NOW() - INTERVAL '24 hours'`

	var totalIncidents, criticalIncidents int
	err := s.postgres.QueryRowContext(ctx, query).Scan(&totalIncidents, &criticalIncidents)
	if err != nil {
		return SafetyMetrics{}, err
	}

	// Calculate safety score (simplified)
	safetyScore := 100.0
	if totalIncidents > 0 {
		safetyScore = math.Max(0, 100.0 - float64(totalIncidents*5) - float64(criticalIncidents*15))
	}

	return SafetyMetrics{
		SafetyScore:        safetyScore,
		Incidents:          totalIncidents,
		CriticalIncidents:  criticalIncidents,
		NearMisses:         totalIncidents / 3, // Estimated
		HumanInterventions: totalIncidents / 2, // Estimated
		MTBFHours:          168.0, // Placeholder - Mean Time Between Failures
		SafetyTraining:     95,    // Placeholder - % of staff with current training
	}, nil
}

func (s *AnalyticsService) getComplianceMetricsData(ctx context.Context) (ComplianceMetrics, error) {
	query := `
	SELECT 
		COUNT(*) as total_violations,
		COUNT(CASE WHEN status = 'active' THEN 1 END) as active_violations,
		COUNT(CASE WHEN status = 'resolved' THEN 1 END) as resolved_violations
	FROM fleet_core.vehicle_alerts 
	WHERE category = 'compliance' 
	AND occurred_at >= NOW() - INTERVAL '30 days'`

	var totalViolations, activeViolations, resolvedViolations int
	err := s.postgres.QueryRowContext(ctx, query).Scan(&totalViolations, &activeViolations, &resolvedViolations)
	if err != nil {
		return ComplianceMetrics{}, err
	}

	// Calculate compliance score
	complianceScore := 100.0
	if totalViolations > 0 {
		complianceScore = math.Max(0, 100.0 - float64(activeViolations*10))
	}

	auditReadiness := 95.0 // Placeholder - would calculate based on documentation completeness
	if activeViolations > 0 {
		auditReadiness = math.Max(0, auditReadiness - float64(activeViolations*5))
	}

	return ComplianceMetrics{
		ComplianceScore:     complianceScore,
		ActiveViolations:    activeViolations,
		ResolvedViolations:  resolvedViolations,
		AuditReadiness:      auditReadiness,
		CertificationStatus: "compliant",
		RegulatoryUpdates:   2, // Placeholder
	}, nil
}

func (s *AnalyticsService) getEnvironmentalMetricsData(ctx context.Context) (EnvironmentalMetrics, error) {
	// Get distance data for environmental calculations
	query := `
	SELECT 
		sum(speed * 0.016667) as total_distance
	FROM telemetry 
	WHERE timestamp >= now() - INTERVAL 24 HOUR`

	var totalDistance float64
	err := s.clickhouse.QueryRow(ctx, query).Scan(&totalDistance)
	if err != nil {
		return EnvironmentalMetrics{}, err
	}

	// Calculate environmental metrics (simplified)
	fuelConsumption := totalDistance * 0.08 // 8L/100km
	co2Emissions := fuelConsumption * 2.31  // 2.31 kg CO2 per liter

	energyEfficiency := 0.0
	if totalDistance > 0 {
		energyEfficiency = totalDistance / fuelConsumption // km per liter
	}

	sustainabilityScore := 75.0 // Placeholder - would calculate based on multiple factors

	return EnvironmentalMetrics{
		CO2Emissions:        co2Emissions,
		FuelConsumption:     fuelConsumption,
		EnergyEfficiency:    energyEfficiency,
		CarbonFootprint:     co2Emissions,
		SustainabilityScore: sustainabilityScore,
		WeatherImpact:       "moderate", // Placeholder - would get from weather service
	}, nil
}

func (s *AnalyticsService) getTrendsData(ctx context.Context) map[string][]DataPoint {
	trends := make(map[string][]DataPoint)

	// Get hourly utilization trend for last 24 hours
	query := `
	SELECT 
		toStartOfHour(timestamp) as hour,
		uniq(vehicle_id) as active_vehicles
	FROM telemetry 
	WHERE timestamp >= now() - INTERVAL 24 HOUR
	GROUP BY hour
	ORDER BY hour`

	rows, err := s.clickhouse.Query(ctx, query)
	if err != nil {
		log.Printf("❌ Failed to get trends data: %v", err)
		return trends
	}
	defer rows.Close()

	var utilizationTrend []DataPoint
	for rows.Next() {
		var hour time.Time
		var activeVehicles int
		if err := rows.Scan(&hour, &activeVehicles); err == nil {
			utilizationTrend = append(utilizationTrend, DataPoint{
				Timestamp: hour,
				Value:     float64(activeVehicles),
			})
		}
	}

	trends["utilization"] = utilizationTrend
	trends["efficiency"] = s.generateMockTrend(24) // Placeholder
	trends["safety"] = s.generateMockTrend(24)     // Placeholder

	return trends
}

func (s *AnalyticsService) generateMockTrend(hours int) []DataPoint {
	var trend []DataPoint
	now := time.Now()
	
	for i := hours; i >= 0; i-- {
		timestamp := now.Add(time.Duration(-i) * time.Hour)
		value := 75.0 + (rand.Float64()-0.5)*20 // Random value between 65-85
		trend = append(trend, DataPoint{
			Timestamp: timestamp,
			Value:     value,
		})
	}
	
	return trend
}

func (s *AnalyticsService) getActiveAlertsData(ctx context.Context) []Alert {
	query := `
	SELECT alert_id, alert_type, severity, title, message, vehicle_id, occurred_at, status
	FROM fleet_core.vehicle_alerts 
	WHERE status = 'active' 
	AND severity IN ('high', 'critical')
	ORDER BY occurred_at DESC 
	LIMIT 10`

	rows, err := s.postgres.QueryContext(ctx, query)
	if err != nil {
		log.Printf("❌ Failed to get active alerts: %v", err)
		return []Alert{}
	}
	defer rows.Close()

	var alerts []Alert
	for rows.Next() {
		var alert Alert
		var vehicleID sql.NullString
		
		err := rows.Scan(&alert.ID, &alert.Type, &alert.Severity, &alert.Title,
			&alert.Description, &vehicleID, &alert.Timestamp, &alert.Status)
		if err == nil {
			if vehicleID.Valid {
				alert.VehicleID = vehicleID.String
			}
			alerts = append(alerts, alert)
		}
	}

	return alerts
}

func (s *AnalyticsService) generateRecommendations(dashboard ExecutiveDashboard) []string {
	var recommendations []string

	// Fleet utilization recommendations
	if dashboard.FleetOverview.UtilizationRate < 70 {
		recommendations = append(recommendations, "Fleet utilization is below optimal (70%). Consider route optimization or fleet size adjustment.")
	}

	// Safety recommendations
	if dashboard.SafetyMetrics.SafetyScore < 90 {
		recommendations = append(recommendations, "Safety score is below target. Review recent incidents and enhance safety protocols.")
	}

	// Financial recommendations
	if dashboard.FinancialMetrics.ROI < 15 {
		recommendations = append(recommendations, "ROI is below target (15%). Analyze cost optimization opportunities.")
	}

	// Compliance recommendations
	if dashboard.ComplianceMetrics.ActiveViolations > 0 {
		recommendations = append(recommendations, fmt.Sprintf("Address %d active compliance violations to maintain regulatory standing.", dashboard.ComplianceMetrics.ActiveViolations))
	}

	// Abu Dhabi specific recommendations
	if s.config.AbuDhabiMode {
		if dashboard.EnvironmentalMetrics.SustainabilityScore < 80 {
			recommendations = append(recommendations, "Consider implementing additional sustainability measures for UAE Vision 2071 alignment.")
		}
	}

	// Default recommendation if none generated
	if len(recommendations) == 0 {
		recommendations = append(recommendations, "Fleet operations are performing well. Continue monitoring key metrics.")
	}

	return recommendations
}

// Background Services

func (s *AnalyticsService) startScheduledReports() {
	// Run daily reports at 6 AM
	ticker := time.NewTicker(24 * time.Hour)
	defer ticker.Stop()

	// Calculate time until next 6 AM
	now := time.Now()
	next6AM := time.Date(now.Year(), now.Month(), now.Day(), 6, 0, 0, 0, now.Location())
	if now.After(next6AM) {
		next6AM = next6AM.Add(24 * time.Hour)
	}
	
	// Initial timer to sync with 6 AM
	initialTimer := time.NewTimer(time.Until(next6AM))
	
	select {
	case <-initialTimer.C:
		s.generateScheduledReports()
	}

	for range ticker.C {
		s.generateScheduledReports()
	}
}

func (s *AnalyticsService) generateScheduledReports() {
	log.Printf("📊 Generating scheduled reports...")
	
	ctx := context.Background()
	
	// Generate executive summary report
	dashboard, err := s.generateExecutiveDashboard(ctx)
	if err != nil {
		log.Printf("❌ Failed to generate executive dashboard: %v", err)
		s.metrics.ReportsGenerated.WithLabelValues("executive", "failed").Inc()
	} else {
		s.metrics.ReportsGenerated.WithLabelValues("executive", "success").Inc()
		log.Printf("✅ Generated executive dashboard report")
	}

	// Store report for historical analysis
	s.storeHistoricalReport("executive", dashboard)
}

func (s *AnalyticsService) storeHistoricalReport(reportType string, data interface{}) {
	ctx := context.Background()
	
	reportJSON, err := json.Marshal(data)
	if err != nil {
		log.Printf("❌ Failed to marshal report: %v", err)
		return
	}

	key := fmt.Sprintf("report:%s:%s", reportType, time.Now().Format("2006-01-02"))
	err = s.redis.Set(ctx, key, reportJSON, 30*24*time.Hour).Err() // Keep for 30 days
	if err != nil {
		log.Printf("❌ Failed to store historical report: %v", err)
	}
}

func (s *AnalyticsService) startMetricsCollection() {
	ticker := time.NewTicker(60 * time.Second)
	defer ticker.Stop()

	for range ticker.C {
		s.collectSystemMetrics()
	}
}

func (s *AnalyticsService) collectSystemMetrics() {
	// Update cache hit rates
	ctx := context.Background()
	
	// Check data freshness
	lastTelemetry := s.getLastTelemetryTime(ctx)
	if !lastTelemetry.IsZero() {
		freshness := time.Since(lastTelemetry).Minutes()
		s.metrics.DataFreshness.WithLabelValues("telemetry").Set(freshness)
	}
}

func (s *AnalyticsService) getLastTelemetryTime(ctx context.Context) time.Time {
	query := `SELECT max(timestamp) FROM telemetry`
	
	var lastTime time.Time
	err := s.clickhouse.QueryRow(ctx, query).Scan(&lastTime)
	if err != nil {
		return time.Time{}
	}
	
	return lastTime
}

func (s *AnalyticsService) startDataFreshnessMonitor() {
	ticker := time.NewTicker(5 * time.Minute)
	defer ticker.Stop()

	for range ticker.C {
		s.monitorDataFreshness()
	}
}

func (s *AnalyticsService) monitorDataFreshness() {
	ctx := context.Background()
	
	// Check various data sources
	dataSources := map[string]string{
		"telemetry": "SELECT max(timestamp) FROM telemetry",
		"trips":     "SELECT max(updated_at) FROM trips",
		"alerts":    "SELECT max(occurred_at) FROM fleet_core.vehicle_alerts",
	}

	for source, query := range dataSources {
		var lastTime time.Time
		var err error
		
		if source == "telemetry" {
			err = s.clickhouse.QueryRow(ctx, query).Scan(&lastTime)
		} else {
			err = s.postgres.QueryRowContext(ctx, query).Scan(&lastTime)
		}
		
		if err == nil && !lastTime.IsZero() {
			freshness := time.Since(lastTime).Minutes()
			s.metrics.DataFreshness.WithLabelValues(source).Set(freshness)
			
			// Alert if data is too stale
			if freshness > 30 { // 30 minutes
				log.Printf("⚠️ Stale data detected in %s: %.1f minutes old", source, freshness)
			}
		}
	}
}

// Utility Functions

func (s *AnalyticsService) healthCheck(w http.ResponseWriter, r *http.Request) {
	health := map[string]interface{}{
		"status":      "healthy",
		"timestamp":   time.Now(),
		"service":     "analytics-service",
		"version":     "1.0.0",
		"region":      "abu-dhabi",
		"cache_timeout": s.config.CacheTimeout,
	}

	// Check PostgreSQL connection
	if err := s.postgres.Ping(); err != nil {
		health["status"] = "unhealthy"
		health["postgres"] = "failed"
	} else {
		health["postgres"] = "healthy"
	}

	// Check ClickHouse connection
	ctx := context.Background()
	if err := s.clickhouse.Ping(ctx); err != nil {
		health["status"] = "unhealthy"
		health["clickhouse"] = "failed"
	} else {
		health["clickhouse"] = "healthy"
	}

	// Check Redis connection
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

func (s *AnalyticsService) respondJSON(w http.ResponseWriter, data interface{}) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(data)
}

func (s *AnalyticsService) handleError(w http.ResponseWriter, message string, err error, statusCode int) {
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
func (s *AnalyticsService) getOperationalDashboard(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Operational dashboard not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getFleetDashboard(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Fleet dashboard not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) generateOperationalReport(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Operational report not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) generateComplianceReport(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Compliance report not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) generateFinancialReport(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Financial report not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) generateSafetyReport(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Safety report not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getFleetUtilization(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Fleet utilization not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getVehiclePerformance(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Vehicle performance not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getRouteEfficiency(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Route efficiency not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getCostAnalysis(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Cost analysis not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getRealtimeMetrics(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Realtime metrics not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getActiveAlerts(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Active alerts not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getKPIs(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "KPIs not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getEnvironmentalImpact(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Environmental impact not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getRegulatoryCompliance(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Regulatory compliance not yet implemented", nil, http.StatusNotImplemented)
}

func (s *AnalyticsService) getWeatherAnalytics(w http.ResponseWriter, r *http.Request) {
	s.handleError(w, "Weather analytics not yet implemented", nil, http.StatusNotImplemented)
}
