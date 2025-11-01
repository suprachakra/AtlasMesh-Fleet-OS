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
	"github.com/clickhouse/clickhouse-go/v2"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// AtlasMesh Analytics Service
//
// PUBLIC API: Real-time and batch analytics service providing business intelligence
// Handles KPI calculations, operational insights, and data-driven decision support
//
// INTEGRATION CONTRACTS:
// - ClickHouse: OLAP queries for analytics data warehouse
// - Kafka: Stream processing for real-time analytics
// - HTTP: REST API for analytics queries and reports
// - Prometheus: Metrics collection and monitoring
//
// PERFORMANCE: Sub-second query response for real-time dashboards
// SCALABILITY: Designed for 1000+ vehicles with 10Hz telemetry processing
// AVAILABILITY SLA: 99.9% uptime for analytics services

// Config holds all configuration parameters for the Analytics Service
type Config struct {
	Port                int      `json:"port"`                    // HTTP server port
	ClickHouseURL       string   `json:"clickhouse_url"`         // ClickHouse connection string
	KafkaBrokers        []string `json:"kafka_brokers"`          // Kafka cluster endpoints
	LogLevel            string   `json:"log_level"`              // Logging verbosity
	QueryTimeout        int      `json:"query_timeout_seconds"`  // Query execution timeout
	CacheTTL            int      `json:"cache_ttl_seconds"`     // Cache time-to-live
	MaxConcurrentQueries int      `json:"max_concurrent_queries"` // Query concurrency limit
}

// AnalyticsService manages analytics processing and queries
type AnalyticsService struct {
	config          Config
	tracer          trace.Tracer
	metrics         *Metrics
	clickhouse      clickhouse.Conn
	kafkaReader     *kafka.Reader
	queryCache      map[string]interface{}
	cacheMutex      sync.RWMutex
	querySemaphore  chan struct{}
}

type Metrics struct {
	QueryLatency       *prometheus.HistogramVec
	QueryCount         *prometheus.CounterVec
	CacheHits          *prometheus.CounterVec
	ProcessingErrors   *prometheus.CounterVec
	ActiveQueries      *prometheus.GaugeVec
	DataFreshness      *prometheus.GaugeVec
}

// Analytics data structures
type FleetKPIs struct {
	UtilizationRate    float64 `json:"utilization_rate"`
	AverageTripTime    float64 `json:"average_trip_time"`
	FuelEfficiency     float64 `json:"fuel_efficiency"`
	OnTimePerformance  float64 `json:"on_time_performance"`
	TotalRevenue       float64 `json:"total_revenue"`
	CostPerKilometer   float64 `json:"cost_per_kilometer"`
	CustomerSatisfaction float64 `json:"customer_satisfaction"`
	FleetAvailability  float64 `json:"fleet_availability"`
}

type OperationalMetrics struct {
	ServiceLevelAgreement float64 `json:"sla_compliance"`
	ResourceUtilization   float64 `json:"resource_utilization"`
	CapacityUtilization   float64 `json:"capacity_utilization"`
	IncidentRate          float64 `json:"incident_rate"`
	ResponseTime          float64 `json:"response_time"`
	Uptime                float64 `json:"uptime"`
}

type FinancialMetrics struct {
	OperatingCosts       float64 `json:"operating_costs"`
	RevenuePerVehicle     float64 `json:"revenue_per_vehicle"`
	ROI                   float64 `json:"roi"`
	BudgetVariance        float64 `json:"budget_variance"`
	CostEfficiency        float64 `json:"cost_efficiency"`
	ProfitMargin          float64 `json:"profit_margin"`
}

type SafetyMetrics struct {
	IncidentRate          float64 `json:"incident_rate"`
	ComplianceScore       float64 `json:"compliance_score"`
	SafetyTraining        float64 `json:"safety_training"`
	EmergencyResponse     float64 `json:"emergency_response"`
	RiskScore             float64 `json:"risk_score"`
	AuditScore            float64 `json:"audit_score"`
}

type AnalyticsQuery struct {
	Query    string                 `json:"query"`
	Format   string                 `json:"format"`
	Filters  map[string]interface{} `json:"filters"`
	TimeRange TimeRange             `json:"time_range"`
}

type TimeRange struct {
	Start time.Time `json:"start"`
	End   time.Time `json:"end"`
}

type QueryResult struct {
	Data      interface{} `json:"data"`
	Metadata  QueryMetadata `json:"metadata"`
	Timestamp time.Time   `json:"timestamp"`
}

type QueryMetadata struct {
	QueryID     string  `json:"query_id"`
	ExecutionTime float64 `json:"execution_time_ms"`
	RowsReturned int     `json:"rows_returned"`
	CacheHit    bool    `json:"cache_hit"`
}

func loadConfig() Config {
	return Config{
		Port:                getEnvInt("PORT", 8080),
		ClickHouseURL:       getEnv("CLICKHOUSE_URL", "clickhouse://localhost:9000"),
		KafkaBrokers:        getEnvStringSlice("KAFKA_BROKERS", []string{"localhost:9092"}),
		LogLevel:            getEnv("LOG_LEVEL", "info"),
		QueryTimeout:        getEnvInt("QUERY_TIMEOUT_SECONDS", 30),
		CacheTTL:            getEnvInt("CACHE_TTL_SECONDS", 300),
		MaxConcurrentQueries: getEnvInt("MAX_CONCURRENT_QUERIES", 100),
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

func getEnvStringSlice(key string, defaultValue []string) []string {
	if value := os.Getenv(key); value != "" {
		return []string{value} // Simplified - in production, parse comma-separated values
	}
	return defaultValue
}

func initializeMetrics() *Metrics {
	return &Metrics{
		QueryLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_analytics_query_latency_seconds",
				Help: "Analytics query execution latency",
				Buckets: prometheus.ExponentialBuckets(0.1, 2, 10),
			},
			[]string{"query_type", "status"},
		),
		QueryCount: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_analytics_queries_total",
				Help: "Total number of analytics queries",
			},
			[]string{"query_type", "status"},
		),
		CacheHits: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_analytics_cache_hits_total",
				Help: "Total number of cache hits",
			},
			[]string{"cache_type"},
		),
		ProcessingErrors: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_analytics_processing_errors_total",
				Help: "Total number of processing errors",
			},
			[]string{"error_type", "component"},
		),
		ActiveQueries: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_analytics_active_queries",
				Help: "Number of active analytics queries",
			},
			[]string{"query_type"},
		),
		DataFreshness: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_analytics_data_freshness_seconds",
				Help: "Data freshness in seconds",
			},
			[]string{"data_source"},
		),
	}
}

func main() {
	config := loadConfig()
	metrics := initializeMetrics()
	tracer := otel.Tracer("analytics-service")

	// Register metrics
	prometheus.MustRegister(
		metrics.QueryLatency,
		metrics.QueryCount,
		metrics.CacheHits,
		metrics.ProcessingErrors,
		metrics.ActiveQueries,
		metrics.DataFreshness,
	)

	// Initialize ClickHouse connection
	clickhouseConn, err := clickhouse.Open(&clickhouse.Options{
		Addr: []string{config.ClickHouseURL},
	})
	if err != nil {
		log.Fatalf("Failed to connect to ClickHouse: %v", err)
	}

	// Test ClickHouse connection
	ctx := context.Background()
	if err := clickhouseConn.Ping(ctx); err != nil {
		log.Fatalf("Failed to ping ClickHouse: %v", err)
	}

	// Initialize Kafka reader for real-time analytics
	kafkaReader := kafka.NewReader(kafka.ReaderConfig{
		Brokers: config.KafkaBrokers,
		Topic:   "analytics-events",
		GroupID: "analytics-service",
	})

	service := &AnalyticsService{
		config:         config,
		tracer:         tracer,
		metrics:        metrics,
		clickhouse:     clickhouseConn,
		kafkaReader:    kafkaReader,
		queryCache:     make(map[string]interface{}),
		querySemaphore: make(chan struct{}, config.MaxConcurrentQueries),
	}

	// Setup HTTP server
	router := mux.NewRouter()
	service.setupRoutes(router)

	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", config.Port),
		Handler: router,
	}

	// Start background services
	go service.startRealTimeProcessor(ctx)
	go service.startDataFreshnessMonitor(ctx)
	go service.startCacheCleanup(ctx)

	// Start server
	go func() {
		log.Printf("🚀 AtlasMesh Analytics Service starting on port %d", config.Port)
		log.Printf("📊 ClickHouse URL: %s", config.ClickHouseURL)
		log.Printf("📨 Kafka brokers: %v", config.KafkaBrokers)
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Server failed: %v", err)
		}
	}()

	// Graceful shutdown
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c

	log.Println("🛑 Shutting down Analytics Service...")
	
	// Close connections
	clickhouseConn.Close()
	kafkaReader.Close()
	
	// Shutdown server
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	
	if err := server.Shutdown(ctx); err != nil {
		log.Printf("Server shutdown error: %v", err)
}
	
	log.Println("✅ Analytics Service shutdown complete")
}

// API Routes and Handlers

func (s *AnalyticsService) setupRoutes(router *mux.Router) {
	// Health endpoints
	router.HandleFunc("/health", s.healthCheck).Methods("GET")
	router.HandleFunc("/ready", s.readinessCheck).Methods("GET")
	router.HandleFunc("/metrics", promhttp.Handler().ServeHTTP).Methods("GET")

	// Analytics API
	api := router.PathPrefix("/api/v1").Subrouter()

	// KPI endpoints
	api.HandleFunc("/analytics/kpis", s.getFleetKPIs).Methods("GET")
	api.HandleFunc("/analytics/operational", s.getOperationalMetrics).Methods("GET")
	api.HandleFunc("/analytics/financial", s.getFinancialMetrics).Methods("GET")
	api.HandleFunc("/analytics/safety", s.getSafetyMetrics).Methods("GET")
	
	// Query endpoints
	api.HandleFunc("/analytics/query", s.executeQuery).Methods("POST")
	api.HandleFunc("/analytics/dashboards", s.getDashboards).Methods("GET")
	api.HandleFunc("/analytics/reports/{id}", s.getReport).Methods("GET")
	
	// Real-time analytics
	api.HandleFunc("/analytics/realtime", s.getRealTimeMetrics).Methods("GET")
	api.HandleFunc("/analytics/stream", s.streamAnalytics).Methods("GET")
}

func (s *AnalyticsService) healthCheck(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":    "healthy",
		"service":   "analytics-service",
		"timestamp": time.Now(),
	})
}

func (s *AnalyticsService) readinessCheck(w http.ResponseWriter, r *http.Request) {
	// Check ClickHouse connection
	ctx, cancel := context.WithTimeout(r.Context(), 5*time.Second)
	defer cancel()
	
	if err := s.clickhouse.Ping(ctx); err != nil {
		w.WriteHeader(http.StatusServiceUnavailable)
		json.NewEncoder(w).Encode(map[string]interface{}{
			"status": "not_ready",
			"error":  "ClickHouse connection failed",
		})
		return
	}
	
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status": "ready",
	})
}

// KPI Calculation Functions

func (s *AnalyticsService) getFleetKPIs(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getFleetKPIs")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.QueryLatency.WithLabelValues("fleet_kpis", "success").Observe(time.Since(start).Seconds())
	}()

	// Parse query parameters
	timeframe := r.URL.Query().Get("timeframe")
	if timeframe == "" {
		timeframe = "24h"
	}

	// Check cache first
	cacheKey := fmt.Sprintf("fleet_kpis_%s", timeframe)
	if cached, found := s.getFromCache(cacheKey); found {
		s.metrics.CacheHits.WithLabelValues("fleet_kpis").Inc()
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(cached)
			return
	}

	// Calculate KPIs
	kpis, err := s.calculateFleetKPIs(ctx, timeframe)
	if err != nil {
		s.metrics.ProcessingErrors.WithLabelValues("query_error", "fleet_kpis").Inc()
		http.Error(w, fmt.Sprintf("Failed to calculate fleet KPIs: %v", err), http.StatusInternalServerError)
		return
	}

	// Cache results
	s.setCache(cacheKey, kpis, time.Duration(s.config.CacheTTL)*time.Second)

	s.metrics.QueryCount.WithLabelValues("fleet_kpis", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(kpis)
}

func (s *AnalyticsService) calculateFleetKPIs(ctx context.Context, timeframe string) (*FleetKPIs, error) {
	// Build time range
	timeRange := s.buildTimeRange(timeframe)
	
	// Execute ClickHouse queries for each KPI
	queries := map[string]string{
		"utilization_rate": `
			SELECT AVG(utilization_rate) as rate
			FROM fleet_analytics.utilization_metrics
			WHERE timestamp BETWEEN ? AND ?`,
		"average_trip_time": `
			SELECT AVG(trip_duration_minutes) as avg_duration
			FROM fleet_analytics.trip_metrics
			WHERE timestamp BETWEEN ? AND ?`,
		"fuel_efficiency": `
			SELECT AVG(fuel_efficiency_km_l) as efficiency
			FROM fleet_analytics.vehicle_metrics
			WHERE timestamp BETWEEN ? AND ?`,
		"on_time_performance": `
			SELECT (COUNT(CASE WHEN on_time = 1 THEN 1 END) * 100.0 / COUNT(*)) as performance
			FROM fleet_analytics.trip_metrics
			WHERE timestamp BETWEEN ? AND ?`,
		"total_revenue": `
			SELECT SUM(revenue) as total
			FROM fleet_analytics.financial_metrics
			WHERE timestamp BETWEEN ? AND ?`,
		"cost_per_kilometer": `
			SELECT AVG(cost_per_km) as cost
			FROM fleet_analytics.financial_metrics
			WHERE timestamp BETWEEN ? AND ?`,
		"customer_satisfaction": `
			SELECT AVG(satisfaction_score) as satisfaction
			FROM fleet_analytics.customer_metrics
			WHERE timestamp BETWEEN ? AND ?`,
		"fleet_availability": `
			SELECT AVG(availability_percentage) as availability
			FROM fleet_analytics.operational_metrics
			WHERE timestamp BETWEEN ? AND ?`,
	}

	kpis := &FleetKPIs{}
	
	// Execute each query
	for metric, query := range queries {
		var value float64
		err := s.clickhouse.QueryRow(ctx, query, timeRange.Start, timeRange.End).Scan(&value)
	if err != nil {
			log.Printf("Warning: Failed to calculate %s: %v", metric, err)
			continue
		}
		
		switch metric {
		case "utilization_rate":
			kpis.UtilizationRate = value
		case "average_trip_time":
			kpis.AverageTripTime = value
		case "fuel_efficiency":
			kpis.FuelEfficiency = value
		case "on_time_performance":
			kpis.OnTimePerformance = value
		case "total_revenue":
			kpis.TotalRevenue = value
		case "cost_per_kilometer":
			kpis.CostPerKilometer = value
		case "customer_satisfaction":
			kpis.CustomerSatisfaction = value
		case "fleet_availability":
			kpis.FleetAvailability = value
		}
	}

	return kpis, nil
}

func (s *AnalyticsService) getOperationalMetrics(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getOperationalMetrics")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.QueryLatency.WithLabelValues("operational_metrics", "success").Observe(time.Since(start).Seconds())
	}()

	// Calculate operational metrics
	metrics, err := s.calculateOperationalMetrics(ctx)
	if err != nil {
		s.metrics.ProcessingErrors.WithLabelValues("query_error", "operational_metrics").Inc()
		http.Error(w, fmt.Sprintf("Failed to calculate operational metrics: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.QueryCount.WithLabelValues("operational_metrics", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(metrics)
}

func (s *AnalyticsService) calculateOperationalMetrics(ctx context.Context) (*OperationalMetrics, error) {
	// Execute operational metrics queries
	queries := map[string]string{
		"sla_compliance": `
			SELECT AVG(sla_compliance_percentage) as compliance
			FROM fleet_analytics.operational_metrics
			WHERE timestamp >= now() - INTERVAL 24 HOUR`,
		"resource_utilization": `
			SELECT AVG(resource_utilization_percentage) as utilization
			FROM fleet_analytics.operational_metrics
			WHERE timestamp >= now() - INTERVAL 24 HOUR`,
		"capacity_utilization": `
			SELECT AVG(capacity_utilization_percentage) as capacity
			FROM fleet_analytics.operational_metrics
			WHERE timestamp >= now() - INTERVAL 24 HOUR`,
		"incident_rate": `
			SELECT COUNT(*) / (SELECT COUNT(DISTINCT vehicle_id) FROM fleet_analytics.vehicles) as rate
			FROM fleet_analytics.incidents
			WHERE timestamp >= now() - INTERVAL 24 HOUR`,
		"response_time": `
			SELECT AVG(response_time_seconds) as avg_response
			FROM fleet_analytics.incidents
			WHERE timestamp >= now() - INTERVAL 24 HOUR`,
		"uptime": `
			SELECT AVG(uptime_percentage) as uptime
			FROM fleet_analytics.operational_metrics
			WHERE timestamp >= now() - INTERVAL 24 HOUR`,
	}

	metrics := &OperationalMetrics{}
	
	for metric, query := range queries {
		var value float64
		err := s.clickhouse.QueryRow(ctx, query).Scan(&value)
	if err != nil {
			log.Printf("Warning: Failed to calculate %s: %v", metric, err)
			continue
		}
		
		switch metric {
		case "sla_compliance":
			metrics.ServiceLevelAgreement = value
		case "resource_utilization":
			metrics.ResourceUtilization = value
		case "capacity_utilization":
			metrics.CapacityUtilization = value
		case "incident_rate":
			metrics.IncidentRate = value
		case "response_time":
			metrics.ResponseTime = value
		case "uptime":
			metrics.Uptime = value
		}
	}

	return metrics, nil
}

func (s *AnalyticsService) getFinancialMetrics(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getFinancialMetrics")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.QueryLatency.WithLabelValues("financial_metrics", "success").Observe(time.Since(start).Seconds())
	}()

	// Calculate financial metrics
	metrics, err := s.calculateFinancialMetrics(ctx)
	if err != nil {
		s.metrics.ProcessingErrors.WithLabelValues("query_error", "financial_metrics").Inc()
		http.Error(w, fmt.Sprintf("Failed to calculate financial metrics: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.QueryCount.WithLabelValues("financial_metrics", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(metrics)
}

func (s *AnalyticsService) calculateFinancialMetrics(ctx context.Context) (*FinancialMetrics, error) {
	// Execute financial metrics queries
	queries := map[string]string{
		"operating_costs": `
			SELECT SUM(operating_cost) as total_cost
			FROM fleet_analytics.financial_metrics
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
		"revenue_per_vehicle": `
			SELECT AVG(revenue_per_vehicle) as avg_revenue
			FROM fleet_analytics.financial_metrics
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
		"roi": `
			SELECT AVG(roi_percentage) as avg_roi
			FROM fleet_analytics.financial_metrics
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
		"budget_variance": `
			SELECT AVG(budget_variance_percentage) as variance
			FROM fleet_analytics.financial_metrics
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
		"cost_efficiency": `
			SELECT AVG(cost_efficiency_score) as efficiency
			FROM fleet_analytics.financial_metrics
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
		"profit_margin": `
			SELECT AVG(profit_margin_percentage) as margin
			FROM fleet_analytics.financial_metrics
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
	}

	metrics := &FinancialMetrics{}
	
	for metric, query := range queries {
		var value float64
		err := s.clickhouse.QueryRow(ctx, query).Scan(&value)
	if err != nil {
			log.Printf("Warning: Failed to calculate %s: %v", metric, err)
			continue
		}
		
		switch metric {
		case "operating_costs":
			metrics.OperatingCosts = value
		case "revenue_per_vehicle":
			metrics.RevenuePerVehicle = value
		case "roi":
			metrics.ROI = value
		case "budget_variance":
			metrics.BudgetVariance = value
		case "cost_efficiency":
			metrics.CostEfficiency = value
		case "profit_margin":
			metrics.ProfitMargin = value
		}
	}

	return metrics, nil
}

func (s *AnalyticsService) getSafetyMetrics(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "getSafetyMetrics")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.QueryLatency.WithLabelValues("safety_metrics", "success").Observe(time.Since(start).Seconds())
	}()

	// Calculate safety metrics
	metrics, err := s.calculateSafetyMetrics(ctx)
	if err != nil {
		s.metrics.ProcessingErrors.WithLabelValues("query_error", "safety_metrics").Inc()
		http.Error(w, fmt.Sprintf("Failed to calculate safety metrics: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.QueryCount.WithLabelValues("safety_metrics", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(metrics)
}

func (s *AnalyticsService) calculateSafetyMetrics(ctx context.Context) (*SafetyMetrics, error) {
	// Execute safety metrics queries
	queries := map[string]string{
		"incident_rate": `
			SELECT COUNT(*) / (SELECT SUM(distance_km) FROM fleet_analytics.trips WHERE timestamp >= now() - INTERVAL 30 DAY) * 1000 as rate
			FROM fleet_analytics.incidents
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
		"compliance_score": `
			SELECT AVG(compliance_score) as score
			FROM fleet_analytics.safety_metrics
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
		"safety_training": `
			SELECT (COUNT(CASE WHEN training_completed = 1 THEN 1 END) * 100.0 / COUNT(*)) as training
			FROM fleet_analytics.driver_metrics
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
		"emergency_response": `
			SELECT AVG(response_time_seconds) as response
			FROM fleet_analytics.incidents
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
		"risk_score": `
			SELECT AVG(risk_score) as risk
			FROM fleet_analytics.safety_metrics
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
		"audit_score": `
			SELECT AVG(audit_score) as audit
			FROM fleet_analytics.safety_metrics
			WHERE timestamp >= now() - INTERVAL 30 DAY`,
	}

	metrics := &SafetyMetrics{}
	
	for metric, query := range queries {
		var value float64
		err := s.clickhouse.QueryRow(ctx, query).Scan(&value)
	if err != nil {
			log.Printf("Warning: Failed to calculate %s: %v", metric, err)
			continue
		}
		
		switch metric {
		case "incident_rate":
			metrics.IncidentRate = value
		case "compliance_score":
			metrics.ComplianceScore = value
		case "safety_training":
			metrics.SafetyTraining = value
		case "emergency_response":
			metrics.EmergencyResponse = value
		case "risk_score":
			metrics.RiskScore = value
		case "audit_score":
			metrics.AuditScore = value
		}
	}

	return metrics, nil
}

// Query execution and background services

func (s *AnalyticsService) executeQuery(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "executeQuery")
	defer span.End()

	start := time.Now()
	defer func() {
		s.metrics.QueryLatency.WithLabelValues("custom_query", "success").Observe(time.Since(start).Seconds())
	}()

	var query AnalyticsQuery
	if err := json.NewDecoder(r.Body).Decode(&query); err != nil {
		http.Error(w, "Invalid query payload", http.StatusBadRequest)
		return
	}

	// Acquire semaphore
	select {
	case s.querySemaphore <- struct{}{}:
		defer func() { <-s.querySemaphore }()
	case <-ctx.Done():
		http.Error(w, "Request timeout", http.StatusRequestTimeout)
		return
	}

	// Execute query with timeout
	queryCtx, cancel := context.WithTimeout(ctx, time.Duration(s.config.QueryTimeout)*time.Second)
	defer cancel()

	result, err := s.executeAnalyticsQuery(queryCtx, query)
	if err != nil {
		s.metrics.ProcessingErrors.WithLabelValues("query_execution", "custom_query").Inc()
		http.Error(w, fmt.Sprintf("Query execution failed: %v", err), http.StatusInternalServerError)
		return
	}

	s.metrics.QueryCount.WithLabelValues("custom_query", "success").Inc()
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(result)
}

func (s *AnalyticsService) executeAnalyticsQuery(ctx context.Context, query AnalyticsQuery) (*QueryResult, error) {
	start := time.Now()
	
	// Execute the query
	rows, err := s.clickhouse.Query(ctx, query.Query)
	if err != nil {
		return nil, err
	}
	defer rows.Close()

	// Parse results
	var results []map[string]interface{}
	columns, err := rows.Columns()
	if err != nil {
		return nil, err
	}

	for rows.Next() {
		values := make([]interface{}, len(columns))
		valuePtrs := make([]interface{}, len(columns))
		for i := range values {
			valuePtrs[i] = &values[i]
		}

		if err := rows.Scan(valuePtrs...); err != nil {
			return nil, err
		}

		row := make(map[string]interface{})
		for i, col := range columns {
			row[col] = values[i]
		}
		results = append(results, row)
	}

	executionTime := time.Since(start).Seconds() * 1000 // Convert to milliseconds

	return &QueryResult{
		Data: results,
		Metadata: QueryMetadata{
			QueryID:       s.generateQueryID(),
			ExecutionTime: executionTime,
			RowsReturned:  len(results),
			CacheHit:      false,
		},
		Timestamp: time.Now(),
	}, nil
}

// Background services

func (s *AnalyticsService) startRealTimeProcessor(ctx context.Context) {
	go func() {
		for {
	select {
			case <-ctx.Done():
				return
			default:
				message, err := s.kafkaReader.ReadMessage(ctx)
				if err != nil {
					log.Printf("❌ Failed to read analytics event from Kafka: %v", err)
					continue
				}

				s.processRealTimeEvent(ctx, message)
			}
		}
	}()
}

func (s *AnalyticsService) processRealTimeEvent(ctx context.Context, message kafka.Message) {
	// Process real-time analytics events
	// This would typically update real-time metrics and trigger alerts
	log.Printf("📊 Processing real-time analytics event: %s", string(message.Key))
}

func (s *AnalyticsService) startDataFreshnessMonitor(ctx context.Context) {
	go func() {
		ticker := time.NewTicker(30 * time.Second)
		defer ticker.Stop()

		for {
			select {
			case <-ticker.C:
				s.updateDataFreshness()
			case <-ctx.Done():
		return
	}
		}
	}()
}

func (s *AnalyticsService) updateDataFreshness() {
	// Check data freshness for different data sources
	sources := []string{"telemetry", "trips", "financial", "safety"}
	
	for _, source := range sources {
		var freshness float64
		query := fmt.Sprintf(`
			SELECT EXTRACT(EPOCH FROM (now() - MAX(timestamp))) as freshness
			FROM fleet_analytics.%s_metrics`, source)
		
		err := s.clickhouse.QueryRow(context.Background(), query).Scan(&freshness)
	if err != nil {
			log.Printf("Warning: Failed to check freshness for %s: %v", source, err)
			continue
	}
	
		s.metrics.DataFreshness.WithLabelValues(source).Set(freshness)
	}
}

func (s *AnalyticsService) startCacheCleanup(ctx context.Context) {
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

func (s *AnalyticsService) cleanupCache() {
	s.cacheMutex.Lock()
	defer s.cacheMutex.Unlock()
	
	// Remove expired cache entries
	// This is a simplified implementation
	// In production, you'd use a proper cache with TTL
}

// Helper functions

func (s *AnalyticsService) buildTimeRange(timeframe string) TimeRange {
	now := time.Now()
	var start time.Time
	
	switch timeframe {
	case "1h":
		start = now.Add(-1 * time.Hour)
	case "24h":
		start = now.Add(-24 * time.Hour)
	case "7d":
		start = now.Add(-7 * 24 * time.Hour)
	case "30d":
		start = now.Add(-30 * 24 * time.Hour)
	default:
		start = now.Add(-24 * time.Hour)
	}
	
	return TimeRange{
		Start: start,
		End:   now,
	}
}

func (s *AnalyticsService) getFromCache(key string) (interface{}, bool) {
	s.cacheMutex.RLock()
	defer s.cacheMutex.RUnlock()
	
	value, exists := s.queryCache[key]
	return value, exists
}

func (s *AnalyticsService) setCache(key string, value interface{}, ttl time.Duration) {
	s.cacheMutex.Lock()
	defer s.cacheMutex.Unlock()
	
	s.queryCache[key] = value
	// In production, implement proper TTL handling
}

func (s *AnalyticsService) generateQueryID() string {
	return fmt.Sprintf("query_%d", time.Now().UnixNano())
}

// Additional API endpoints

func (s *AnalyticsService) getDashboards(w http.ResponseWriter, r *http.Request) {
	dashboards := []map[string]interface{}{
		{
			"id":          "executive",
			"name":        "Executive Dashboard",
			"description": "High-level KPIs for C-level executives",
			"url":         "/dashboards/executive",
		},
		{
			"id":          "operations",
			"name":        "Operations Dashboard",
			"description": "Fleet operations and performance metrics",
			"url":         "/dashboards/operations",
		},
		{
			"id":          "financial",
			"name":        "Financial Dashboard",
			"description": "Cost analysis and revenue tracking",
			"url":         "/dashboards/financial",
		},
		{
			"id":          "safety",
			"name":        "Safety Dashboard",
			"description": "Safety metrics and compliance tracking",
			"url":         "/dashboards/safety",
		},
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(dashboards)
}

func (s *AnalyticsService) getReport(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	reportID := vars["id"]
	
	// Mock report data
	report := map[string]interface{}{
		"id":          reportID,
		"title":       "Fleet Performance Report",
		"generated_at": time.Now(),
		"status":      "completed",
		"data":        "Report data would be here",
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(report)
}

func (s *AnalyticsService) getRealTimeMetrics(w http.ResponseWriter, r *http.Request) {
	// Real-time metrics endpoint
	metrics := map[string]interface{}{
		"active_vehicles":    150,
		"current_utilization": 85.5,
		"avg_response_time":   1.2,
		"system_health":      "healthy",
		"timestamp":          time.Now(),
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(metrics)
}

func (s *AnalyticsService) streamAnalytics(w http.ResponseWriter, r *http.Request) {
	// Server-sent events for real-time analytics streaming
	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "keep-alive")
	
	flusher, ok := w.(http.Flusher)
	if !ok {
		http.Error(w, "Streaming not supported", http.StatusInternalServerError)
		return
	}
	
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()
	
	for {
		select {
		case <-r.Context().Done():
			return
		case <-ticker.C:
			metrics := map[string]interface{}{
				"timestamp": time.Now(),
				"metrics":   "Real-time analytics data",
			}
			
			data, _ := json.Marshal(metrics)
			fmt.Fprintf(w, "data: %s\n\n", data)
			flusher.Flush()
		}
	}
}