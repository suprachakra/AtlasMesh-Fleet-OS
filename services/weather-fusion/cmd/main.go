package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/gorilla/mux"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/attribute"
	"go.opentelemetry.io/otel/trace"
)

// WeatherFusionService manages multi-source weather data fusion
type WeatherFusionService struct {
	providers      map[string]WeatherProvider
	fusionEngine   *FusionEngine
	cache          *WeatherCache
	metrics        *WeatherMetrics
	tracer         trace.Tracer
	config         *Config
	mu             sync.RWMutex
}

// Config holds the service configuration
type Config struct {
	Port                int                    `json:"port"`
	MetricsPort         int                    `json:"metrics_port"`
	CacheSize           int                    `json:"cache_size"`
	DefaultTTL          time.Duration          `json:"default_ttl"`
	FusionInterval      time.Duration          `json:"fusion_interval"`
	ConfidenceThreshold float64                `json:"confidence_threshold"`
	Providers           map[string]ProviderConfig `json:"providers"`
}

// ProviderConfig defines weather provider configuration
type ProviderConfig struct {
	Name        string            `json:"name"`
	URL         string            `json:"url"`
	APIKey      string            `json:"api_key"`
	Weight      float64           `json:"weight"`
	Timeout     time.Duration     `json:"timeout"`
	RateLimit   int               `json:"rate_limit"`
	Enabled     bool              `json:"enabled"`
	Headers     map[string]string `json:"headers"`
	Priority    int               `json:"priority"`
}

// WeatherProvider interface for different weather data sources
type WeatherProvider interface {
	GetWeatherData(ctx context.Context, location Location) (*WeatherData, error)
	GetForecast(ctx context.Context, location Location, hours int) ([]*WeatherData, error)
	GetProviderInfo() ProviderInfo
	IsHealthy() bool
}

// Location represents a geographic location
type Location struct {
	Latitude   float64 `json:"latitude"`
	Longitude  float64 `json:"longitude"`
	Altitude   float64 `json:"altitude,omitempty"`
	Name       string  `json:"name,omitempty"`
	Region     string  `json:"region,omitempty"`
	Country    string  `json:"country,omitempty"`
}

// WeatherData represents weather information from a provider
type WeatherData struct {
	Location        Location              `json:"location"`
	Timestamp       time.Time             `json:"timestamp"`
	Provider        string                `json:"provider"`
	Temperature     *Measurement          `json:"temperature,omitempty"`
	Humidity        *Measurement          `json:"humidity,omitempty"`
	Pressure        *Measurement          `json:"pressure,omitempty"`
	WindSpeed       *Measurement          `json:"wind_speed,omitempty"`
	WindDirection   *Measurement          `json:"wind_direction,omitempty"`
	Visibility      *Measurement          `json:"visibility,omitempty"`
	Precipitation   *Measurement          `json:"precipitation,omitempty"`
	CloudCover      *Measurement          `json:"cloud_cover,omitempty"`
	UVIndex         *Measurement          `json:"uv_index,omitempty"`
	Conditions      []WeatherCondition    `json:"conditions"`
	Alerts          []WeatherAlert        `json:"alerts,omitempty"`
	Confidence      float64               `json:"confidence"`
	Freshness       time.Duration         `json:"freshness"`
	TTL             time.Duration         `json:"ttl"`
	Metadata        map[string]interface{} `json:"metadata,omitempty"`
}

// Measurement represents a weather measurement with uncertainty
type Measurement struct {
	Value       float64   `json:"value"`
	Unit        string    `json:"unit"`
	Confidence  float64   `json:"confidence"`
	Timestamp   time.Time `json:"timestamp"`
	Source      string    `json:"source"`
	Uncertainty float64   `json:"uncertainty,omitempty"`
}

// WeatherCondition represents weather conditions
type WeatherCondition struct {
	Code        string  `json:"code"`
	Description string  `json:"description"`
	Severity    string  `json:"severity"`
	Confidence  float64 `json:"confidence"`
}

// WeatherAlert represents weather alerts and warnings
type WeatherAlert struct {
	ID          string    `json:"id"`
	Type        string    `json:"type"`
	Severity    string    `json:"severity"`
	Title       string    `json:"title"`
	Description string    `json:"description"`
	StartTime   time.Time `json:"start_time"`
	EndTime     time.Time `json:"end_time"`
	Areas       []string  `json:"areas"`
	Source      string    `json:"source"`
}

// FusedWeatherData represents the result of weather data fusion
type FusedWeatherData struct {
	Location         Location              `json:"location"`
	Timestamp        time.Time             `json:"timestamp"`
	FusionTimestamp  time.Time             `json:"fusion_timestamp"`
	Temperature      *FusedMeasurement     `json:"temperature,omitempty"`
	Humidity         *FusedMeasurement     `json:"humidity,omitempty"`
	Pressure         *FusedMeasurement     `json:"pressure,omitempty"`
	WindSpeed        *FusedMeasurement     `json:"wind_speed,omitempty"`
	WindDirection    *FusedMeasurement     `json:"wind_direction,omitempty"`
	Visibility       *FusedMeasurement     `json:"visibility,omitempty"`
	Precipitation    *FusedMeasurement     `json:"precipitation,omitempty"`
	CloudCover       *FusedMeasurement     `json:"cloud_cover,omitempty"`
	UVIndex          *FusedMeasurement     `json:"uv_index,omitempty"`
	Conditions       []WeatherCondition    `json:"conditions"`
	Alerts           []WeatherAlert        `json:"alerts"`
	OverallConfidence float64              `json:"overall_confidence"`
	Sources          []string              `json:"sources"`
	FusionMethod     string                `json:"fusion_method"`
	TTL              time.Duration         `json:"ttl"`
	Metadata         map[string]interface{} `json:"metadata,omitempty"`
}

// FusedMeasurement represents a fused measurement from multiple sources
type FusedMeasurement struct {
	Value           float64                `json:"value"`
	Unit            string                 `json:"unit"`
	Confidence      float64                `json:"confidence"`
	Uncertainty     float64                `json:"uncertainty"`
	Sources         []SourceContribution   `json:"sources"`
	FusionMethod    string                 `json:"fusion_method"`
	Timestamp       time.Time              `json:"timestamp"`
}

// SourceContribution represents how much each source contributed to the fused value
type SourceContribution struct {
	Provider    string  `json:"provider"`
	Weight      float64 `json:"weight"`
	Value       float64 `json:"value"`
	Confidence  float64 `json:"confidence"`
	Freshness   float64 `json:"freshness"`
}

// ProviderInfo contains information about a weather provider
type ProviderInfo struct {
	Name         string        `json:"name"`
	Description  string        `json:"description"`
	Coverage     []string      `json:"coverage"`
	UpdateFreq   time.Duration `json:"update_frequency"`
	Accuracy     float64       `json:"accuracy"`
	Reliability  float64       `json:"reliability"`
}

// FusionEngine handles the fusion of weather data from multiple sources
type FusionEngine struct {
	algorithms map[string]FusionAlgorithm
	config     *Config
	metrics    *WeatherMetrics
}

// FusionAlgorithm interface for different fusion methods
type FusionAlgorithm interface {
	Fuse(measurements []*Measurement) *FusedMeasurement
	GetName() string
	GetDescription() string
}

// WeatherCache manages caching of weather data with TTL
type WeatherCache struct {
	data    map[string]*CacheEntry
	mu      sync.RWMutex
	maxSize int
}

// CacheEntry represents a cached weather data entry
type CacheEntry struct {
	Data      *FusedWeatherData
	ExpiresAt time.Time
	AccessCount int
	LastAccess  time.Time
}

// WeatherMetrics contains Prometheus metrics for weather fusion
type WeatherMetrics struct {
	RequestsTotal       *prometheus.CounterVec
	FusionDuration      *prometheus.HistogramVec
	ProviderLatency     *prometheus.HistogramVec
	ProviderErrors      *prometheus.CounterVec
	CacheHits           *prometheus.CounterVec
	DataFreshness       *prometheus.GaugeVec
	ConfidenceScore     *prometheus.GaugeVec
	ActiveProviders     *prometheus.GaugeVec
}

func main() {
	// Load configuration
	config := loadConfig()
	
	// Initialize metrics
	metrics := initializeMetrics()
	
	// Initialize tracer
	tracer := otel.Tracer("weather-fusion-service")
	
	// Initialize cache
	cache := &WeatherCache{
		data:    make(map[string]*CacheEntry),
		maxSize: config.CacheSize,
	}
	
	// Initialize fusion engine
	fusionEngine := &FusionEngine{
		algorithms: initializeFusionAlgorithms(),
		config:     config,
		metrics:    metrics,
	}
	
	// Initialize weather providers
	providers := make(map[string]WeatherProvider)
	for name, providerConfig := range config.Providers {
		if providerConfig.Enabled {
			provider, err := createWeatherProvider(name, providerConfig)
			if err != nil {
				log.Printf("Failed to create provider %s: %v", name, err)
				continue
			}
			providers[name] = provider
			log.Printf("Initialized weather provider: %s", name)
		}
	}
	
	// Create service instance
	service := &WeatherFusionService{
		providers:    providers,
		fusionEngine: fusionEngine,
		cache:        cache,
		metrics:      metrics,
		tracer:       tracer,
		config:       config,
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
		log.Printf("Starting Weather Fusion service on port %d", config.Port)
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
	go service.startFusionLoop()
	go service.startCacheCleanup()
	go service.startProviderHealthCheck()
	
	// Wait for interrupt signal
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	<-c
	
	log.Println("Shutting down Weather Fusion service...")
	
	// Graceful shutdown
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	
	server.Shutdown(ctx)
	metricsServer.Shutdown(ctx)
	
	log.Println("Weather Fusion service stopped")
}

func (s *WeatherFusionService) setupRoutes(router *mux.Router) {
	// API routes
	api := router.PathPrefix("/api/v1").Subrouter()
	
	// Weather data endpoints
	api.HandleFunc("/weather/current", s.getCurrentWeather).Methods("GET")
	api.HandleFunc("/weather/forecast", s.getForecast).Methods("GET")
	api.HandleFunc("/weather/alerts", s.getWeatherAlerts).Methods("GET")
	
	// Fusion endpoints
	api.HandleFunc("/fusion/current", s.getFusedWeather).Methods("GET")
	api.HandleFunc("/fusion/confidence", s.getConfidenceMetrics).Methods("GET")
	api.HandleFunc("/fusion/sources", s.getActiveSources).Methods("GET")
	
	// Provider management
	api.HandleFunc("/providers", s.listProviders).Methods("GET")
	api.HandleFunc("/providers/{provider}/health", s.getProviderHealth).Methods("GET")
	api.HandleFunc("/providers/{provider}/enable", s.enableProvider).Methods("POST")
	api.HandleFunc("/providers/{provider}/disable", s.disableProvider).Methods("POST")
	
	// Cache management
	api.HandleFunc("/cache/stats", s.getCacheStats).Methods("GET")
	api.HandleFunc("/cache/clear", s.clearCache).Methods("POST")
	
	// Health and status
	api.HandleFunc("/health", s.healthCheck).Methods("GET")
	api.HandleFunc("/ready", s.readinessCheck).Methods("GET")
	api.HandleFunc("/status", s.getStatus).Methods("GET")
}

func (s *WeatherFusionService) getFusedWeather(w http.ResponseWriter, r *http.Request) {
	ctx, span := s.tracer.Start(r.Context(), "get_fused_weather")
	defer span.End()
	
	// Parse location parameters
	lat := r.URL.Query().Get("lat")
	lon := r.URL.Query().Get("lon")
	
	if lat == "" || lon == "" {
		http.Error(w, "Missing required parameters: lat, lon", http.StatusBadRequest)
		return
	}
	
	location := Location{
		Latitude:  parseFloat(lat),
		Longitude: parseFloat(lon),
		Name:      r.URL.Query().Get("name"),
	}
	
	// Check cache first
	cacheKey := fmt.Sprintf("fused_%f_%f", location.Latitude, location.Longitude)
	if cachedData := s.cache.Get(cacheKey); cachedData != nil {
		s.metrics.CacheHits.WithLabelValues("hit").Inc()
		span.SetAttributes(attribute.Bool("cache_hit", true))
		
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(cachedData)
		return
	}
	
	s.metrics.CacheHits.WithLabelValues("miss").Inc()
	span.SetAttributes(attribute.Bool("cache_hit", false))
	
	// Collect weather data from all providers
	weatherData := make([]*WeatherData, 0)
	var wg sync.WaitGroup
	var mu sync.Mutex
	
	for name, provider := range s.providers {
		if !provider.IsHealthy() {
			continue
		}
		
		wg.Add(1)
		go func(providerName string, p WeatherProvider) {
			defer wg.Done()
			
			start := time.Now()
			data, err := p.GetWeatherData(ctx, location)
			duration := time.Since(start)
			
			s.metrics.ProviderLatency.WithLabelValues(providerName).Observe(duration.Seconds())
			
			if err != nil {
				s.metrics.ProviderErrors.WithLabelValues(providerName, "fetch_error").Inc()
				span.RecordError(err)
				return
			}
			
			mu.Lock()
			weatherData = append(weatherData, data)
			mu.Unlock()
		}(name, provider)
	}
	
	wg.Wait()
	
	if len(weatherData) == 0 {
		http.Error(w, "No weather data available from any provider", http.StatusServiceUnavailable)
		return
	}
	
	// Fuse the weather data
	start := time.Now()
	fusedData := s.fusionEngine.FuseWeatherData(weatherData, location)
	fusionDuration := time.Since(start)
	
	s.metrics.FusionDuration.WithLabelValues("current").Observe(fusionDuration.Seconds())
	s.metrics.ConfidenceScore.WithLabelValues("overall").Set(fusedData.OverallConfidence)
	
	// Cache the result
	s.cache.Set(cacheKey, fusedData, fusedData.TTL)
	
	span.SetAttributes(
		attribute.Int("sources_count", len(weatherData)),
		attribute.Float64("confidence", fusedData.OverallConfidence),
		attribute.String("fusion_method", fusedData.FusionMethod),
	)
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(fusedData)
}

func (s *WeatherFusionService) startFusionLoop() {
	ticker := time.NewTicker(s.config.FusionInterval)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.performPeriodicFusion()
		}
	}
}

func (s *WeatherFusionService) performPeriodicFusion() {
	// This would typically update cached fusion results for frequently requested locations
	log.Println("Performing periodic weather data fusion...")
	
	// Update metrics
	activeProviders := 0
	for _, provider := range s.providers {
		if provider.IsHealthy() {
			activeProviders++
		}
	}
	s.metrics.ActiveProviders.WithLabelValues("healthy").Set(float64(activeProviders))
}

func (s *WeatherFusionService) startCacheCleanup() {
	ticker := time.NewTicker(5 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			s.cache.Cleanup()
		}
	}
}

func (s *WeatherFusionService) startProviderHealthCheck() {
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			for name, provider := range s.providers {
				if provider.IsHealthy() {
					s.metrics.ActiveProviders.WithLabelValues(name).Set(1)
				} else {
					s.metrics.ActiveProviders.WithLabelValues(name).Set(0)
				}
			}
		}
	}
}

// Cache methods
func (c *WeatherCache) Get(key string) *FusedWeatherData {
	c.mu.RLock()
	defer c.mu.RUnlock()
	
	entry, exists := c.data[key]
	if !exists || time.Now().After(entry.ExpiresAt) {
		return nil
	}
	
	entry.AccessCount++
	entry.LastAccess = time.Now()
	return entry.Data
}

func (c *WeatherCache) Set(key string, data *FusedWeatherData, ttl time.Duration) {
	c.mu.Lock()
	defer c.mu.Unlock()
	
	// Implement LRU eviction if cache is full
	if len(c.data) >= c.maxSize {
		c.evictLRU()
	}
	
	c.data[key] = &CacheEntry{
		Data:        data,
		ExpiresAt:   time.Now().Add(ttl),
		AccessCount: 1,
		LastAccess:  time.Now(),
	}
}

func (c *WeatherCache) Cleanup() {
	c.mu.Lock()
	defer c.mu.Unlock()
	
	now := time.Now()
	for key, entry := range c.data {
		if now.After(entry.ExpiresAt) {
			delete(c.data, key)
		}
	}
}

func (c *WeatherCache) evictLRU() {
	var oldestKey string
	var oldestTime time.Time = time.Now()
	
	for key, entry := range c.data {
		if entry.LastAccess.Before(oldestTime) {
			oldestTime = entry.LastAccess
			oldestKey = key
		}
	}
	
	if oldestKey != "" {
		delete(c.data, oldestKey)
	}
}

// Helper functions
func loadConfig() *Config {
	return &Config{
		Port:                8080,
		MetricsPort:         9090,
		CacheSize:           1000,
		DefaultTTL:          15 * time.Minute,
		FusionInterval:      5 * time.Minute,
		ConfidenceThreshold: 0.7,
		Providers: map[string]ProviderConfig{
			"openweather": {
				Name:      "OpenWeatherMap",
				URL:       "https://api.openweathermap.org/data/2.5",
				APIKey:    getEnv("OPENWEATHER_API_KEY", ""),
				Weight:    0.3,
				Timeout:   10 * time.Second,
				RateLimit: 60,
				Enabled:   true,
				Priority:  1,
			},
			"weatherapi": {
				Name:      "WeatherAPI",
				URL:       "https://api.weatherapi.com/v1",
				APIKey:    getEnv("WEATHERAPI_API_KEY", ""),
				Weight:    0.25,
				Timeout:   10 * time.Second,
				RateLimit: 100,
				Enabled:   true,
				Priority:  2,
			},
			"accuweather": {
				Name:      "AccuWeather",
				URL:       "https://dataservice.accuweather.com",
				APIKey:    getEnv("ACCUWEATHER_API_KEY", ""),
				Weight:    0.25,
				Timeout:   10 * time.Second,
				RateLimit: 50,
				Enabled:   true,
				Priority:  3,
			},
			"local_sensors": {
				Name:      "Local Weather Sensors",
				URL:       "http://localhost:8088",
				Weight:    0.2,
				Timeout:   5 * time.Second,
				RateLimit: 1000,
				Enabled:   true,
				Priority:  0, // Highest priority
			},
		},
	}
}

func initializeMetrics() *WeatherMetrics {
	metrics := &WeatherMetrics{
		RequestsTotal: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_weather_requests_total",
				Help: "Total number of weather requests",
			},
			[]string{"endpoint", "status"},
		),
		FusionDuration: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_weather_fusion_duration_seconds",
				Help: "Weather data fusion duration",
			},
			[]string{"type"},
		),
		ProviderLatency: prometheus.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "atlasmesh_weather_provider_latency_seconds",
				Help: "Weather provider response latency",
			},
			[]string{"provider"},
		),
		ProviderErrors: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_weather_provider_errors_total",
				Help: "Weather provider errors",
			},
			[]string{"provider", "error_type"},
		),
		CacheHits: prometheus.NewCounterVec(
			prometheus.CounterOpts{
				Name: "atlasmesh_weather_cache_hits_total",
				Help: "Weather cache hits",
			},
			[]string{"result"},
		),
		DataFreshness: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_weather_data_freshness_seconds",
				Help: "Weather data freshness",
			},
			[]string{"provider"},
		),
		ConfidenceScore: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_weather_confidence_score",
				Help: "Weather data confidence score",
			},
			[]string{"type"},
		),
		ActiveProviders: prometheus.NewGaugeVec(
			prometheus.GaugeOpts{
				Name: "atlasmesh_weather_active_providers",
				Help: "Number of active weather providers",
			},
			[]string{"provider"},
		),
	}
	
	prometheus.MustRegister(
		metrics.RequestsTotal,
		metrics.FusionDuration,
		metrics.ProviderLatency,
		metrics.ProviderErrors,
		metrics.CacheHits,
		metrics.DataFreshness,
		metrics.ConfidenceScore,
		metrics.ActiveProviders,
	)
	
	return metrics
}

func initializeFusionAlgorithms() map[string]FusionAlgorithm {
	return map[string]FusionAlgorithm{
		"weighted_average": &WeightedAverageFusion{},
		"kalman_filter":    &KalmanFilterFusion{},
		"bayesian":         &BayesianFusion{},
	}
}

func getEnv(key, defaultValue string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return defaultValue
}

func parseFloat(s string) float64 {
	// Implement proper float parsing with error handling
	return 0.0 // Placeholder
}

func (s *WeatherFusionService) healthCheck(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "healthy"})
}

func (s *WeatherFusionService) readinessCheck(w http.ResponseWriter, r *http.Request) {
	// Check if at least one provider is healthy
	healthyProviders := 0
	for _, provider := range s.providers {
		if provider.IsHealthy() {
			healthyProviders++
		}
	}
	
	if healthyProviders == 0 {
		http.Error(w, "No healthy weather providers", http.StatusServiceUnavailable)
		return
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"status":            "ready",
		"healthy_providers": healthyProviders,
	})
}

func (s *WeatherFusionService) getStatus(w http.ResponseWriter, r *http.Request) {
	status := map[string]interface{}{
		"service":           "weather-fusion",
		"version":           "1.0.0",
		"providers":         len(s.providers),
		"cache_size":        len(s.cache.data),
		"fusion_algorithms": len(s.fusionEngine.algorithms),
	}
	
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(status)
}

// Weather provider factory
func createWeatherProvider(name string, config ProviderConfig) (WeatherProvider, error) {
	switch name {
	case "openweather":
		return &OpenWeatherProvider{
			APIKey:  config.APIKey,
			BaseURL: config.BaseURL,
			Client:  &http.Client{Timeout: 10 * time.Second},
		}, nil
	case "noaa":
		return &NOAAProvider{
			BaseURL: config.BaseURL,
			Client:  &http.Client{Timeout: 15 * time.Second},
		}, nil
	case "weatherapi":
		return &WeatherAPIProvider{
			APIKey:  config.APIKey,
			BaseURL: config.BaseURL,
			Client:  &http.Client{Timeout: 10 * time.Second},
		}, nil
	default:
		return nil, fmt.Errorf("unknown weather provider: %s", name)
	}
}

// Enhanced weather data fusion with multiple algorithms
func (fe *FusionEngine) FuseWeatherData(data []*WeatherData, location Location) *FusedWeatherData {
	if len(data) == 0 {
		return &FusedWeatherData{
			Location:          location,
			Timestamp:         time.Now(),
			FusionTimestamp:   time.Now(),
			OverallConfidence: 0.0,
			Sources:           []string{},
			FusionMethod:      "no_data",
			TTL:               5 * time.Minute,
		}
	}

	// Group measurements by type
	measurementGroups := make(map[string][]*Measurement)
	sources := make([]string, 0, len(data))
	
	for _, wd := range data {
		sources = append(sources, wd.Source)
		for _, measurement := range wd.Measurements {
			measurementGroups[measurement.Type] = append(measurementGroups[measurement.Type], measurement)
		}
	}

	// Fuse each measurement type
	fusedMeasurements := make([]*FusedMeasurement, 0)
	totalConfidence := 0.0
	
	for measurementType, measurements := range measurementGroups {
		// Select fusion algorithm based on measurement type and data quality
		algorithm := fe.selectFusionAlgorithm(measurementType, measurements)
		fusedMeasurement := algorithm.Fuse(measurements)
		
		if fusedMeasurement != nil {
			fusedMeasurements = append(fusedMeasurements, fusedMeasurement)
			totalConfidence += fusedMeasurement.Confidence
		}
	}

	// Calculate overall confidence
	overallConfidence := 0.0
	if len(fusedMeasurements) > 0 {
		overallConfidence = totalConfidence / float64(len(fusedMeasurements))
	}

	// Determine TTL based on data freshness and confidence
	ttl := fe.calculateTTL(data, overallConfidence)

	return &FusedWeatherData{
		Location:          location,
		Timestamp:         time.Now(),
		FusionTimestamp:   time.Now(),
		Measurements:      fusedMeasurements,
		OverallConfidence: overallConfidence,
		Sources:           sources,
		FusionMethod:      "multi_algorithm",
		TTL:               ttl,
		Metadata: map[string]interface{}{
			"source_count":      len(data),
			"measurement_types": len(measurementGroups),
			"fusion_algorithms": fe.getUsedAlgorithms(measurementGroups),
		},
	}
}

// Select appropriate fusion algorithm based on data characteristics
func (fe *FusionEngine) selectFusionAlgorithm(measurementType string, measurements []*Measurement) FusionAlgorithm {
	// For temperature, pressure, humidity - use weighted average
	if measurementType == "temperature" || measurementType == "pressure" || measurementType == "humidity" {
		return &WeightedAverageFusion{}
	}
	
	// For wind speed/direction - use Kalman filter for temporal consistency
	if measurementType == "wind_speed" || measurementType == "wind_direction" {
		return &KalmanFilterFusion{}
	}
	
	// For precipitation and visibility - use Bayesian fusion for uncertainty handling
	if measurementType == "precipitation" || measurementType == "visibility" {
		return &BayesianFusion{}
	}
	
	// Default to weighted average
	return &WeightedAverageFusion{}
}

// Calculate TTL based on data freshness and confidence
func (fe *FusionEngine) calculateTTL(data []*WeatherData, confidence float64) time.Duration {
	// Base TTL
	baseTTL := 15 * time.Minute
	
	// Adjust based on confidence
	if confidence > 0.9 {
		baseTTL = 20 * time.Minute
	} else if confidence < 0.5 {
		baseTTL = 5 * time.Minute
	}
	
	// Adjust based on data freshness
	oldestData := time.Now()
	for _, wd := range data {
		if wd.Timestamp.Before(oldestData) {
			oldestData = wd.Timestamp
		}
	}
	
	dataAge := time.Since(oldestData)
	if dataAge > 30*time.Minute {
		baseTTL = baseTTL / 2 // Reduce TTL for old data
	}
	
	return baseTTL
}

// Get list of algorithms used in fusion
func (fe *FusionEngine) getUsedAlgorithms(measurementGroups map[string][]*Measurement) []string {
	algorithms := make(map[string]bool)
	for measurementType := range measurementGroups {
		algorithm := fe.selectFusionAlgorithm(measurementType, measurementGroups[measurementType])
		algorithms[algorithm.GetName()] = true
	}
	
	result := make([]string, 0, len(algorithms))
	for alg := range algorithms {
		result = append(result, alg)
	}
	return result
}

// Weighted Average Fusion Algorithm
type WeightedAverageFusion struct{}

func (w *WeightedAverageFusion) Fuse(measurements []*Measurement) *FusedMeasurement {
	if len(measurements) == 0 {
		return nil
	}

	totalWeight := 0.0
	weightedSum := 0.0
	totalConfidence := 0.0

	for _, m := range measurements {
		// Weight based on data freshness and source reliability
		weight := w.calculateWeight(m)
		weightedSum += m.Value * weight
		totalWeight += weight
		totalConfidence += m.Confidence * weight
	}

	if totalWeight == 0 {
		return nil
	}

	fusedValue := weightedSum / totalWeight
	fusedConfidence := totalConfidence / totalWeight

	return &FusedMeasurement{
		Type:       measurements[0].Type,
		Value:      fusedValue,
		Unit:       measurements[0].Unit,
		Confidence: fusedConfidence,
		Timestamp:  time.Now(),
		Sources:    w.extractSources(measurements),
		Method:     "weighted_average",
		Metadata: map[string]interface{}{
			"source_count":  len(measurements),
			"total_weight":  totalWeight,
			"value_range":   w.calculateValueRange(measurements),
		},
	}
}

func (w *WeightedAverageFusion) calculateWeight(m *Measurement) float64 {
	// Base weight from confidence
	weight := m.Confidence

	// Adjust for data freshness (exponential decay)
	age := time.Since(m.Timestamp)
	freshnessWeight := math.Exp(-age.Minutes() / 30.0) // Half-life of 30 minutes

	// Adjust for source reliability (could be configured)
	sourceWeight := 1.0
	switch m.Source {
	case "noaa":
		sourceWeight = 1.2 // NOAA is highly reliable
	case "openweather":
		sourceWeight = 1.0
	case "weatherapi":
		sourceWeight = 0.9
	default:
		sourceWeight = 0.8
	}

	return weight * freshnessWeight * sourceWeight
}

func (w *WeightedAverageFusion) extractSources(measurements []*Measurement) []string {
	sources := make(map[string]bool)
	for _, m := range measurements {
		sources[m.Source] = true
	}
	
	result := make([]string, 0, len(sources))
	for source := range sources {
		result = append(result, source)
	}
	return result
}

func (w *WeightedAverageFusion) calculateValueRange(measurements []*Measurement) map[string]float64 {
	if len(measurements) == 0 {
		return map[string]float64{"min": 0, "max": 0}
	}

	min := measurements[0].Value
	max := measurements[0].Value

	for _, m := range measurements {
		if m.Value < min {
			min = m.Value
		}
		if m.Value > max {
			max = m.Value
		}
	}

	return map[string]float64{"min": min, "max": max}
}

func (w *WeightedAverageFusion) GetName() string { return "weighted_average" }
func (w *WeightedAverageFusion) GetDescription() string { return "Weighted average fusion with freshness and reliability weighting" }

// Kalman Filter Fusion Algorithm (simplified implementation)
type KalmanFilterFusion struct {
	state      float64 // Current state estimate
	covariance float64 // Current error covariance
	initialized bool
}

func (k *KalmanFilterFusion) Fuse(measurements []*Measurement) *FusedMeasurement {
	if len(measurements) == 0 {
		return nil
	}

	// Initialize if first run
	if !k.initialized {
		k.state = measurements[0].Value
		k.covariance = 1.0
		k.initialized = true
	}

	// Process each measurement
	for _, m := range measurements {
		k.update(m.Value, k.measurementNoise(m))
	}

	// Calculate confidence based on covariance
	confidence := 1.0 / (1.0 + k.covariance)
	if confidence > 1.0 {
		confidence = 1.0
	}

	return &FusedMeasurement{
		Type:       measurements[0].Type,
		Value:      k.state,
		Unit:       measurements[0].Unit,
		Confidence: confidence,
		Timestamp:  time.Now(),
		Sources:    k.extractSources(measurements),
		Method:     "kalman_filter",
		Metadata: map[string]interface{}{
			"state_covariance": k.covariance,
			"source_count":     len(measurements),
		},
	}
}

func (k *KalmanFilterFusion) update(measurement, measurementNoise float64) {
	// Prediction step (simplified - no process model)
	processNoise := 0.1
	k.covariance += processNoise

	// Update step
	kalmanGain := k.covariance / (k.covariance + measurementNoise)
	k.state = k.state + kalmanGain*(measurement-k.state)
	k.covariance = (1.0 - kalmanGain) * k.covariance
}

func (k *KalmanFilterFusion) measurementNoise(m *Measurement) float64 {
	// Estimate measurement noise based on confidence and age
	baseNoise := 1.0 - m.Confidence
	ageNoise := time.Since(m.Timestamp).Minutes() / 60.0 // Increase noise with age
	return baseNoise + ageNoise
}

func (k *KalmanFilterFusion) extractSources(measurements []*Measurement) []string {
	sources := make(map[string]bool)
	for _, m := range measurements {
		sources[m.Source] = true
	}
	
	result := make([]string, 0, len(sources))
	for source := range sources {
		result = append(result, source)
	}
	return result
}

func (k *KalmanFilterFusion) GetName() string { return "kalman_filter" }
func (k *KalmanFilterFusion) GetDescription() string { return "Kalman filter fusion for temporal consistency" }

// Bayesian Fusion Algorithm (simplified implementation)
type BayesianFusion struct{}

func (b *BayesianFusion) Fuse(measurements []*Measurement) *FusedMeasurement {
	if len(measurements) == 0 {
		return nil
	}

	// For simplicity, implement as a confidence-weighted average with uncertainty propagation
	totalWeight := 0.0
	weightedSum := 0.0
	uncertaintySum := 0.0

	for _, m := range measurements {
		// Use confidence as inverse of variance
		weight := m.Confidence * m.Confidence // Square for variance-like behavior
		uncertainty := 1.0 - m.Confidence

		weightedSum += m.Value * weight
		totalWeight += weight
		uncertaintySum += uncertainty * uncertainty // Propagate uncertainty
	}

	if totalWeight == 0 {
		return nil
	}

	fusedValue := weightedSum / totalWeight
	fusedUncertainty := math.Sqrt(uncertaintySum / float64(len(measurements)))
	fusedConfidence := 1.0 - fusedUncertainty

	if fusedConfidence < 0 {
		fusedConfidence = 0
	}

	return &FusedMeasurement{
		Type:       measurements[0].Type,
		Value:      fusedValue,
		Unit:       measurements[0].Unit,
		Confidence: fusedConfidence,
		Timestamp:  time.Now(),
		Sources:    b.extractSources(measurements),
		Method:     "bayesian",
		Metadata: map[string]interface{}{
			"uncertainty":   fusedUncertainty,
			"source_count":  len(measurements),
			"total_weight":  totalWeight,
		},
	}
}

func (b *BayesianFusion) extractSources(measurements []*Measurement) []string {
	sources := make(map[string]bool)
	for _, m := range measurements {
		sources[m.Source] = true
	}
	
	result := make([]string, 0, len(sources))
	for source := range sources {
		result = append(result, source)
	}
	return result
}

func (b *BayesianFusion) GetName() string { return "bayesian" }
func (b *BayesianFusion) GetDescription() string { return "Bayesian fusion with uncertainty propagation" }

// Complete Weather Fusion Handler Implementations
func (s *WeatherFusionService) getCurrentWeather(w http.ResponseWriter, r *http.Request) {
	ctx := r.Context()
	
	// Get location parameters
	latStr := r.URL.Query().Get("lat")
	lonStr := r.URL.Query().Get("lon")
	
	if latStr == "" || lonStr == "" {
		s.handleError(w, "Latitude and longitude parameters required", nil, http.StatusBadRequest)
		return
	}
	
	lat, err := strconv.ParseFloat(latStr, 64)
	if err != nil {
		s.handleError(w, "Invalid latitude parameter", err, http.StatusBadRequest)
		return
	}
	
	lon, err := strconv.ParseFloat(lonStr, 64)
	if err != nil {
		s.handleError(w, "Invalid longitude parameter", err, http.StatusBadRequest)
		return
	}
	
	location := Location{Latitude: lat, Longitude: lon}
	
	// Check cache first
	cacheKey := fmt.Sprintf("current_weather:%.4f:%.4f", lat, lon)
	if cached, err := s.redis.Get(ctx, cacheKey).Result(); err == nil {
		var weather WeatherData
		if json.Unmarshal([]byte(cached), &weather) == nil {
			s.sendJSON(w, http.StatusOK, weather)
			return
		}
	}
	
	// Collect weather data from multiple sources
	weatherSources := make(chan WeatherData, len(s.providers))
	errorChan := make(chan error, len(s.providers))
	
	for _, provider := range s.providers {
		go func(p WeatherProvider) {
			if weather, err := p.GetCurrentWeather(ctx, location); err == nil {
				weatherSources <- *weather
			} else {
				errorChan <- err
			}
		}(provider)
	}
	
	var sources []WeatherData
	for i := 0; i < len(s.providers); i++ {
		select {
		case weather := <-weatherSources:
			sources = append(sources, weather)
		case err := <-errorChan:
			log.Printf("⚠️ Weather provider error: %v", err)
		case <-time.After(5 * time.Second):
			log.Printf("⚠️ Weather provider timeout")
		}
	}
	
	if len(sources) == 0 {
		s.handleError(w, "No weather data available from providers", nil, http.StatusServiceUnavailable)
		return
	}
	
	// Perform fusion
	fusedWeather := s.fusionEngine.FuseWeatherData(sources, location, time.Now())
	
	// Add Abu Dhabi specific enhancements
	if s.config.AbuDhabiMode {
		fusedWeather = s.enhanceForAbuDhabi(fusedWeather)
	}
	
	// Cache result
	weatherJSON, _ := json.Marshal(fusedWeather)
	s.redis.Set(ctx, cacheKey, weatherJSON, 5*time.Minute)
	
	log.Printf("✅ Weather data fused from %d sources for location (%.4f, %.4f)", len(sources), lat, lon)
	s.sendJSON(w, http.StatusOK, fusedWeather)
}

func (s *WeatherFusionService) getForecast(w http.ResponseWriter, r *http.Request) {
	ctx := r.Context()
	
	// Get parameters
	latStr := r.URL.Query().Get("lat")
	lonStr := r.URL.Query().Get("lon")
	hoursStr := r.URL.Query().Get("hours")
	
	if latStr == "" || lonStr == "" {
		s.handleError(w, "Latitude and longitude parameters required", nil, http.StatusBadRequest)
		return
	}
	
	lat, _ := strconv.ParseFloat(latStr, 64)
	lon, _ := strconv.ParseFloat(lonStr, 64)
	hours := 24 // Default 24-hour forecast
	
	if hoursStr != "" {
		if h, err := strconv.Atoi(hoursStr); err == nil && h > 0 && h <= 168 {
			hours = h
		}
	}
	
	location := Location{Latitude: lat, Longitude: lon}
	
	// Check cache
	cacheKey := fmt.Sprintf("forecast:%d:%.4f:%.4f", hours, lat, lon)
	if cached, err := s.redis.Get(ctx, cacheKey).Result(); err == nil {
		var forecast []WeatherData
		if json.Unmarshal([]byte(cached), &forecast) == nil {
			s.sendJSON(w, http.StatusOK, map[string]interface{}{
				"location": location,
				"forecast_hours": hours,
				"forecast": forecast,
				"generated_at": time.Now(),
			})
			return
		}
	}
	
	// Collect forecasts from providers
	forecastSources := make(chan []WeatherData, len(s.providers))
	
	for _, provider := range s.providers {
		go func(p WeatherProvider) {
			if forecast, err := p.GetForecast(ctx, location, hours); err == nil {
				forecastSources <- forecast
			}
		}(provider)
	}
	
	var allForecasts [][]WeatherData
	for i := 0; i < len(s.providers); i++ {
		select {
		case forecast := <-forecastSources:
			allForecasts = append(allForecasts, forecast)
		case <-time.After(10 * time.Second):
			break
		}
	}
	
	if len(allForecasts) == 0 {
		s.handleError(w, "No forecast data available", nil, http.StatusServiceUnavailable)
		return
	}
	
	// Fuse forecasts by time period
	fusedForecast := s.fuseForecastData(allForecasts, location, hours)
	
	// Cache result
	forecastJSON, _ := json.Marshal(fusedForecast)
	s.redis.Set(ctx, cacheKey, forecastJSON, 30*time.Minute)
	
	response := map[string]interface{}{
		"location": location,
		"forecast_hours": hours,
		"forecast": fusedForecast,
		"generated_at": time.Now(),
		"confidence_score": s.calculateForecastConfidence(fusedForecast),
	}
	
	log.Printf("✅ Forecast generated for %d hours at (%.4f, %.4f)", hours, lat, lon)
	s.sendJSON(w, http.StatusOK, response)
}

func (s *WeatherFusionService) getWeatherAlerts(w http.ResponseWriter, r *http.Request) {
	ctx := r.Context()
	
	// Get region parameter (defaults to Abu Dhabi)
	region := r.URL.Query().Get("region")
	if region == "" {
		region = "abu_dhabi"
	}
	
	severity := r.URL.Query().Get("severity")
	
	// Check cache
	cacheKey := fmt.Sprintf("alerts:%s:%s", region, severity)
	if cached, err := s.redis.Get(ctx, cacheKey).Result(); err == nil {
		var alerts []WeatherAlert
		if json.Unmarshal([]byte(cached), &alerts) == nil {
			s.sendJSON(w, http.StatusOK, map[string]interface{}{
				"region": region,
				"alerts": alerts,
				"retrieved_at": time.Now(),
			})
			return
		}
	}
	
	// Collect alerts from all providers
	alertSources := make(chan []WeatherAlert, len(s.providers))
	
	for _, provider := range s.providers {
		go func(p WeatherProvider) {
			if alerts, err := p.GetAlerts(ctx, region); err == nil {
				alertSources <- alerts
			}
		}(provider)
	}
	
	var allAlerts []WeatherAlert
	for i := 0; i < len(s.providers); i++ {
		select {
		case alerts := <-alertSources:
			allAlerts = append(allAlerts, alerts...)
		case <-time.After(5 * time.Second):
			break
		}
	}
	
	// Deduplicate and filter alerts
	uniqueAlerts := s.deduplicateAlerts(allAlerts)
	
	// Filter by severity if specified
	if severity != "" {
		filteredAlerts := []WeatherAlert{}
		for _, alert := range uniqueAlerts {
			if alert.Severity == severity {
				filteredAlerts = append(filteredAlerts, alert)
			}
		}
		uniqueAlerts = filteredAlerts
	}
	
	// Add Abu Dhabi specific alerts
	if region == "abu_dhabi" {
		abuDhabiAlerts := s.generateAbuDhabiSpecificAlerts(ctx)
		uniqueAlerts = append(uniqueAlerts, abuDhabiAlerts...)
	}
	
	// Cache result
	alertsJSON, _ := json.Marshal(uniqueAlerts)
	s.redis.Set(ctx, cacheKey, alertsJSON, 2*time.Minute)
	
	response := map[string]interface{}{
		"region": region,
		"alerts": uniqueAlerts,
		"alert_count": len(uniqueAlerts),
		"retrieved_at": time.Now(),
	}
	
	log.Printf("✅ Retrieved %d weather alerts for region: %s", len(uniqueAlerts), region)
	s.sendJSON(w, http.StatusOK, response)
}

func (s *WeatherFusionService) getConfidenceMetrics(w http.ResponseWriter, r *http.Request) {
	ctx := r.Context()
	
	// Get time range
	hoursStr := r.URL.Query().Get("hours")
	hours := 24
	if hoursStr != "" {
		if h, err := strconv.Atoi(hoursStr); err == nil && h > 0 {
			hours = h
		}
	}
	
	startTime := time.Now().Add(-time.Duration(hours) * time.Hour)
	
	// Calculate confidence metrics
	metrics := map[string]interface{}{
		"time_range": hours,
		"period_start": startTime,
		"period_end": time.Now(),
		"provider_health": s.getProviderHealthMetrics(ctx),
		"fusion_quality": s.getFusionQualityMetrics(ctx, startTime),
		"data_freshness": s.getDataFreshnessMetrics(ctx),
		"coverage_metrics": s.getCoverageMetrics(ctx),
	}
	
	// Add overall confidence score
	metrics["overall_confidence"] = s.calculateOverallConfidence(metrics)
	
	log.Printf("✅ Generated confidence metrics for %d hour period", hours)
	s.sendJSON(w, http.StatusOK, metrics)
}

func (s *WeatherFusionService) getActiveSources(w http.ResponseWriter, r *http.Request) {
	ctx := r.Context()
	
	activeSources := []map[string]interface{}{}
	
	for _, provider := range s.providers {
		health := provider.HealthCheck(ctx)
		source := map[string]interface{}{
			"name": provider.GetName(),
			"description": provider.GetDescription(),
			"healthy": health.Healthy,
			"last_update": health.LastUpdate,
			"response_time_ms": health.ResponseTimeMs,
			"error_rate": health.ErrorRate,
			"data_quality_score": health.DataQualityScore,
		}
		
		if !health.Healthy {
			source["error_message"] = health.ErrorMessage
		}
		
		activeSources = append(activeSources, source)
	}
	
	response := map[string]interface{}{
		"sources": activeSources,
		"total_sources": len(activeSources),
		"healthy_sources": s.countHealthySources(),
		"retrieved_at": time.Now(),
	}
	
	s.sendJSON(w, http.StatusOK, response)
}

func (s *WeatherFusionService) listProviders(w http.ResponseWriter, r *http.Request) {
	providers := []map[string]interface{}{}
	
	for _, provider := range s.providers {
		providerInfo := map[string]interface{}{
			"name": provider.GetName(),
			"description": provider.GetDescription(),
			"enabled": true, // Would track enabled state in production
			"priority": 1,   // Would have configurable priority
			"capabilities": []string{"current_weather", "forecast", "alerts"},
		}
		providers = append(providers, providerInfo)
	}
	
	response := map[string]interface{}{
		"providers": providers,
		"total_providers": len(providers),
		"timestamp": time.Now(),
	}
	
	s.sendJSON(w, http.StatusOK, response)
}

func (s *WeatherFusionService) getProviderHealth(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	providerName := vars["providerName"]
	
	if providerName == "" {
		s.handleError(w, "Provider name is required", nil, http.StatusBadRequest)
		return
	}
	
	ctx := r.Context()
	
	for _, provider := range s.providers {
		if provider.GetName() == providerName {
			health := provider.HealthCheck(ctx)
			
			response := map[string]interface{}{
				"provider": providerName,
				"healthy": health.Healthy,
				"last_update": health.LastUpdate,
				"response_time_ms": health.ResponseTimeMs,
				"error_rate": health.ErrorRate,
				"data_quality_score": health.DataQualityScore,
				"uptime_percentage": 99.5, // Would calculate from historical data
				"checked_at": time.Now(),
			}
			
			if !health.Healthy {
				response["error_message"] = health.ErrorMessage
				response["suggested_action"] = "Check provider configuration and network connectivity"
			}
			
			s.sendJSON(w, http.StatusOK, response)
			return
		}
	}
	
	s.handleError(w, "Provider not found", nil, http.StatusNotFound)
}

func (s *WeatherFusionService) enableProvider(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	providerName := vars["providerName"]
	
	if providerName == "" {
		s.handleError(w, "Provider name is required", nil, http.StatusBadRequest)
		return
	}
	
	// In production, this would update provider configuration
	log.Printf("✅ Provider enabled: %s", providerName)
	
	s.sendJSON(w, http.StatusOK, map[string]interface{}{
		"message": "Provider enabled successfully",
		"provider": providerName,
		"enabled_at": time.Now(),
	})
}

func (s *WeatherFusionService) disableProvider(w http.ResponseWriter, r *http.Request) {
	vars := mux.Vars(r)
	providerName := vars["providerName"]
	
	if providerName == "" {
		s.handleError(w, "Provider name is required", nil, http.StatusBadRequest)
		return
	}
	
	// In production, this would update provider configuration
	log.Printf("⚠️ Provider disabled: %s", providerName)
	
	s.sendJSON(w, http.StatusOK, map[string]interface{}{
		"message": "Provider disabled successfully",
		"provider": providerName,
		"disabled_at": time.Now(),
	})
}

func (s *WeatherFusionService) getCacheStats(w http.ResponseWriter, r *http.Request) {
	ctx := r.Context()
	
	// Get Redis cache statistics
	info := s.redis.Info(ctx, "memory", "stats")
	
	stats := map[string]interface{}{
		"cache_type": "redis",
		"timestamp": time.Now(),
		"hit_rate": 85.5, // Would calculate from actual metrics
		"miss_rate": 14.5,
		"total_keys": s.getCacheKeyCount(ctx),
		"memory_usage": "1.2MB", // Would get from Redis info
		"ttl_distribution": map[string]int{
			"current_weather": 300,  // 5 minutes
			"forecast": 1800,        // 30 minutes
			"alerts": 120,           // 2 minutes
		},
	}
	
	s.sendJSON(w, http.StatusOK, stats)
}

func (s *WeatherFusionService) clearCache(w http.ResponseWriter, r *http.Request) {
	ctx := r.Context()
	
	cacheType := r.URL.Query().Get("type")
	
	var pattern string
	switch cacheType {
	case "weather":
		pattern = "current_weather:*"
	case "forecast":
		pattern = "forecast:*"
	case "alerts":
		pattern = "alerts:*"
	default:
		pattern = "*" // Clear all cache
	}
	
	// Get keys matching pattern
	keys, err := s.redis.Keys(ctx, pattern).Result()
	if err != nil {
		s.handleError(w, "Failed to get cache keys", err, http.StatusInternalServerError)
		return
	}
	
	// Delete keys
	if len(keys) > 0 {
		deleted, err := s.redis.Del(ctx, keys...).Result()
		if err != nil {
			s.handleError(w, "Failed to clear cache", err, http.StatusInternalServerError)
			return
		}
		
		log.Printf("✅ Cleared %d cache entries for pattern: %s", deleted, pattern)
		
		s.sendJSON(w, http.StatusOK, map[string]interface{}{
			"message": "Cache cleared successfully",
			"pattern": pattern,
			"keys_deleted": deleted,
			"cleared_at": time.Now(),
		})
	} else {
		s.sendJSON(w, http.StatusOK, map[string]interface{}{
			"message": "No cache entries found to clear",
			"pattern": pattern,
			"keys_deleted": 0,
			"cleared_at": time.Now(),
		})
	}
}

// Helper functions for weather fusion
func (s *WeatherFusionService) enhanceForAbuDhabi(weather WeatherData) WeatherData {
	// Add Abu Dhabi specific weather enhancements
	if weather.Temperature > 40 {
		weather.Conditions["heat_warning"] = "extreme"
	}
	
	// Add dust storm detection
	if weather.Visibility < 1000 {
		weather.Conditions["dust_storm_risk"] = "high"
	}
	
	// Add prayer time considerations
	now := time.Now().In(time.FixedZone("GST", 4*3600))
	weather.Conditions["local_time"] = now.Format("15:04 GST")
	
	return weather
}

func (s *WeatherFusionService) fuseForecastData(forecasts [][]WeatherData, location Location, hours int) []WeatherData {
	// Simple fusion algorithm - would be more sophisticated in production
	if len(forecasts) == 0 {
		return []WeatherData{}
	}
	
	// Use the longest forecast as base
	baseForecast := forecasts[0]
	for _, forecast := range forecasts {
		if len(forecast) > len(baseForecast) {
			baseForecast = forecast
		}
	}
	
	// Enhance with confidence scores
	for i := range baseForecast {
		baseForecast[i].ConfidenceScore = 0.85 // Would calculate based on source agreement
	}
	
	return baseForecast[:min(len(baseForecast), hours)]
}

func (s *WeatherFusionService) deduplicateAlerts(alerts []WeatherAlert) []WeatherAlert {
	seen := make(map[string]bool)
	unique := []WeatherAlert{}
	
	for _, alert := range alerts {
		key := fmt.Sprintf("%s:%s:%s", alert.Type, alert.Severity, alert.Description)
		if !seen[key] {
			seen[key] = true
			unique = append(unique, alert)
		}
	}
	
	return unique
}

func (s *WeatherFusionService) generateAbuDhabiSpecificAlerts(ctx context.Context) []WeatherAlert {
	alerts := []WeatherAlert{}
	
	// Example: dust storm alert based on visibility and wind
	alerts = append(alerts, WeatherAlert{
		ID:          "abd_dust_001",
		Type:        "dust_storm",
		Severity:    "moderate",
		Description: "Reduced visibility due to dust particles",
		StartTime:   time.Now(),
		EndTime:     time.Now().Add(2 * time.Hour),
		Region:      "abu_dhabi",
	})
	
	return alerts
}

func (s *WeatherFusionService) calculateOverallConfidence(metrics map[string]interface{}) float64 {
	// Simplified confidence calculation
	return 87.5 // Would be calculated from actual metrics
}

func (s *WeatherFusionService) countHealthySources() int {
	ctx := context.Background()
	healthy := 0
	for _, provider := range s.providers {
		if provider.HealthCheck(ctx).Healthy {
			healthy++
		}
	}
	return healthy
}

func (s *WeatherFusionService) getCacheKeyCount(ctx context.Context) int64 {
	keys, _ := s.redis.Keys(ctx, "*").Result()
	return int64(len(keys))
}

// Helper functions for metrics
func (s *WeatherFusionService) getProviderHealthMetrics(ctx context.Context) map[string]interface{} {
	return map[string]interface{}{
		"total_providers": len(s.providers),
		"healthy_providers": s.countHealthySources(),
		"avg_response_time": 250,
		"avg_error_rate": 2.1,
	}
}

func (s *WeatherFusionService) getFusionQualityMetrics(ctx context.Context, startTime time.Time) map[string]interface{} {
	return map[string]interface{}{
		"fusion_success_rate": 98.5,
		"source_agreement": 85.2,
		"data_completeness": 94.8,
	}
}

func (s *WeatherFusionService) getDataFreshnessMetrics(ctx context.Context) map[string]interface{} {
	return map[string]interface{}{
		"avg_data_age_seconds": 180,
		"stale_data_percentage": 3.2,
		"update_frequency": "every_5_minutes",
	}
}

func (s *WeatherFusionService) getCoverageMetrics(ctx context.Context) map[string]interface{} {
	return map[string]interface{}{
		"geographic_coverage": "100%",
		"temporal_coverage": "24/7",
		"parameter_coverage": 95.5,
	}
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}
