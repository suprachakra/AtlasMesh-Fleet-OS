package metrics

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.uber.org/zap"
	"github.com/atlasmesh/mining-adapters/wenco/internal/config"
)

// Metrics represents the metrics collector
type Metrics struct {
	config *config.MetricsConfig
	logger *zap.Logger
	server *http.Server
	
	// Prometheus metrics
	EquipmentStatus    prometheus.Counter
	ProductionTonnes   prometheus.Counter
	OptimizationTime   prometheus.Histogram
	EquipmentHealth    prometheus.Gauge
	LocationUpdates    prometheus.Counter
}

// New creates a new metrics collector
func New(cfg *config.MetricsConfig) *Metrics {
	metrics := &Metrics{
		config: cfg,
		logger: zap.NewNop(),
	}

	// Initialize Prometheus metrics
	metrics.EquipmentStatus = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "wenco_equipment_status_total",
		Help: "Total number of equipment status updates",
	})

	metrics.ProductionTonnes = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "wenco_production_tonnes",
		Help: "Total production in tonnes",
	})

	metrics.OptimizationTime = prometheus.NewHistogram(prometheus.HistogramOpts{
		Name:    "wenco_optimization_time_seconds",
		Help:    "Duration of fleet optimization",
		Buckets: prometheus.DefBuckets,
	})

	metrics.EquipmentHealth = prometheus.NewGauge(prometheus.GaugeOpts{
		Name: "wenco_equipment_health_score",
		Help: "Equipment health score (0-100)",
	})

	metrics.LocationUpdates = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "wenco_location_updates_total",
		Help: "Total number of location updates",
	})

	// Register metrics
	prometheus.MustRegister(
		metrics.EquipmentStatus,
		metrics.ProductionTonnes,
		metrics.OptimizationTime,
		metrics.EquipmentHealth,
		metrics.LocationUpdates,
	)

	return metrics
}

// Start starts the metrics server
func (m *Metrics) Start(ctx context.Context) error {
	if !m.config.Enabled {
		return nil
	}

	mux := http.NewServeMux()
	mux.Handle(m.config.Path, promhttp.Handler())

	m.server = &http.Server{
		Addr:    fmt.Sprintf(":%d", m.config.Port),
		Handler: mux,
	}

	go func() {
		if err := m.server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			m.logger.Error("Failed to start metrics server", zap.Error(err))
		}
	}()

	m.logger.Info("Metrics server started", zap.Int("port", m.config.Port))
	return nil
}

// Stop stops the metrics server
func (m *Metrics) Stop() error {
	if m.server != nil {
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		return m.server.Shutdown(ctx)
	}
	return nil
}

// IncrementEquipmentStatus increments the equipment status counter
func (m *Metrics) IncrementEquipmentStatus() {
	m.EquipmentStatus.Inc()
}

// AddProductionTonnes adds production tonnes
func (m *Metrics) AddProductionTonnes(tonnes float64) {
	m.ProductionTonnes.Add(tonnes)
}

// ObserveOptimizationTime records optimization duration
func (m *Metrics) ObserveOptimizationTime(duration time.Duration) {
	m.OptimizationTime.Observe(duration.Seconds())
}

// SetEquipmentHealth sets the equipment health gauge
func (m *Metrics) SetEquipmentHealth(score float64) {
	m.EquipmentHealth.Set(score)
}

// IncrementLocationUpdates increments the location updates counter
func (m *Metrics) IncrementLocationUpdates() {
	m.LocationUpdates.Inc()
}
