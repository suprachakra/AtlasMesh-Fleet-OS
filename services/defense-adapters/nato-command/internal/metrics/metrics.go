package metrics

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.uber.org/zap"
	"github.com/atlasmesh/defense-adapters/nato-command/internal/config"
)

// Metrics represents the metrics collector
type Metrics struct {
	config *config.MetricsConfig
	logger *zap.Logger
	server *http.Server
	
	// Prometheus metrics
	NATORequests      prometheus.Counter
	NATOResponseTime  prometheus.Histogram
	NATOErrors        prometheus.Counter
	ActiveConnections prometheus.Gauge
	SecurityEvents    prometheus.Counter
}

// New creates a new metrics collector
func New(cfg *config.MetricsConfig) *Metrics {
	metrics := &Metrics{
		config: cfg,
		logger: zap.NewNop(),
	}

	// Initialize Prometheus metrics
	metrics.NATORequests = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "nato_command_requests_total",
		Help: "Total number of NATO command requests",
	})

	metrics.NATOResponseTime = prometheus.NewHistogram(prometheus.HistogramOpts{
		Name:    "nato_command_response_time_seconds",
		Help:    "Response time for NATO command requests",
		Buckets: prometheus.DefBuckets,
	})

	metrics.NATOErrors = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "nato_command_errors_total",
		Help: "Total number of NATO command errors",
	})

	metrics.ActiveConnections = prometheus.NewGauge(prometheus.GaugeOpts{
		Name: "nato_command_active_connections",
		Help: "Current number of active connections",
	})

	metrics.SecurityEvents = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "nato_command_security_events_total",
		Help: "Total number of security events",
	})

	// Register metrics
	prometheus.MustRegister(
		metrics.NATORequests,
		metrics.NATOResponseTime,
		metrics.NATOErrors,
		metrics.ActiveConnections,
		metrics.SecurityEvents,
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

// IncrementNATORequests increments the NATO requests counter
func (m *Metrics) IncrementNATORequests() {
	m.NATORequests.Inc()
}

// ObserveNATOResponseTime records NATO response time
func (m *Metrics) ObserveNATOResponseTime(duration time.Duration) {
	m.NATOResponseTime.Observe(duration.Seconds())
}

// IncrementNATOErrors increments the NATO errors counter
func (m *Metrics) IncrementNATOErrors() {
	m.NATOErrors.Inc()
}

// SetActiveConnections sets the active connections gauge
func (m *Metrics) SetActiveConnections(count float64) {
	m.ActiveConnections.Set(count)
}

// IncrementSecurityEvents increments the security events counter
func (m *Metrics) IncrementSecurityEvents() {
	m.SecurityEvents.Inc()
}
