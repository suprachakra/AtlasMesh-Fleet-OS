package metrics

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.uber.org/zap"
	"github.com/atlasmesh/control-center-ui/internal/config"
)

// Metrics represents the metrics collector
type Metrics struct {
	config *config.MetricsConfig
	logger *zap.Logger
	server *http.Server
	
	// Prometheus metrics
	DashboardViews     prometheus.Counter
	WebSocketConnections prometheus.Gauge
	ResponseTime       prometheus.Histogram
	ActiveUsers        prometheus.Gauge
	ErrorRate          prometheus.Counter
}

// New creates a new metrics collector
func New(cfg *config.MetricsConfig) *Metrics {
	metrics := &Metrics{
		config: cfg,
		logger: zap.NewNop(),
	}

	// Initialize Prometheus metrics
	metrics.DashboardViews = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "control_center_dashboard_views_total",
		Help: "Total number of dashboard views",
	})

	metrics.WebSocketConnections = prometheus.NewGauge(prometheus.GaugeOpts{
		Name: "control_center_websocket_connections",
		Help: "Current number of WebSocket connections",
	})

	metrics.ResponseTime = prometheus.NewHistogram(prometheus.HistogramOpts{
		Name:    "control_center_response_time_seconds",
		Help:    "Response time for control center requests",
		Buckets: prometheus.DefBuckets,
	})

	metrics.ActiveUsers = prometheus.NewGauge(prometheus.GaugeOpts{
		Name: "control_center_active_users",
		Help: "Current number of active users",
	})

	metrics.ErrorRate = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "control_center_errors_total",
		Help: "Total number of errors",
	})

	// Register metrics
	prometheus.MustRegister(
		metrics.DashboardViews,
		metrics.WebSocketConnections,
		metrics.ResponseTime,
		metrics.ActiveUsers,
		metrics.ErrorRate,
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

// IncrementDashboardViews increments the dashboard views counter
func (m *Metrics) IncrementDashboardViews() {
	m.DashboardViews.Inc()
}

// SetWebSocketConnections sets the WebSocket connections gauge
func (m *Metrics) SetWebSocketConnections(count float64) {
	m.WebSocketConnections.Set(count)
}

// ObserveResponseTime records response time
func (m *Metrics) ObserveResponseTime(duration time.Duration) {
	m.ResponseTime.Observe(duration.Seconds())
}

// SetActiveUsers sets the active users gauge
func (m *Metrics) SetActiveUsers(count float64) {
	m.ActiveUsers.Set(count)
}

// IncrementErrorRate increments the error rate counter
func (m *Metrics) IncrementErrorRate() {
	m.ErrorRate.Inc()
}
