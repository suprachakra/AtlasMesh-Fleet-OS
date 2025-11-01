package metrics

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.uber.org/zap"
	"github.com/atlasmesh/wms-adapters/manhattan-scale/internal/config"
)

// Metrics represents the metrics collector
type Metrics struct {
	config *config.MetricsConfig
	logger *zap.Logger
	server *http.Server
	
	// Prometheus metrics
	TasksTotal      prometheus.Counter
	TasksProcessed  prometheus.Counter
	TasksFailed     prometheus.Counter
	TaskDuration    prometheus.Histogram
	ActiveTasks     prometheus.Gauge
	InventoryItems  prometheus.Gauge
	ResourcesTotal  prometheus.Gauge
}

// New creates a new metrics collector
func New(cfg *config.MetricsConfig) *Metrics {
	metrics := &Metrics{
		config: cfg,
		logger: zap.NewNop(),
	}

	// Initialize Prometheus metrics
	metrics.TasksTotal = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "manhattan_tasks_total",
		Help: "Total number of tasks processed",
	})

	metrics.TasksProcessed = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "manhattan_tasks_processed_total",
		Help: "Total number of tasks successfully processed",
	})

	metrics.TasksFailed = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "manhattan_tasks_failed_total",
		Help: "Total number of tasks that failed",
	})

	metrics.TaskDuration = prometheus.NewHistogram(prometheus.HistogramOpts{
		Name:    "manhattan_task_duration_seconds",
		Help:    "Duration of task processing in seconds",
		Buckets: prometheus.DefBuckets,
	})

	metrics.ActiveTasks = prometheus.NewGauge(prometheus.GaugeOpts{
		Name: "manhattan_active_tasks",
		Help: "Number of active tasks",
	})

	metrics.InventoryItems = prometheus.NewGauge(prometheus.GaugeOpts{
		Name: "manhattan_inventory_items",
		Help: "Number of inventory items",
	})

	metrics.ResourcesTotal = prometheus.NewGauge(prometheus.GaugeOpts{
		Name: "manhattan_resources_total",
		Help: "Total number of resources",
	})

	// Register metrics
	prometheus.MustRegister(
		metrics.TasksTotal,
		metrics.TasksProcessed,
		metrics.TasksFailed,
		metrics.TaskDuration,
		metrics.ActiveTasks,
		metrics.InventoryItems,
		metrics.ResourcesTotal,
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

// IncrementTasksTotal increments the total tasks counter
func (m *Metrics) IncrementTasksTotal() {
	m.TasksTotal.Inc()
}

// IncrementTasksProcessed increments the processed tasks counter
func (m *Metrics) IncrementTasksProcessed() {
	m.TasksProcessed.Inc()
}

// IncrementTasksFailed increments the failed tasks counter
func (m *Metrics) IncrementTasksFailed() {
	m.TasksFailed.Inc()
}

// ObserveTaskDuration records task processing duration
func (m *Metrics) ObserveTaskDuration(duration time.Duration) {
	m.TaskDuration.Observe(duration.Seconds())
}

// SetActiveTasks sets the active tasks gauge
func (m *Metrics) SetActiveTasks(count float64) {
	m.ActiveTasks.Set(count)
}

// SetInventoryItems sets the inventory items gauge
func (m *Metrics) SetInventoryItems(count float64) {
	m.InventoryItems.Set(count)
}

// SetResourcesTotal sets the resources total gauge
func (m *Metrics) SetResourcesTotal(count float64) {
	m.ResourcesTotal.Set(count)
}
