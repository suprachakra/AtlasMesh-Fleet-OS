package metrics

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.uber.org/zap"
	"github.com/atlasmesh/dispatch-service/internal/config"
)

// Metrics represents the metrics collector
type Metrics struct {
	config *config.MetricsConfig
	logger *zap.Logger
	server *http.Server
	
	// Prometheus metrics
	TasksDispatched    prometheus.Counter
	TasksCompleted     prometheus.Counter
	TasksFailed        prometheus.Counter
	DispatchDuration   prometheus.Histogram
	ActiveTasks        prometheus.Gauge
	QueueSize          prometheus.Gauge
	OptimizationTime   prometheus.Histogram
}

// New creates a new metrics collector
func New(cfg *config.MetricsConfig) *Metrics {
	metrics := &Metrics{
		config: cfg,
		logger: zap.NewNop(),
	}

	// Initialize Prometheus metrics
	metrics.TasksDispatched = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "dispatch_tasks_dispatched_total",
		Help: "Total number of tasks dispatched",
	})

	metrics.TasksCompleted = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "dispatch_tasks_completed_total",
		Help: "Total number of tasks completed",
	})

	metrics.TasksFailed = prometheus.NewCounter(prometheus.CounterOpts{
		Name: "dispatch_tasks_failed_total",
		Help: "Total number of tasks that failed",
	})

	metrics.DispatchDuration = prometheus.NewHistogram(prometheus.HistogramOpts{
		Name:    "dispatch_task_duration_seconds",
		Help:    "Duration of task dispatch in seconds",
		Buckets: prometheus.DefBuckets,
	})

	metrics.ActiveTasks = prometheus.NewGauge(prometheus.GaugeOpts{
		Name: "dispatch_active_tasks",
		Help: "Number of active tasks",
	})

	metrics.QueueSize = prometheus.NewGauge(prometheus.GaugeOpts{
		Name: "dispatch_queue_size",
		Help: "Current queue size",
	})

	metrics.OptimizationTime = prometheus.NewHistogram(prometheus.HistogramOpts{
		Name:    "dispatch_optimization_duration_seconds",
		Help:    "Duration of optimization in seconds",
		Buckets: prometheus.DefBuckets,
	})

	// Register metrics
	prometheus.MustRegister(
		metrics.TasksDispatched,
		metrics.TasksCompleted,
		metrics.TasksFailed,
		metrics.DispatchDuration,
		metrics.ActiveTasks,
		metrics.QueueSize,
		metrics.OptimizationTime,
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

// IncrementTasksDispatched increments the dispatched tasks counter
func (m *Metrics) IncrementTasksDispatched() {
	m.TasksDispatched.Inc()
}

// IncrementTasksCompleted increments the completed tasks counter
func (m *Metrics) IncrementTasksCompleted() {
	m.TasksCompleted.Inc()
}

// IncrementTasksFailed increments the failed tasks counter
func (m *Metrics) IncrementTasksFailed() {
	m.TasksFailed.Inc()
}

// ObserveDispatchDuration records task dispatch duration
func (m *Metrics) ObserveDispatchDuration(duration time.Duration) {
	m.DispatchDuration.Observe(duration.Seconds())
}

// SetActiveTasks sets the active tasks gauge
func (m *Metrics) SetActiveTasks(count float64) {
	m.ActiveTasks.Set(count)
}

// SetQueueSize sets the queue size gauge
func (m *Metrics) SetQueueSize(count float64) {
	m.QueueSize.Set(count)
}

// ObserveOptimizationTime records optimization duration
func (m *Metrics) ObserveOptimizationTime(duration time.Duration) {
	m.OptimizationTime.Observe(duration.Seconds())
}
