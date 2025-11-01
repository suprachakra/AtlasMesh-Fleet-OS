package metrics

import (
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promauto"
)

// Metrics represents the metrics collector
type Metrics struct {
	EmergencyStops prometheus.Counter
	ControlCommands prometheus.Counter
	TelemetryMessages prometheus.Counter
	SafetyViolations prometheus.Counter
	ProcessingLatency prometheus.Histogram
}

// New creates a new metrics collector
func New(config interface{}) *Metrics {
	return &Metrics{
		EmergencyStops: promauto.NewCounter(prometheus.CounterOpts{
			Name: "vehicle_agent_emergency_stops_total",
			Help: "Total number of emergency stops",
		}),
		ControlCommands: promauto.NewCounter(prometheus.CounterOpts{
			Name: "vehicle_agent_control_commands_total",
			Help: "Total number of control commands executed",
		}),
		TelemetryMessages: promauto.NewCounter(prometheus.CounterOpts{
			Name: "vehicle_agent_telemetry_messages_total",
			Help: "Total number of telemetry messages sent",
		}),
		SafetyViolations: promauto.NewCounter(prometheus.CounterOpts{
			Name: "vehicle_agent_safety_violations_total",
			Help: "Total number of safety violations",
		}),
		ProcessingLatency: promauto.NewHistogram(prometheus.HistogramOpts{
			Name: "vehicle_agent_processing_latency_seconds",
			Help: "Processing latency in seconds",
		}),
	}
}
