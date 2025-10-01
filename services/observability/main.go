// AtlasMesh Observability Service - Distributed tracing, metrics, and SLO monitoring
package main

import (
	"context"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
)

type ObservabilityService struct {
	registry        *prometheus.Registry
	requestDuration *prometheus.HistogramVec
	sloCompliance   *prometheus.GaugeVec
}

func main() {
	log.Println("AtlasMesh Observability Service starting...")

	service := &ObservabilityService{
		registry: prometheus.NewRegistry(),
	}
	service.initMetrics()

	router := gin.New()
	router.Use(gin.Logger(), gin.Recovery())
	
	router.GET("/health", func(c *gin.Context) {
		c.JSON(200, gin.H{"status": "healthy", "service": "observability"})
	})
	
	router.GET("/metrics", gin.WrapH(promhttp.HandlerFor(service.registry, promhttp.HandlerOpts{})))
	
	router.GET("/api/v1/slos", func(c *gin.Context) {
		c.JSON(200, gin.H{
			"slos": []map[string]interface{}{
				{"name": "fleet_availability", "target": 0.99, "current": 0.995, "status": "healthy"},
				{"name": "control_loop_latency", "target": 0.95, "current": 0.98, "status": "healthy"},
			},
		})
	})

	go service.startMonitoring()

	server := &http.Server{Addr: ":8081", Handler: router}
	
	go func() {
		log.Println("HTTP server listening on port 8081")
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Server error: %v", err)
		}
	}()

	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("Shutting down...")
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	server.Shutdown(ctx)
}

func (s *ObservabilityService) initMetrics() {
	s.requestDuration = prometheus.NewHistogramVec(
		prometheus.HistogramOpts{
			Namespace: "atlasmesh",
			Name:      "request_duration_seconds",
			Help:      "Request duration in seconds",
		},
		[]string{"service", "method", "status"},
	)

	s.sloCompliance = prometheus.NewGaugeVec(
		prometheus.GaugeOpts{
			Namespace: "atlasmesh",
			Name:      "slo_compliance_ratio",
			Help:      "SLO compliance ratio",
		},
		[]string{"slo_name", "service"},
	)

	s.registry.MustRegister(s.requestDuration, s.sloCompliance)
}

func (s *ObservabilityService) startMonitoring() {
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			s.sloCompliance.WithLabelValues("fleet_availability", "fleet-manager").Set(0.995)
			s.sloCompliance.WithLabelValues("control_loop_latency", "vehicle-control").Set(0.98)
			s.sloCompliance.WithLabelValues("policy_evaluation", "policy-engine").Set(0.997)
		}
	}
}
