package main

import (
	"context"
	"fmt"
	"log"
	"net"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"google.golang.org/grpc"
	"google.golang.org/grpc/health"
	"google.golang.org/grpc/health/grpc_health_v1"

	"github.com/atlasmesh/fleet-os/services/feature-flags/internal/config"
	"github.com/atlasmesh/fleet-os/services/feature-flags/internal/service"
	"github.com/atlasmesh/fleet-os/services/feature-flags/internal/storage"
	pb "github.com/atlasmesh/fleet-os/services/feature-flags/proto"
)

func main() {
	// Load configuration
	cfg, err := config.Load()
	if err != nil {
		log.Fatalf("Failed to load configuration: %v", err)
	}

	// Initialize storage
	store, err := storage.NewRedisStorage(cfg.Redis)
	if err != nil {
		log.Fatalf("Failed to initialize storage: %v", err)
	}
	defer store.Close()

	// Initialize feature flags service
	featureFlagsService := service.NewFeatureFlagsService(store, cfg)

	// Start gRPC server
	go startGRPCServer(featureFlagsService, cfg.GRPC.Port)

	// Start HTTP server for REST API and metrics
	go startHTTPServer(featureFlagsService, cfg.HTTP.Port)

	// Wait for shutdown signal
	waitForShutdown()
}

func startGRPCServer(svc *service.FeatureFlagsService, port int) {
	lis, err := net.Listen("tcp", fmt.Sprintf(":%d", port))
	if err != nil {
		log.Fatalf("Failed to listen on port %d: %v", port, err)
	}

	s := grpc.NewServer()
	
	// Register feature flags service
	pb.RegisterFeatureFlagsServiceServer(s, svc)
	
	// Register health check service
	healthServer := health.NewServer()
	grpc_health_v1.RegisterHealthServer(s, healthServer)
	healthServer.SetServingStatus("", grpc_health_v1.HealthCheckResponse_SERVING)

	log.Printf("gRPC server listening on port %d", port)
	if err := s.Serve(lis); err != nil {
		log.Fatalf("Failed to serve gRPC: %v", err)
	}
}

func startHTTPServer(svc *service.FeatureFlagsService, port int) {
	gin.SetMode(gin.ReleaseMode)
	r := gin.New()
	r.Use(gin.Logger(), gin.Recovery())

	// Health check endpoints
	r.GET("/health", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"status":    "healthy",
			"timestamp": time.Now().UTC().Format(time.RFC3339),
			"service":   "feature-flags",
			"version":   "1.0.0",
		})
	})

	r.GET("/ready", func(c *gin.Context) {
		// Check if service is ready (e.g., database connections)
		if err := svc.HealthCheck(c.Request.Context()); err != nil {
			c.JSON(http.StatusServiceUnavailable, gin.H{
				"status": "not ready",
				"error":  err.Error(),
			})
			return
		}

		c.JSON(http.StatusOK, gin.H{
			"status":    "ready",
			"timestamp": time.Now().UTC().Format(time.RFC3339),
		})
	})

	// Metrics endpoint
	r.GET("/metrics", gin.WrapH(promhttp.Handler()))

	// REST API endpoints
	api := r.Group("/api/v1")
	{
		// Feature flag evaluation
		api.POST("/evaluate", svc.EvaluateHTTP)
		api.POST("/evaluate/batch", svc.BatchEvaluateHTTP)
		
		// Feature flag management
		api.GET("/flags", svc.ListFlagsHTTP)
		api.POST("/flags", svc.CreateFlagHTTP)
		api.GET("/flags/:flag_key", svc.GetFlagHTTP)
		api.PUT("/flags/:flag_key", svc.UpdateFlagHTTP)
		api.DELETE("/flags/:flag_key", svc.DeleteFlagHTTP)
		
		// Cohort management
		api.GET("/cohorts", svc.ListCohortsHTTP)
		api.POST("/cohorts", svc.CreateCohortHTTP)
		api.GET("/cohorts/:cohort_id", svc.GetCohortHTTP)
		api.PUT("/cohorts/:cohort_id", svc.UpdateCohortHTTP)
		api.DELETE("/cohorts/:cohort_id", svc.DeleteCohortHTTP)
		
		// User/entity management
		api.POST("/entities/:entity_id/cohorts", svc.AssignEntityToCohortHTTP)
		api.DELETE("/entities/:entity_id/cohorts/:cohort_id", svc.RemoveEntityFromCohortHTTP)
		api.GET("/entities/:entity_id/flags", svc.GetEntityFlagsHTTP)
		
		// Analytics and reporting
		api.GET("/analytics/flags/:flag_key", svc.GetFlagAnalyticsHTTP)
		api.GET("/analytics/cohorts/:cohort_id", svc.GetCohortAnalyticsHTTP)
		
		// Kill switches
		api.POST("/flags/:flag_key/kill", svc.KillSwitchHTTP)
		api.POST("/flags/:flag_key/revive", svc.ReviveFlagHTTP)
	}

	// Admin endpoints
	admin := r.Group("/admin")
	{
		admin.POST("/cache/clear", svc.ClearCacheHTTP)
		admin.GET("/stats", svc.GetStatsHTTP)
		admin.POST("/export", svc.ExportConfigHTTP)
		admin.POST("/import", svc.ImportConfigHTTP)
	}

	log.Printf("HTTP server listening on port %d", port)
	if err := r.Run(fmt.Sprintf(":%d", port)); err != nil {
		log.Fatalf("Failed to serve HTTP: %v", err)
	}
}

func waitForShutdown() {
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)
	
	sig := <-sigChan
	log.Printf("Received signal %v, shutting down gracefully...", sig)
	
	// Add graceful shutdown logic here
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	
	// Perform cleanup
	_ = ctx
	
	log.Println("Shutdown complete")
}
