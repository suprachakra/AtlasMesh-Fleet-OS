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
	"google.golang.org/grpc/reflection"

	"github.com/atlasmesh/fleet-os/services/trip-service/internal/config"
	"github.com/atlasmesh/fleet-os/services/trip-service/internal/handler"
	"github.com/atlasmesh/fleet-os/services/trip-service/internal/repository"
	"github.com/atlasmesh/fleet-os/services/trip-service/internal/service"
	pb "github.com/atlasmesh/fleet-os/services/trip-service/proto"
)

var (
	version   = "dev"
	commit    = "unknown"
	buildDate = "unknown"
)

func main() {
	log.Printf("AtlasMesh Trip Service starting - version: %s, commit: %s, built: %s", version, commit, buildDate)

	// Load configuration
	cfg, err := config.Load()
	if err != nil {
		log.Fatalf("Failed to load configuration: %v", err)
	}

	// Initialize repositories
	tripRepo, err := repository.NewTripRepository(cfg.Database.URL)
	if err != nil {
		log.Fatalf("Failed to initialize trip repository: %v", err)
	}
	defer tripRepo.Close()

	auditRepo, err := repository.NewAuditRepository(cfg.Database.URL)
	if err != nil {
		log.Fatalf("Failed to initialize audit repository: %v", err)
	}
	defer auditRepo.Close()

	eventRepo, err := repository.NewEventRepository(cfg.EventStore.URL)
	if err != nil {
		log.Fatalf("Failed to initialize event repository: %v", err)
	}
	defer eventRepo.Close()

	// Initialize external service clients
	policyClient, err := service.NewPolicyEngineClient(cfg.PolicyEngine.Address)
	if err != nil {
		log.Fatalf("Failed to initialize policy engine client: %v", err)
	}
	defer policyClient.Close()

	// Initialize services
	tripService := service.NewTripService(tripRepo, auditRepo, eventRepo, policyClient, cfg)

	// Initialize gRPC server
	grpcServer := grpc.NewServer(
		grpc.UnaryInterceptor(service.UnaryLoggingInterceptor),
		grpc.StreamInterceptor(service.StreamLoggingInterceptor),
	)

	// Register services
	pb.RegisterTripServiceServer(grpcServer, handler.NewTripServiceHandler(tripService))
	
	// Register health check
	healthServer := health.NewServer()
	grpc_health_v1.RegisterHealthServer(grpcServer, healthServer)
	healthServer.SetServingStatus("", grpc_health_v1.HealthCheckResponse_SERVING)
	
	// Enable reflection for development
	if cfg.Environment == "development" {
		reflection.Register(grpcServer)
	}

	// Start gRPC server
	lis, err := net.Listen("tcp", fmt.Sprintf(":%d", cfg.Server.GRPCPort))
	if err != nil {
		log.Fatalf("Failed to listen on gRPC port %d: %v", cfg.Server.GRPCPort, err)
	}

	go func() {
		log.Printf("gRPC server listening on port %d", cfg.Server.GRPCPort)
		if err := grpcServer.Serve(lis); err != nil {
			log.Fatalf("Failed to serve gRPC: %v", err)
		}
	}()

	// Start HTTP server for health checks and metrics
	httpRouter := gin.New()
	httpRouter.Use(gin.Logger(), gin.Recovery())

	// Health endpoint
	httpRouter.GET("/health", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"status":     "healthy",
			"service":    "trip-service",
			"version":    version,
			"commit":     commit,
			"build_date": buildDate,
			"timestamp":  time.Now().UTC().Format(time.RFC3339),
		})
	})

	// Readiness endpoint
	httpRouter.GET("/ready", func(c *gin.Context) {
		// Check database connectivity
		if err := tripRepo.HealthCheck(c.Request.Context()); err != nil {
			c.JSON(http.StatusServiceUnavailable, gin.H{
				"status": "not_ready",
				"error":  err.Error(),
			})
			return
		}

		// Check policy engine connectivity
		if err := policyClient.HealthCheck(c.Request.Context()); err != nil {
			c.JSON(http.StatusServiceUnavailable, gin.H{
				"status": "not_ready",
				"error":  fmt.Sprintf("policy engine unavailable: %v", err),
			})
			return
		}

		c.JSON(http.StatusOK, gin.H{
			"status": "ready",
		})
	})

	// Metrics endpoint
	httpRouter.GET("/metrics", gin.WrapH(promhttp.Handler()))

	httpServer := &http.Server{
		Addr:    fmt.Sprintf(":%d", cfg.Server.HTTPPort),
		Handler: httpRouter,
	}

	go func() {
		log.Printf("HTTP server listening on port %d", cfg.Server.HTTPPort)
		if err := httpServer.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Failed to serve HTTP: %v", err)
		}
	}()

	// Wait for interrupt signal to gracefully shutdown
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("Shutting down servers...")

	// Graceful shutdown
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()

	// Shutdown HTTP server
	if err := httpServer.Shutdown(ctx); err != nil {
		log.Printf("HTTP server forced to shutdown: %v", err)
	}

	// Shutdown gRPC server
	grpcServer.GracefulStop()

	log.Println("Servers shutdown complete")
}
