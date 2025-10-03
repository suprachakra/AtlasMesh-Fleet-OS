// AtlasMesh Fleet OS - Policy Engine Service
//
// PUBLIC API: Core policy evaluation engine for autonomous fleet operations
// Provides policy-as-code capabilities using OPA/Rego for real-time decision making
// 
// INTEGRATION CONTRACTS:
// - gRPC: PolicyEngine service on port 50051 (proto/policy_engine.proto)
// - HTTP: REST API on port 8080 with OpenAPI spec (api/openapi.yaml)
// - Database: PostgreSQL with policies/audit tables (migrations/*.sql)
// - Message Bus: Kafka for policy evaluation events (topic: policy.evaluations)
// - Metrics: Prometheus metrics on /metrics endpoint
//
// SAFETY: All policy evaluations are logged for audit and safety compliance
// SECURITY: Policy modifications require admin role with audit trail
// CONCURRENCY: OPA evaluation engine is thread-safe for concurrent policy evaluation
// PERF: Policy cache with TTL to reduce database load on frequent evaluations
//
// POLICY EVALUATION LATENCY BUDGET: <10ms for real-time vehicle decisions
// AVAILABILITY SLA: 99.9% uptime for safety-critical policy evaluations
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

	"github.com/atlasmesh/fleet-os/services/policy-engine/internal/config"
	"github.com/atlasmesh/fleet-os/services/policy-engine/internal/handler"
	"github.com/atlasmesh/fleet-os/services/policy-engine/internal/repository"
	"github.com/atlasmesh/fleet-os/services/policy-engine/internal/service"
	pb "github.com/atlasmesh/fleet-os/services/policy-engine/proto"
)

// Build-time variables injected during compilation
// These provide version information for debugging and monitoring
var (
	version   = "dev"        // Git tag or version number
	commit    = "unknown"    // Git commit hash
	buildDate = "unknown"    // Build timestamp
)

// main is the entry point for the Policy Engine service
// It initializes all components, starts servers, and handles graceful shutdown
func main() {
	log.Printf("AtlasMesh Policy Engine starting - version: %s, commit: %s, built: %s", version, commit, buildDate)

	// Load configuration from environment variables and config files
	// This includes database URLs, server ports, and feature flags
	cfg, err := config.Load()
	if err != nil {
		log.Fatalf("Failed to load configuration: %v", err)
	}

	// Initialize data access layer repositories
	// Repository pattern provides abstraction over data storage (PostgreSQL)
	policyRepo, err := repository.NewPolicyRepository(cfg.Database.URL)
	if err != nil {
		log.Fatalf("Failed to initialize policy repository: %v", err)
	}
	defer policyRepo.Close() // Ensure database connections are properly closed

	// Audit repository for compliance and traceability logging
	auditRepo, err := repository.NewAuditRepository(cfg.Database.URL)
	if err != nil {
		log.Fatalf("Failed to initialize audit repository: %v", err)
	}
	defer auditRepo.Close() // Ensure audit database connections are properly closed

	// Initialize business logic service layer
	// Service layer encapsulates policy evaluation logic and orchestrates repository calls
	policyService := service.NewPolicyService(policyRepo, auditRepo, cfg)

	// Initialize gRPC server
	grpcServer := grpc.NewServer(
		grpc.UnaryInterceptor(service.UnaryLoggingInterceptor),
		grpc.StreamInterceptor(service.StreamLoggingInterceptor),
	)

	// Register services
	pb.RegisterPolicyEngineServer(grpcServer, handler.NewPolicyEngineHandler(policyService))
	
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
			"service":    "policy-engine",
			"version":    version,
			"commit":     commit,
			"build_date": buildDate,
			"timestamp":  time.Now().UTC().Format(time.RFC3339),
		})
	})

	// Readiness endpoint
	httpRouter.GET("/ready", func(c *gin.Context) {
		// Check database connectivity
		if err := policyRepo.HealthCheck(c.Request.Context()); err != nil {
			c.JSON(http.StatusServiceUnavailable, gin.H{
				"status": "not_ready",
				"error":  err.Error(),
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
