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
	"go.uber.org/zap"

	"github.com/atlasmesh/fleet-os/services/telemetry-ingestion/internal/config"
	"github.com/atlasmesh/fleet-os/services/telemetry-ingestion/internal/ingestion"
	"github.com/atlasmesh/fleet-os/services/telemetry-ingestion/internal/kafka"
	"github.com/atlasmesh/fleet-os/services/telemetry-ingestion/internal/metrics"
	"github.com/atlasmesh/fleet-os/services/telemetry-ingestion/internal/schema"
	"github.com/atlasmesh/fleet-os/services/telemetry-ingestion/internal/storage"
)

var (
	version   = "dev"
	commit    = "unknown"
	buildDate = "unknown"
)

func main() {
	// Initialize logger
	logger, err := zap.NewProduction()
	if err != nil {
		log.Fatalf("Failed to initialize logger: %v", err)
	}
	defer logger.Sync()

	logger.Info("Starting AtlasMesh Telemetry Ingestion Service",
		zap.String("version", version),
		zap.String("commit", commit),
		zap.String("buildDate", buildDate),
	)

	// Load configuration
	cfg, err := config.Load()
	if err != nil {
		logger.Fatal("Failed to load configuration", zap.Error(err))
	}

	// Initialize metrics
	metricsCollector := metrics.NewCollector()
	prometheus.MustRegister(metricsCollector)

	// Initialize schema registry client
	schemaRegistry, err := schema.NewRegistryClient(cfg.SchemaRegistry)
	if err != nil {
		logger.Fatal("Failed to initialize schema registry", zap.Error(err))
	}

	// Initialize Kafka producer
	kafkaProducer, err := kafka.NewProducer(cfg.Kafka, logger)
	if err != nil {
		logger.Fatal("Failed to initialize Kafka producer", zap.Error(err))
	}
	defer kafkaProducer.Close()

	// Initialize storage clients
	hotStorage, err := storage.NewClickHouseClient(cfg.ClickHouse, logger)
	if err != nil {
		logger.Fatal("Failed to initialize ClickHouse client", zap.Error(err))
	}
	defer hotStorage.Close()

	coldStorage, err := storage.NewMinIOClient(cfg.MinIO, logger)
	if err != nil {
		logger.Fatal("Failed to initialize MinIO client", zap.Error(err))
	}

	// Initialize ingestion service
	ingestionService := ingestion.NewService(ingestion.ServiceConfig{
		Logger:         logger,
		SchemaRegistry: schemaRegistry,
		KafkaProducer:  kafkaProducer,
		HotStorage:     hotStorage,
		ColdStorage:    coldStorage,
		Metrics:        metricsCollector,
		Config:         cfg.Ingestion,
	})

	// Initialize HTTP server
	router := setupRouter(ingestionService, metricsCollector, logger)
	server := &http.Server{
		Addr:         fmt.Sprintf(":%d", cfg.Server.Port),
		Handler:      router,
		ReadTimeout:  cfg.Server.ReadTimeout,
		WriteTimeout: cfg.Server.WriteTimeout,
		IdleTimeout:  cfg.Server.IdleTimeout,
	}

	// Start server in a goroutine
	go func() {
		logger.Info("Starting HTTP server", zap.Int("port", cfg.Server.Port))
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			logger.Fatal("Failed to start HTTP server", zap.Error(err))
		}
	}()

	// Start background processors
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	go ingestionService.StartBatchProcessor(ctx)
	go ingestionService.StartMetricsCollector(ctx)

	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	logger.Info("Shutting down server...")

	// Graceful shutdown with timeout
	shutdownCtx, shutdownCancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer shutdownCancel()

	if err := server.Shutdown(shutdownCtx); err != nil {
		logger.Error("Server forced to shutdown", zap.Error(err))
	}

	// Cancel background processes
	cancel()

	logger.Info("Server exited")
}

func setupRouter(ingestionService *ingestion.Service, metricsCollector *metrics.Collector, logger *zap.Logger) *gin.Engine {
	// Set Gin mode based on environment
	if os.Getenv("GIN_MODE") == "" {
		gin.SetMode(gin.ReleaseMode)
	}

	router := gin.New()

	// Middleware
	router.Use(gin.Logger())
	router.Use(gin.Recovery())
	router.Use(corsMiddleware())
	router.Use(metricsMiddleware(metricsCollector))

	// Health endpoints
	router.GET("/health", healthHandler)
	router.GET("/ready", readinessHandler(ingestionService))

	// Metrics endpoint
	router.GET("/metrics", gin.WrapH(promhttp.Handler()))

	// API routes
	v1 := router.Group("/api/v1")
	{
		// Telemetry ingestion endpoints
		v1.POST("/telemetry/vehicle/:vehicleId", ingestionService.IngestVehicleTelemetry)
		v1.POST("/telemetry/batch", ingestionService.IngestBatch)

		// Command endpoints
		v1.POST("/commands/vehicle/:vehicleId", ingestionService.SendVehicleCommand)

		// Schema endpoints
		v1.GET("/schemas", ingestionService.ListSchemas)
		v1.GET("/schemas/:subject/versions/:version", ingestionService.GetSchema)

		// Query endpoints (for debugging and monitoring)
		v1.GET("/telemetry/vehicle/:vehicleId/latest", ingestionService.GetLatestTelemetry)
		v1.GET("/telemetry/vehicle/:vehicleId/history", ingestionService.GetTelemetryHistory)

		// Analytics endpoints
		v1.GET("/analytics/fleet/summary", ingestionService.GetFleetSummary)
		v1.GET("/analytics/vehicle/:vehicleId/metrics", ingestionService.GetVehicleMetrics)
	}

	// WebSocket endpoints for real-time streaming
	router.GET("/ws/telemetry", ingestionService.HandleWebSocketTelemetry)
	router.GET("/ws/commands", ingestionService.HandleWebSocketCommands)

	return router
}

func corsMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Header("Access-Control-Allow-Origin", "*")
		c.Header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
		c.Header("Access-Control-Allow-Headers", "Origin, Content-Type, Accept, Authorization, X-Requested-With")
		c.Header("Access-Control-Allow-Credentials", "true")

		if c.Request.Method == "OPTIONS" {
			c.AbortWithStatus(http.StatusNoContent)
			return
		}

		c.Next()
	}
}

func metricsMiddleware(collector *metrics.Collector) gin.HandlerFunc {
	return func(c *gin.Context) {
		start := time.Now()

		c.Next()

		duration := time.Since(start)
		collector.RecordHTTPRequest(c.Request.Method, c.FullPath(), c.Writer.Status(), duration)
	}
}

func healthHandler(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{
		"status":    "healthy",
		"timestamp": time.Now().UTC(),
		"version":   version,
		"commit":    commit,
		"buildDate": buildDate,
	})
}

func readinessHandler(service *ingestion.Service) gin.HandlerFunc {
	return func(c *gin.Context) {
		if service.IsReady() {
			c.JSON(http.StatusOK, gin.H{
				"status": "ready",
				"checks": service.GetHealthChecks(),
			})
		} else {
			c.JSON(http.StatusServiceUnavailable, gin.H{
				"status": "not ready",
				"checks": service.GetHealthChecks(),
			})
		}
	}
}
