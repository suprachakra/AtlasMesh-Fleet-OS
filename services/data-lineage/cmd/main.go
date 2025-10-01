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

	"github.com/atlasmesh/fleet-os/services/data-lineage/internal/config"
	"github.com/atlasmesh/fleet-os/services/data-lineage/internal/lineage"
	"github.com/atlasmesh/fleet-os/services/data-lineage/internal/storage"
	"github.com/atlasmesh/fleet-os/services/data-lineage/internal/kafka"
	"github.com/atlasmesh/fleet-os/services/data-lineage/internal/metrics"
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

	logger.Info("Starting AtlasMesh Data Lineage Service",
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

	// Initialize storage
	neo4jClient, err := storage.NewNeo4jClient(cfg.Neo4j, logger)
	if err != nil {
		logger.Fatal("Failed to initialize Neo4j client", zap.Error(err))
	}
	defer neo4jClient.Close()

	// Initialize Kafka consumer
	kafkaConsumer, err := kafka.NewConsumer(cfg.Kafka, logger)
	if err != nil {
		logger.Fatal("Failed to initialize Kafka consumer", zap.Error(err))
	}
	defer kafkaConsumer.Close()

	// Initialize lineage service
	lineageService := lineage.NewService(lineage.ServiceConfig{
		Logger:    logger,
		Storage:   neo4jClient,
		Consumer:  kafkaConsumer,
		Metrics:   metricsCollector,
		Config:    cfg.Lineage,
	})

	// Initialize HTTP server
	router := setupRouter(lineageService, metricsCollector, logger)
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

	go lineageService.StartLineageProcessor(ctx)
	go lineageService.StartMetricsCollector(ctx)

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

func setupRouter(lineageService *lineage.Service, metricsCollector *metrics.Collector, logger *zap.Logger) *gin.Engine {
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
	router.GET("/ready", readinessHandler(lineageService))

	// Metrics endpoint
	router.GET("/metrics", gin.WrapH(promhttp.Handler()))

	// API routes
	v1 := router.Group("/api/v1")
	{
		// Lineage tracking endpoints
		v1.POST("/lineage/track", lineageService.TrackDataLineage)
		v1.POST("/lineage/batch", lineageService.TrackBatchLineage)

		// Lineage query endpoints
		v1.GET("/lineage/dataset/:datasetId", lineageService.GetDatasetLineage)
		v1.GET("/lineage/upstream/:datasetId", lineageService.GetUpstreamLineage)
		v1.GET("/lineage/downstream/:datasetId", lineageService.GetDownstreamLineage)
		v1.GET("/lineage/impact/:datasetId", lineageService.GetImpactAnalysis)

		// Data quality endpoints
		v1.GET("/quality/dataset/:datasetId", lineageService.GetDataQuality)
		v1.POST("/quality/report", lineageService.ReportDataQuality)

		// Governance endpoints
		v1.GET("/governance/datasets", lineageService.ListDatasets)
		v1.GET("/governance/pipelines", lineageService.ListPipelines)
		v1.GET("/governance/compliance/:datasetId", lineageService.GetComplianceStatus)

		// Search endpoints
		v1.GET("/search/datasets", lineageService.SearchDatasets)
		v1.GET("/search/columns", lineageService.SearchColumns)
	}

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

func readinessHandler(service *lineage.Service) gin.HandlerFunc {
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
