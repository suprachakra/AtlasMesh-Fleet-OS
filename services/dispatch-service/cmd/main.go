package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"go.uber.org/zap"
	"go.uber.org/zap/zapcore"
	"atlasmesh/dispatch-service/internal/config"
	"atlasmesh/dispatch-service/internal/database"
	"atlasmesh/dispatch-service/internal/logger"
	"atlasmesh/dispatch-service/internal/metrics"
	"atlasmesh/dispatch-service/internal/server"
	"atlasmesh/dispatch-service/internal/dispatch"
	"atlasmesh/dispatch-service/internal/routing"
	"atlasmesh/dispatch-service/internal/optimization"
)

// DispatchService represents the main dispatch service
type DispatchService struct {
	config        *config.Config
	sectorConfig  *config.SectorConfig
	logger        *zap.Logger
	db            *database.Database
	metrics       *metrics.Metrics
	dispatchMgr   *dispatch.Manager
	routingMgr    *routing.Manager
	optimizationMgr *optimization.Manager
	server        *server.Server
	ctx           context.Context
	cancel        context.CancelFunc
}

func main() {
	// Load configuration
	cfg, err := config.Load()
	if err != nil {
		log.Fatalf("Failed to load configuration: %v", err)
	}

	// Initialize logger
	logger, err := logger.New(cfg.LogLevel)
	if err != nil {
		log.Fatalf("Failed to initialize logger: %v", err)
	}
	defer logger.Sync()

	// Load sector-specific configuration
	sector := os.Getenv("ATLASMESH_SECTOR")
	if sector == "" {
		sector = "logistics" // Default to logistics
	}
	
	sectorConfig, err := config.LoadSectorConfig(config.SectorType(sector))
	if err != nil {
		logger.Fatal("Failed to load sector configuration", zap.Error(err))
	}

	// Validate sector configuration
	if err := sectorConfig.ValidateSectorConfig(); err != nil {
		logger.Fatal("Invalid sector configuration", zap.Error(err))
	}

	logger.Info("Starting Dispatch Service",
		zap.String("sector", sector),
		zap.String("version", "1.0.0"))

	// Initialize database
	db, err := database.New(cfg.Database)
	if err != nil {
		logger.Fatal("Failed to initialize database", zap.Error(err))
	}
	defer db.Close()

	// Initialize metrics
	metrics := metrics.New(cfg.Metrics)

	// Create context
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	// Initialize managers
	dispatchMgr := dispatch.NewManager(cfg.Dispatch, sectorConfig, db, logger)
	routingMgr := routing.NewManager(cfg.Routing, sectorConfig, db, logger)
	optimizationMgr := optimization.NewManager(cfg.Optimization, sectorConfig, db, logger)

	// Initialize HTTP server
	httpServer := server.New(cfg.Server, logger, dispatchMgr, routingMgr, optimizationMgr)

	// Initialize service
	service := &DispatchService{
		config:         cfg,
		sectorConfig:   sectorConfig,
		logger:         logger,
		db:             db,
		metrics:        metrics,
		dispatchMgr:    dispatchMgr,
		routingMgr:     routingMgr,
		optimizationMgr: optimizationMgr,
		server:         httpServer,
		ctx:            ctx,
		cancel:         cancel,
	}

	// Start service
	if err := service.start(); err != nil {
		logger.Fatal("Failed to start service", zap.Error(err))
	}

	// Wait for shutdown signal
	service.waitForShutdown()
}

func (s *DispatchService) start() error {
	s.logger.Info("Starting Dispatch Service components")

	// Start managers
	if err := s.dispatchMgr.Start(s.ctx); err != nil {
		return err
	}

	if err := s.routingMgr.Start(s.ctx); err != nil {
		return err
	}

	if err := s.optimizationMgr.Start(s.ctx); err != nil {
		return err
	}

	// Start HTTP server
	if err := s.server.Start(s.ctx); err != nil {
		return err
	}

	s.logger.Info("Dispatch Service started successfully")
	return nil
}

func (s *DispatchService) waitForShutdown() {
	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	s.logger.Info("Shutting down Dispatch Service")

	// Cancel context
	s.cancel()

	// Stop components
	s.server.Stop()
	s.optimizationMgr.Stop()
	s.routingMgr.Stop()
	s.dispatchMgr.Stop()

	s.logger.Info("Dispatch Service stopped")
}
