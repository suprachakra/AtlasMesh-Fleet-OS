package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"go.uber.org/zap"
	"atlasmesh/control-center-ui/internal/config"
	"atlasmesh/control-center-ui/internal/database"
	"atlasmesh/control-center-ui/internal/logger"
	"atlasmesh/control-center-ui/internal/metrics"
	"atlasmesh/control-center-ui/internal/server"
	"atlasmesh/control-center-ui/internal/dashboard"
	"atlasmesh/control-center-ui/internal/websocket"
)

// ControlCenterUIService represents the main control center UI service
type ControlCenterUIService struct {
	config        *config.Config
	sectorConfig  *config.SectorConfig
	logger        *zap.Logger
	db            *database.Database
	metrics       *metrics.Metrics
	dashboardMgr  *dashboard.Manager
	websocketMgr  *websocket.Manager
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

	logger.Info("Starting Control Center UI Service",
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
	dashboardMgr := dashboard.NewManager(cfg.Dashboard, sectorConfig, db, logger)
	websocketMgr := websocket.NewManager(cfg.WebSocket, sectorConfig, logger)

	// Initialize HTTP server
	httpServer := server.New(cfg.Server, logger, dashboardMgr, websocketMgr)

	// Initialize service
	service := &ControlCenterUIService{
		config:        cfg,
		sectorConfig:  sectorConfig,
		logger:        logger,
		db:            db,
		metrics:       metrics,
		dashboardMgr:  dashboardMgr,
		websocketMgr:  websocketMgr,
		server:        httpServer,
		ctx:           ctx,
		cancel:        cancel,
	}

	// Start service
	if err := service.start(); err != nil {
		logger.Fatal("Failed to start service", zap.Error(err))
	}

	// Wait for shutdown signal
	service.waitForShutdown()
}

func (s *ControlCenterUIService) start() error {
	s.logger.Info("Starting Control Center UI Service components")

	// Start managers
	if err := s.dashboardMgr.Start(s.ctx); err != nil {
		return err
	}

	if err := s.websocketMgr.Start(s.ctx); err != nil {
		return err
	}

	// Start HTTP server
	if err := s.server.Start(s.ctx); err != nil {
		return err
	}

	s.logger.Info("Control Center UI Service started successfully")
	return nil
}

func (s *ControlCenterUIService) waitForShutdown() {
	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	s.logger.Info("Shutting down Control Center UI Service")

	// Cancel context
	s.cancel()

	// Stop components
	s.server.Stop()
	s.websocketMgr.Stop()
	s.dashboardMgr.Stop()

	s.logger.Info("Control Center UI Service stopped")
}
