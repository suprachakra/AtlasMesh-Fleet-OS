package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"go.uber.org/zap"
	"atlasmesh/mining-adapters/wenco/internal/config"
	"atlasmesh/mining-adapters/wenco/internal/database"
	"atlasmesh/mining-adapters/wenco/internal/logger"
	"atlasmesh/mining-adapters/wenco/internal/metrics"
	"atlasmesh/mining-adapters/wenco/internal/server"
	"atlasmesh/mining-adapters/wenco/internal/wenco"
	"atlasmesh/mining-adapters/wenco/internal/production"
	"atlasmesh/mining-adapters/wenco/internal/equipment"
)

// WencoMiningAdapterService represents the main Wenco mining adapter service
type WencoMiningAdapterService struct {
	config        *config.Config
	logger        *zap.Logger
	db            *database.Database
	metrics       *metrics.Metrics
	wencoClient   *wenco.Client
	productionMgr *production.Manager
	equipmentMgr  *equipment.Manager
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

	logger.Info("Starting Wenco Mining Adapter Service",
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
	wencoClient := wenco.NewClient(cfg.Wenco, logger)
	productionMgr := production.NewManager(cfg.Production, db, logger)
	equipmentMgr := equipment.NewManager(cfg.Equipment, db, logger)

	// Initialize HTTP server
	httpServer := server.New(cfg.Server, logger, wencoClient, productionMgr, equipmentMgr)

	// Initialize service
	service := &WencoMiningAdapterService{
		config:        cfg,
		logger:        logger,
		db:            db,
		metrics:       metrics,
		wencoClient:   wencoClient,
		productionMgr: productionMgr,
		equipmentMgr:  equipmentMgr,
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

func (s *WencoMiningAdapterService) start() error {
	s.logger.Info("Starting Wenco Mining Adapter Service components")

	// Start managers
	if err := s.wencoClient.Start(s.ctx); err != nil {
		return err
	}

	if err := s.productionMgr.Start(s.ctx); err != nil {
		return err
	}

	if err := s.equipmentMgr.Start(s.ctx); err != nil {
		return err
	}

	// Start HTTP server
	if err := s.server.Start(s.ctx); err != nil {
		return err
	}

	s.logger.Info("Wenco Mining Adapter Service started successfully")
	return nil
}

func (s *WencoMiningAdapterService) waitForShutdown() {
	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	s.logger.Info("Shutting down Wenco Mining Adapter Service")

	// Cancel context
	s.cancel()

	// Stop components
	s.server.Stop()
	s.equipmentMgr.Stop()
	s.productionMgr.Stop()
	s.wencoClient.Stop()

	s.logger.Info("Wenco Mining Adapter Service stopped")
}
