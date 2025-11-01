package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"go.uber.org/zap"
	"atlasmesh/defense-adapters/nato-command/internal/config"
	"atlasmesh/defense-adapters/nato-command/internal/database"
	"atlasmesh/defense-adapters/nato-command/internal/logger"
	"atlasmesh/defense-adapters/nato-command/internal/metrics"
	"atlasmesh/defense-adapters/nato-command/internal/server"
	"atlasmesh/defense-adapters/nato-command/internal/nato"
	"atlasmesh/defense-adapters/nato-command/internal/security"
	"atlasmesh/defense-adapters/nato-command/internal/audit"
)

// NATOCommandAdapterService represents the main NATO Command adapter service
type NATOCommandAdapterService struct {
	config        *config.Config
	logger        *zap.Logger
	db            *database.Database
	metrics       *metrics.Metrics
	natoClient    *nato.Client
	securityMgr   *security.Manager
	auditMgr      *audit.Manager
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

	logger.Info("Starting NATO Command Adapter Service",
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
	natoClient := nato.NewClient(cfg.NATO, logger)
	securityMgr := security.NewManager(cfg.Security, logger)
	auditMgr := audit.NewManager(cfg.Audit, db, logger)

	// Initialize HTTP server
	httpServer := server.New(cfg.Server, logger, natoClient, securityMgr, auditMgr)

	// Initialize service
	service := &NATOCommandAdapterService{
		config:      cfg,
		logger:      logger,
		db:          db,
		metrics:     metrics,
		natoClient:  natoClient,
		securityMgr: securityMgr,
		auditMgr:    auditMgr,
		server:      httpServer,
		ctx:         ctx,
		cancel:      cancel,
	}

	// Start service
	if err := service.start(); err != nil {
		logger.Fatal("Failed to start service", zap.Error(err))
	}

	// Wait for shutdown signal
	service.waitForShutdown()
}

func (s *NATOCommandAdapterService) start() error {
	s.logger.Info("Starting NATO Command Adapter Service components")

	// Start managers
	if err := s.natoClient.Start(s.ctx); err != nil {
		return err
	}

	if err := s.securityMgr.Start(s.ctx); err != nil {
		return err
	}

	if err := s.auditMgr.Start(s.ctx); err != nil {
		return err
	}

	// Start HTTP server
	if err := s.server.Start(s.ctx); err != nil {
		return err
	}

	s.logger.Info("NATO Command Adapter Service started successfully")
	return nil
}

func (s *NATOCommandAdapterService) waitForShutdown() {
	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	s.logger.Info("Shutting down NATO Command Adapter Service")

	// Cancel context
	s.cancel()

	// Stop components
	s.server.Stop()
	s.auditMgr.Stop()
	s.securityMgr.Stop()
	s.natoClient.Stop()

	s.logger.Info("NATO Command Adapter Service stopped")
}
