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

	"atlasmesh/wms-adapters/oracle-wms/internal/config"
	"atlasmesh/wms-adapters/oracle-wms/internal/database"
	"atlasmesh/wms-adapters/oracle-wms/internal/logger"
	"atlasmesh/wms-adapters/oracle-wms/internal/metrics"
	"atlasmesh/wms-adapters/oracle-wms/internal/oracle"
	"atlasmesh/wms-adapters/oracle-wms/internal/server"
	"atlasmesh/wms-adapters/oracle-wms/internal/task"
	"atlasmesh/wms-adapters/oracle-wms/internal/inventory"
	"atlasmesh/wms-adapters/oracle-wms/internal/resource"
)

// OracleWMSAdapter represents the main Oracle WMS adapter service
type OracleWMSAdapter struct {
	config     *config.Config
	logger     *zap.Logger
	db         *database.Database
	metrics    *metrics.Metrics
	oracleClient  *oracle.Client
	taskMgr    *task.Manager
	inventoryMgr *inventory.Manager
	resourceMgr  *resource.Manager
	server     *server.Server
	ctx        context.Context
	cancel     context.CancelFunc
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

	logger.Info("Starting Oracle WMS Adapter",
		zap.String("version", "1.0.0"),
		zap.String("oracle_host", cfg.OracleWMS.Host))

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

	// Initialize Oracle client
	oracleClient, err := oracle.NewClient(cfg.OracleWMS, logger)
	if err != nil {
		logger.Fatal("Failed to initialize Oracle client", zap.Error(err))
	}

	// Initialize managers
	taskMgr := task.NewManager(cfg.Task, oracleClient, db, logger)
	inventoryMgr := inventory.NewManager(cfg.Inventory, oracleClient, db, logger)
	resourceMgr := resource.NewManager(cfg.Resource, oracleClient, db, logger)

	// Initialize HTTP server
	httpServer := server.New(cfg.Server, logger, taskMgr, inventoryMgr, resourceMgr)

	// Initialize adapter
	adapter := &OracleWMSAdapter{
		config:       cfg,
		logger:       logger,
		db:           db,
		metrics:      metrics,
		oracleClient: oracleClient,
		taskMgr:      taskMgr,
		inventoryMgr: inventoryMgr,
		resourceMgr:  resourceMgr,
		server:       httpServer,
		ctx:          ctx,
		cancel:       cancel,
	}

	// Start adapter
	if err := adapter.start(); err != nil {
		logger.Fatal("Failed to start adapter", zap.Error(err))
	}

	// Wait for shutdown signal
	adapter.waitForShutdown()
}

func (a *OracleWMSAdapter) start() error {
	a.logger.Info("Starting Oracle WMS Adapter components")

	// Start Oracle client
	if err := a.oracleClient.Start(a.ctx); err != nil {
		return err
	}

	// Start managers
	if err := a.taskMgr.Start(a.ctx); err != nil {
		return err
	}

	if err := a.inventoryMgr.Start(a.ctx); err != nil {
		return err
	}

	if err := a.resourceMgr.Start(a.ctx); err != nil {
		return err
	}

	// Start HTTP server
	if err := a.server.Start(a.ctx); err != nil {
		return err
	}

	a.logger.Info("Oracle WMS Adapter started successfully")
	return nil
}

func (a *OracleWMSAdapter) waitForShutdown() {
	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	a.logger.Info("Shutting down Oracle WMS Adapter")

	// Cancel context
	a.cancel()

	// Stop components
	a.server.Stop()
	a.resourceMgr.Stop()
	a.inventoryMgr.Stop()
	a.taskMgr.Stop()
	a.oracleClient.Stop()

	a.logger.Info("Oracle WMS Adapter stopped")
}
