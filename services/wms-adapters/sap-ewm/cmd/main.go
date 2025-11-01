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

	"atlasmesh/wms-adapters/sap-ewm/internal/config"
	"atlasmesh/wms-adapters/sap-ewm/internal/database"
	"atlasmesh/wms-adapters/sap-ewm/internal/logger"
	"atlasmesh/wms-adapters/sap-ewm/internal/metrics"
	"atlasmesh/wms-adapters/sap-ewm/internal/sap"
	"atlasmesh/wms-adapters/sap-ewm/internal/server"
	"atlasmesh/wms-adapters/sap-ewm/internal/task"
	"atlasmesh/wms-adapters/sap-ewm/internal/inventory"
	"atlasmesh/wms-adapters/sap-ewm/internal/resource"
)

// SAPEWMAdapter represents the main SAP EWM adapter service
type SAPEWMAdapter struct {
	config     *config.Config
	logger     *zap.Logger
	db         *database.Database
	metrics    *metrics.Metrics
	sapClient  *sap.Client
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

	logger.Info("Starting SAP EWM Adapter",
		zap.String("version", "1.0.0"),
		zap.String("sap_host", cfg.SAPEWM.Host))

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

	// Initialize SAP client
	sapClient, err := sap.NewClient(cfg.SAPEWM, logger)
	if err != nil {
		logger.Fatal("Failed to initialize SAP client", zap.Error(err))
	}

	// Initialize managers
	taskMgr := task.NewManager(cfg.Task, sapClient, db, logger)
	inventoryMgr := inventory.NewManager(cfg.Inventory, sapClient, db, logger)
	resourceMgr := resource.NewManager(cfg.Resource, sapClient, db, logger)

	// Initialize HTTP server
	httpServer := server.New(cfg.Server, logger, taskMgr, inventoryMgr, resourceMgr)

	// Initialize adapter
	adapter := &SAPEWMAdapter{
		config:       cfg,
		logger:       logger,
		db:           db,
		metrics:      metrics,
		sapClient:    sapClient,
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

func (a *SAPEWMAdapter) start() error {
	a.logger.Info("Starting SAP EWM Adapter components")

	// Start SAP client
	if err := a.sapClient.Start(a.ctx); err != nil {
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

	a.logger.Info("SAP EWM Adapter started successfully")
	return nil
}

func (a *SAPEWMAdapter) waitForShutdown() {
	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	a.logger.Info("Shutting down SAP EWM Adapter")

	// Cancel context
	a.cancel()

	// Stop components
	a.server.Stop()
	a.resourceMgr.Stop()
	a.inventoryMgr.Stop()
	a.taskMgr.Stop()
	a.sapClient.Stop()

	a.logger.Info("SAP EWM Adapter stopped")
}
