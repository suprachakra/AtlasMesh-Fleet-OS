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

	"atlasmesh/wms-adapters/manhattan-scale/internal/config"
	"atlasmesh/wms-adapters/manhattan-scale/internal/database"
	"atlasmesh/wms-adapters/manhattan-scale/internal/logger"
	"atlasmesh/wms-adapters/manhattan-scale/internal/metrics"
	"atlasmesh/wms-adapters/manhattan-scale/internal/manhattan"
	"atlasmesh/wms-adapters/manhattan-scale/internal/server"
	"atlasmesh/wms-adapters/manhattan-scale/internal/task"
	"atlasmesh/wms-adapters/manhattan-scale/internal/inventory"
	"atlasmesh/wms-adapters/manhattan-scale/internal/resource"
)

// ManhattanScaleAdapter represents the main Manhattan SCALE adapter service
type ManhattanScaleAdapter struct {
	config     *config.Config
	logger     *zap.Logger
	db         *database.Database
	metrics    *metrics.Metrics
	manhattanClient  *manhattan.Client
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

	logger.Info("Starting Manhattan SCALE Adapter",
		zap.String("version", "1.0.0"),
		zap.String("manhattan_host", cfg.Manhattan.Host))

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

	// Initialize Manhattan client
	manhattanClient, err := manhattan.NewClient(cfg.Manhattan, logger)
	if err != nil {
		logger.Fatal("Failed to initialize Manhattan client", zap.Error(err))
	}

	// Initialize managers
	taskMgr := task.NewManager(cfg.Task, manhattanClient, db, logger)
	inventoryMgr := inventory.NewManager(cfg.Inventory, manhattanClient, db, logger)
	resourceMgr := resource.NewManager(cfg.Resource, manhattanClient, db, logger)

	// Initialize HTTP server
	httpServer := server.New(cfg.Server, logger, taskMgr, inventoryMgr, resourceMgr)

	// Initialize adapter
	adapter := &ManhattanScaleAdapter{
		config:       cfg,
		logger:       logger,
		db:           db,
		metrics:      metrics,
		manhattanClient: manhattanClient,
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

func (a *ManhattanScaleAdapter) start() error {
	a.logger.Info("Starting Manhattan SCALE Adapter components")

	// Start Manhattan client
	if err := a.manhattanClient.Start(a.ctx); err != nil {
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

	a.logger.Info("Manhattan SCALE Adapter started successfully")
	return nil
}

func (a *ManhattanScaleAdapter) waitForShutdown() {
	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	a.logger.Info("Shutting down Manhattan SCALE Adapter")

	// Cancel context
	a.cancel()

	// Stop components
	a.server.Stop()
	a.resourceMgr.Stop()
	a.inventoryMgr.Stop()
	a.taskMgr.Stop()
	a.manhattanClient.Stop()

	a.logger.Info("Manhattan SCALE Adapter stopped")
}
