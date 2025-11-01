package server

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"
	"go.uber.org/zap"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/config"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/task"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/inventory"
	"github.com/atlasmesh/wms-adapters/sap-ewm/internal/resource"
)

// Server represents the HTTP server
type Server struct {
	config       *config.ServerConfig
	logger       *zap.Logger
	taskMgr      *task.Manager
	inventoryMgr *inventory.Manager
	resourceMgr  *resource.Manager
	httpServer   *http.Server
	router       *gin.Engine
}

// New creates a new HTTP server
func New(cfg *config.ServerConfig, logger *zap.Logger, taskMgr *task.Manager, inventoryMgr *inventory.Manager, resourceMgr *resource.Manager) *Server {
	// Set Gin mode
	gin.SetMode(gin.ReleaseMode)
	
	router := gin.New()
	router.Use(gin.Logger())
	router.Use(gin.Recovery())

	// Setup routes
	setupRoutes(router, taskMgr, inventoryMgr, resourceMgr, logger)

	httpServer := &http.Server{
		Addr:         fmt.Sprintf("%s:%d", cfg.Host, cfg.Port),
		Handler:      router,
		ReadTimeout:  cfg.ReadTimeout,
		WriteTimeout: cfg.WriteTimeout,
		IdleTimeout:  cfg.IdleTimeout,
	}

	return &Server{
		config:       cfg,
		logger:       logger,
		taskMgr:      taskMgr,
		inventoryMgr: inventoryMgr,
		resourceMgr:  resourceMgr,
		httpServer:   httpServer,
		router:       router,
	}
}

// Start starts the HTTP server
func (s *Server) Start(ctx context.Context) error {
	go func() {
		if err := s.httpServer.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			s.logger.Error("Failed to start HTTP server", zap.Error(err))
		}
	}()

	s.logger.Info("HTTP server started", 
		zap.String("host", s.config.Host),
		zap.Int("port", s.config.Port))

	return nil
}

// Stop stops the HTTP server
func (s *Server) Stop() error {
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	if err := s.httpServer.Shutdown(ctx); err != nil {
		return err
	}

	s.logger.Info("HTTP server stopped")
	return nil
}

// setupRoutes sets up HTTP routes
func setupRoutes(router *gin.Engine, taskMgr *task.Manager, inventoryMgr *inventory.Manager, resourceMgr *resource.Manager, logger *zap.Logger) {
	// Health check
	router.GET("/health", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"status":    "healthy",
			"timestamp": time.Now().UTC(),
			"service":   "sap-ewm-adapter",
		})
	})

	// API routes
	api := router.Group("/api/v1")
	{
		// Task routes
		tasks := api.Group("/tasks")
		{
			tasks.GET("", func(c *gin.Context) {
				c.JSON(http.StatusOK, gin.H{
					"message": "Tasks endpoint - integration pending",
					"tasks":   []interface{}{},
				})
			})
		}

		// Inventory routes
		inventory := api.Group("/inventory")
		{
			inventory.GET("", func(c *gin.Context) {
				c.JSON(http.StatusOK, gin.H{
					"message": "Inventory endpoint - integration pending",
					"items":   []interface{}{},
				})
			})
		}

		// Resource routes
		resources := api.Group("/resources")
		{
			resources.GET("", func(c *gin.Context) {
				c.JSON(http.StatusOK, gin.H{
					"message":   "Resources endpoint - integration pending",
					"resources": []interface{}{},
				})
			})
		}
	}
}
