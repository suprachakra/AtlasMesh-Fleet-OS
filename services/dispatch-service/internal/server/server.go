package server

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"go.uber.org/zap"
	"github.com/atlasmesh/dispatch-service/internal/dispatch"
	"github.com/atlasmesh/dispatch-service/internal/routing"
	"github.com/atlasmesh/dispatch-service/internal/optimization"
)

// Server represents the HTTP server
type Server struct {
	config         *ServerConfig
	logger         *zap.Logger
	dispatchMgr    *dispatch.Manager
	routingMgr     *routing.Manager
	optimizationMgr *optimization.Manager
	httpServer     *http.Server
}

// ServerConfig represents server configuration
type ServerConfig struct {
	Host         string        `yaml:"host"`
	Port         int           `yaml:"port"`
	ReadTimeout  time.Duration `yaml:"read_timeout"`
	WriteTimeout time.Duration `yaml:"write_timeout"`
	IdleTimeout  time.Duration `yaml:"idle_timeout"`
}

// New creates a new HTTP server
func New(cfg *ServerConfig, logger *zap.Logger, dispatchMgr *dispatch.Manager, routingMgr *routing.Manager, optimizationMgr *optimization.Manager) *Server {
	return &Server{
		config:         cfg,
		logger:         logger,
		dispatchMgr:    dispatchMgr,
		routingMgr:     routingMgr,
		optimizationMgr: optimizationMgr,
	}
}

// Start starts the HTTP server
func (s *Server) Start(ctx context.Context) error {
	// Setup routes
	mux := http.NewServeMux()
	s.setupRoutes(mux)

	// Create HTTP server
	s.httpServer = &http.Server{
		Addr:         fmt.Sprintf("%s:%d", s.config.Host, s.config.Port),
		Handler:      mux,
		ReadTimeout:  s.config.ReadTimeout,
		WriteTimeout: s.config.WriteTimeout,
		IdleTimeout:  s.config.IdleTimeout,
	}

	// Start server in goroutine
	go func() {
		s.logger.Info("Starting HTTP server", zap.String("addr", s.httpServer.Addr))
		if err := s.httpServer.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			s.logger.Error("Failed to start HTTP server", zap.Error(err))
		}
	}()

	return nil
}

// Stop stops the HTTP server
func (s *Server) Stop() error {
	if s.httpServer != nil {
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		return s.httpServer.Shutdown(ctx)
	}
	return nil
}

// setupRoutes sets up HTTP routes
func (s *Server) setupRoutes(mux *http.ServeMux) {
	// Health check endpoint
	mux.HandleFunc("/health", s.healthHandler)
	
	// Dispatch endpoints
	mux.HandleFunc("/api/v1/dispatch/tasks", s.dispatchTasksHandler)
	mux.HandleFunc("/api/v1/dispatch/queue", s.queueHandler)
	
	// Routing endpoints
	mux.HandleFunc("/api/v1/routing/route", s.routeHandler)
	mux.HandleFunc("/api/v1/routing/optimize", s.optimizeRouteHandler)
	
	// Optimization endpoints
	mux.HandleFunc("/api/v1/optimization/run", s.runOptimizationHandler)
	mux.HandleFunc("/api/v1/optimization/status", s.optimizationStatusHandler)
}

// HTTP handlers

func (s *Server) healthHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"status":"healthy"}`))
}

func (s *Server) dispatchTasksHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Dispatch tasks endpoint"}`))
}

func (s *Server) queueHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Queue status endpoint"}`))
}

func (s *Server) routeHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Route calculation endpoint"}`))
}

func (s *Server) optimizeRouteHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Route optimization endpoint"}`))
}

func (s *Server) runOptimizationHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Run optimization endpoint"}`))
}

func (s *Server) optimizationStatusHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Optimization status endpoint"}`))
}
