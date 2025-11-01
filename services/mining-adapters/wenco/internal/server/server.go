package server

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"go.uber.org/zap"
	"github.com/atlasmesh/mining-adapters/wenco/internal/wenco"
	"github.com/atlasmesh/mining-adapters/wenco/internal/production"
	"github.com/atlasmesh/mining-adapters/wenco/internal/equipment"
)

// Server represents the HTTP server
type Server struct {
	config        *ServerConfig
	logger        *zap.Logger
	wencoClient   *wenco.Client
	productionMgr *production.Manager
	equipmentMgr  *equipment.Manager
	httpServer    *http.Server
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
func New(cfg *ServerConfig, logger *zap.Logger, wencoClient *wenco.Client, productionMgr *production.Manager, equipmentMgr *equipment.Manager) *Server {
	return &Server{
		config:        cfg,
		logger:        logger,
		wencoClient:   wencoClient,
		productionMgr: productionMgr,
		equipmentMgr:  equipmentMgr,
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
	
	// Wenco endpoints
	mux.HandleFunc("/api/v1/wenco/equipment", s.equipmentHandler)
	mux.HandleFunc("/api/v1/wenco/production", s.productionHandler)
	mux.HandleFunc("/api/v1/wenco/locations", s.locationsHandler)
	mux.HandleFunc("/api/v1/wenco/optimization", s.optimizationHandler)
	mux.HandleFunc("/api/v1/wenco/health", s.healthStatusHandler)
}

// HTTP handlers

func (s *Server) healthHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"status":"healthy"}`))
}

func (s *Server) equipmentHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Equipment endpoint"}`))
}

func (s *Server) productionHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Production endpoint"}`))
}

func (s *Server) locationsHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Locations endpoint"}`))
}

func (s *Server) optimizationHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Optimization endpoint"}`))
}

func (s *Server) healthStatusHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Health status endpoint"}`))
}
