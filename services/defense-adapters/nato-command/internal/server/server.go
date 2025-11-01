package server

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"go.uber.org/zap"
	"github.com/atlasmesh/defense-adapters/nato-command/internal/nato"
	"github.com/atlasmesh/defense-adapters/nato-command/internal/security"
	"github.com/atlasmesh/defense-adapters/nato-command/internal/audit"
)

// Server represents the HTTP server
type Server struct {
	config      *ServerConfig
	logger      *zap.Logger
	natoClient  *nato.Client
	securityMgr *security.Manager
	auditMgr    *audit.Manager
	httpServer  *http.Server
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
func New(cfg *ServerConfig, logger *zap.Logger, natoClient *nato.Client, securityMgr *security.Manager, auditMgr *audit.Manager) *Server {
	return &Server{
		config:      cfg,
		logger:      logger,
		natoClient:  natoClient,
		securityMgr: securityMgr,
		auditMgr:    auditMgr,
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
	
	// NATO Command endpoints
	mux.HandleFunc("/api/v1/nato/command", s.natoCommandHandler)
	mux.HandleFunc("/api/v1/nato/missions", s.natoMissionsHandler)
	mux.HandleFunc("/api/v1/nato/threats", s.natoThreatsHandler)
	mux.HandleFunc("/api/v1/nato/reports", s.natoReportsHandler)
	mux.HandleFunc("/api/v1/nato/status", s.natoStatusHandler)
}

// HTTP handlers

func (s *Server) healthHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"status":"healthy"}`))
}

func (s *Server) natoCommandHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"NATO command endpoint"}`))
}

func (s *Server) natoMissionsHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"NATO missions endpoint"}`))
}

func (s *Server) natoThreatsHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"NATO threats endpoint"}`))
}

func (s *Server) natoReportsHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"NATO reports endpoint"}`))
}

func (s *Server) natoStatusHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"NATO status endpoint"}`))
}
