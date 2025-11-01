package server

import (
	"context"
	"fmt"
	"net/http"
	"time"

	"go.uber.org/zap"
	"github.com/atlasmesh/control-center-ui/internal/dashboard"
	"github.com/atlasmesh/control-center-ui/internal/websocket"
)

// Server represents the HTTP server
type Server struct {
	config        *ServerConfig
	logger        *zap.Logger
	dashboardMgr  *dashboard.Manager
	websocketMgr  *websocket.Manager
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
func New(cfg *ServerConfig, logger *zap.Logger, dashboardMgr *dashboard.Manager, websocketMgr *websocket.Manager) *Server {
	return &Server{
		config:       cfg,
		logger:       logger,
		dashboardMgr: dashboardMgr,
		websocketMgr: websocketMgr,
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
	
	// Dashboard endpoints
	mux.HandleFunc("/api/v1/dashboard", s.dashboardHandler)
	mux.HandleFunc("/api/v1/dashboard/layout", s.dashboardLayoutHandler)
	mux.HandleFunc("/api/v1/dashboard/widgets", s.dashboardWidgetsHandler)
	
	// WebSocket endpoint
	mux.HandleFunc("/ws", s.websocketHandler)
	
	// Static files (for UI)
	mux.Handle("/", http.FileServer(http.Dir("./web/static/")))
}

// HTTP handlers

func (s *Server) healthHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"status":"healthy"}`))
}

func (s *Server) dashboardHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Dashboard endpoint"}`))
}

func (s *Server) dashboardLayoutHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Dashboard layout endpoint"}`))
}

func (s *Server) dashboardWidgetsHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"Dashboard widgets endpoint"}`))
}

func (s *Server) websocketHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)
	w.Write([]byte(`{"message":"WebSocket endpoint"}`))
}
