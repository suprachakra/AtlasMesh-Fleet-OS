// AtlasMesh Fleet OS - Authentication & Authorization Service
//
// PUBLIC API: Core authentication service for fleet operations
// Provides JWT-based authentication, RBAC/ABAC authorization, and session management
//
// INTEGRATION CONTRACTS:
// - HTTP: REST API on port 8080 with JWT token endpoints
// - Database: PostgreSQL for user/role/session storage with audit logging
// - Vault: HashiCorp Vault for secrets management and PKI operations
// - Metrics: Prometheus metrics on /metrics endpoint
// - Message Bus: Kafka for auth events (topic: auth.events)
//
// SECURITY: All passwords hashed with bcrypt, JWT tokens signed with RS256
// SECURITY: Rate limiting on login endpoints to prevent brute force attacks
// SECURITY: Audit logging for all authentication and authorization events
// CONCURRENCY: JWT validation is stateless and thread-safe
// PERF: Redis caching for frequently accessed user sessions and permissions
//
// AUTHENTICATION LATENCY BUDGET: <50ms for JWT validation
// AUTHORIZATION LATENCY BUDGET: <10ms for RBAC/ABAC policy evaluation
// AVAILABILITY SLA: 99.95% uptime for authentication services
package main

import (
	"context"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.uber.org/zap"

	"github.com/atlasmesh/fleet-os/services/auth-service/internal/config"
	"github.com/atlasmesh/fleet-os/services/auth-service/internal/auth"
	"github.com/atlasmesh/fleet-os/services/auth-service/internal/storage"
	"github.com/atlasmesh/fleet-os/services/auth-service/internal/vault"
	"github.com/atlasmesh/fleet-os/services/auth-service/internal/metrics"
	"github.com/atlasmesh/fleet-os/services/auth-service/internal/middleware"
)

// Build-time variables for version tracking and debugging
var (
	version   = "dev"        // Git tag or semantic version
	commit    = "unknown"    // Git commit hash for traceability
	buildDate = "unknown"    // Build timestamp for deployment tracking
)

func main() {
	// Initialize logger
	logger, err := zap.NewProduction()
	if err != nil {
		log.Fatalf("Failed to initialize logger: %v", err)
	}
	defer logger.Sync()

	logger.Info("Starting AtlasMesh Authentication Service",
		zap.String("version", version),
		zap.String("commit", commit),
		zap.String("buildDate", buildDate),
	)

	// Load configuration
	cfg, err := config.Load()
	if err != nil {
		logger.Fatal("Failed to load configuration", zap.Error(err))
	}

	// Initialize metrics
	metricsCollector := metrics.NewCollector()
	prometheus.MustRegister(metricsCollector)

	// Initialize Vault client
	vaultClient, err := vault.NewClient(cfg.Vault, logger)
	if err != nil {
		logger.Fatal("Failed to initialize Vault client", zap.Error(err))
	}

	// Initialize storage
	db, err := storage.NewPostgresClient(cfg.Database, logger)
	if err != nil {
		logger.Fatal("Failed to initialize database", zap.Error(err))
	}
	defer db.Close()

	// Run database migrations
	if err := storage.RunMigrations(db, logger); err != nil {
		logger.Fatal("Failed to run database migrations", zap.Error(err))
	}

	// Initialize auth service
	authService := auth.NewService(auth.ServiceConfig{
		Logger:      logger,
		Database:    db,
		VaultClient: vaultClient,
		Metrics:     metricsCollector,
		Config:      cfg.Auth,
	})

	// Initialize HTTP server
	router := setupRouter(authService, metricsCollector, logger, cfg)
	server := &http.Server{
		Addr:         fmt.Sprintf(":%d", cfg.Server.Port),
		Handler:      router,
		ReadTimeout:  cfg.Server.ReadTimeout,
		WriteTimeout: cfg.Server.WriteTimeout,
		IdleTimeout:  cfg.Server.IdleTimeout,
	}

	// Start server in a goroutine
	go func() {
		logger.Info("Starting HTTP server", zap.Int("port", cfg.Server.Port))
		if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			logger.Fatal("Failed to start HTTP server", zap.Error(err))
		}
	}()

	// Start background processors
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	go authService.StartTokenCleanup(ctx)
	go authService.StartSessionMonitoring(ctx)
	go authService.StartSecurityEventProcessor(ctx)

	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	logger.Info("Shutting down server...")

	// Graceful shutdown with timeout
	shutdownCtx, shutdownCancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer shutdownCancel()

	if err := server.Shutdown(shutdownCtx); err != nil {
		logger.Error("Server forced to shutdown", zap.Error(err))
	}

	// Cancel background processes
	cancel()

	logger.Info("Server exited")
}

func setupRouter(authService *auth.Service, metricsCollector *metrics.Collector, logger *zap.Logger, cfg *config.Config) *gin.Engine {
	// Set Gin mode based on environment
	if os.Getenv("GIN_MODE") == "" {
		gin.SetMode(gin.ReleaseMode)
	}

	router := gin.New()

	// Global middleware
	router.Use(gin.Logger())
	router.Use(gin.Recovery())
	router.Use(middleware.CORS())
	router.Use(middleware.SecurityHeaders())
	router.Use(middleware.RateLimiter(cfg.RateLimit))
	router.Use(middleware.Metrics(metricsCollector))
	router.Use(middleware.RequestID())
	router.Use(middleware.AuditLogging(logger))

	// Health endpoints (no auth required)
	router.GET("/health", healthHandler)
	router.GET("/ready", readinessHandler(authService))
	router.GET("/metrics", gin.WrapH(promhttp.Handler()))

	// Public endpoints (no auth required)
	public := router.Group("/api/v1/auth")
	{
		public.POST("/login", authService.Login)
		public.POST("/refresh", authService.RefreshToken)
		public.POST("/logout", authService.Logout)
		public.POST("/forgot-password", authService.ForgotPassword)
		public.POST("/reset-password", authService.ResetPassword)
		public.GET("/verify-email/:token", authService.VerifyEmail)
		
		// OAuth endpoints
		public.GET("/oauth/:provider", authService.OAuthLogin)
		public.GET("/oauth/:provider/callback", authService.OAuthCallback)
		
		// Public key endpoints for JWT verification
		public.GET("/jwks", authService.GetJWKS)
		public.GET("/public-key", authService.GetPublicKey)
	}

	// Protected endpoints (require authentication)
	protected := router.Group("/api/v1")
	protected.Use(middleware.JWTAuth(authService))
	{
		// User management
		users := protected.Group("/users")
		{
			users.GET("/me", authService.GetCurrentUser)
			users.PUT("/me", authService.UpdateCurrentUser)
			users.POST("/change-password", authService.ChangePassword)
			users.POST("/enable-mfa", authService.EnableMFA)
			users.POST("/disable-mfa", authService.DisableMFA)
			users.POST("/verify-mfa", authService.VerifyMFA)
			users.GET("/sessions", authService.GetUserSessions)
			users.DELETE("/sessions/:sessionId", authService.RevokeSession)
		}

		// Admin endpoints (require admin role)
		admin := protected.Group("/admin")
		admin.Use(middleware.RequireRole("admin"))
		{
			admin.GET("/users", authService.ListUsers)
			admin.POST("/users", authService.CreateUser)
			admin.GET("/users/:userId", authService.GetUser)
			admin.PUT("/users/:userId", authService.UpdateUser)
			admin.DELETE("/users/:userId", authService.DeleteUser)
			admin.POST("/users/:userId/reset-password", authService.AdminResetPassword)
			admin.POST("/users/:userId/disable", authService.DisableUser)
			admin.POST("/users/:userId/enable", authService.EnableUser)
			
			// Role management
			admin.GET("/roles", authService.ListRoles)
			admin.POST("/roles", authService.CreateRole)
			admin.GET("/roles/:roleId", authService.GetRole)
			admin.PUT("/roles/:roleId", authService.UpdateRole)
			admin.DELETE("/roles/:roleId", authService.DeleteRole)
			admin.POST("/users/:userId/roles/:roleId", authService.AssignRole)
			admin.DELETE("/users/:userId/roles/:roleId", authService.UnassignRole)
			
			// Permission management
			admin.GET("/permissions", authService.ListPermissions)
			admin.POST("/permissions", authService.CreatePermission)
			admin.GET("/permissions/:permissionId", authService.GetPermission)
			admin.PUT("/permissions/:permissionId", authService.UpdatePermission)
			admin.DELETE("/permissions/:permissionId", authService.DeletePermission)
			
			// Audit and security
			admin.GET("/audit-logs", authService.GetAuditLogs)
			admin.GET("/security-events", authService.GetSecurityEvents)
			admin.GET("/active-sessions", authService.GetActiveSessions)
			admin.POST("/revoke-all-sessions", authService.RevokeAllSessions)
		}

		// Service-to-service authentication
		service := protected.Group("/service")
		service.Use(middleware.RequireServiceAccount())
		{
			service.POST("/authenticate", authService.ServiceAuthenticate)
			service.POST("/authorize", authService.ServiceAuthorize)
			service.GET("/certificate", authService.GetServiceCertificate)
			service.POST("/certificate/renew", authService.RenewServiceCertificate)
		}

		// Policy endpoints (ABAC)
		policy := protected.Group("/policy")
		policy.Use(middleware.RequirePermission("policy:read"))
		{
			policy.GET("/evaluate", authService.EvaluatePolicy)
			policy.POST("/evaluate", authService.EvaluatePolicyBatch)
			policy.GET("/policies", middleware.RequirePermission("policy:list"), authService.ListPolicies)
			policy.POST("/policies", middleware.RequirePermission("policy:create"), authService.CreatePolicy)
			policy.GET("/policies/:policyId", authService.GetPolicy)
			policy.PUT("/policies/:policyId", middleware.RequirePermission("policy:update"), authService.UpdatePolicy)
			policy.DELETE("/policies/:policyId", middleware.RequirePermission("policy:delete"), authService.DeletePolicy)
		}
	}

	return router
}

func healthHandler(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{
		"status":    "healthy",
		"timestamp": time.Now().UTC(),
		"version":   version,
		"commit":    commit,
		"buildDate": buildDate,
	})
}

func readinessHandler(service *auth.Service) gin.HandlerFunc {
	return func(c *gin.Context) {
		if service.IsReady() {
			c.JSON(http.StatusOK, gin.H{
				"status": "ready",
				"checks": service.GetHealthChecks(),
			})
		} else {
			c.JSON(http.StatusServiceUnavailable, gin.H{
				"status": "not ready",
				"checks": service.GetHealthChecks(),
			})
		}
	}
}
