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
	"github.com/atlasmesh/fleet-os/services/variant-budget/internal/analyzer"
	"github.com/atlasmesh/fleet-os/services/variant-budget/internal/budget"
	"github.com/atlasmesh/fleet-os/services/variant-budget/internal/config"
	"github.com/atlasmesh/fleet-os/services/variant-budget/internal/enforcer"
	"github.com/atlasmesh/fleet-os/services/variant-budget/internal/server"
)

// Variant Budget Enforcement Service
// Automated tracking and enforcement of code delta limits
// Ensures qualified agnosticism stays within defined bounds
func main() {
	// Load configuration
	cfg, err := config.Load()
	if err != nil {
		log.Fatalf("Failed to load configuration: %v", err)
	}

	// Initialize delta analyzer
	deltaAnalyzer, err := analyzer.New(cfg.AnalyzerConfig)
	if err != nil {
		log.Fatalf("Failed to initialize delta analyzer: %v", err)
	}

	// Initialize budget tracker
	budgetTracker := budget.NewTracker(cfg.BudgetConfig)

	// Initialize policy enforcer
	policyEnforcer := enforcer.New(cfg.EnforcementConfig, budgetTracker)

	// Setup HTTP server
	router := gin.New()
	router.Use(gin.Logger())
	router.Use(gin.Recovery())

	// Initialize server with dependencies
	srv := server.New(deltaAnalyzer, budgetTracker, policyEnforcer)
	srv.SetupRoutes(router)

	// Health check endpoint
	router.GET("/health", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"status":    "healthy",
			"service":   "variant-budget",
			"version":   cfg.Version,
			"timestamp": time.Now().UTC(),
			"budget_status": map[string]interface{}{
				"vehicle_budget":  budgetTracker.GetVehicleBudgetStatus(),
				"sector_budget":   budgetTracker.GetSectorBudgetStatus(),
				"platform_budget": budgetTracker.GetPlatformBudgetStatus(),
			},
		})
	})

	// Metrics endpoint for Prometheus
	router.GET("/metrics", func(c *gin.Context) {
		metrics := srv.GetPrometheusMetrics()
		c.String(http.StatusOK, metrics)
	})

	// Budget status endpoint
	router.GET("/api/v1/budget/status", func(c *gin.Context) {
		status := budgetTracker.GetOverallBudgetStatus()
		c.JSON(http.StatusOK, status)
	})

	// Start HTTP server
	httpServer := &http.Server{
		Addr:    fmt.Sprintf(":%d", cfg.Port),
		Handler: router,
	}

	// Graceful shutdown handling
	go func() {
		log.Printf("Variant Budget service starting on port %d", cfg.Port)
		log.Printf("Budget limits - Vehicle: %.1f%%, Sector: %.1f%%, Platform: %.1f%%", 
			cfg.BudgetConfig.VehicleBudgetLimit,
			cfg.BudgetConfig.SectorBudgetLimit,
			cfg.BudgetConfig.PlatformBudgetLimit)
		log.Printf("Enforcement enabled: %v", cfg.EnforcementConfig.Enabled)
		
		if err := httpServer.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Failed to start server: %v", err)
		}
	}()

	// Start background budget monitoring
	go func() {
		ticker := time.NewTicker(cfg.MonitoringInterval)
		defer ticker.Stop()
		
		for {
			select {
			case <-ticker.C:
				// Perform periodic budget analysis
				if err := srv.PerformPeriodicAnalysis(); err != nil {
					log.Printf("Periodic analysis failed: %v", err)
				}
			case <-context.Background().Done():
				return
			}
		}
	}()

	// Wait for interrupt signal to gracefully shutdown
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("Shutting down Variant Budget service...")

	// Graceful shutdown with timeout
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()

	if err := httpServer.Shutdown(ctx); err != nil {
		log.Fatalf("Server forced to shutdown: %v", err)
	}

	log.Println("Variant Budget service stopped")
}
