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
	"github.com/atlasmesh/fleet-os/services/vehicle-hal/internal/config"
	"github.com/atlasmesh/fleet-os/services/vehicle-hal/internal/hal"
	"github.com/atlasmesh/fleet-os/services/vehicle-hal/internal/profiles"
	"github.com/atlasmesh/fleet-os/services/vehicle-hal/internal/safety"
	"github.com/atlasmesh/fleet-os/services/vehicle-hal/internal/server"
)

// Vehicle Hardware Abstraction Layer (HAL) Service
// Provides vehicle-agnostic interface through config-driven profiles
// Enables qualified agnosticism across vehicle classes
func main() {
	// Load configuration
	cfg, err := config.Load()
	if err != nil {
		log.Fatalf("Failed to load configuration: %v", err)
	}

	// Initialize profile loader
	profileLoader, err := profiles.NewLoader(cfg.ProfilesPath)
	if err != nil {
		log.Fatalf("Failed to initialize profile loader: %v", err)
	}

	// Initialize safety monitor
	safetyMonitor := safety.NewMonitor(cfg.SafetyConfig)

	// Initialize HAL interface
	halInterface := hal.NewInterface(profileLoader, safetyMonitor)

	// Setup HTTP server
	router := gin.New()
	router.Use(gin.Logger())
	router.Use(gin.Recovery())

	// Initialize server with dependencies
	srv := server.New(halInterface, profileLoader, safetyMonitor)
	srv.SetupRoutes(router)

	// Health check endpoint
	router.GET("/health", func(c *gin.Context) {
		c.JSON(http.StatusOK, gin.H{
			"status":    "healthy",
			"service":   "vehicle-hal",
			"version":   cfg.Version,
			"timestamp": time.Now().UTC(),
		})
	})

	// Metrics endpoint for Prometheus
	router.GET("/metrics", func(c *gin.Context) {
		// TODO: Implement Prometheus metrics
		c.String(http.StatusOK, "# Vehicle HAL Metrics\n")
	})

	// Start HTTP server
	httpServer := &http.Server{
		Addr:    fmt.Sprintf(":%d", cfg.Port),
		Handler: router,
	}

	// Graceful shutdown handling
	go func() {
		log.Printf("Vehicle HAL service starting on port %d", cfg.Port)
		log.Printf("Profile path: %s", cfg.ProfilesPath)
		log.Printf("Safety monitoring: %v", cfg.SafetyConfig.Enabled)
		
		if err := httpServer.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			log.Fatalf("Failed to start server: %v", err)
		}
	}()

	// Wait for interrupt signal to gracefully shutdown
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Println("Shutting down Vehicle HAL service...")

	// Graceful shutdown with timeout
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()

	if err := httpServer.Shutdown(ctx); err != nil {
		log.Fatalf("Server forced to shutdown: %v", err)
	}

	log.Println("Vehicle HAL service stopped")
}
