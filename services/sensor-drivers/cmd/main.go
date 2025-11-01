package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"go.uber.org/zap"
	"go.uber.org/zap/zapcore"

	"atlasmesh/sensor-drivers/internal/config"
	"atlasmesh/sensor-drivers/internal/database"
	"atlasmesh/sensor-drivers/internal/logger"
	"atlasmesh/sensor-drivers/internal/metrics"
	"atlasmesh/sensor-drivers/internal/sensormanager"
	"atlasmesh/sensor-drivers/internal/dataprocessor"
	"atlasmesh/sensor-drivers/internal/calibration"
)

// SensorDriversService represents the main sensor drivers service
type SensorDriversService struct {
	config         *config.Config
	logger         *zap.Logger
	db             *database.Database
	metrics        *metrics.Metrics
	sensorManager  *sensormanager.Manager
	dataProcessor  *dataprocessor.Processor
	calibration    *calibration.Manager
	router         *gin.Engine
	wsUpgrader     websocket.Upgrader
	ctx            context.Context
	cancel         context.CancelFunc
	wg             sync.WaitGroup
}

// Sensor represents a sensor
type Sensor struct {
	ID          string                 `json:"id"`
	Type        string                 `json:"type"`
	Name        string                 `json:"name"`
	Status      string                 `json:"status"` // IDLE, RUNNING, ERROR, CALIBRATING
	Config      map[string]interface{} `json:"config"`
	Health      SensorHealth           `json:"health"`
	Data        SensorData             `json:"data,omitempty"`
	Created     time.Time              `json:"created"`
	Updated     time.Time              `json:"updated"`
}

type SensorHealth struct {
	OverallHealth    float64 `json:"overall_health"`
	DataQuality      float64 `json:"data_quality"`
	ConnectionStatus string  `json:"connection_status"`
	LastDataTime     time.Time `json:"last_data_time"`
	ErrorCount       int     `json:"error_count"`
	WarningCount     int     `json:"warning_count"`
}

type SensorData struct {
	Timestamp time.Time              `json:"timestamp"`
	SensorID  string                 `json:"sensor_id"`
	Type      string                 `json:"type"`
	Data      map[string]interface{} `json:"data"`
	Metadata  map[string]interface{} `json:"metadata"`
}

// SensorConfig represents sensor configuration
type SensorConfig struct {
	ID          string                 `json:"id"`
	Type        string                 `json:"type"`
	Name        string                 `json:"name"`
	Parameters  map[string]interface{} `json:"parameters"`
	Calibration map[string]interface{} `json:"calibration"`
	Enabled     bool                   `json:"enabled"`
}

// CalibrationData represents calibration data
type CalibrationData struct {
	SensorID    string                 `json:"sensor_id"`
	Type        string                 `json:"type"`
	Parameters  map[string]interface{} `json:"parameters"`
	Quality     float64                `json:"quality"`
	Status      string                 `json:"status"`
	Created     time.Time              `json:"created"`
	Updated     time.Time              `json:"updated"`
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

	// Initialize service
	service := &SensorDriversService{
		config:  cfg,
		logger:  logger,
		db:      db,
		metrics: metrics,
		ctx:     ctx,
		cancel:  cancel,
		wsUpgrader: websocket.Upgrader{
			CheckOrigin: func(r *http.Request) bool {
				return true // In production, implement proper origin checking
			},
		},
	}

	// Initialize modules
	if err := service.initializeModules(); err != nil {
		logger.Fatal("Failed to initialize modules", zap.Error(err))
	}

	// Setup routes
	service.setupRoutes()

	// Start service
	if err := service.start(); err != nil {
		logger.Fatal("Failed to start service", zap.Error(err))
	}

	// Wait for shutdown signal
	service.waitForShutdown()
}

func (s *SensorDriversService) initializeModules() error {
	s.logger.Info("Initializing sensor drivers service modules")

	// Initialize sensor manager
	sensorManager, err := sensormanager.New(s.config.SensorManager, s.logger, s.db)
	if err != nil {
		return fmt.Errorf("failed to initialize sensor manager: %w", err)
	}
	s.sensorManager = sensorManager

	// Initialize data processor
	dataProcessor, err := dataprocessor.New(s.config.DataProcessor, s.logger, s.sensorManager)
	if err != nil {
		return fmt.Errorf("failed to initialize data processor: %w", err)
	}
	s.dataProcessor = dataProcessor

	// Initialize calibration manager
	calibration, err := calibration.New(s.config.Calibration, s.logger, s.db)
	if err != nil {
		return fmt.Errorf("failed to initialize calibration manager: %w", err)
	}
	s.calibration = calibration

	s.logger.Info("All modules initialized successfully")
	return nil
}

func (s *SensorDriversService) setupRoutes() {
	s.router = gin.New()
	s.router.Use(gin.Recovery())
	s.router.Use(s.corsMiddleware())
	s.router.Use(s.loggingMiddleware())

	// Health check
	s.router.GET("/health", s.healthCheck)

	// Sensor management endpoints
	sensors := s.router.Group("/api/v1/sensors")
	{
		sensors.GET("", s.listSensors)
		sensors.GET("/:id", s.getSensor)
		sensors.POST("/:id/start", s.startSensor)
		sensors.POST("/:id/stop", s.stopSensor)
		sensors.POST("/:id/reset", s.resetSensor)
	}

	// Data acquisition endpoints
	data := s.router.Group("/api/v1/sensors")
	{
		data.GET("/:id/data", s.getSensorData)
		data.GET("/:id/stream", s.streamSensorData)
		data.POST("/:id/capture", s.captureSensorData)
	}

	// Calibration endpoints
	calibration := s.router.Group("/api/v1/sensors")
	{
		calibration.POST("/:id/calibrate", s.calibrateSensor)
		calibration.GET("/:id/calibration", s.getCalibrationData)
		calibration.PUT("/:id/calibration", s.updateCalibrationData)
	}

	// Configuration endpoints
	config := s.router.Group("/api/v1/sensors")
	{
		config.GET("/:id/config", s.getSensorConfig)
		config.PUT("/:id/config", s.updateSensorConfig)
		config.POST("/:id/test", s.testSensorConfig)
	}

	// WebSocket endpoints
	s.router.GET("/ws/sensor-data", s.handleSensorDataWebSocket)
}

func (s *SensorDriversService) start() error {
	s.logger.Info("Starting sensor drivers service")

	// Start sensor manager
	s.wg.Add(1)
	go s.startSensorManager()

	// Start data processor
	s.wg.Add(1)
	go s.startDataProcessor()

	// Start calibration manager
	s.wg.Add(1)
	go s.startCalibrationManager()

	// Start HTTP server
	s.wg.Add(1)
	go s.startHTTPServer()

	s.logger.Info("Sensor drivers service started successfully")
	return nil
}

func (s *SensorDriversService) startSensorManager() {
	defer s.wg.Done()
	s.logger.Info("Starting sensor manager")
	
	if err := s.sensorManager.Start(s.ctx); err != nil {
		s.logger.Error("Sensor manager failed", zap.Error(err))
	}
}

func (s *SensorDriversService) startDataProcessor() {
	defer s.wg.Done()
	s.logger.Info("Starting data processor")
	
	if err := s.dataProcessor.Start(s.ctx); err != nil {
		s.logger.Error("Data processor failed", zap.Error(err))
	}
}

func (s *SensorDriversService) startCalibrationManager() {
	defer s.wg.Done()
	s.logger.Info("Starting calibration manager")
	
	if err := s.calibration.Start(s.ctx); err != nil {
		s.logger.Error("Calibration manager failed", zap.Error(err))
	}
}

func (s *SensorDriversService) startHTTPServer() {
	defer s.wg.Done()
	
	server := &http.Server{
		Addr:    fmt.Sprintf(":%d", s.config.Server.Port),
		Handler: s.router,
	}

	s.logger.Info("Starting HTTP server", zap.Int("port", s.config.Server.Port))

	if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		s.logger.Error("HTTP server failed", zap.Error(err))
	}
}

func (s *SensorDriversService) waitForShutdown() {
	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	s.logger.Info("Shutting down sensor drivers service")

	// Cancel context
	s.cancel()

	// Wait for all goroutines to finish
	s.wg.Wait()

	s.logger.Info("Sensor drivers service stopped")
}

// HTTP Handlers

func (s *SensorDriversService) healthCheck(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{
		"status":    "healthy",
		"timestamp": time.Now(),
		"service":   "sensor-drivers",
	})
}

func (s *SensorDriversService) listSensors(c *gin.Context) {
	// Get all sensors
	sensors, err := s.sensorManager.GetAllSensors()
	if err != nil {
		s.logger.Error("Failed to get sensors", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get sensors"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"sensors":   sensors,
		"timestamp": time.Now(),
	})
}

func (s *SensorDriversService) getSensor(c *gin.Context) {
	sensorID := c.Param("id")

	// Get sensor
	sensor, err := s.sensorManager.GetSensor(sensorID)
	if err != nil {
		s.logger.Error("Failed to get sensor", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get sensor"})
		return
	}

	if sensor == nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Sensor not found"})
		return
	}

	c.JSON(http.StatusOK, sensor)
}

func (s *SensorDriversService) startSensor(c *gin.Context) {
	sensorID := c.Param("id")

	// Start sensor
	if err := s.sensorManager.StartSensor(sensorID); err != nil {
		s.logger.Error("Failed to start sensor", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to start sensor"})
		return
	}

	s.logger.Info("Sensor started", zap.String("sensor_id", sensorID))

	c.JSON(http.StatusOK, gin.H{
		"status":    "started",
		"sensor_id": sensorID,
		"timestamp": time.Now(),
	})
}

func (s *SensorDriversService) stopSensor(c *gin.Context) {
	sensorID := c.Param("id")

	// Stop sensor
	if err := s.sensorManager.StopSensor(sensorID); err != nil {
		s.logger.Error("Failed to stop sensor", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to stop sensor"})
		return
	}

	s.logger.Info("Sensor stopped", zap.String("sensor_id", sensorID))

	c.JSON(http.StatusOK, gin.H{
		"status":    "stopped",
		"sensor_id": sensorID,
		"timestamp": time.Now(),
	})
}

func (s *SensorDriversService) resetSensor(c *gin.Context) {
	sensorID := c.Param("id")

	// Reset sensor
	if err := s.sensorManager.ResetSensor(sensorID); err != nil {
		s.logger.Error("Failed to reset sensor", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to reset sensor"})
		return
	}

	s.logger.Info("Sensor reset", zap.String("sensor_id", sensorID))

	c.JSON(http.StatusOK, gin.H{
		"status":    "reset",
		"sensor_id": sensorID,
		"timestamp": time.Now(),
	})
}

func (s *SensorDriversService) getSensorData(c *gin.Context) {
	sensorID := c.Param("id")

	// Get sensor data
	data, err := s.dataProcessor.GetLatestData(sensorID)
	if err != nil {
		s.logger.Error("Failed to get sensor data", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get sensor data"})
		return
	}

	c.JSON(http.StatusOK, data)
}

func (s *SensorDriversService) streamSensorData(c *gin.Context) {
	sensorID := c.Param("id")

	// Stream sensor data
	conn, err := s.wsUpgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		s.logger.Error("WebSocket upgrade failed", zap.Error(err))
		return
	}
	defer conn.Close()

	s.logger.Info("Sensor data WebSocket connected", zap.String("sensor_id", sensorID))

	// Stream data
	ticker := time.NewTicker(100 * time.Millisecond) // 10Hz
	defer ticker.Stop()

	for {
		select {
		case <-s.ctx.Done():
			return
		case <-ticker.C:
			data, err := s.dataProcessor.GetLatestData(sensorID)
			if err != nil {
				s.logger.Error("Failed to get sensor data", zap.Error(err))
				continue
			}

			if err := conn.WriteJSON(data); err != nil {
				s.logger.Error("WebSocket write failed", zap.Error(err))
				return
			}
		}
	}
}

func (s *SensorDriversService) captureSensorData(c *gin.Context) {
	sensorID := c.Param("id")

	// Capture single frame
	data, err := s.dataProcessor.CaptureData(sensorID)
	if err != nil {
		s.logger.Error("Failed to capture sensor data", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to capture sensor data"})
		return
	}

	c.JSON(http.StatusOK, data)
}

func (s *SensorDriversService) calibrateSensor(c *gin.Context) {
	sensorID := c.Param("id")

	// Start calibration
	if err := s.calibration.StartCalibration(sensorID); err != nil {
		s.logger.Error("Failed to start calibration", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to start calibration"})
		return
	}

	s.logger.Info("Calibration started", zap.String("sensor_id", sensorID))

	c.JSON(http.StatusOK, gin.H{
		"status":    "calibration_started",
		"sensor_id": sensorID,
		"timestamp": time.Now(),
	})
}

func (s *SensorDriversService) getCalibrationData(c *gin.Context) {
	sensorID := c.Param("id")

	// Get calibration data
	calibration, err := s.calibration.GetCalibrationData(sensorID)
	if err != nil {
		s.logger.Error("Failed to get calibration data", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get calibration data"})
		return
	}

	c.JSON(http.StatusOK, calibration)
}

func (s *SensorDriversService) updateCalibrationData(c *gin.Context) {
	sensorID := c.Param("id")

	var calibrationData CalibrationData
	if err := c.ShouldBindJSON(&calibrationData); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Update calibration data
	if err := s.calibration.UpdateCalibrationData(sensorID, calibrationData); err != nil {
		s.logger.Error("Failed to update calibration data", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to update calibration data"})
		return
	}

	s.logger.Info("Calibration data updated", zap.String("sensor_id", sensorID))

	c.JSON(http.StatusOK, gin.H{
		"status":    "calibration_updated",
		"sensor_id": sensorID,
		"timestamp": time.Now(),
	})
}

func (s *SensorDriversService) getSensorConfig(c *gin.Context) {
	sensorID := c.Param("id")

	// Get sensor configuration
	config, err := s.sensorManager.GetSensorConfig(sensorID)
	if err != nil {
		s.logger.Error("Failed to get sensor config", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get sensor config"})
		return
	}

	c.JSON(http.StatusOK, config)
}

func (s *SensorDriversService) updateSensorConfig(c *gin.Context) {
	sensorID := c.Param("id")

	var config SensorConfig
	if err := c.ShouldBindJSON(&config); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Update sensor configuration
	if err := s.sensorManager.UpdateSensorConfig(sensorID, config); err != nil {
		s.logger.Error("Failed to update sensor config", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to update sensor config"})
		return
	}

	s.logger.Info("Sensor config updated", zap.String("sensor_id", sensorID))

	c.JSON(http.StatusOK, gin.H{
		"status":    "config_updated",
		"sensor_id": sensorID,
		"timestamp": time.Now(),
	})
}

func (s *SensorDriversService) testSensorConfig(c *gin.Context) {
	sensorID := c.Param("id")

	// Test sensor configuration
	result, err := s.sensorManager.TestSensorConfig(sensorID)
	if err != nil {
		s.logger.Error("Failed to test sensor config", zap.String("sensor_id", sensorID), zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to test sensor config"})
		return
	}

	c.JSON(http.StatusOK, result)
}

// WebSocket Handlers

func (s *SensorDriversService) handleSensorDataWebSocket(c *gin.Context) {
	conn, err := s.wsUpgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		s.logger.Error("WebSocket upgrade failed", zap.Error(err))
		return
	}
	defer conn.Close()

	s.logger.Info("Sensor data WebSocket connected")

	// Stream all sensor data
	ticker := time.NewTicker(100 * time.Millisecond) // 10Hz
	defer ticker.Stop()

	for {
		select {
		case <-s.ctx.Done():
			return
		case <-ticker.C:
			// Get data from all sensors
			sensors, err := s.sensorManager.GetAllSensors()
			if err != nil {
				s.logger.Error("Failed to get sensors", zap.Error(err))
				continue
			}

			for _, sensor := range sensors {
				data, err := s.dataProcessor.GetLatestData(sensor.ID)
				if err != nil {
					continue
				}

				if err := conn.WriteJSON(data); err != nil {
					s.logger.Error("WebSocket write failed", zap.Error(err))
					return
				}
			}
		}
	}
}

// Middleware

func (s *SensorDriversService) corsMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		c.Header("Access-Control-Allow-Origin", "*")
		c.Header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
		c.Header("Access-Control-Allow-Headers", "Origin, Content-Type, Accept, Authorization")

		if c.Request.Method == "OPTIONS" {
			c.AbortWithStatus(204)
			return
		}

		c.Next()
	}
}

func (s *SensorDriversService) loggingMiddleware() gin.HandlerFunc {
	return gin.LoggerWithFormatter(func(param gin.LogFormatterParams) string {
		s.logger.Info("HTTP Request",
			zap.String("method", param.Method),
			zap.String("path", param.Path),
			zap.Int("status", param.StatusCode),
			zap.Duration("latency", param.Latency),
			zap.String("client_ip", param.ClientIP),
		)
		return ""
	})
}
