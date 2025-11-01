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

	"atlasmesh/vehicle-agent/internal/config"
	"atlasmesh/vehicle-agent/internal/database"
	"atlasmesh/vehicle-agent/internal/logger"
	"atlasmesh/vehicle-agent/internal/metrics"
	"atlasmesh/vehicle-agent/internal/perception"
	"atlasmesh/vehicle-agent/internal/localization"
	"atlasmesh/vehicle-agent/internal/control"
	"atlasmesh/vehicle-agent/internal/safety"
	"atlasmesh/vehicle-agent/internal/cloud"
	"atlasmesh/vehicle-agent/internal/ros2"
)

// VehicleAgentService represents the main vehicle agent service
type VehicleAgentService struct {
	config        *config.Config
	sectorConfig  *config.SectorConfig
	logger        *zap.Logger
	db            *database.Database
	metrics       *metrics.Metrics
	perception    *perception.Module
	localization  *localization.Module
	control       *control.Module
	safety        *safety.Monitor
	cloud         *cloud.Bridge
	ros2          *ros2.Node
	router        *gin.Engine
	wsUpgrader    websocket.Upgrader
	ctx           context.Context
	cancel        context.CancelFunc
	wg            sync.WaitGroup
}

// VehicleState represents the current state of the vehicle
type VehicleState struct {
	VehicleID           string                 `json:"vehicle_id"`
	Timestamp           time.Time              `json:"timestamp"`
	OperationalMode     string                 `json:"operational_mode"` // AUTONOMOUS, MANUAL, EMERGENCY_STOP, PAUSED
	Position            Position               `json:"position"`
	Velocity            Velocity               `json:"velocity"`
	Acceleration        Acceleration           `json:"acceleration"`
	Heading             float64                `json:"heading_deg"`
	BatterySOC          float64                `json:"battery_soc_percent"`
	HealthScores        HealthScores           `json:"health_scores"`
	SensorHealth        SensorHealth           `json:"sensor_health"`
	ComputeHealth       ComputeHealth          `json:"compute_health"`
	NetworkHealth       NetworkHealth          `json:"network_health"`
	SafetyStatus        SafetyStatus           `json:"safety_status"`
	ODDCompliance       ODDCompliance          `json:"odd_compliance"`
	CurrentMission      *MissionInfo           `json:"current_mission,omitempty"`
	PerceptionSummary   PerceptionSummary      `json:"perception_summary"`
	LocalizationData    LocalizationData       `json:"localization_data"`
	ControlCommands     []ControlCommand       `json:"control_commands"`
	Alerts              []Alert                `json:"alerts"`
}

type Position struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
	Altitude  float64 `json:"altitude_m"`
	Accuracy  float64 `json:"accuracy_m"`
}

type Velocity struct {
	SpeedMPS     float64 `json:"speed_mps"`
	AngularVel   float64 `json:"angular_velocity_rad_per_s"`
	LateralVel   float64 `json:"lateral_velocity_mps"`
	LongitudinalVel float64 `json:"longitudinal_velocity_mps"`
}

type Acceleration struct {
	Ax float64 `json:"ax_mps2"`
	Ay float64 `json:"ay_mps2"`
	Az float64 `json:"az_mps2"`
}

type HealthScores struct {
	OverallHealthScore    float64 `json:"overall_health_score"`
	PerceptionHealthScore float64 `json:"perception_health_score"`
	LocalizationHealthScore float64 `json:"localization_health_score"`
	ControlHealthScore    float64 `json:"control_health_score"`
	SensorHealthScore     float64 `json:"sensor_health_score"`
	ComputeHealthScore    float64 `json:"compute_health_score"`
	NetworkHealthScore    float64 `json:"network_health_score"`
	BatteryHealthScore    float64 `json:"battery_health_score"`
}

type SensorHealth struct {
	LidarRPM        float64 `json:"lidar_rpm"`
	LidarStatus     string  `json:"lidar_status"`
	CameraStatus    string  `json:"camera_status"`
	RadarStatus     string  `json:"radar_status"`
	GNSSSatellites  int     `json:"gnss_satellites"`
	GNSSHDOP        float64 `json:"gnss_hdop"`
	IMUStatus       string  `json:"imu_status"`
	WheelEncoderStatus string `json:"wheel_encoder_status"`
}

type ComputeHealth struct {
	CPUPercent    float64 `json:"cpu_percent"`
	MemoryPercent float64 `json:"memory_percent"`
	GPUPercent    float64 `json:"gpu_percent"`
	Temperature   float64 `json:"temperature_c"`
	DiskUsage     float64 `json:"disk_usage_percent"`
}

type NetworkHealth struct {
	SignalStrength float64 `json:"signal_strength_dbm"`
	ConnectionType string  `json:"connection_type"` // 4G, 5G, WiFi
	Latency        float64 `json:"latency_ms"`
	Bandwidth      float64 `json:"bandwidth_mbps"`
	CloudConnected bool    `json:"cloud_connected"`
}

type SafetyStatus struct {
	EmergencyStopActive bool      `json:"emergency_stop_active"`
	SafetyViolations    int       `json:"safety_violations"`
	LastViolationTime   time.Time `json:"last_violation_time"`
	SafetyMode          string    `json:"safety_mode"` // NORMAL, CAUTION, WARNING, CRITICAL
}

type ODDCompliance struct {
	WithinODD           bool      `json:"within_odd"`
	ODDViolations       []string  `json:"odd_violations"`
	WeatherCompliant    bool      `json:"weather_compliant"`
	LightingCompliant   bool      `json:"lighting_compliant"`
	InfrastructureCompliant bool  `json:"infrastructure_compliant"`
}

type MissionInfo struct {
	MissionID    string    `json:"mission_id"`
	MissionType  string    `json:"mission_type"`
	Status       string    `json:"status"`
	Progress     float64   `json:"progress_percent"`
	StartTime    time.Time `json:"start_time"`
	EstimatedEnd time.Time `json:"estimated_end_time"`
}

type PerceptionSummary struct {
	NumObstacles           int     `json:"num_obstacles"`
	NearestObstacleDistance float64 `json:"nearest_obstacle_distance_m"`
	LaneDetection          bool    `json:"lane_detection_active"`
	TrafficLightStatus     string  `json:"traffic_light_status"`
	PedestrianDetection    bool    `json:"pedestrian_detection"`
}

type LocalizationData struct {
	PositionAccuracy    float64 `json:"position_accuracy_m"`
	HeadingAccuracy     float64 `json:"heading_accuracy_deg"`
	MapMatchingActive   bool    `json:"map_matching_active"`
	SLAMActive          bool    `json:"slam_active"`
	GPSQuality          string  `json:"gps_quality"`
}

type ControlCommand struct {
	CommandType string    `json:"command_type"`
	Value       float64   `json:"value"`
	Timestamp   time.Time `json:"timestamp"`
	Source      string    `json:"source"` // AUTONOMOUS, MANUAL, EMERGENCY
}

type Alert struct {
	AlertID     string    `json:"alert_id"`
	Severity    string    `json:"severity"` // INFO, WARNING, ERROR, CRITICAL
	Type        string    `json:"type"`
	Message     string    `json:"message"`
	Timestamp   time.Time `json:"timestamp"`
	Acknowledged bool     `json:"acknowledged"`
}

// EmergencyStopRequest represents an emergency stop request
type EmergencyStopRequest struct {
	Reason      string `json:"reason"`
	Source      string `json:"source"` // MANUAL, SYSTEM, REMOTE
	Timestamp   time.Time `json:"timestamp"`
}

// ControlCommandRequest represents a control command request
type ControlCommandRequest struct {
	CommandType string  `json:"command_type"` // SET_SPEED, STEER, BRAKE, ACCELERATE
	Value       float64 `json:"value"`
	Duration    float64 `json:"duration_seconds,omitempty"`
	Source      string  `json:"source"` // MANUAL, REMOTE, AUTONOMOUS
}

func main() {
	// Get sector from environment variable
	sector := os.Getenv("ATLASMESH_SECTOR")
	if sector == "" {
		sector = "logistics" // Default to logistics
	}

	// Load base configuration
	cfg, err := config.Load()
	if err != nil {
		log.Fatalf("Failed to load configuration: %v", err)
	}

	// Load sector-specific configuration
	sectorConfig, err := config.LoadSectorConfig(config.SectorType(sector))
	if err != nil {
		log.Fatalf("Failed to load sector configuration: %v", err)
	}

	// Validate sector configuration
	if err := sectorConfig.ValidateSectorConfig(); err != nil {
		log.Fatalf("Invalid sector configuration: %v", err)
	}

	// Initialize logger
	logger, err := logger.New(cfg.LogLevel)
	if err != nil {
		log.Fatalf("Failed to initialize logger: %v", err)
	}
	defer logger.Sync()

	logger.Info("Starting Vehicle Agent Service", 
		zap.String("sector", sector),
		zap.String("vehicle_id", cfg.VehicleID))

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
	service := &VehicleAgentService{
		config:        cfg,
		sectorConfig:  sectorConfig,
		logger:        logger,
		db:            db,
		metrics:       metrics,
		ctx:           ctx,
		cancel:        cancel,
		wsUpgrader: websocket.Upgrader{
			CheckOrigin: func(r *http.Request) bool {
				return true // In production, implement proper origin checking
			},
		},
	}

	// Initialize modules with sector-specific configuration
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

func (s *VehicleAgentService) initializeModules() error {
	s.logger.Info("Initializing vehicle agent modules with sector-specific configuration",
		zap.String("sector", string(s.sectorConfig.Sector)))

	// Initialize ROS2 node
	ros2Node, err := ros2.NewNode(s.config.ROS2, s.logger)
	if err != nil {
		return fmt.Errorf("failed to initialize ROS2 node: %w", err)
	}
	s.ros2 = ros2Node

	// Get sector-specific service configuration
	serviceConfig, err := s.sectorConfig.GetServiceConfig("vehicle-agent")
	if err != nil {
		return fmt.Errorf("failed to get vehicle-agent service configuration: %w", err)
	}

	// Initialize perception module with sector-specific configuration
	perceptionConfig := s.config.Perception
	if serviceConfig.Perception != nil {
		// Apply sector-specific perception configuration
		perceptionConfig = s.applySectorPerceptionConfig(perceptionConfig, serviceConfig.Perception)
	}
	perceptionModule, err := perception.New(perceptionConfig, s.logger, s.ros2)
	if err != nil {
		return fmt.Errorf("failed to initialize perception module: %w", err)
	}
	s.perception = perceptionModule

	// Initialize localization module with sector-specific configuration
	localizationConfig := s.config.Localization
	if serviceConfig.Localization != nil {
		// Apply sector-specific localization configuration
		localizationConfig = s.applySectorLocalizationConfig(localizationConfig, serviceConfig.Localization)
	}
	localizationModule, err := localization.New(localizationConfig, s.logger, s.ros2)
	if err != nil {
		return fmt.Errorf("failed to initialize localization module: %w", err)
	}
	s.localization = localizationModule

	// Initialize control module with sector-specific configuration
	controlConfig := s.config.Control
	if serviceConfig.Control != nil {
		// Apply sector-specific control configuration
		controlConfig = s.applySectorControlConfig(controlConfig, serviceConfig.Control)
	}
	controlModule, err := control.New(controlConfig, s.logger, s.ros2)
	if err != nil {
		return fmt.Errorf("failed to initialize control module: %w", err)
	}
	s.control = controlModule

	// Initialize safety monitor with sector-specific configuration
	safetyConfig := s.config.Safety
	if serviceConfig.Safety != nil {
		// Apply sector-specific safety configuration
		safetyConfig = s.applySectorSafetyConfig(safetyConfig, serviceConfig.Safety)
	}
	safetyMonitor, err := safety.New(safetyConfig, s.logger, s.ros2)
	if err != nil {
		return fmt.Errorf("failed to initialize safety monitor: %w", err)
	}
	s.safety = safetyMonitor

	// Initialize cloud bridge
	cloudBridge, err := cloud.New(s.config.Cloud, s.logger)
	if err != nil {
		return fmt.Errorf("failed to initialize cloud bridge: %w", err)
	}
	s.cloud = cloudBridge

	s.logger.Info("All modules initialized successfully with sector-specific configuration",
		zap.String("sector", string(s.sectorConfig.Sector)))
	return nil
}

func (s *VehicleAgentService) setupRoutes() {
	s.router = gin.New()
	s.router.Use(gin.Recovery())
	s.router.Use(s.corsMiddleware())
	s.router.Use(s.loggingMiddleware())

	// Health check
	s.router.GET("/health", s.healthCheck)

	// Vehicle control endpoints
	control := s.router.Group("/api/v1/vehicle-agent")
	{
		control.POST("/emergency-stop", s.emergencyStop)
		control.POST("/resume", s.resume)
		control.POST("/set-speed-limit", s.setSpeedLimit)
		control.POST("/update-route", s.updateRoute)
		control.POST("/control-command", s.controlCommand)
	}

	// Status endpoints
	status := s.router.Group("/api/v1/vehicle-agent")
	{
		status.GET("/status", s.getStatus)
		status.GET("/health", s.getHealth)
		status.GET("/sensors", s.getSensorStatus)
		status.GET("/position", s.getPosition)
		status.GET("/alerts", s.getAlerts)
	}

	// Configuration endpoints
	config := s.router.Group("/api/v1/vehicle-agent")
	{
		config.POST("/configure", s.configure)
		config.GET("/config", s.getConfig)
		config.POST("/calibrate", s.calibrate)
	}

	// WebSocket endpoints
	s.router.GET("/ws/telemetry", s.handleTelemetryWebSocket)
	s.router.GET("/ws/control", s.handleControlWebSocket)
}

func (s *VehicleAgentService) start() error {
	s.logger.Info("Starting vehicle agent service")

	// Start ROS2 node
	if err := s.ros2.Start(); err != nil {
		return fmt.Errorf("failed to start ROS2 node: %w", err)
	}

	// Start modules
	s.wg.Add(5)
	go s.startPerceptionModule()
	go s.startLocalizationModule()
	go s.startControlModule()
	go s.startSafetyMonitor()
	go s.startCloudBridge()

	// Start HTTP server
	s.wg.Add(1)
	go s.startHTTPServer()

	s.logger.Info("Vehicle agent service started successfully")
	return nil
}

func (s *VehicleAgentService) startPerceptionModule() {
	defer s.wg.Done()
	s.logger.Info("Starting perception module")
	
	if err := s.perception.Start(s.ctx); err != nil {
		s.logger.Error("Perception module failed", zap.Error(err))
	}
}

func (s *VehicleAgentService) startLocalizationModule() {
	defer s.wg.Done()
	s.logger.Info("Starting localization module")
	
	if err := s.localization.Start(s.ctx); err != nil {
		s.logger.Error("Localization module failed", zap.Error(err))
	}
}

func (s *VehicleAgentService) startControlModule() {
	defer s.wg.Done()
	s.logger.Info("Starting control module")
	
	if err := s.control.Start(s.ctx); err != nil {
		s.logger.Error("Control module failed", zap.Error(err))
	}
}

func (s *VehicleAgentService) startSafetyMonitor() {
	defer s.wg.Done()
	s.logger.Info("Starting safety monitor")
	
	if err := s.safety.Start(s.ctx); err != nil {
		s.logger.Error("Safety monitor failed", zap.Error(err))
	}
}

func (s *VehicleAgentService) startCloudBridge() {
	defer s.wg.Done()
	s.logger.Info("Starting cloud bridge")
	
	if err := s.cloud.Start(s.ctx); err != nil {
		s.logger.Error("Cloud bridge failed", zap.Error(err))
	}
}

func (s *VehicleAgentService) startHTTPServer() {
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

func (s *VehicleAgentService) waitForShutdown() {
	// Wait for interrupt signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	s.logger.Info("Shutting down vehicle agent service")

	// Cancel context
	s.cancel()

	// Wait for all goroutines to finish
	s.wg.Wait()

	s.logger.Info("Vehicle agent service stopped")
}

// HTTP Handlers

func (s *VehicleAgentService) healthCheck(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{
		"status":    "healthy",
		"timestamp": time.Now(),
		"service":   "vehicle-agent",
	})
}

func (s *VehicleAgentService) emergencyStop(c *gin.Context) {
	var req EmergencyStopRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	req.Timestamp = time.Now()

	// Execute emergency stop
	if err := s.control.EmergencyStop(req.Reason, req.Source); err != nil {
		s.logger.Error("Emergency stop failed", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Emergency stop failed"})
		return
	}

	// Log emergency stop
	s.logger.Warn("Emergency stop executed", 
		zap.String("reason", req.Reason),
		zap.String("source", req.Source))

	// Update metrics
	s.metrics.EmergencyStops.Inc()

	c.JSON(http.StatusOK, gin.H{
		"status":    "emergency_stop_executed",
		"timestamp": req.Timestamp,
	})
}

func (s *VehicleAgentService) resume(c *gin.Context) {
	// Resume operations
	if err := s.control.Resume(); err != nil {
		s.logger.Error("Resume failed", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Resume failed"})
		return
	}

	s.logger.Info("Operations resumed")

	c.JSON(http.StatusOK, gin.H{
		"status":    "resumed",
		"timestamp": time.Now(),
	})
}

func (s *VehicleAgentService) setSpeedLimit(c *gin.Context) {
	var req struct {
		SpeedLimit float64 `json:"speed_limit_mps"`
	}
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Set speed limit
	if err := s.control.SetSpeedLimit(req.SpeedLimit); err != nil {
		s.logger.Error("Set speed limit failed", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Set speed limit failed"})
		return
	}

	s.logger.Info("Speed limit set", zap.Float64("speed_limit", req.SpeedLimit))

	c.JSON(http.StatusOK, gin.H{
		"status":       "speed_limit_set",
		"speed_limit":  req.SpeedLimit,
		"timestamp":    time.Now(),
	})
}

func (s *VehicleAgentService) updateRoute(c *gin.Context) {
	var req struct {
		Route []Position `json:"route"`
	}
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Update route
	if err := s.control.UpdateRoute(req.Route); err != nil {
		s.logger.Error("Update route failed", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Update route failed"})
		return
	}

	s.logger.Info("Route updated", zap.Int("waypoints", len(req.Route)))

	c.JSON(http.StatusOK, gin.H{
		"status":    "route_updated",
		"waypoints": len(req.Route),
		"timestamp": time.Now(),
	})
}

func (s *VehicleAgentService) controlCommand(c *gin.Context) {
	var req ControlCommandRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Execute control command
	if err := s.control.ExecuteCommand(req.CommandType, req.Value, req.Source); err != nil {
		s.logger.Error("Control command failed", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Control command failed"})
		return
	}

	s.logger.Info("Control command executed",
		zap.String("type", req.CommandType),
		zap.Float64("value", req.Value),
		zap.String("source", req.Source))

	c.JSON(http.StatusOK, gin.H{
		"status":    "command_executed",
		"timestamp": time.Now(),
	})
}

func (s *VehicleAgentService) getStatus(c *gin.Context) {
	// Get current vehicle state
	state, err := s.getCurrentVehicleState()
	if err != nil {
		s.logger.Error("Failed to get vehicle state", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get vehicle state"})
		return
	}

	c.JSON(http.StatusOK, state)
}

func (s *VehicleAgentService) getHealth(c *gin.Context) {
	// Get system health
	health := s.getSystemHealth()
	c.JSON(http.StatusOK, health)
}

func (s *VehicleAgentService) getSensorStatus(c *gin.Context) {
	// Get sensor status
	sensorStatus := s.getSensorStatus()
	c.JSON(http.StatusOK, sensorStatus)
}

func (s *VehicleAgentService) getPosition(c *gin.Context) {
	// Get current position
	position, err := s.localization.GetCurrentPosition()
	if err != nil {
		s.logger.Error("Failed to get position", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get position"})
		return
	}

	c.JSON(http.StatusOK, position)
}

func (s *VehicleAgentService) getAlerts(c *gin.Context) {
	// Get active alerts
	alerts, err := s.safety.GetActiveAlerts()
	if err != nil {
		s.logger.Error("Failed to get alerts", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to get alerts"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"alerts":    alerts,
		"timestamp": time.Now(),
	})
}

func (s *VehicleAgentService) configure(c *gin.Context) {
	var config map[string]interface{}
	if err := c.ShouldBindJSON(&config); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Update configuration
	if err := s.updateConfiguration(config); err != nil {
		s.logger.Error("Configuration update failed", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Configuration update failed"})
		return
	}

	s.logger.Info("Configuration updated")

	c.JSON(http.StatusOK, gin.H{
		"status":    "configuration_updated",
		"timestamp": time.Now(),
	})
}

func (s *VehicleAgentService) getConfig(c *gin.Context) {
	// Get current configuration
	config := s.getCurrentConfiguration()
	c.JSON(http.StatusOK, config)
}

func (s *VehicleAgentService) calibrate(c *gin.Context) {
	var req struct {
		SensorType string `json:"sensor_type"`
	}
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Start calibration
	if err := s.startCalibration(req.SensorType); err != nil {
		s.logger.Error("Calibration failed", zap.Error(err))
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Calibration failed"})
		return
	}

	s.logger.Info("Calibration started", zap.String("sensor_type", req.SensorType))

	c.JSON(http.StatusOK, gin.H{
		"status":      "calibration_started",
		"sensor_type": req.SensorType,
		"timestamp":   time.Now(),
	})
}

// WebSocket Handlers

func (s *VehicleAgentService) handleTelemetryWebSocket(c *gin.Context) {
	conn, err := s.wsUpgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		s.logger.Error("WebSocket upgrade failed", zap.Error(err))
		return
	}
	defer conn.Close()

	s.logger.Info("Telemetry WebSocket connected")

	// Send telemetry data
	ticker := time.NewTicker(100 * time.Millisecond) // 10Hz
	defer ticker.Stop()

	for {
		select {
		case <-s.ctx.Done():
			return
		case <-ticker.C:
			state, err := s.getCurrentVehicleState()
			if err != nil {
				s.logger.Error("Failed to get vehicle state", zap.Error(err))
				continue
			}

			if err := conn.WriteJSON(state); err != nil {
				s.logger.Error("WebSocket write failed", zap.Error(err))
				return
			}
		}
	}
}

func (s *VehicleAgentService) handleControlWebSocket(c *gin.Context) {
	conn, err := s.wsUpgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		s.logger.Error("WebSocket upgrade failed", zap.Error(err))
		return
	}
	defer conn.Close()

	s.logger.Info("Control WebSocket connected")

	for {
		select {
		case <-s.ctx.Done():
			return
		default:
			var cmd ControlCommandRequest
			if err := conn.ReadJSON(&cmd); err != nil {
				if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
					s.logger.Error("WebSocket read error", zap.Error(err))
				}
				return
			}

			// Execute control command
			if err := s.control.ExecuteCommand(cmd.CommandType, cmd.Value, cmd.Source); err != nil {
				s.logger.Error("Control command failed", zap.Error(err))
				conn.WriteJSON(gin.H{"error": err.Error()})
				continue
			}

			// Send acknowledgment
			conn.WriteJSON(gin.H{
				"status":    "command_executed",
				"timestamp": time.Now(),
			})
		}
	}
}

// Helper Methods

func (s *VehicleAgentService) getCurrentVehicleState() (*VehicleState, error) {
	// This would integrate with ROS2 to get real-time vehicle state
	// For now, return a mock state
	state := &VehicleState{
		VehicleID:       s.config.VehicleID,
		Timestamp:       time.Now(),
		OperationalMode: "AUTONOMOUS",
		Position: Position{
			Latitude:  25.2048,
			Longitude: 55.2744,
			Altitude:  10.0,
			Accuracy:  1.0,
		},
		Velocity: Velocity{
			SpeedMPS: 2.0,
		},
		BatterySOC: 85.0,
		HealthScores: HealthScores{
			OverallHealthScore: 95.0,
		},
		SafetyStatus: SafetyStatus{
			SafetyMode: "NORMAL",
		},
		ODDCompliance: ODDCompliance{
			WithinODD: true,
		},
	}

	return state, nil
}

func (s *VehicleAgentService) getSystemHealth() map[string]interface{} {
	return map[string]interface{}{
		"overall_health": 95.0,
		"perception":     98.0,
		"localization":   92.0,
		"control":        96.0,
		"sensors":        94.0,
		"compute":        88.0,
		"network":        90.0,
		"battery":        85.0,
		"timestamp":      time.Now(),
	}
}

func (s *VehicleAgentService) getSensorStatus() map[string]interface{} {
	return map[string]interface{}{
		"lidar": map[string]interface{}{
			"status": "healthy",
			"rpm":    600.0,
		},
		"camera": map[string]interface{}{
			"status": "healthy",
		},
		"radar": map[string]interface{}{
			"status": "healthy",
		},
		"gnss": map[string]interface{}{
			"status":     "healthy",
			"satellites": 12,
			"hdop":       1.2,
		},
		"imu": map[string]interface{}{
			"status": "healthy",
		},
		"timestamp": time.Now(),
	}
}

func (s *VehicleAgentService) updateConfiguration(config map[string]interface{}) error {
	// Update configuration in database and notify modules
	return nil
}

func (s *VehicleAgentService) getCurrentConfiguration() map[string]interface{} {
	// Get current configuration from database
	return map[string]interface{}{
		"vehicle_id": s.config.VehicleID,
		"timestamp":  time.Now(),
	}
}

func (s *VehicleAgentService) startCalibration(sensorType string) error {
	// Start sensor calibration process
	return nil
}

// Sector-specific configuration application methods

func (s *VehicleAgentService) applySectorPerceptionConfig(baseConfig *config.PerceptionConfig, sectorConfig *config.PerceptionConfig) *config.PerceptionConfig {
	// Create a copy of base config
	perceptionConfig := *baseConfig
	
	// Apply sector-specific settings
	if sectorConfig.PalletDetection {
		perceptionConfig.PalletDetection = true
	}
	if sectorConfig.DockRecognition {
		perceptionConfig.DockRecognition = true
	}
	if sectorConfig.ThreatDetection {
		perceptionConfig.ThreatDetection = true
	}
	if sectorConfig.FormationAwareness {
		perceptionConfig.FormationAwareness = true
	}
	if sectorConfig.BlastZoneDetection {
		perceptionConfig.BlastZoneDetection = true
	}
	if sectorConfig.EquipmentRecognition {
		perceptionConfig.EquipmentRecognition = true
	}
	if sectorConfig.PedestrianDetection {
		perceptionConfig.PedestrianDetection = true
	}
	if sectorConfig.TrafficSignRecognition {
		perceptionConfig.TrafficSignRecognition = true
	}
	
	// Apply sector-specific parameters
	if sectorConfig.Parameters != nil {
		for key, value := range sectorConfig.Parameters {
			perceptionConfig.Parameters[key] = value
		}
	}
	
	return &perceptionConfig
}

func (s *VehicleAgentService) applySectorLocalizationConfig(baseConfig *config.LocalizationConfig, sectorConfig *config.LocalizationConfig) *config.LocalizationConfig {
	// Create a copy of base config
	localizationConfig := *baseConfig
	
	// Apply sector-specific settings
	if sectorConfig.Strategy != "" {
		localizationConfig.Strategy = sectorConfig.Strategy
	}
	if sectorConfig.Accuracy != "" {
		localizationConfig.Accuracy = sectorConfig.Accuracy
	}
	
	// Apply sector-specific parameters
	if sectorConfig.Parameters != nil {
		for key, value := range sectorConfig.Parameters {
			localizationConfig.Parameters[key] = value
		}
	}
	
	// Apply fallback configuration
	if sectorConfig.Fallback != nil {
		localizationConfig.Fallback = sectorConfig.Fallback
	}
	
	return &localizationConfig
}

func (s *VehicleAgentService) applySectorControlConfig(baseConfig *config.ControlConfig, sectorConfig *config.ControlConfig) *config.ControlConfig {
	// Create a copy of base config
	controlConfig := *baseConfig
	
	// Apply sector-specific settings
	if sectorConfig.MaxSpeed != "" {
		controlConfig.MaxSpeed = sectorConfig.MaxSpeed
	}
	if sectorConfig.Precision != "" {
		controlConfig.Precision = sectorConfig.Precision
	}
	if sectorConfig.FormationControl {
		controlConfig.FormationControl = true
	}
	if sectorConfig.ConvoyCoordination {
		controlConfig.ConvoyCoordination = true
	}
	if sectorConfig.HeavyVehicleDynamics {
		controlConfig.HeavyVehicleDynamics = true
	}
	if sectorConfig.SlopeHandling {
		controlConfig.SlopeHandling = true
	}
	if sectorConfig.SmoothRide {
		controlConfig.SmoothRide = true
	}
	if sectorConfig.TrafficCompliance {
		controlConfig.TrafficCompliance = true
	}
	
	// Apply sector-specific parameters
	if sectorConfig.Parameters != nil {
		for key, value := range sectorConfig.Parameters {
			controlConfig.Parameters[key] = value
		}
	}
	
	return &controlConfig
}

func (s *VehicleAgentService) applySectorSafetyConfig(baseConfig *config.SafetyConfig, sectorConfig *config.SafetyConfig) *config.SafetyConfig {
	// Create a copy of base config
	safetyConfig := *baseConfig
	
	// Apply sector-specific settings
	if sectorConfig.EmergencyStop {
		safetyConfig.EmergencyStop = true
	}
	if sectorConfig.CollisionAvoidance {
		safetyConfig.CollisionAvoidance = true
	}
	if sectorConfig.FormationSafety {
		safetyConfig.FormationSafety = true
	}
	if sectorConfig.MissionSafety {
		safetyConfig.MissionSafety = true
	}
	if sectorConfig.MiningSafety {
		safetyConfig.MiningSafety = true
	}
	if sectorConfig.BlastZoneSafety {
		safetyConfig.BlastZoneSafety = true
	}
	if sectorConfig.PassengerSafety {
		safetyConfig.PassengerSafety = true
	}
	if sectorConfig.TrafficSafety {
		safetyConfig.TrafficSafety = true
	}
	
	// Apply sector-specific parameters
	if sectorConfig.Parameters != nil {
		for key, value := range sectorConfig.Parameters {
			safetyConfig.Parameters[key] = value
		}
	}
	
	return &safetyConfig
}

// Middleware

func (s *VehicleAgentService) corsMiddleware() gin.HandlerFunc {
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

func (s *VehicleAgentService) loggingMiddleware() gin.HandlerFunc {
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
