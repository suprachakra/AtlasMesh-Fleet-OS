package cloud

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"sync"
	"time"

	"go.uber.org/zap"
)

// Bridge represents the cloud bridge for vehicle agent
type Bridge struct {
	config     *Config
	logger     *zap.Logger
	ctx        context.Context
	cancel     context.CancelFunc
	wg         sync.WaitGroup
	started    bool
	mu         sync.RWMutex

	// HTTP client
	httpClient *http.Client

	// State
	connected     bool
	lastHeartbeat time.Time
	vehicleID     string
}

// Config represents cloud bridge configuration
type Config struct {
	CloudURL        string `yaml:"cloud_url"`
	VehicleID       string `yaml:"vehicle_id"`
	APIKey          string `yaml:"api_key"`
	HeartbeatInterval int  `yaml:"heartbeat_interval_s"`
	TelemetryInterval int  `yaml:"telemetry_interval_s"`
	CommandTimeout  int    `yaml:"command_timeout_s"`
	RetryAttempts   int    `yaml:"retry_attempts"`
	RetryDelay      int    `yaml:"retry_delay_s"`
}

// TelemetryMessage represents telemetry data sent to cloud
type TelemetryMessage struct {
	VehicleID     string                 `json:"vehicle_id"`
	Timestamp     time.Time              `json:"timestamp"`
	Position      Position               `json:"position"`
	Velocity      Velocity               `json:"velocity"`
	Acceleration  Acceleration           `json:"acceleration"`
	Heading       float64                `json:"heading_deg"`
	BatterySOC    float64                `json:"battery_soc_percent"`
	HealthScores  HealthScores           `json:"health_scores"`
	SensorHealth  SensorHealth           `json:"sensor_health"`
	ComputeHealth ComputeHealth          `json:"compute_health"`
	NetworkHealth NetworkHealth          `json:"network_health"`
	SafetyStatus  SafetyStatus           `json:"safety_status"`
	ODDCompliance ODDCompliance          `json:"odd_compliance"`
	PerceptionSummary PerceptionSummary  `json:"perception_summary"`
	LocalizationData LocalizationData    `json:"localization_data"`
	Alerts        []Alert                `json:"alerts"`
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
	ConnectionType string  `json:"connection_type"`
	Latency        float64 `json:"latency_ms"`
	Bandwidth      float64 `json:"bandwidth_mbps"`
	CloudConnected bool    `json:"cloud_connected"`
}

type SafetyStatus struct {
	EmergencyStopActive bool      `json:"emergency_stop_active"`
	SafetyViolations    int       `json:"safety_violations"`
	LastViolationTime   time.Time `json:"last_violation_time"`
	SafetyMode          string    `json:"safety_mode"`
}

type ODDCompliance struct {
	WithinODD           bool      `json:"within_odd"`
	ODDViolations       []string  `json:"odd_violations"`
	WeatherCompliant    bool      `json:"weather_compliant"`
	LightingCompliant   bool      `json:"lighting_compliant"`
	InfrastructureCompliant bool  `json:"infrastructure_compliant"`
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

type Alert struct {
	AlertID     string    `json:"alert_id"`
	Severity    string    `json:"severity"`
	Type        string    `json:"type"`
	Message     string    `json:"message"`
	Timestamp   time.Time `json:"timestamp"`
	Acknowledged bool     `json:"acknowledged"`
}

// VehicleCommand represents a command from the cloud
type VehicleCommand struct {
	CommandID   string                 `json:"command_id"`
	VehicleID   string                 `json:"vehicle_id"`
	CommandType string                 `json:"command_type"`
	Parameters  map[string]interface{} `json:"parameters"`
	Timestamp   time.Time              `json:"timestamp"`
	Priority    int                    `json:"priority"`
	Source      string                 `json:"source"`
}

// New creates a new cloud bridge
func New(config *Config, logger *zap.Logger) (*Bridge, error) {
	if config == nil {
		return nil, fmt.Errorf("config cannot be nil")
	}
	if logger == nil {
		return nil, fmt.Errorf("logger cannot be nil")
	}

	ctx, cancel := context.WithCancel(context.Background())

	bridge := &Bridge{
		config:     config,
		logger:     logger,
		ctx:        ctx,
		cancel:     cancel,
		vehicleID:  config.VehicleID,
		httpClient: &http.Client{
			Timeout: time.Duration(config.CommandTimeout) * time.Second,
		},
	}

	return bridge, nil
}

// Start starts the cloud bridge
func (b *Bridge) Start(ctx context.Context) error {
	b.mu.Lock()
	defer b.mu.Unlock()

	if b.started {
		return fmt.Errorf("cloud bridge already started")
	}

	b.logger.Info("Starting cloud bridge")

	// Start heartbeat loop
	b.wg.Add(1)
	go b.heartbeatLoop()

	// Start telemetry loop
	b.wg.Add(1)
	go b.telemetryLoop()

	// Start command polling loop
	b.wg.Add(1)
	go b.commandPollingLoop()

	b.started = true
	b.logger.Info("Cloud bridge started successfully")

	return nil
}

// Stop stops the cloud bridge
func (b *Bridge) Stop() error {
	b.mu.Lock()
	defer b.mu.Unlock()

	if !b.started {
		return nil
	}

	b.logger.Info("Stopping cloud bridge")
	b.cancel()
	b.wg.Wait()

	b.started = false
	b.logger.Info("Cloud bridge stopped")

	return nil
}

// SendTelemetry sends telemetry data to the cloud
func (b *Bridge) SendTelemetry(telemetry *TelemetryMessage) error {
	if !b.isConnected() {
		return fmt.Errorf("not connected to cloud")
	}

	// Add vehicle ID and timestamp
	telemetry.VehicleID = b.vehicleID
	telemetry.Timestamp = time.Now()

	// Serialize to JSON
	data, err := json.Marshal(telemetry)
	if err != nil {
		return fmt.Errorf("failed to marshal telemetry: %w", err)
	}

	// Send to cloud
	url := fmt.Sprintf("%s/api/v1/vehicles/%s/telemetry", b.config.CloudURL, b.vehicleID)
	
	req, err := http.NewRequestWithContext(b.ctx, "POST", url, bytes.NewBuffer(data))
	if err != nil {
		return fmt.Errorf("failed to create request: %w", err)
	}

	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Authorization", "Bearer "+b.config.APIKey)

	resp, err := b.httpClient.Do(req)
	if err != nil {
		b.setConnected(false)
		return fmt.Errorf("failed to send telemetry: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		return fmt.Errorf("telemetry send failed with status: %d", resp.StatusCode)
	}

	b.logger.Debug("Telemetry sent successfully")
	return nil
}

// PollCommands polls for commands from the cloud
func (b *Bridge) PollCommands() ([]VehicleCommand, error) {
	if !b.isConnected() {
		return nil, fmt.Errorf("not connected to cloud")
	}

	url := fmt.Sprintf("%s/api/v1/vehicles/%s/commands", b.config.CloudURL, b.vehicleID)
	
	req, err := http.NewRequestWithContext(b.ctx, "GET", url, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to create request: %w", err)
	}

	req.Header.Set("Authorization", "Bearer "+b.config.APIKey)

	resp, err := b.httpClient.Do(req)
	if err != nil {
		b.setConnected(false)
		return nil, fmt.Errorf("failed to poll commands: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("command polling failed with status: %d", resp.StatusCode)
	}

	var commands []VehicleCommand
	if err := json.NewDecoder(resp.Body).Decode(&commands); err != nil {
		return nil, fmt.Errorf("failed to decode commands: %w", err)
	}

	return commands, nil
}

// SendCommandResponse sends a command response to the cloud
func (b *Bridge) SendCommandResponse(commandID string, success bool, message string) error {
	if !b.isConnected() {
		return fmt.Errorf("not connected to cloud")
	}

	response := map[string]interface{}{
		"command_id": commandID,
		"vehicle_id": b.vehicleID,
		"success":    success,
		"message":    message,
		"timestamp":  time.Now(),
	}

	data, err := json.Marshal(response)
	if err != nil {
		return fmt.Errorf("failed to marshal response: %w", err)
	}

	url := fmt.Sprintf("%s/api/v1/vehicles/%s/commands/%s/response", b.config.CloudURL, b.vehicleID, commandID)
	
	req, err := http.NewRequestWithContext(b.ctx, "POST", url, bytes.NewBuffer(data))
	if err != nil {
		return fmt.Errorf("failed to create request: %w", err)
	}

	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Authorization", "Bearer "+b.config.APIKey)

	resp, err := b.httpClient.Do(req)
	if err != nil {
		b.setConnected(false)
		return fmt.Errorf("failed to send response: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		return fmt.Errorf("response send failed with status: %d", resp.StatusCode)
	}

	b.logger.Debug("Command response sent successfully", zap.String("command_id", commandID))
	return nil
}

// heartbeatLoop sends periodic heartbeats to the cloud
func (b *Bridge) heartbeatLoop() {
	defer b.wg.Done()

	ticker := time.NewTicker(time.Duration(b.config.HeartbeatInterval) * time.Second)
	defer ticker.Stop()

	b.logger.Info("Heartbeat loop started")

	for {
		select {
		case <-b.ctx.Done():
			b.logger.Info("Heartbeat loop stopped")
			return
		case <-ticker.C:
			if err := b.sendHeartbeat(); err != nil {
				b.logger.Error("Heartbeat failed", zap.Error(err))
				b.setConnected(false)
			} else {
				b.setConnected(true)
				b.lastHeartbeat = time.Now()
			}
		}
	}
}

// telemetryLoop sends periodic telemetry data to the cloud
func (b *Bridge) telemetryLoop() {
	defer b.wg.Done()

	ticker := time.NewTicker(time.Duration(b.config.TelemetryInterval) * time.Second)
	defer ticker.Stop()

	b.logger.Info("Telemetry loop started")

	for {
		select {
		case <-b.ctx.Done():
			b.logger.Info("Telemetry loop stopped")
			return
		case <-ticker.C:
			if b.isConnected() {
				// Create mock telemetry data
				telemetry := b.createMockTelemetry()
				if err := b.SendTelemetry(telemetry); err != nil {
					b.logger.Error("Telemetry send failed", zap.Error(err))
				}
			}
		}
	}
}

// commandPollingLoop polls for commands from the cloud
func (b *Bridge) commandPollingLoop() {
	defer b.wg.Done()

	ticker := time.NewTicker(5 * time.Second) // Poll every 5 seconds
	defer ticker.Stop()

	b.logger.Info("Command polling loop started")

	for {
		select {
		case <-b.ctx.Done():
			b.logger.Info("Command polling loop stopped")
			return
		case <-ticker.C:
			if b.isConnected() {
				commands, err := b.PollCommands()
				if err != nil {
					b.logger.Error("Command polling failed", zap.Error(err))
					continue
				}

				// Process commands
				for _, cmd := range commands {
					if err := b.processCommand(cmd); err != nil {
						b.logger.Error("Command processing failed", 
							zap.String("command_id", cmd.CommandID),
							zap.Error(err))
					}
				}
			}
		}
	}
}

// sendHeartbeat sends a heartbeat to the cloud
func (b *Bridge) sendHeartbeat() error {
	heartbeat := map[string]interface{}{
		"vehicle_id": b.vehicleID,
		"timestamp":  time.Now(),
		"status":     "online",
	}

	data, err := json.Marshal(heartbeat)
	if err != nil {
		return fmt.Errorf("failed to marshal heartbeat: %w", err)
	}

	url := fmt.Sprintf("%s/api/v1/vehicles/%s/heartbeat", b.config.CloudURL, b.vehicleID)
	
	req, err := http.NewRequestWithContext(b.ctx, "POST", url, bytes.NewBuffer(data))
	if err != nil {
		return fmt.Errorf("failed to create request: %w", err)
	}

	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Authorization", "Bearer "+b.config.APIKey)

	resp, err := b.httpClient.Do(req)
	if err != nil {
		return fmt.Errorf("failed to send heartbeat: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		return fmt.Errorf("heartbeat failed with status: %d", resp.StatusCode)
	}

	return nil
}

// processCommand processes a command from the cloud
func (b *Bridge) processCommand(cmd VehicleCommand) error {
	b.logger.Info("Processing command",
		zap.String("command_id", cmd.CommandID),
		zap.String("command_type", cmd.CommandType))

	// In a real implementation, this would:
	// 1. Validate the command
	// 2. Execute the command
	// 3. Send response back to cloud

	// Mock command processing
	success := true
	message := "Command executed successfully"

	// Send response
	if err := b.SendCommandResponse(cmd.CommandID, success, message); err != nil {
		return fmt.Errorf("failed to send command response: %w", err)
	}

	return nil
}

// createMockTelemetry creates mock telemetry data for testing
func (b *Bridge) createMockTelemetry() *TelemetryMessage {
	return &TelemetryMessage{
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
}

// Helper methods

func (b *Bridge) isConnected() bool {
	b.mu.RLock()
	defer b.mu.RUnlock()
	return b.connected
}

func (b *Bridge) setConnected(connected bool) {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.connected = connected
}

// IsStarted returns whether the bridge is started
func (b *Bridge) IsStarted() bool {
	b.mu.RLock()
	defer b.mu.RUnlock()
	return b.started
}

// IsConnected returns whether the bridge is connected to the cloud
func (b *Bridge) IsConnected() bool {
	return b.isConnected()
}

// GetLastHeartbeat returns the time of the last successful heartbeat
func (b *Bridge) GetLastHeartbeat() time.Time {
	b.mu.RLock()
	defer b.mu.RUnlock()
	return b.lastHeartbeat
}
