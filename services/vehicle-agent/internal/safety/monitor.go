package safety

import (
	"context"
	"fmt"
	"sync"
	"time"

	"go.uber.org/zap"

	"atlasmesh/vehicle-agent/internal/ros2"
)

// Monitor represents the safety monitor
type Monitor struct {
	config     *Config
	logger     *zap.Logger
	ros2       *ros2.Node
	ctx        context.Context
	cancel     context.CancelFunc
	wg         sync.WaitGroup
	started    bool
	mu         sync.RWMutex

	// Publishers
	alertPublisher      *ros2.Publisher
	emergencyPublisher  *ros2.Publisher

	// Subscribers
	poseSubscriber      *ros2.Subscriber
	odometrySubscriber  *ros2.Subscriber
	perceptionSubscriber *ros2.Subscriber

	// State
	activeAlerts        []Alert
	safetyViolations    int
	lastViolationTime   time.Time
	safetyMode          string
	oddCompliance       ODDCompliance
	emergencyStopActive bool
	lastUpdateTime      time.Time
}

// Config represents safety monitor configuration
type Config struct {
	PoseTopic           string  `yaml:"pose_topic"`
	OdometryTopic       string  `yaml:"odometry_topic"`
	PerceptionTopic     string  `yaml:"perception_topic"`
	AlertTopic          string  `yaml:"alert_topic"`
	EmergencyTopic      string  `yaml:"emergency_topic"`
	ProcessingRate      int     `yaml:"processing_rate_hz"`
	MaxSpeed            float64 `yaml:"max_speed_mps"`
	MinObstacleDistance float64 `yaml:"min_obstacle_distance_m"`
	MaxAcceleration     float64 `yaml:"max_acceleration_mps2"`
	MaxDeceleration     float64 `yaml:"max_deceleration_mps2"`
	SafetyTimeout       float64 `yaml:"safety_timeout_s"`
	AlertThreshold      int     `yaml:"alert_threshold"`
	CriticalThreshold   int     `yaml:"critical_threshold"`
}

// Alert represents a safety alert
type Alert struct {
	ID          string    `json:"id"`
	Severity    string    `json:"severity"` // INFO, WARNING, ERROR, CRITICAL
	Type        string    `json:"type"`
	Message     string    `json:"message"`
	Timestamp   time.Time `json:"timestamp"`
	Acknowledged bool     `json:"acknowledged"`
	Source      string    `json:"source"`
	Details     map[string]interface{} `json:"details"`
}

// ODDCompliance represents Operational Design Domain compliance
type ODDCompliance struct {
	WithinODD           bool      `json:"within_odd"`
	ODDViolations       []string  `json:"odd_violations"`
	WeatherCompliant    bool      `json:"weather_compliant"`
	LightingCompliant   bool      `json:"lighting_compliant"`
	InfrastructureCompliant bool  `json:"infrastructure_compliant"`
	Timestamp           time.Time `json:"timestamp"`
}

// SafetyStatus represents safety status
type SafetyStatus struct {
	EmergencyStopActive bool      `json:"emergency_stop_active"`
	SafetyViolations    int       `json:"safety_violations"`
	LastViolationTime   time.Time `json:"last_violation_time"`
	SafetyMode          string    `json:"safety_mode"` // NORMAL, CAUTION, WARNING, CRITICAL
	ActiveAlerts        []Alert   `json:"active_alerts"`
	ODDCompliance       ODDCompliance `json:"odd_compliance"`
	Timestamp           time.Time `json:"timestamp"`
}

// New creates a new safety monitor
func New(config *Config, logger *zap.Logger, ros2Node *ros2.Node) (*Monitor, error) {
	if config == nil {
		return nil, fmt.Errorf("config cannot be nil")
	}
	if logger == nil {
		return nil, fmt.Errorf("logger cannot be nil")
	}
	if ros2Node == nil {
		return nil, fmt.Errorf("ros2 node cannot be nil")
	}

	ctx, cancel := context.WithCancel(context.Background())

	monitor := &Monitor{
		config:     config,
		logger:     logger,
		ros2:       ros2Node,
		ctx:        ctx,
		cancel:     cancel,
		safetyMode: "NORMAL",
		oddCompliance: ODDCompliance{
			WithinODD: true,
			WeatherCompliant: true,
			LightingCompliant: true,
			InfrastructureCompliant: true,
		},
	}

	return monitor, nil
}

// Start starts the safety monitor
func (m *Monitor) Start(ctx context.Context) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.started {
		return fmt.Errorf("safety monitor already started")
	}

	m.logger.Info("Starting safety monitor")

	// Create publishers
	if err := m.createPublishers(); err != nil {
		return fmt.Errorf("failed to create publishers: %w", err)
	}

	// Create subscribers
	if err := m.createSubscribers(); err != nil {
		return fmt.Errorf("failed to create subscribers: %w", err)
	}

	// Start processing loop
	m.wg.Add(1)
	go m.processingLoop()

	m.started = true
	m.logger.Info("Safety monitor started successfully")

	return nil
}

// Stop stops the safety monitor
func (m *Monitor) Stop() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if !m.started {
		return nil
	}

	m.logger.Info("Stopping safety monitor")
	m.cancel()
	m.wg.Wait()

	m.started = false
	m.logger.Info("Safety monitor stopped")

	return nil
}

// GetActiveAlerts returns the current active alerts
func (m *Monitor) GetActiveAlerts() ([]Alert, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	// Return a copy to avoid race conditions
	alerts := make([]Alert, len(m.activeAlerts))
	copy(alerts, m.activeAlerts)
	return alerts, nil
}

// GetSafetyStatus returns the current safety status
func (m *Monitor) GetSafetyStatus() *SafetyStatus {
	m.mu.RLock()
	defer m.mu.RUnlock()

	// Return a copy to avoid race conditions
	alerts := make([]Alert, len(m.activeAlerts))
	copy(alerts, m.activeAlerts)

	return &SafetyStatus{
		EmergencyStopActive: m.emergencyStopActive,
		SafetyViolations:    m.safetyViolations,
		LastViolationTime:   m.lastViolationTime,
		SafetyMode:          m.safetyMode,
		ActiveAlerts:        alerts,
		ODDCompliance:       m.oddCompliance,
		Timestamp:           m.lastUpdateTime,
	}
}

// AcknowledgeAlert acknowledges an alert
func (m *Monitor) AcknowledgeAlert(alertID string) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	for i, alert := range m.activeAlerts {
		if alert.ID == alertID {
			m.activeAlerts[i].Acknowledged = true
			m.logger.Info("Alert acknowledged", zap.String("alert_id", alertID))
			return nil
		}
	}

	return fmt.Errorf("alert not found: %s", alertID)
}

// createPublishers creates ROS2 publishers
func (m *Monitor) createPublishers() error {
	var err error

	// Alert publisher
	m.alertPublisher, err = m.ros2.CreatePublisher(m.config.AlertTopic, "std_msgs/msg/String", 1)
	if err != nil {
		return fmt.Errorf("failed to create alert publisher: %w", err)
	}

	// Emergency publisher
	m.emergencyPublisher, err = m.ros2.CreatePublisher(m.config.EmergencyTopic, "std_msgs/msg/Bool", 1)
	if err != nil {
		return fmt.Errorf("failed to create emergency publisher: %w", err)
	}

	return nil
}

// createSubscribers creates ROS2 subscribers
func (m *Monitor) createSubscribers() error {
	var err error

	// Pose subscriber
	m.poseSubscriber, err = m.ros2.CreatePoseSubscriber(m.config.PoseTopic, m.onPoseData)
	if err != nil {
		return fmt.Errorf("failed to create pose subscriber: %w", err)
	}

	// Odometry subscriber
	m.odometrySubscriber, err = m.ros2.CreateOdometrySubscriber(m.config.OdometryTopic, m.onOdometryData)
	if err != nil {
		return fmt.Errorf("failed to create odometry subscriber: %w", err)
	}

	// Perception subscriber
	m.perceptionSubscriber, err = m.ros2.CreateSubscriber(m.config.PerceptionTopic, "autoware_msgs/msg/DetectedObjectArray", 1, m.onPerceptionData)
	if err != nil {
		return fmt.Errorf("failed to create perception subscriber: %w", err)
	}

	// Start subscribers
	if err := m.poseSubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start pose subscriber: %w", err)
	}

	if err := m.odometrySubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start odometry subscriber: %w", err)
	}

	if err := m.perceptionSubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start perception subscriber: %w", err)
	}

	return nil
}

// processingLoop runs the main safety processing loop
func (m *Monitor) processingLoop() {
	defer m.wg.Done()

	ticker := time.NewTicker(time.Duration(1000/m.config.ProcessingRate) * time.Millisecond)
	defer ticker.Stop()

	m.logger.Info("Safety processing loop started")

	for {
		select {
		case <-m.ctx.Done():
			m.logger.Info("Safety processing loop stopped")
			return
		case <-ticker.C:
			m.processSafetyData()
		}
	}
}

// processSafetyData processes the current safety data
func (m *Monitor) processSafetyData() {
	// In a real implementation, this would:
	// 1. Check for safety violations
	// 2. Monitor ODD compliance
	// 3. Detect emergency conditions
	// 4. Update safety status
	// 5. Publish alerts and emergency signals

	m.mu.Lock()
	defer m.mu.Unlock()

	// Update timestamp
	m.lastUpdateTime = time.Now()

	// Mock safety processing - in real implementation, this would be actual safety algorithms
	m.mockSafetyProcessing()
}

// mockSafetyProcessing provides mock safety data for testing
func (m *Monitor) mockSafetyProcessing() {
	// Mock safety checks
	// In a real implementation, this would check:
	// - Speed limits
	// - Obstacle distances
	// - Acceleration limits
	// - ODD compliance
	// - System health

	// Mock ODD compliance check
	m.oddCompliance.WithinODD = true
	m.oddCompliance.WeatherCompliant = true
	m.oddCompliance.LightingCompliant = true
	m.oddCompliance.InfrastructureCompliant = true
	m.oddCompliance.Timestamp = time.Now()

	// Mock safety mode
	m.safetyMode = "NORMAL"
}

// checkSpeedViolation checks for speed violations
func (m *Monitor) checkSpeedViolation(speed float64) bool {
	if speed > m.config.MaxSpeed {
		m.createAlert("SPEED_VIOLATION", "WARNING", 
			fmt.Sprintf("Speed exceeds limit: %.2f > %.2f m/s", speed, m.config.MaxSpeed),
			map[string]interface{}{
				"current_speed": speed,
				"speed_limit": m.config.MaxSpeed,
			})
		return true
	}
	return false
}

// checkObstacleDistance checks for obstacle distance violations
func (m *Monitor) checkObstacleDistance(distance float64) bool {
	if distance < m.config.MinObstacleDistance {
		m.createAlert("OBSTACLE_TOO_CLOSE", "CRITICAL",
			fmt.Sprintf("Obstacle too close: %.2f < %.2f m", distance, m.config.MinObstacleDistance),
			map[string]interface{}{
				"obstacle_distance": distance,
				"min_distance": m.config.MinObstacleDistance,
			})
		return true
	}
	return false
}

// checkAccelerationViolation checks for acceleration violations
func (m *Monitor) checkAccelerationViolation(acceleration float64) bool {
	if acceleration > m.config.MaxAcceleration {
		m.createAlert("ACCELERATION_VIOLATION", "WARNING",
			fmt.Sprintf("Acceleration exceeds limit: %.2f > %.2f m/s²", acceleration, m.config.MaxAcceleration),
			map[string]interface{}{
				"current_acceleration": acceleration,
				"max_acceleration": m.config.MaxAcceleration,
			})
		return true
	}
	if acceleration < -m.config.MaxDeceleration {
		m.createAlert("DECELERATION_VIOLATION", "WARNING",
			fmt.Sprintf("Deceleration exceeds limit: %.2f < %.2f m/s²", acceleration, -m.config.MaxDeceleration),
			map[string]interface{}{
				"current_deceleration": acceleration,
				"max_deceleration": m.config.MaxDeceleration,
			})
		return true
	}
	return false
}

// createAlert creates a new safety alert
func (m *Monitor) createAlert(alertType, severity, message string, details map[string]interface{}) {
	alert := Alert{
		ID:          fmt.Sprintf("alert_%d", time.Now().UnixNano()),
		Severity:    severity,
		Type:        alertType,
		Message:     message,
		Timestamp:   time.Now(),
		Acknowledged: false,
		Source:      "SAFETY_MONITOR",
		Details:     details,
	}

	m.activeAlerts = append(m.activeAlerts, alert)
	m.safetyViolations++
	m.lastViolationTime = time.Now()

	// Update safety mode based on severity
	switch severity {
	case "CRITICAL":
		m.safetyMode = "CRITICAL"
		m.emergencyStopActive = true
	case "ERROR":
		if m.safetyMode != "CRITICAL" {
			m.safetyMode = "WARNING"
		}
	case "WARNING":
		if m.safetyMode == "NORMAL" {
			m.safetyMode = "CAUTION"
		}
	}

	m.logger.Warn("Safety alert created",
		zap.String("type", alertType),
		zap.String("severity", severity),
		zap.String("message", message))

	// Publish alert
	if m.alertPublisher != nil {
		// Convert to ROS2 message format and publish
		// This would be implemented with actual ROS2 message serialization
	}

	// Publish emergency signal if critical
	if severity == "CRITICAL" && m.emergencyPublisher != nil {
		// Publish emergency signal
		// This would be implemented with actual ROS2 message serialization
	}
}

// Sensor data callbacks

func (m *Monitor) onPoseData(poseStamped *ros2.PoseStamped) {
	m.logger.Debug("Received pose data for safety monitoring",
		zap.Float64("x", poseStamped.Pose.Position.X),
		zap.Float64("y", poseStamped.Pose.Position.Y),
		zap.Float64("z", poseStamped.Pose.Position.Z))

	// In a real implementation, this would:
	// 1. Check position against ODD boundaries
	// 2. Validate pose accuracy
	// 3. Check for localization failures
}

func (m *Monitor) onOdometryData(odometry *ros2.Odometry) {
	m.logger.Debug("Received odometry data for safety monitoring",
		zap.Float64("linear_x", odometry.Twist.Twist.Linear.X),
		zap.Float64("angular_z", odometry.Twist.Twist.Angular.Z))

	// Check for speed violations
	speed := math.Abs(odometry.Twist.Twist.Linear.X)
	if m.checkSpeedViolation(speed) {
		return
	}

	// Check for acceleration violations
	// In a real implementation, this would calculate acceleration from velocity changes
	acceleration := 0.0 // Mock value
	if m.checkAccelerationViolation(acceleration) {
		return
	}
}

func (m *Monitor) onPerceptionData(data interface{}) {
	m.logger.Debug("Received perception data for safety monitoring")

	// In a real implementation, this would:
	// 1. Check obstacle distances
	// 2. Validate object detection
	// 3. Check for perception failures
	// 4. Monitor tracking quality

	// Mock obstacle distance check
	obstacleDistance := 10.0 // Mock value
	if m.checkObstacleDistance(obstacleDistance) {
		return
	}
}

// Utility functions

// clearOldAlerts removes old acknowledged alerts
func (m *Monitor) clearOldAlerts() {
	cutoff := time.Now().Add(-time.Duration(m.config.SafetyTimeout) * time.Second)
	
	var activeAlerts []Alert
	for _, alert := range m.activeAlerts {
		if !alert.Acknowledged || alert.Timestamp.After(cutoff) {
			activeAlerts = append(activeAlerts, alert)
		}
	}
	
	m.activeAlerts = activeAlerts
}

// updateSafetyMode updates the safety mode based on current conditions
func (m *Monitor) updateSafetyMode() {
	criticalCount := 0
	errorCount := 0
	warningCount := 0

	for _, alert := range m.activeAlerts {
		if !alert.Acknowledged {
			switch alert.Severity {
			case "CRITICAL":
				criticalCount++
			case "ERROR":
				errorCount++
			case "WARNING":
				warningCount++
			}
		}
	}

	if criticalCount > 0 {
		m.safetyMode = "CRITICAL"
	} else if errorCount >= m.config.CriticalThreshold {
		m.safetyMode = "WARNING"
	} else if warningCount >= m.config.AlertThreshold {
		m.safetyMode = "CAUTION"
	} else {
		m.safetyMode = "NORMAL"
	}
}

// IsStarted returns whether the monitor is started
func (m *Monitor) IsStarted() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.started
}

// GetSafetyMode returns the current safety mode
func (m *Monitor) GetSafetyMode() string {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.safetyMode
}

// IsEmergencyStopActive returns whether emergency stop is active
func (m *Monitor) IsEmergencyStopActive() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.emergencyStopActive
}
