package control

import (
	"context"
	"fmt"
	"sync"
	"time"

	"go.uber.org/zap"

	"atlasmesh/vehicle-agent/internal/ros2"
)

// Module represents the control module
type Module struct {
	config     *Config
	logger     *zap.Logger
	ros2       *ros2.Node
	ctx        context.Context
	cancel     context.CancelFunc
	wg         sync.WaitGroup
	started    bool
	mu         sync.RWMutex

	// Publishers
	twistPublisher      *ros2.Publisher
	pathPublisher       *ros2.Publisher
	cmdVelPublisher     *ros2.Publisher

	// Subscribers
	pathSubscriber      *ros2.Subscriber
	poseSubscriber      *ros2.Subscriber
	odometrySubscriber  *ros2.Subscriber

	// State
	currentPath         []PathPoint
	currentPose         Pose
	currentOdometry     Odometry
	emergencyStopActive bool
	speedLimit          float64
	operationalMode     string
	lastCommandTime     time.Time
}

// Config represents control module configuration
type Config struct {
	PathTopic           string  `yaml:"path_topic"`
	PoseTopic           string  `yaml:"pose_topic"`
	OdometryTopic       string  `yaml:"odometry_topic"`
	TwistTopic          string  `yaml:"twist_topic"`
	PathOutTopic        string  `yaml:"path_out_topic"`
	CmdVelTopic         string  `yaml:"cmd_vel_topic"`
	ProcessingRate      int     `yaml:"processing_rate_hz"`
	MaxSpeed            float64 `yaml:"max_speed_mps"`
	MaxAcceleration     float64 `yaml:"max_acceleration_mps2"`
	MaxDeceleration     float64 `yaml:"max_deceleration_mps2"`
	MaxAngularVelocity  float64 `yaml:"max_angular_velocity_rad_per_s"`
	LookaheadDistance   float64 `yaml:"lookahead_distance_m"`
	ControlGain         float64 `yaml:"control_gain"`
	EmergencyStopTime   float64 `yaml:"emergency_stop_time_s"`
}

// PathPoint represents a point in the planned path
type PathPoint struct {
	X         float64   `json:"x"`
	Y         float64   `json:"y"`
	Z         float64   `json:"z"`
	Yaw       float64   `json:"yaw"`
	Speed     float64   `json:"speed_mps"`
	Timestamp time.Time `json:"timestamp"`
}

// Pose represents vehicle pose
type Pose struct {
	Position    Position  `json:"position"`
	Orientation Quaternion `json:"orientation"`
	Timestamp   time.Time `json:"timestamp"`
}

type Position struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type Quaternion struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
	W float64 `json:"w"`
}

// Odometry represents vehicle odometry
type Odometry struct {
	Pose                PoseWithCovariance `json:"pose"`
	Twist               TwistWithCovariance `json:"twist"`
	Timestamp           time.Time          `json:"timestamp"`
}

type PoseWithCovariance struct {
	Pose       Pose      `json:"pose"`
	Covariance []float64 `json:"covariance"`
}

type TwistWithCovariance struct {
	Twist      Twist     `json:"twist"`
	Covariance []float64 `json:"covariance"`
}

type Twist struct {
	Linear  Vector3 `json:"linear"`
	Angular Vector3 `json:"angular"`
}

type Vector3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// ControlCommand represents a control command
type ControlCommand struct {
	LinearVelocity  Vector3 `json:"linear_velocity"`
	AngularVelocity Vector3 `json:"angular_velocity"`
	Timestamp       time.Time `json:"timestamp"`
	Source          string    `json:"source"` // AUTONOMOUS, MANUAL, EMERGENCY
}

// New creates a new control module
func New(config *Config, logger *zap.Logger, ros2Node *ros2.Node) (*Module, error) {
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

	module := &Module{
		config:         config,
		logger:         logger,
		ros2:           ros2Node,
		ctx:            ctx,
		cancel:         cancel,
		speedLimit:     config.MaxSpeed,
		operationalMode: "AUTONOMOUS",
	}

	return module, nil
}

// Start starts the control module
func (m *Module) Start(ctx context.Context) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.started {
		return fmt.Errorf("control module already started")
	}

	m.logger.Info("Starting control module")

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
	m.logger.Info("Control module started successfully")

	return nil
}

// Stop stops the control module
func (m *Module) Stop() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if !m.started {
		return nil
	}

	m.logger.Info("Stopping control module")
	m.cancel()
	m.wg.Wait()

	m.started = false
	m.logger.Info("Control module stopped")

	return nil
}

// EmergencyStop executes an emergency stop
func (m *Module) EmergencyStop(reason, source string) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	m.logger.Warn("Emergency stop executed",
		zap.String("reason", reason),
		zap.String("source", source))

	m.emergencyStopActive = true
	m.operationalMode = "EMERGENCY_STOP"

	// Send emergency stop command
	cmd := ControlCommand{
		LinearVelocity: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
		AngularVelocity: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
		Timestamp: time.Now(),
		Source: "EMERGENCY",
	}

	if err := m.publishControlCommand(cmd); err != nil {
		return fmt.Errorf("failed to publish emergency stop command: %w", err)
	}

	return nil
}

// Resume resumes normal operations
func (m *Module) Resume() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	m.logger.Info("Resuming normal operations")

	m.emergencyStopActive = false
	m.operationalMode = "AUTONOMOUS"

	return nil
}

// SetSpeedLimit sets the speed limit
func (m *Module) SetSpeedLimit(speedLimit float64) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if speedLimit < 0 || speedLimit > m.config.MaxSpeed {
		return fmt.Errorf("invalid speed limit: %f", speedLimit)
	}

	m.speedLimit = speedLimit
	m.logger.Info("Speed limit set", zap.Float64("speed_limit", speedLimit))

	return nil
}

// UpdateRoute updates the planned route
func (m *Module) UpdateRoute(route []PathPoint) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	m.currentPath = make([]PathPoint, len(route))
	copy(m.currentPath, route)

	m.logger.Info("Route updated", zap.Int("waypoints", len(route)))

	// Publish updated path
	if m.pathPublisher != nil {
		// Convert to ROS2 message format and publish
		// This would be implemented with actual ROS2 message serialization
	}

	return nil
}

// ExecuteCommand executes a control command
func (m *Module) ExecuteCommand(commandType string, value float64, source string) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.emergencyStopActive {
		return fmt.Errorf("emergency stop active, cannot execute commands")
	}

	var cmd ControlCommand
	cmd.Timestamp = time.Now()
	cmd.Source = source

	switch commandType {
	case "SET_SPEED":
		cmd.LinearVelocity = Vector3{X: value, Y: 0.0, Z: 0.0}
		cmd.AngularVelocity = Vector3{X: 0.0, Y: 0.0, Z: 0.0}
	case "STEER":
		cmd.LinearVelocity = Vector3{X: m.speedLimit, Y: 0.0, Z: 0.0}
		cmd.AngularVelocity = Vector3{X: 0.0, Y: 0.0, Z: value}
	case "BRAKE":
		cmd.LinearVelocity = Vector3{X: -value, Y: 0.0, Z: 0.0}
		cmd.AngularVelocity = Vector3{X: 0.0, Y: 0.0, Z: 0.0}
	case "ACCELERATE":
		cmd.LinearVelocity = Vector3{X: value, Y: 0.0, Z: 0.0}
		cmd.AngularVelocity = Vector3{X: 0.0, Y: 0.0, Z: 0.0}
	default:
		return fmt.Errorf("unknown command type: %s", commandType)
	}

	// Apply speed limit
	if cmd.LinearVelocity.X > m.speedLimit {
		cmd.LinearVelocity.X = m.speedLimit
	}

	// Apply acceleration limits
	if cmd.LinearVelocity.X > m.config.MaxAcceleration {
		cmd.LinearVelocity.X = m.config.MaxAcceleration
	}
	if cmd.LinearVelocity.X < -m.config.MaxDeceleration {
		cmd.LinearVelocity.X = -m.config.MaxDeceleration
	}

	// Apply angular velocity limits
	if cmd.AngularVelocity.Z > m.config.MaxAngularVelocity {
		cmd.AngularVelocity.Z = m.config.MaxAngularVelocity
	}
	if cmd.AngularVelocity.Z < -m.config.MaxAngularVelocity {
		cmd.AngularVelocity.Z = -m.config.MaxAngularVelocity
	}

	if err := m.publishControlCommand(cmd); err != nil {
		return fmt.Errorf("failed to publish control command: %w", err)
	}

	m.lastCommandTime = time.Now()

	m.logger.Debug("Control command executed",
		zap.String("type", commandType),
		zap.Float64("value", value),
		zap.String("source", source))

	return nil
}

// createPublishers creates ROS2 publishers
func (m *Module) createPublishers() error {
	var err error

	// Twist publisher
	m.twistPublisher, err = m.ros2.CreateTwistPublisher(m.config.TwistTopic)
	if err != nil {
		return fmt.Errorf("failed to create twist publisher: %w", err)
	}

	// Path publisher
	m.pathPublisher, err = m.ros2.CreatePathPublisher(m.config.PathOutTopic)
	if err != nil {
		return fmt.Errorf("failed to create path publisher: %w", err)
	}

	// Command velocity publisher
	m.cmdVelPublisher, err = m.ros2.CreateTwistPublisher(m.config.CmdVelTopic)
	if err != nil {
		return fmt.Errorf("failed to create cmd_vel publisher: %w", err)
	}

	return nil
}

// createSubscribers creates ROS2 subscribers
func (m *Module) createSubscribers() error {
	var err error

	// Path subscriber
	m.pathSubscriber, err = m.ros2.CreatePathSubscriber(m.config.PathTopic, m.onPathData)
	if err != nil {
		return fmt.Errorf("failed to create path subscriber: %w", err)
	}

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

	// Start subscribers
	if err := m.pathSubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start path subscriber: %w", err)
	}

	if err := m.poseSubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start pose subscriber: %w", err)
	}

	if err := m.odometrySubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start odometry subscriber: %w", err)
	}

	return nil
}

// processingLoop runs the main control processing loop
func (m *Module) processingLoop() {
	defer m.wg.Done()

	ticker := time.NewTicker(time.Duration(1000/m.config.ProcessingRate) * time.Millisecond)
	defer ticker.Stop()

	m.logger.Info("Control processing loop started")

	for {
		select {
		case <-m.ctx.Done():
			m.logger.Info("Control processing loop stopped")
			return
		case <-ticker.C:
			m.processControlData()
		}
	}
}

// processControlData processes the current control data
func (m *Module) processControlData() {
	// In a real implementation, this would:
	// 1. Check for emergency stop conditions
	// 2. Plan path following
	// 3. Calculate control commands
	// 4. Apply safety limits
	// 5. Publish control commands

	m.mu.Lock()
	defer m.mu.Unlock()

	// Check for emergency stop timeout
	if m.emergencyStopActive {
		if time.Since(m.lastCommandTime) > time.Duration(m.config.EmergencyStopTime)*time.Second {
			m.logger.Warn("Emergency stop timeout, resuming operations")
			m.emergencyStopActive = false
			m.operationalMode = "AUTONOMOUS"
		}
		return
	}

	// Mock control processing - in real implementation, this would be actual control algorithms
	m.mockControlProcessing()
}

// mockControlProcessing provides mock control data for testing
func (m *Module) mockControlProcessing() {
	// Mock path following control
	if len(m.currentPath) > 0 {
		// Simple path following algorithm
		cmd := ControlCommand{
			LinearVelocity: Vector3{X: m.speedLimit, Y: 0.0, Z: 0.0},
			AngularVelocity: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
			Timestamp: time.Now(),
			Source: "AUTONOMOUS",
		}

		// Publish control command
		if err := m.publishControlCommand(cmd); err != nil {
			m.logger.Error("Failed to publish control command", zap.Error(err))
		}
	}
}

// publishControlCommand publishes a control command
func (m *Module) publishControlCommand(cmd ControlCommand) error {
	// Convert to ROS2 message format and publish
	// This would be implemented with actual ROS2 message serialization

	if m.twistPublisher != nil {
		// Publish twist command
		// m.twistPublisher.Publish(twistMsg)
	}

	if m.cmdVelPublisher != nil {
		// Publish cmd_vel command
		// m.cmdVelPublisher.Publish(cmdVelMsg)
	}

	return nil
}

// Sensor data callbacks

func (m *Module) onPathData(path *ros2.Path) {
	m.logger.Debug("Received path data",
		zap.Int("poses_count", len(path.Poses)))

	// Convert ROS2 path to internal format
	m.currentPath = make([]PathPoint, len(path.Poses))
	for i, poseStamped := range path.Poses {
		m.currentPath[i] = PathPoint{
			X: poseStamped.Pose.Position.X,
			Y: poseStamped.Pose.Position.Y,
			Z: poseStamped.Pose.Position.Z,
			// Convert quaternion to yaw
			Yaw: m.quaternionToYaw(poseStamped.Pose.Orientation),
			Speed: m.speedLimit, // Default speed
			Timestamp: time.Now(),
		}
	}

	m.logger.Info("Path updated", zap.Int("waypoints", len(m.currentPath)))
}

func (m *Module) onPoseData(poseStamped *ros2.PoseStamped) {
	m.logger.Debug("Received pose data",
		zap.Float64("x", poseStamped.Pose.Position.X),
		zap.Float64("y", poseStamped.Pose.Position.Y),
		zap.Float64("z", poseStamped.Pose.Position.Z))

	// Update current pose
	m.currentPose = Pose{
		Position: Position{
			X: poseStamped.Pose.Position.X,
			Y: poseStamped.Pose.Position.Y,
			Z: poseStamped.Pose.Position.Z,
		},
		Orientation: Quaternion{
			X: poseStamped.Pose.Orientation.X,
			Y: poseStamped.Pose.Orientation.Y,
			Z: poseStamped.Pose.Orientation.Z,
			W: poseStamped.Pose.Orientation.W,
		},
		Timestamp: time.Now(),
	}
}

func (m *Module) onOdometryData(odometry *ros2.Odometry) {
	m.logger.Debug("Received odometry data",
		zap.Float64("linear_x", odometry.Twist.Twist.Linear.X),
		zap.Float64("angular_z", odometry.Twist.Twist.Angular.Z))

	// Update current odometry
	m.currentOdometry = Odometry{
		Pose: PoseWithCovariance{
			Pose: Pose{
				Position: Position{
					X: odometry.Pose.Pose.Position.X,
					Y: odometry.Pose.Pose.Position.Y,
					Z: odometry.Pose.Pose.Position.Z,
				},
				Orientation: Quaternion{
					X: odometry.Pose.Pose.Orientation.X,
					Y: odometry.Pose.Pose.Orientation.Y,
					Z: odometry.Pose.Pose.Orientation.Z,
					W: odometry.Pose.Pose.Orientation.W,
				},
				Timestamp: time.Now(),
			},
			Covariance: odometry.Pose.Covariance,
		},
		Twist: TwistWithCovariance{
			Twist: Twist{
				Linear: Vector3{
					X: odometry.Twist.Twist.Linear.X,
					Y: odometry.Twist.Twist.Linear.Y,
					Z: odometry.Twist.Twist.Linear.Z,
				},
				Angular: Vector3{
					X: odometry.Twist.Twist.Angular.X,
					Y: odometry.Twist.Twist.Angular.Y,
					Z: odometry.Twist.Twist.Angular.Z,
				},
			},
			Covariance: odometry.Twist.Covariance,
		},
		Timestamp: time.Now(),
	}
}

// Utility functions

// quaternionToYaw converts quaternion to yaw angle
func (m *Module) quaternionToYaw(q ros2.Quaternion) float64 {
	// Convert quaternion to yaw (rotation around Z axis)
	// yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
	yaw := math.Atan2(2*(q.W*q.Z + q.X*q.Y), 1 - 2*(q.Y*q.Y + q.Z*q.Z))
	return yaw * 180.0 / math.Pi // Convert to degrees
}

// yawToQuaternion converts yaw angle to quaternion
func (m *Module) yawToQuaternion(yaw float64) Quaternion {
	// Convert yaw (degrees) to quaternion
	yawRad := yaw * math.Pi / 180.0
	cy := math.Cos(yawRad * 0.5)
	sy := math.Sin(yawRad * 0.5)
	
	return Quaternion{
		X: 0.0,
		Y: 0.0,
		Z: sy,
		W: cy,
	}
}

// calculateDistance calculates distance between two points
func (m *Module) calculateDistance(x1, y1, x2, y2 float64) float64 {
	dx := x2 - x1
	dy := y2 - y1
	return math.Sqrt(dx*dx + dy*dy)
}

// calculateHeading calculates heading between two points
func (m *Module) calculateHeading(x1, y1, x2, y2 float64) float64 {
	dx := x2 - x1
	dy := y2 - y1
	return math.Atan2(dy, dx) * 180.0 / math.Pi
}

// IsStarted returns whether the module is started
func (m *Module) IsStarted() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.started
}

// GetOperationalMode returns the current operational mode
func (m *Module) GetOperationalMode() string {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.operationalMode
}

// IsEmergencyStopActive returns whether emergency stop is active
func (m *Module) IsEmergencyStopActive() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.emergencyStopActive
}
