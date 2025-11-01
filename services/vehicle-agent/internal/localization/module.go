package localization

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"go.uber.org/zap"

	"atlasmesh/vehicle-agent/internal/ros2"
)

// Module represents the localization module
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
	posePublisher       *ros2.Publisher
	odometryPublisher   *ros2.Publisher
	pathPublisher       *ros2.Publisher

	// Subscribers
	gpsSubscriber       *ros2.Subscriber
	imuSubscriber       *ros2.Subscriber
	odometrySubscriber  *ros2.Subscriber
	mapSubscriber       *ros2.Subscriber

	// State
	currentPose         Pose
	currentOdometry     Odometry
	positionAccuracy    float64
	headingAccuracy     float64
	mapMatchingActive   bool
	slamActive          bool
	gpsQuality          string
	lastUpdateTime      time.Time
}

// Config represents localization module configuration
type Config struct {
	GPSTopic            string  `yaml:"gps_topic"`
	IMUTopic            string  `yaml:"imu_topic"`
	OdometryTopic       string  `yaml:"odometry_topic"`
	MapTopic            string  `yaml:"map_topic"`
	PoseTopic           string  `yaml:"pose_topic"`
	OdometryOutTopic    string  `yaml:"odometry_out_topic"`
	PathTopic           string  `yaml:"path_topic"`
	ProcessingRate      int     `yaml:"processing_rate_hz"`
	MinGPSAccuracy      float64 `yaml:"min_gps_accuracy_m"`
	MinSatellites       int     `yaml:"min_satellites"`
	EnableSLAM          bool    `yaml:"enable_slam"`
	EnableMapMatching   bool    `yaml:"enable_map_matching"`
	MapMatchingRadius   float64 `yaml:"map_matching_radius_m"`
	SLAMKeyframeDist    float64 `yaml:"slam_keyframe_distance_m"`
	SLAMKeyframeAngle   float64 `yaml:"slam_keyframe_angle_deg"`
}

// Pose represents vehicle pose
type Pose struct {
	Position    Position  `json:"position"`
	Orientation Quaternion `json:"orientation"`
	Timestamp   time.Time `json:"timestamp"`
	FrameID     string    `json:"frame_id"`
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
	FrameID             string             `json:"frame_id"`
	ChildFrameID        string             `json:"child_frame_id"`
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

// GPSData represents GPS data
type GPSData struct {
	Latitude     float64   `json:"latitude"`
	Longitude    float64   `json:"longitude"`
	Altitude     float64   `json:"altitude"`
	Accuracy     float64   `json:"accuracy_m"`
	Satellites   int       `json:"satellites"`
	HDOP         float64   `json:"hdop"`
	Status       string    `json:"status"`
	Timestamp    time.Time `json:"timestamp"`
}

// IMUData represents IMU data
type IMUData struct {
	Orientation           Quaternion `json:"orientation"`
	AngularVelocity       Vector3    `json:"angular_velocity"`
	LinearAcceleration    Vector3    `json:"linear_acceleration"`
	OrientationCovariance []float64  `json:"orientation_covariance"`
	AngularVelocityCovariance []float64 `json:"angular_velocity_covariance"`
	LinearAccelerationCovariance []float64 `json:"linear_acceleration_covariance"`
	Timestamp             time.Time  `json:"timestamp"`
}

// LocalizationData represents localization summary data
type LocalizationData struct {
	PositionAccuracy    float64   `json:"position_accuracy_m"`
	HeadingAccuracy     float64   `json:"heading_accuracy_deg"`
	MapMatchingActive   bool      `json:"map_matching_active"`
	SLAMActive          bool      `json:"slam_active"`
	GPSQuality          string    `json:"gps_quality"`
	Satellites          int       `json:"satellites"`
	HDOP                float64   `json:"hdop"`
	Timestamp           time.Time `json:"timestamp"`
}

// New creates a new localization module
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
		config: config,
		logger: logger,
		ros2:   ros2Node,
		ctx:    ctx,
		cancel: cancel,
	}

	return module, nil
}

// Start starts the localization module
func (m *Module) Start(ctx context.Context) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.started {
		return fmt.Errorf("localization module already started")
	}

	m.logger.Info("Starting localization module")

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
	m.logger.Info("Localization module started successfully")

	return nil
}

// Stop stops the localization module
func (m *Module) Stop() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if !m.started {
		return nil
	}

	m.logger.Info("Stopping localization module")
	m.cancel()
	m.wg.Wait()

	m.started = false
	m.logger.Info("Localization module stopped")

	return nil
}

// GetCurrentPosition returns the current vehicle position
func (m *Module) GetCurrentPosition() (*Pose, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	if m.currentPose.Timestamp.IsZero() {
		return nil, fmt.Errorf("no position data available")
	}

	// Return a copy
	pose := m.currentPose
	return &pose, nil
}

// GetCurrentOdometry returns the current vehicle odometry
func (m *Module) GetCurrentOdometry() (*Odometry, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	if m.currentOdometry.Timestamp.IsZero() {
		return nil, fmt.Errorf("no odometry data available")
	}

	// Return a copy
	odometry := m.currentOdometry
	return &odometry, nil
}

// GetLocalizationData returns the current localization data
func (m *Module) GetLocalizationData() *LocalizationData {
	m.mu.RLock()
	defer m.mu.RUnlock()

	return &LocalizationData{
		PositionAccuracy:    m.positionAccuracy,
		HeadingAccuracy:     m.headingAccuracy,
		MapMatchingActive:   m.mapMatchingActive,
		SLAMActive:          m.slamActive,
		GPSQuality:          m.gpsQuality,
		Satellites:          0, // Would be updated from GPS data
		HDOP:                0.0, // Would be updated from GPS data
		Timestamp:           m.lastUpdateTime,
	}
}

// createPublishers creates ROS2 publishers
func (m *Module) createPublishers() error {
	var err error

	// Pose publisher
	m.posePublisher, err = m.ros2.CreatePosePublisher(m.config.PoseTopic)
	if err != nil {
		return fmt.Errorf("failed to create pose publisher: %w", err)
	}

	// Odometry publisher
	m.odometryPublisher, err = m.ros2.CreateOdometryPublisher(m.config.OdometryOutTopic)
	if err != nil {
		return fmt.Errorf("failed to create odometry publisher: %w", err)
	}

	// Path publisher
	m.pathPublisher, err = m.ros2.CreatePathPublisher(m.config.PathTopic)
	if err != nil {
		return fmt.Errorf("failed to create path publisher: %w", err)
	}

	return nil
}

// createSubscribers creates ROS2 subscribers
func (m *Module) createSubscribers() error {
	var err error

	// GPS subscriber
	m.gpsSubscriber, err = m.ros2.CreateNavSatFixSubscriber(m.config.GPSTopic, m.onGPSData)
	if err != nil {
		return fmt.Errorf("failed to create GPS subscriber: %w", err)
	}

	// IMU subscriber
	m.imuSubscriber, err = m.ros2.CreateImuSubscriber(m.config.IMUTopic, m.onIMUData)
	if err != nil {
		return fmt.Errorf("failed to create IMU subscriber: %w", err)
	}

	// Odometry subscriber
	m.odometrySubscriber, err = m.ros2.CreateOdometrySubscriber(m.config.OdometryTopic, m.onOdometryData)
	if err != nil {
		return fmt.Errorf("failed to create odometry subscriber: %w", err)
	}

	// Map subscriber
	m.mapSubscriber, err = m.ros2.CreateOccupancyGridSubscriber(m.config.MapTopic, m.onMapData)
	if err != nil {
		return fmt.Errorf("failed to create map subscriber: %w", err)
	}

	// Start subscribers
	if err := m.gpsSubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start GPS subscriber: %w", err)
	}

	if err := m.imuSubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start IMU subscriber: %w", err)
	}

	if err := m.odometrySubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start odometry subscriber: %w", err)
	}

	if err := m.mapSubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start map subscriber: %w", err)
	}

	return nil
}

// processingLoop runs the main localization processing loop
func (m *Module) processingLoop() {
	defer m.wg.Done()

	ticker := time.NewTicker(time.Duration(1000/m.config.ProcessingRate) * time.Millisecond)
	defer ticker.Stop()

	m.logger.Info("Localization processing loop started")

	for {
		select {
		case <-m.ctx.Done():
			m.logger.Info("Localization processing loop stopped")
			return
		case <-ticker.C:
			m.processLocalizationData()
		}
	}
}

// processLocalizationData processes the current localization data
func (m *Module) processLocalizationData() {
	// In a real implementation, this would:
	// 1. Fuse GPS, IMU, and odometry data
	// 2. Run SLAM algorithms if enabled
	// 3. Perform map matching if enabled
	// 4. Update pose and odometry estimates
	// 5. Publish results

	m.mu.Lock()
	defer m.mu.Unlock()

	// Update timestamp
	m.lastUpdateTime = time.Now()

	// Mock processing - in real implementation, this would be actual localization algorithms
	m.mockLocalizationProcessing()
}

// mockLocalizationProcessing provides mock localization data for testing
func (m *Module) mockLocalizationProcessing() {
	// Mock pose estimation
	m.currentPose = Pose{
		Position: Position{
			X: 0.0,
			Y: 0.0,
			Z: 0.0,
		},
		Orientation: Quaternion{
			X: 0.0,
			Y: 0.0,
			Z: 0.0,
			W: 1.0,
		},
		Timestamp: time.Now(),
		FrameID:   "map",
	}

	// Mock odometry
	m.currentOdometry = Odometry{
		Pose: PoseWithCovariance{
			Pose: m.currentPose,
			Covariance: []float64{
				1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
			},
		},
		Twist: TwistWithCovariance{
			Twist: Twist{
				Linear: Vector3{X: 2.0, Y: 0.0, Z: 0.0},
				Angular: Vector3{X: 0.0, Y: 0.0, Z: 0.0},
			},
			Covariance: []float64{
				0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
			},
		},
		Timestamp:    time.Now(),
		FrameID:      "map",
		ChildFrameID: "base_link",
	}

	// Mock accuracy estimates
	m.positionAccuracy = 1.0
	m.headingAccuracy = 2.0

	// Mock status
	m.mapMatchingActive = m.config.EnableMapMatching
	m.slamActive = m.config.EnableSLAM
	m.gpsQuality = "GOOD"
}

// Sensor data callbacks

func (m *Module) onGPSData(navSatFix *ros2.NavSatFix) {
	m.logger.Debug("Received GPS data",
		zap.Float64("latitude", navSatFix.Latitude),
		zap.Float64("longitude", navSatFix.Longitude),
		zap.Float64("altitude", navSatFix.Altitude),
		zap.Int8("status", navSatFix.Status.Status))

	// In a real implementation, this would:
	// 1. Validate GPS data quality
	// 2. Convert to local coordinates
	// 3. Update position estimate
}

func (m *Module) onIMUData(imu *ros2.Imu) {
	m.logger.Debug("Received IMU data",
		zap.Float64("orientation_x", imu.Orientation.X),
		zap.Float64("orientation_y", imu.Orientation.Y),
		zap.Float64("orientation_z", imu.Orientation.Z),
		zap.Float64("orientation_w", imu.Orientation.W))

	// In a real implementation, this would:
	// 1. Process IMU data
	// 2. Update orientation estimate
	// 3. Integrate angular velocity
}

func (m *Module) onOdometryData(odometry *ros2.Odometry) {
	m.logger.Debug("Received odometry data",
		zap.Float64("position_x", odometry.Pose.Pose.Position.X),
		zap.Float64("position_y", odometry.Pose.Pose.Position.Y),
		zap.Float64("linear_x", odometry.Twist.Twist.Linear.X))

	// In a real implementation, this would:
	// 1. Process wheel odometry
	// 2. Update position estimate
	// 3. Integrate velocity
}

func (m *Module) onMapData(occupancyGrid *ros2.OccupancyGrid) {
	m.logger.Debug("Received map data",
		zap.Uint32("width", occupancyGrid.Info.Width),
		zap.Uint32("height", occupancyGrid.Info.Height),
		zap.Float32("resolution", occupancyGrid.Info.Resolution))

	// In a real implementation, this would:
	// 1. Process map data
	// 2. Update map matching
	// 3. Perform localization against map
}

// Utility functions

// ConvertLatLonToXY converts latitude/longitude to local X/Y coordinates
func (m *Module) ConvertLatLonToXY(lat, lon, refLat, refLon float64) (float64, float64) {
	// Simple flat-earth approximation for small areas
	// In production, use proper map projection (UTM, etc.)
	
	const earthRadius = 6371000.0 // meters
	
	// Convert to radians
	latRad := lat * math.Pi / 180.0
	lonRad := lon * math.Pi / 180.0
	refLatRad := refLat * math.Pi / 180.0
	refLonRad := refLon * math.Pi / 180.0
	
	// Calculate differences
	dLat := latRad - refLatRad
	dLon := lonRad - refLonRad
	
	// Calculate X and Y (meters)
	x := dLon * earthRadius * math.Cos(refLatRad)
	y := dLat * earthRadius
	
	return x, y
}

// ConvertXYToLatLon converts local X/Y coordinates to latitude/longitude
func (m *Module) ConvertXYToLatLon(x, y, refLat, refLon float64) (float64, float64) {
	// Simple flat-earth approximation for small areas
	// In production, use proper map projection (UTM, etc.)
	
	const earthRadius = 6371000.0 // meters
	
	// Convert reference to radians
	refLatRad := refLat * math.Pi / 180.0
	refLonRad := refLon * math.Pi / 180.0
	
	// Calculate differences
	dLat := y / earthRadius
	dLon := x / (earthRadius * math.Cos(refLatRad))
	
	// Calculate latitude and longitude
	lat := (refLatRad + dLat) * 180.0 / math.Pi
	lon := (refLonRad + dLon) * 180.0 / math.Pi
	
	return lat, lon
}

// CalculateDistance calculates distance between two points
func (m *Module) CalculateDistance(x1, y1, x2, y2 float64) float64 {
	dx := x2 - x1
	dy := y2 - y1
	return math.Sqrt(dx*dx + dy*dy)
}

// CalculateHeading calculates heading between two points
func (m *Module) CalculateHeading(x1, y1, x2, y2 float64) float64 {
	dx := x2 - x1
	dy := y2 - y1
	return math.Atan2(dy, dx) * 180.0 / math.Pi
}

// Publish localization results
func (m *Module) publishResults() {
	// Publish pose
	if m.posePublisher != nil {
		// Convert to ROS2 message format and publish
		// This would be implemented with actual ROS2 message serialization
	}

	// Publish odometry
	if m.odometryPublisher != nil {
		// Convert to ROS2 message format and publish
	}

	// Publish path (if available)
	if m.pathPublisher != nil {
		// Convert to ROS2 message format and publish
	}
}

// IsStarted returns whether the module is started
func (m *Module) IsStarted() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.started
}
