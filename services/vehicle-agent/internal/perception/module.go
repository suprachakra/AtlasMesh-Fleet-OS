package perception

import (
	"context"
	"fmt"
	"sync"
	"time"

	"go.uber.org/zap"

	"atlasmesh/vehicle-agent/internal/ros2"
)

// Module represents the perception module
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
	objectPublisher     *ros2.Publisher
	obstaclePublisher   *ros2.Publisher
	lanesPublisher      *ros2.Publisher
	trafficLightPublisher *ros2.Publisher

	// Subscribers
	lidarSubscriber     *ros2.Subscriber
	cameraSubscriber    *ros2.Subscriber
	radarSubscriber     *ros2.Subscriber

	// State
	detectedObjects     []DetectedObject
	detectedObstacles   []DetectedObstacle
	detectedLanes       []DetectedLane
	trafficLightStatus  TrafficLightStatus
	lastUpdateTime      time.Time
}

// Config represents perception module configuration
type Config struct {
	LidarTopic          string  `yaml:"lidar_topic"`
	CameraTopic         string  `yaml:"camera_topic"`
	RadarTopic          string  `yaml:"radar_topic"`
	ObjectTopic         string  `yaml:"object_topic"`
	ObstacleTopic       string  `yaml:"obstacle_topic"`
	LanesTopic          string  `yaml:"lanes_topic"`
	TrafficLightTopic   string  `yaml:"traffic_light_topic"`
	ProcessingRate      int     `yaml:"processing_rate_hz"`
	MaxDetectionRange   float64 `yaml:"max_detection_range_m"`
	MinObjectSize       float64 `yaml:"min_object_size_m"`
	ConfidenceThreshold float64 `yaml:"confidence_threshold"`
	EnableTracking      bool    `yaml:"enable_tracking"`
	TrackingTimeout     float64 `yaml:"tracking_timeout_s"`
}

// DetectedObject represents a detected object
type DetectedObject struct {
	ID           string    `json:"id"`
	Type         string    `json:"type"` // VEHICLE, PEDESTRIAN, CYCLIST, STATIC
	Position     Position  `json:"position"`
	Velocity     Velocity  `json:"velocity"`
	Size         Size      `json:"size"`
	Confidence   float64   `json:"confidence"`
	Timestamp    time.Time `json:"timestamp"`
	TrackingID   string    `json:"tracking_id,omitempty"`
	Age          float64   `json:"age_s"`
}

type Position struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type Velocity struct {
	Vx float64 `json:"vx"`
	Vy float64 `json:"vy"`
	Vz float64 `json:"vz"`
}

type Size struct {
	Length float64 `json:"length"`
	Width  float64 `json:"width"`
	Height float64 `json:"height"`
}

// DetectedObstacle represents a detected obstacle
type DetectedObstacle struct {
	ID          string    `json:"id"`
	Position    Position  `json:"position"`
	Size        Size      `json:"size"`
	Distance    float64   `json:"distance_m"`
	Confidence  float64   `json:"confidence"`
	Timestamp   time.Time `json:"timestamp"`
	Type        string    `json:"type"` // STATIC, DYNAMIC, UNKNOWN
}

// DetectedLane represents a detected lane
type DetectedLane struct {
	ID          string    `json:"id"`
	Type        string    `json:"type"` // SOLID, DASHED, DOUBLE
	Color       string    `json:"color"` // WHITE, YELLOW
	Points      []Position `json:"points"`
	Confidence  float64   `json:"confidence"`
	Timestamp   time.Time `json:"timestamp"`
}

// TrafficLightStatus represents traffic light status
type TrafficLightStatus struct {
	ID          string    `json:"id"`
	Position    Position  `json:"position"`
	State       string    `json:"state"` // RED, YELLOW, GREEN, UNKNOWN
	Confidence  float64   `json:"confidence"`
	Timestamp   time.Time `json:"timestamp"`
}

// PerceptionSummary represents a summary of perception data
type PerceptionSummary struct {
	NumObjects           int                   `json:"num_objects"`
	NumObstacles         int                   `json:"num_obstacles"`
	NumLanes             int                   `json:"num_lanes"`
	NearestObstacle      *DetectedObstacle     `json:"nearest_obstacle,omitempty"`
	TrafficLightStatus   *TrafficLightStatus   `json:"traffic_light_status,omitempty"`
	LaneDetectionActive  bool                  `json:"lane_detection_active"`
	PedestrianDetection  bool                  `json:"pedestrian_detection"`
	Timestamp            time.Time             `json:"timestamp"`
}

// New creates a new perception module
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

// Start starts the perception module
func (m *Module) Start(ctx context.Context) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.started {
		return fmt.Errorf("perception module already started")
	}

	m.logger.Info("Starting perception module")

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
	m.logger.Info("Perception module started successfully")

	return nil
}

// Stop stops the perception module
func (m *Module) Stop() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if !m.started {
		return nil
	}

	m.logger.Info("Stopping perception module")
	m.cancel()
	m.wg.Wait()

	m.started = false
	m.logger.Info("Perception module stopped")

	return nil
}

// GetPerceptionSummary returns the current perception summary
func (m *Module) GetPerceptionSummary() *PerceptionSummary {
	m.mu.RLock()
	defer m.mu.RUnlock()

	summary := &PerceptionSummary{
		NumObjects:          len(m.detectedObjects),
		NumObstacles:        len(m.detectedObstacles),
		NumLanes:            len(m.detectedLanes),
		LaneDetectionActive: len(m.detectedLanes) > 0,
		PedestrianDetection: m.hasPedestrians(),
		Timestamp:           m.lastUpdateTime,
	}

	// Find nearest obstacle
	if len(m.detectedObstacles) > 0 {
		nearest := &m.detectedObstacles[0]
		for i := 1; i < len(m.detectedObstacles); i++ {
			if m.detectedObstacles[i].Distance < nearest.Distance {
				nearest = &m.detectedObstacles[i]
			}
		}
		summary.NearestObstacle = nearest
	}

	// Get traffic light status
	if m.trafficLightStatus.ID != "" {
		summary.TrafficLightStatus = &m.trafficLightStatus
	}

	return summary
}

// GetDetectedObjects returns the current detected objects
func (m *Module) GetDetectedObjects() []DetectedObject {
	m.mu.RLock()
	defer m.mu.RUnlock()

	// Return a copy to avoid race conditions
	objects := make([]DetectedObject, len(m.detectedObjects))
	copy(objects, m.detectedObjects)
	return objects
}

// GetDetectedObstacles returns the current detected obstacles
func (m *Module) GetDetectedObstacles() []DetectedObstacle {
	m.mu.RLock()
	defer m.mu.RUnlock()

	// Return a copy to avoid race conditions
	obstacles := make([]DetectedObstacle, len(m.detectedObstacles))
	copy(obstacles, m.detectedObstacles)
	return obstacles
}

// GetDetectedLanes returns the current detected lanes
func (m *Module) GetDetectedLanes() []DetectedLane {
	m.mu.RLock()
	defer m.mu.RUnlock()

	// Return a copy to avoid race conditions
	lanes := make([]DetectedLane, len(m.detectedLanes))
	copy(lanes, m.detectedLanes)
	return lanes
}

// GetTrafficLightStatus returns the current traffic light status
func (m *Module) GetTrafficLightStatus() *TrafficLightStatus {
	m.mu.RLock()
	defer m.mu.RUnlock()

	if m.trafficLightStatus.ID == "" {
		return nil
	}

	// Return a copy
	status := m.trafficLightStatus
	return &status
}

// createPublishers creates ROS2 publishers
func (m *Module) createPublishers() error {
	var err error

	// Object detection publisher
	m.objectPublisher, err = m.ros2.CreatePublisher(m.config.ObjectTopic, "autoware_msgs/msg/DetectedObjectArray", 1)
	if err != nil {
		return fmt.Errorf("failed to create object publisher: %w", err)
	}

	// Obstacle publisher
	m.obstaclePublisher, err = m.ros2.CreatePublisher(m.config.ObstacleTopic, "autoware_msgs/msg/DetectedObjectArray", 1)
	if err != nil {
		return fmt.Errorf("failed to create obstacle publisher: %w", err)
	}

	// Lanes publisher
	m.lanesPublisher, err = m.ros2.CreatePublisher(m.config.LanesTopic, "autoware_msgs/msg/LaneArray", 1)
	if err != nil {
		return fmt.Errorf("failed to create lanes publisher: %w", err)
	}

	// Traffic light publisher
	m.trafficLightPublisher, err = m.ros2.CreatePublisher(m.config.TrafficLightTopic, "autoware_msgs/msg/TrafficLightResultArray", 1)
	if err != nil {
		return fmt.Errorf("failed to create traffic light publisher: %w", err)
	}

	return nil
}

// createSubscribers creates ROS2 subscribers
func (m *Module) createSubscribers() error {
	var err error

	// LiDAR subscriber
	m.lidarSubscriber, err = m.ros2.CreatePointCloudSubscriber(m.config.LidarTopic, m.onLidarData)
	if err != nil {
		return fmt.Errorf("failed to create LiDAR subscriber: %w", err)
	}

	// Camera subscriber
	m.cameraSubscriber, err = m.ros2.CreateImageSubscriber(m.config.CameraTopic, m.onCameraData)
	if err != nil {
		return fmt.Errorf("failed to create camera subscriber: %w", err)
	}

	// Radar subscriber
	m.radarSubscriber, err = m.ros2.CreateLaserScanSubscriber(m.config.RadarTopic, m.onRadarData)
	if err != nil {
		return fmt.Errorf("failed to create radar subscriber: %w", err)
	}

	// Start subscribers
	if err := m.lidarSubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start LiDAR subscriber: %w", err)
	}

	if err := m.cameraSubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start camera subscriber: %w", err)
	}

	if err := m.radarSubscriber.Start(); err != nil {
		return fmt.Errorf("failed to start radar subscriber: %w", err)
	}

	return nil
}

// processingLoop runs the main perception processing loop
func (m *Module) processingLoop() {
	defer m.wg.Done()

	ticker := time.NewTicker(time.Duration(1000/m.config.ProcessingRate) * time.Millisecond)
	defer ticker.Stop()

	m.logger.Info("Perception processing loop started")

	for {
		select {
		case <-m.ctx.Done():
			m.logger.Info("Perception processing loop stopped")
			return
		case <-ticker.C:
			m.processPerceptionData()
		}
	}
}

// processPerceptionData processes the current perception data
func (m *Module) processPerceptionData() {
	// In a real implementation, this would:
	// 1. Process sensor data
	// 2. Run object detection algorithms
	// 3. Track objects over time
	// 4. Update perception state
	// 5. Publish results

	m.mu.Lock()
	defer m.mu.Unlock()

	// Update timestamp
	m.lastUpdateTime = time.Now()

	// Mock processing - in real implementation, this would be actual perception algorithms
	m.mockPerceptionProcessing()
}

// mockPerceptionProcessing provides mock perception data for testing
func (m *Module) mockPerceptionProcessing() {
	// Mock object detection
	m.detectedObjects = []DetectedObject{
		{
			ID: "obj_1",
			Type: "VEHICLE",
			Position: Position{X: 10.0, Y: 0.0, Z: 0.0},
			Velocity: Velocity{Vx: 2.0, Vy: 0.0, Vz: 0.0},
			Size: Size{Length: 4.5, Width: 2.0, Height: 1.8},
			Confidence: 0.95,
			Timestamp: time.Now(),
		},
	}

	// Mock obstacle detection
	m.detectedObstacles = []DetectedObstacle{
		{
			ID: "obs_1",
			Position: Position{X: 5.0, Y: 1.0, Z: 0.0},
			Size: Size{Length: 1.0, Width: 1.0, Height: 1.0},
			Distance: 5.1,
			Confidence: 0.90,
			Timestamp: time.Now(),
			Type: "STATIC",
		},
	}

	// Mock lane detection
	m.detectedLanes = []DetectedLane{
		{
			ID: "lane_1",
			Type: "DASHED",
			Color: "WHITE",
			Points: []Position{
				{X: 0.0, Y: -1.5, Z: 0.0},
				{X: 20.0, Y: -1.5, Z: 0.0},
			},
			Confidence: 0.85,
			Timestamp: time.Now(),
		},
	}

	// Mock traffic light
	m.trafficLightStatus = TrafficLightStatus{
		ID: "tl_1",
		Position: Position{X: 15.0, Y: 0.0, Z: 3.0},
		State: "GREEN",
		Confidence: 0.92,
		Timestamp: time.Now(),
	}
}

// Sensor data callbacks

func (m *Module) onLidarData(pointCloud *ros2.PointCloud2) {
	m.logger.Debug("Received LiDAR data",
		zap.Uint32("width", pointCloud.Width),
		zap.Uint32("height", pointCloud.Height),
		zap.Int("data_size", len(pointCloud.Data)))

	// In a real implementation, this would:
	// 1. Process point cloud data
	// 2. Extract features
	// 3. Update object detection
}

func (m *Module) onCameraData(image *ros2.Image) {
	m.logger.Debug("Received camera data",
		zap.Uint32("width", image.Width),
		zap.Uint32("height", image.Height),
		zap.String("encoding", image.Encoding),
		zap.Int("data_size", len(image.Data)))

	// In a real implementation, this would:
	// 1. Process image data
	// 2. Run computer vision algorithms
	// 3. Detect objects, lanes, traffic lights
}

func (m *Module) onRadarData(laserScan *ros2.LaserScan) {
	m.logger.Debug("Received radar data",
		zap.Float32("angle_min", laserScan.AngleMin),
		zap.Float32("angle_max", laserScan.AngleMax),
		zap.Int("ranges_count", len(laserScan.Ranges)))

	// In a real implementation, this would:
	// 1. Process radar data
	// 2. Detect moving objects
	// 3. Estimate velocities
}

// Helper methods

func (m *Module) hasPedestrians() bool {
	for _, obj := range m.detectedObjects {
		if obj.Type == "PEDESTRIAN" {
			return true
		}
	}
	return false
}

// Publish perception results
func (m *Module) publishResults() {
	// Publish detected objects
	if m.objectPublisher != nil {
		// Convert to ROS2 message format and publish
		// This would be implemented with actual ROS2 message serialization
	}

	// Publish obstacles
	if m.obstaclePublisher != nil {
		// Convert to ROS2 message format and publish
	}

	// Publish lanes
	if m.lanesPublisher != nil {
		// Convert to ROS2 message format and publish
	}

	// Publish traffic light status
	if m.trafficLightPublisher != nil {
		// Convert to ROS2 message format and publish
	}
}

// IsStarted returns whether the module is started
func (m *Module) IsStarted() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.started
}
