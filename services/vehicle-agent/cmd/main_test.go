package main

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/mock"
	"go.uber.org/zap/zaptest"

	"atlasmesh/vehicle-agent/internal/config"
	"atlasmesh/vehicle-agent/internal/ros2"
)

// MockROS2Node is a mock implementation of the ROS2 node
type MockROS2Node struct {
	mock.Mock
}

func (m *MockROS2Node) Start() error {
	args := m.Called()
	return args.Error(0)
}

func (m *MockROS2Node) Stop() error {
	args := m.Called()
	return args.Error(0)
}

func (m *MockROS2Node) CreatePublisher(topic, msgType string, qos int) (*ros2.Publisher, error) {
	args := m.Called(topic, msgType, qos)
	return args.Get(0).(*ros2.Publisher), args.Error(1)
}

func (m *MockROS2Node) CreateSubscriber(topic, msgType string, qos int, callback interface{}) (*ros2.Subscriber, error) {
	args := m.Called(topic, msgType, qos, callback)
	return args.Get(0).(*ros2.Subscriber), args.Error(1)
}

func (m *MockROS2Node) IsStarted() bool {
	args := m.Called()
	return args.Bool(0)
}

// MockPerceptionModule is a mock implementation of the perception module
type MockPerceptionModule struct {
	mock.Mock
}

func (m *MockPerceptionModule) Start(ctx context.Context) error {
	args := m.Called(ctx)
	return args.Error(0)
}

func (m *MockPerceptionModule) Stop() error {
	args := m.Called()
	return args.Error(0)
}

func (m *MockPerceptionModule) GetPerceptionSummary() interface{} {
	args := m.Called()
	return args.Get(0)
}

func (m *MockPerceptionModule) IsStarted() bool {
	args := m.Called()
	return args.Bool(0)
}

// MockLocalizationModule is a mock implementation of the localization module
type MockLocalizationModule struct {
	mock.Mock
}

func (m *MockLocalizationModule) Start(ctx context.Context) error {
	args := m.Called(ctx)
	return args.Error(0)
}

func (m *MockLocalizationModule) Stop() error {
	args := m.Called()
	return args.Error(0)
}

func (m *MockLocalizationModule) GetCurrentPosition() (interface{}, error) {
	args := m.Called()
	return args.Get(0), args.Error(1)
}

func (m *MockLocalizationModule) IsStarted() bool {
	args := m.Called()
	return args.Bool(0)
}

// MockControlModule is a mock implementation of the control module
type MockControlModule struct {
	mock.Mock
}

func (m *MockControlModule) Start(ctx context.Context) error {
	args := m.Called(ctx)
	return args.Error(0)
}

func (m *MockControlModule) Stop() error {
	args := m.Called()
	return args.Error(0)
}

func (m *MockControlModule) EmergencyStop(reason, source string) error {
	args := m.Called(reason, source)
	return args.Error(0)
}

func (m *MockControlModule) Resume() error {
	args := m.Called()
	return args.Error(0)
}

func (m *MockControlModule) SetSpeedLimit(speedLimit float64) error {
	args := m.Called(speedLimit)
	return args.Error(0)
}

func (m *MockControlModule) UpdateRoute(route interface{}) error {
	args := m.Called(route)
	return args.Error(0)
}

func (m *MockControlModule) ExecuteCommand(commandType string, value float64, source string) error {
	args := m.Called(commandType, value, source)
	return args.Error(0)
}

func (m *MockControlModule) IsStarted() bool {
	args := m.Called()
	return args.Bool(0)
}

// MockSafetyMonitor is a mock implementation of the safety monitor
type MockSafetyMonitor struct {
	mock.Mock
}

func (m *MockSafetyMonitor) Start(ctx context.Context) error {
	args := m.Called(ctx)
	return args.Error(0)
}

func (m *MockSafetyMonitor) Stop() error {
	args := m.Called()
	return args.Error(0)
}

func (m *MockSafetyMonitor) GetActiveAlerts() (interface{}, error) {
	args := m.Called()
	return args.Get(0), args.Error(1)
}

func (m *MockSafetyMonitor) IsStarted() bool {
	args := m.Called()
	return args.Bool(0)
}

// MockCloudBridge is a mock implementation of the cloud bridge
type MockCloudBridge struct {
	mock.Mock
}

func (m *MockCloudBridge) Start(ctx context.Context) error {
	args := m.Called(ctx)
	return args.Error(0)
}

func (m *MockCloudBridge) Stop() error {
	args := m.Called()
	return args.Error(0)
}

func (m *MockCloudBridge) IsStarted() bool {
	args := m.Called()
	return args.Bool(0)
}

func (m *MockCloudBridge) IsConnected() bool {
	args := m.Called()
	return args.Bool(0)
}

// TestVehicleAgentService tests the VehicleAgentService
func TestVehicleAgentService(t *testing.T) {
	logger := zaptest.NewLogger(t)
	
	// Create test configuration
	cfg := &config.Config{
		Server: config.ServerConfig{
			Port: 8080,
		},
		VehicleID: "test-vehicle-001",
		LogLevel:  "debug",
	}

	// Create mock modules
	mockROS2 := &MockROS2Node{}
	mockPerception := &MockPerceptionModule{}
	mockLocalization := &MockLocalizationModule{}
	mockControl := &MockControlModule{}
	mockSafety := &MockSafetyMonitor{}
	mockCloud := &MockCloudBridge{}

	// Create service
	service := &VehicleAgentService{
		config:     cfg,
		logger:     logger,
		ros2:       mockROS2,
		perception: mockPerception,
		localization: mockLocalization,
		control:    mockControl,
		safety:     mockSafety,
		cloud:      mockCloud,
		router:     gin.New(),
	}

	// Setup routes
	service.setupRoutes()

	t.Run("HealthCheck", func(t *testing.T) {
		w := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/health", nil)
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, "healthy", response["status"])
		assert.Equal(t, "vehicle-agent", response["service"])
	})

	t.Run("EmergencyStop", func(t *testing.T) {
		// Setup mock expectations
		mockControl.On("EmergencyStop", "test reason", "MANUAL").Return(nil)

		// Create request
		reqBody := EmergencyStopRequest{
			Reason:    "test reason",
			Source:    "MANUAL",
			Timestamp: time.Now(),
		}
		reqJSON, _ := json.Marshal(reqBody)

		w := httptest.NewRecorder()
		req, _ := http.NewRequest("POST", "/api/v1/vehicle-agent/emergency-stop", bytes.NewBuffer(reqJSON))
		req.Header.Set("Content-Type", "application/json")
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, "emergency_stop_executed", response["status"])

		mockControl.AssertExpectations(t)
	})

	t.Run("Resume", func(t *testing.T) {
		// Setup mock expectations
		mockControl.On("Resume").Return(nil)

		w := httptest.NewRecorder()
		req, _ := http.NewRequest("POST", "/api/v1/vehicle-agent/resume", nil)
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, "resumed", response["status"])

		mockControl.AssertExpectations(t)
	})

	t.Run("SetSpeedLimit", func(t *testing.T) {
		// Setup mock expectations
		mockControl.On("SetSpeedLimit", 3.5).Return(nil)

		// Create request
		reqBody := map[string]interface{}{
			"speed_limit_mps": 3.5,
		}
		reqJSON, _ := json.Marshal(reqBody)

		w := httptest.NewRecorder()
		req, _ := http.NewRequest("POST", "/api/v1/vehicle-agent/set-speed-limit", bytes.NewBuffer(reqJSON))
		req.Header.Set("Content-Type", "application/json")
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, "speed_limit_set", response["status"])
		assert.Equal(t, 3.5, response["speed_limit"])

		mockControl.AssertExpectations(t)
	})

	t.Run("UpdateRoute", func(t *testing.T) {
		// Setup mock expectations
		mockControl.On("UpdateRoute", mock.AnythingOfType("[]Position")).Return(nil)

		// Create request
		reqBody := map[string]interface{}{
			"route": []map[string]interface{}{
				{"latitude": 25.2048, "longitude": 55.2744, "altitude": 10.0, "accuracy": 1.0},
				{"latitude": 25.2050, "longitude": 55.2746, "altitude": 10.0, "accuracy": 1.0},
			},
		}
		reqJSON, _ := json.Marshal(reqBody)

		w := httptest.NewRecorder()
		req, _ := http.NewRequest("POST", "/api/v1/vehicle-agent/update-route", bytes.NewBuffer(reqJSON))
		req.Header.Set("Content-Type", "application/json")
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, "route_updated", response["status"])
		assert.Equal(t, float64(2), response["waypoints"])

		mockControl.AssertExpectations(t)
	})

	t.Run("ControlCommand", func(t *testing.T) {
		// Setup mock expectations
		mockControl.On("ExecuteCommand", "SET_SPEED", 2.5, "MANUAL").Return(nil)

		// Create request
		reqBody := ControlCommandRequest{
			CommandType: "SET_SPEED",
			Value:       2.5,
			Source:      "MANUAL",
		}
		reqJSON, _ := json.Marshal(reqBody)

		w := httptest.NewRecorder()
		req, _ := http.NewRequest("POST", "/api/v1/vehicle-agent/control-command", bytes.NewBuffer(reqJSON))
		req.Header.Set("Content-Type", "application/json")
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, "command_executed", response["status"])

		mockControl.AssertExpectations(t)
	})

	t.Run("GetStatus", func(t *testing.T) {
		w := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/api/v1/vehicle-agent/status", nil)
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response VehicleState
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, "test-vehicle-001", response.VehicleID)
		assert.Equal(t, "AUTONOMOUS", response.OperationalMode)
	})

	t.Run("GetHealth", func(t *testing.T) {
		w := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/api/v1/vehicle-agent/health", nil)
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Contains(t, response, "overall_health")
		assert.Contains(t, response, "perception")
		assert.Contains(t, response, "localization")
		assert.Contains(t, response, "control")
	})

	t.Run("GetSensorStatus", func(t *testing.T) {
		w := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/api/v1/vehicle-agent/sensors", nil)
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Contains(t, response, "lidar")
		assert.Contains(t, response, "camera")
		assert.Contains(t, response, "radar")
		assert.Contains(t, response, "gnss")
		assert.Contains(t, response, "imu")
	})

	t.Run("GetPosition", func(t *testing.T) {
		// Setup mock expectations
		mockPosition := &Position{
			Latitude:  25.2048,
			Longitude: 55.2744,
			Altitude:  10.0,
			Accuracy:  1.0,
		}
		mockLocalization.On("GetCurrentPosition").Return(mockPosition, nil)

		w := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/api/v1/vehicle-agent/position", nil)
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response Position
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, 25.2048, response.Latitude)
		assert.Equal(t, 55.2744, response.Longitude)

		mockLocalization.AssertExpectations(t)
	})

	t.Run("GetAlerts", func(t *testing.T) {
		// Setup mock expectations
		mockAlerts := []Alert{
			{
				ID:          "alert_1",
				Severity:    "WARNING",
				Type:        "SPEED_VIOLATION",
				Message:     "Speed exceeds limit",
				Timestamp:   time.Now(),
				Acknowledged: false,
			},
		}
		mockSafety.On("GetActiveAlerts").Return(mockAlerts, nil)

		w := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/api/v1/vehicle-agent/alerts", nil)
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Contains(t, response, "alerts")
		assert.Contains(t, response, "timestamp")

		mockSafety.AssertExpectations(t)
	})

	t.Run("Configure", func(t *testing.T) {
		// Create request
		reqBody := map[string]interface{}{
			"max_speed": 4.0,
			"processing_rate": 50,
		}
		reqJSON, _ := json.Marshal(reqBody)

		w := httptest.NewRecorder()
		req, _ := http.NewRequest("POST", "/api/v1/vehicle-agent/configure", bytes.NewBuffer(reqJSON))
		req.Header.Set("Content-Type", "application/json")
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, "configuration_updated", response["status"])
	})

	t.Run("GetConfig", func(t *testing.T) {
		w := httptest.NewRecorder()
		req, _ := http.NewRequest("GET", "/api/v1/vehicle-agent/config", nil)
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, "test-vehicle-001", response["vehicle_id"])
	})

	t.Run("Calibrate", func(t *testing.T) {
		// Create request
		reqBody := map[string]interface{}{
			"sensor_type": "lidar",
		}
		reqJSON, _ := json.Marshal(reqBody)

		w := httptest.NewRecorder()
		req, _ := http.NewRequest("POST", "/api/v1/vehicle-agent/calibrate", bytes.NewBuffer(reqJSON))
		req.Header.Set("Content-Type", "application/json")
		service.router.ServeHTTP(w, req)

		assert.Equal(t, http.StatusOK, w.Code)
		
		var response map[string]interface{}
		err := json.Unmarshal(w.Body.Bytes(), &response)
		assert.NoError(t, err)
		assert.Equal(t, "calibration_started", response["status"])
		assert.Equal(t, "lidar", response["sensor_type"])
	})
}

// TestVehicleState tests the VehicleState structure
func TestVehicleState(t *testing.T) {
	state := VehicleState{
		VehicleID:       "test-vehicle-001",
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

	// Test JSON marshaling
	jsonData, err := json.Marshal(state)
	assert.NoError(t, err)
	assert.NotEmpty(t, jsonData)

	// Test JSON unmarshaling
	var unmarshaledState VehicleState
	err = json.Unmarshal(jsonData, &unmarshaledState)
	assert.NoError(t, err)
	assert.Equal(t, state.VehicleID, unmarshaledState.VehicleID)
	assert.Equal(t, state.OperationalMode, unmarshaledState.OperationalMode)
	assert.Equal(t, state.Position.Latitude, unmarshaledState.Position.Latitude)
}

// TestEmergencyStopRequest tests the EmergencyStopRequest structure
func TestEmergencyStopRequest(t *testing.T) {
	req := EmergencyStopRequest{
		Reason:    "test reason",
		Source:    "MANUAL",
		Timestamp: time.Now(),
	}

	// Test JSON marshaling
	jsonData, err := json.Marshal(req)
	assert.NoError(t, err)
	assert.NotEmpty(t, jsonData)

	// Test JSON unmarshaling
	var unmarshaledReq EmergencyStopRequest
	err = json.Unmarshal(jsonData, &unmarshaledReq)
	assert.NoError(t, err)
	assert.Equal(t, req.Reason, unmarshaledReq.Reason)
	assert.Equal(t, req.Source, unmarshaledReq.Source)
}

// TestControlCommandRequest tests the ControlCommandRequest structure
func TestControlCommandRequest(t *testing.T) {
	req := ControlCommandRequest{
		CommandType: "SET_SPEED",
		Value:       2.5,
		Duration:    10.0,
		Source:      "MANUAL",
	}

	// Test JSON marshaling
	jsonData, err := json.Marshal(req)
	assert.NoError(t, err)
	assert.NotEmpty(t, jsonData)

	// Test JSON unmarshaling
	var unmarshaledReq ControlCommandRequest
	err = json.Unmarshal(jsonData, &unmarshaledReq)
	assert.NoError(t, err)
	assert.Equal(t, req.CommandType, unmarshaledReq.CommandType)
	assert.Equal(t, req.Value, unmarshaledReq.Value)
	assert.Equal(t, req.Source, unmarshaledReq.Source)
}

// Benchmark tests

func BenchmarkVehicleStateJSONMarshal(b *testing.B) {
	state := VehicleState{
		VehicleID:       "test-vehicle-001",
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

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, err := json.Marshal(state)
		if err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkVehicleStateJSONUnmarshal(b *testing.B) {
	state := VehicleState{
		VehicleID:       "test-vehicle-001",
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

	jsonData, _ := json.Marshal(state)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		var unmarshaledState VehicleState
		err := json.Unmarshal(jsonData, &unmarshaledState)
		if err != nil {
			b.Fatal(err)
		}
	}
}
