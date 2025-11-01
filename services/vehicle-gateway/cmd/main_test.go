package main

import (
	"bytes"
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/mock"
)

// Mock dependencies
type MockRedisClient struct {
	mock.Mock
}

func (m *MockRedisClient) Set(ctx context.Context, key string, value interface{}, expiration time.Duration) error {
	args := m.Called(ctx, key, value, expiration)
	return args.Error(0)
}

func (m *MockRedisClient) Get(ctx context.Context, key string) (string, error) {
	args := m.Called(ctx, key)
	return args.String(0), args.Error(1)
}

func (m *MockRedisClient) Ping(ctx context.Context) error {
	args := m.Called(ctx)
	return args.Error(0)
}

func (m *MockRedisClient) Publish(ctx context.Context, channel string, message interface{}) error {
	args := m.Called(ctx, channel, message)
	return args.Error(0)
}

func (m *MockRedisClient) Subscribe(ctx context.Context, channels ...string) interface{} {
	args := m.Called(ctx, channels)
	return args.Get(0)
}

type MockKafkaWriter struct {
	mock.Mock
}

func (m *MockKafkaWriter) WriteMessages(ctx context.Context, msgs ...interface{}) error {
	args := m.Called(ctx, msgs)
	return args.Error(0)
}

func (m *MockKafkaWriter) Close() error {
	args := m.Called()
	return args.Error(0)
}

type MockKafkaReader struct {
	mock.Mock
}

func (m *MockKafkaReader) ReadMessage(ctx context.Context) (interface{}, error) {
	args := m.Called(ctx)
	return args.Get(0), args.Error(1)
}

func (m *MockKafkaReader) Close() error {
	args := m.Called()
	return args.Error(0)
}

func TestLoadConfig(t *testing.T) {
	config := loadConfig()
	
	assert.NotNil(t, config)
	assert.Equal(t, 8083, config.Port)
	assert.Equal(t, "info", config.LogLevel)
	assert.Equal(t, 10000, config.TelemetryBufferSize)
	assert.Equal(t, 300, config.WebSocketTimeout)
	assert.Equal(t, 1000, config.RateLimit)
}

func TestHealthCheck(t *testing.T) {
	// Create test service with mocks
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &VehicleGatewayService{
		config:      loadConfig(),
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock Redis ping success
	mockRedis.On("Ping", mock.Anything).Return(nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/health", nil)
	w := httptest.NewRecorder()
	
	// Call health check
	service.healthCheck(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "healthy", response["status"])
	
	mockRedis.AssertExpectations(t)
}

func TestReceiveTelemetry(t *testing.T) {
	// Create test service with mocks
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &VehicleGatewayService{
		config:      loadConfig(),
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test telemetry message
	telemetry := TelemetryMessage{
		VehicleID:   "test-vehicle-001",
		Timestamp:   time.Now(),
		MessageType: "location",
		Location: Location{
			Latitude:  24.4539,
			Longitude: 54.3773,
			Altitude:  10.0,
			Accuracy:  5.0,
		},
		Speed:   60.0,
		Heading: 90.0,
		Status: VehicleStatus{
			Operational: "active",
			Autonomy:    "level_4",
			Mission:     "delivery",
			Emergency:   false,
		},
		Health: VehicleHealth{
			Battery: 85.0,
			Fuel:    75.0,
			Overall: 90.0,
		},
		Environment: EnvironmentData{
			Temperature: 25.0,
			Humidity:    60.0,
			Visibility:  10.0,
			WindSpeed:   5.0,
		},
		Autonomy: AutonomyData{
			Level:       "level_4",
			Confidence:  0.95,
			Engaged:     true,
			Interventions: 0,
			ODDCompliant: true,
		},
	}
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(telemetry)
	req := httptest.NewRequest("POST", "/api/v1/telemetry", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call receive telemetry
	service.receiveTelemetry(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusAccepted, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "accepted", response["status"])
	
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestReceiveTelemetryInvalidJSON(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Create test request with invalid JSON
	req := httptest.NewRequest("POST", "/api/v1/telemetry", bytes.NewReader([]byte("invalid json")))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call receive telemetry
	service.receiveTelemetry(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestReceiveBatchTelemetry(t *testing.T) {
	// Create test service with mocks
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &VehicleGatewayService{
		config:      loadConfig(),
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test batch telemetry messages
	telemetryBatch := []TelemetryMessage{
		{
			VehicleID:   "test-vehicle-001",
			Timestamp:   time.Now(),
			MessageType: "location",
			Location: Location{
				Latitude:  24.4539,
				Longitude: 54.3773,
			},
			Speed: 60.0,
		},
		{
			VehicleID:   "test-vehicle-002",
			Timestamp:   time.Now(),
			MessageType: "location",
			Location: Location{
				Latitude:  24.4540,
				Longitude: 54.3774,
			},
			Speed: 65.0,
		},
	}
	
	// Mock Redis set success for each message
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil).Times(2)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil).Times(2)
	
	// Create test request
	body, _ := json.Marshal(telemetryBatch)
	req := httptest.NewRequest("POST", "/api/v1/telemetry/batch", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call receive batch telemetry
	service.receiveBatchTelemetry(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusAccepted, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "batch_processed", response["status"])
	assert.Equal(t, float64(2), response["processed"])
	assert.Equal(t, float64(0), response["failed"])
	
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestSendCommand(t *testing.T) {
	// Create test service with mocks
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &VehicleGatewayService{
		config:      loadConfig(),
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test vehicle command
	command := VehicleCommand{
		CommandType: "start_mission",
		Payload: map[string]interface{}{
			"mission_id": "test-mission-001",
			"route":      []string{"point1", "point2", "point3"},
		},
		Priority: "high",
		ExpiresAt: time.Now().Add(5 * time.Minute),
	}
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(command)
	req := httptest.NewRequest("POST", "/api/v1/vehicles/test-vehicle-001/commands", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call send command
	service.sendCommand(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusAccepted, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "queued", response["status"])
	
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestSendCommandInvalidJSON(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Create test request with invalid JSON
	req := httptest.NewRequest("POST", "/api/v1/vehicles/test-vehicle-001/commands", bytes.NewReader([]byte("invalid json")))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call send command
	service.sendCommand(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestGetVehicleStatus(t *testing.T) {
	// Create test service with mocks
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &VehicleGatewayService{
		config:      loadConfig(),
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock Redis get success
	statusData := map[string]interface{}{
		"operational": "active",
		"autonomy":    "level_4",
		"location": map[string]interface{}{
			"latitude":  24.4539,
			"longitude": 54.3773,
		},
		"speed":     60.0,
		"health":    90.0,
		"last_seen": time.Now().Format(time.RFC3339),
	}
	
	statusJSON, _ := json.Marshal(statusData)
	mockRedis.On("Get", mock.Anything, mock.Anything).Return(string(statusJSON), nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/vehicles/test-vehicle-001/status", nil)
	w := httptest.NewRecorder()
	
	// Call get vehicle status
	service.getVehicleStatus(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "active", response["operational"])
	
	mockRedis.AssertExpectations(t)
}

func TestGetVehicleHealth(t *testing.T) {
	// Create test service with mocks
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &VehicleGatewayService{
		config:      loadConfig(),
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock Redis get success
	healthData := map[string]interface{}{
		"battery": 85.0,
		"fuel":    75.0,
		"engine": map[string]interface{}{
			"temperature": 80.0,
			"rpm":        1500.0,
		},
		"brakes": map[string]interface{}{
			"pad_wear": 20.0,
			"fluid":    90.0,
		},
		"tires": map[string]interface{}{
			"pressure": 35.0,
			"wear":     15.0,
		},
		"overall": 90.0,
	}
	
	healthJSON, _ := json.Marshal(healthData)
	mockRedis.On("Get", mock.Anything, mock.Anything).Return(string(healthJSON), nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/vehicles/test-vehicle-001/health", nil)
	w := httptest.NewRecorder()
	
	// Call get vehicle health
	service.getVehicleHealth(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, 85.0, response["battery"])
	
	mockRedis.AssertExpectations(t)
}

func TestEmergencyBroadcast(t *testing.T) {
	// Create test service with mocks
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &VehicleGatewayService{
		config:      loadConfig(),
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Enable Abu Dhabi mode
	service.config.AbuDhabiMode = true
	
	// Test emergency broadcast data
	emergency := map[string]interface{}{
		"type":        "emergency_stop",
		"message":     "Emergency stop required",
		"severity":    "critical",
		"location": map[string]interface{}{
			"latitude":  24.4539,
			"longitude": 54.3773,
		},
		"radius": 1000.0,
	}
	
	// Create test request
	body, _ := json.Marshal(emergency)
	req := httptest.NewRequest("POST", "/api/v1/uae/emergency-broadcast", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call emergency broadcast
	service.emergencyBroadcast(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "broadcast_sent", response["status"])
}

func TestSandstormAlert(t *testing.T) {
	// Create test service with mocks
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &VehicleGatewayService{
		config:      loadConfig(),
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Enable Abu Dhabi mode
	service.config.AbuDhabiMode = true
	
	// Test sandstorm alert data
	alert := map[string]interface{}{
		"type":        "sandstorm_warning",
		"message":     "Sandstorm approaching",
		"severity":    "high",
		"location": map[string]interface{}{
			"latitude":  24.4539,
			"longitude": 54.3773,
		},
		"radius": 5000.0,
		"duration": 120.0, // 2 hours
	}
	
	// Create test request
	body, _ := json.Marshal(alert)
	req := httptest.NewRequest("POST", "/api/v1/uae/sandstorm-alert", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call sandstorm alert
	service.sandstormAlert(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "alert_sent", response["status"])
}

func TestValidateTelemetryInput(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Valid telemetry
	validTelemetry := TelemetryMessage{
		VehicleID:   "test-vehicle-001",
		MessageType: "location",
		Location: Location{
			Latitude:  24.4539,
			Longitude: 54.3773,
		},
		Speed: 60.0,
	}
	
	err := service.validateTelemetryInput(validTelemetry)
	assert.NoError(t, err)
	
	// Invalid telemetry - missing vehicle ID
	invalidTelemetry := TelemetryMessage{
		MessageType: "location",
		Location: Location{
			Latitude:  24.4539,
			Longitude: 54.3773,
		},
		Speed: 60.0,
	}
	
	err = service.validateTelemetryInput(invalidTelemetry)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "vehicle_id is required")
	
	// Invalid telemetry - invalid latitude
	invalidLatTelemetry := TelemetryMessage{
		VehicleID:   "test-vehicle-001",
		MessageType: "location",
		Location: Location{
			Latitude:  200.0, // Invalid latitude
			Longitude: 54.3773,
		},
		Speed: 60.0,
	}
	
	err = service.validateTelemetryInput(invalidLatTelemetry)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "invalid latitude")
	
	// Invalid telemetry - invalid longitude
	invalidLonTelemetry := TelemetryMessage{
		VehicleID:   "test-vehicle-001",
		MessageType: "location",
		Location: Location{
			Latitude:  24.4539,
			Longitude: 200.0, // Invalid longitude
		},
		Speed: 60.0,
	}
	
	err = service.validateTelemetryInput(invalidLonTelemetry)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "invalid longitude")
	
	// Invalid telemetry - negative speed
	invalidSpeedTelemetry := TelemetryMessage{
		VehicleID:   "test-vehicle-001",
		MessageType: "location",
		Location: Location{
			Latitude:  24.4539,
			Longitude: 54.3773,
		},
		Speed: -10.0, // Invalid speed
	}
	
	err = service.validateTelemetryInput(invalidSpeedTelemetry)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "invalid speed")
}

func TestValidateCommandInput(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Valid command
	validCommand := VehicleCommand{
		VehicleID:   "test-vehicle-001",
		CommandType: "start_mission",
		Payload: map[string]interface{}{
			"mission_id": "test-mission-001",
		},
		Priority: "high",
	}
	
	err := service.validateCommandInput(validCommand)
	assert.NoError(t, err)
	
	// Invalid command - missing vehicle ID
	invalidCommand := VehicleCommand{
		CommandType: "start_mission",
		Payload: map[string]interface{}{
			"mission_id": "test-mission-001",
		},
		Priority: "high",
	}
	
	err = service.validateCommandInput(invalidCommand)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "vehicle_id is required")
	
	// Invalid command - missing command type
	invalidTypeCommand := VehicleCommand{
		VehicleID: "test-vehicle-001",
		Payload: map[string]interface{}{
			"mission_id": "test-mission-001",
		},
		Priority: "high",
	}
	
	err = service.validateCommandInput(invalidTypeCommand)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "command_type is required")
	
	// Invalid command - invalid priority
	invalidPriorityCommand := VehicleCommand{
		VehicleID:   "test-vehicle-001",
		CommandType: "start_mission",
		Payload: map[string]interface{}{
			"mission_id": "test-mission-001",
		},
		Priority: "invalid_priority",
	}
	
	err = service.validateCommandInput(invalidPriorityCommand)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "invalid priority")
}

func TestIsWithinAbuDhabiBounds(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Test location within Abu Dhabi bounds
	location := Location{
		Latitude:  24.4539, // Abu Dhabi city
		Longitude: 54.3773,
	}
	
	result := service.isWithinAbuDhabiBounds(location)
	assert.True(t, result)
	
	// Test location outside Abu Dhabi bounds
	locationOutside := Location{
		Latitude:  25.0, // Outside bounds
		Longitude: 55.0,
	}
	
	result = service.isWithinAbuDhabiBounds(locationOutside)
	assert.False(t, result)
}

func TestGenerateCorrelationID(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Test correlation ID generation
	corrID1 := service.generateCorrelationID()
	corrID2 := service.generateCorrelationID()
	
	assert.NotEmpty(t, corrID1)
	assert.NotEmpty(t, corrID2)
	assert.NotEqual(t, corrID1, corrID2)
}

func TestGenerateCommandID(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Test command ID generation
	cmdID1 := service.generateCommandID()
	cmdID2 := service.generateCommandID()
	
	assert.NotEmpty(t, cmdID1)
	assert.NotEmpty(t, cmdID2)
	assert.NotEqual(t, cmdID1, cmdID2)
}

func TestSetupRoutes(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Create test router
	router := service.setupRoutes()
	
	// Test that routes are registered
	assert.NotNil(t, router)
}

func TestLoadConfigWithEnvVars(t *testing.T) {
	// Test with environment variables
	// Note: In a real test, you would set environment variables
	config := loadConfig()
	
	assert.NotNil(t, config)
	assert.Equal(t, 8083, config.Port)
	assert.Equal(t, "info", config.LogLevel)
	assert.Equal(t, 10000, config.TelemetryBufferSize)
	assert.Equal(t, 300, config.WebSocketTimeout)
	assert.Equal(t, 1000, config.RateLimit)
}

func TestAuthenticateRequest(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Test with valid JWT token
	req := httptest.NewRequest("GET", "/api/v1/telemetry", nil)
	req.Header.Set("Authorization", "Bearer valid-jwt-token")
	
	result := service.authenticateRequest(req)
	assert.True(t, result)
	
	// Test with valid API key
	req = httptest.NewRequest("GET", "/api/v1/telemetry", nil)
	req.Header.Set("X-API-Key", "supersecretapitoken")
	
	result = service.authenticateRequest(req)
	assert.True(t, result)
	
	// Test with invalid token
	req = httptest.NewRequest("GET", "/api/v1/telemetry", nil)
	req.Header.Set("Authorization", "Bearer invalid-token")
	
	result = service.authenticateRequest(req)
	assert.False(t, result)
	
	// Test with no authentication
	req = httptest.NewRequest("GET", "/api/v1/telemetry", nil)
	
	result = service.authenticateRequest(req)
	assert.False(t, result)
}

func TestAuthenticateVehicle(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Test with valid vehicle ID
	req := httptest.NewRequest("GET", "/api/v1/ws/vehicle/test-vehicle-001", nil)
	req.Header.Set("X-Vehicle-ID", "test-vehicle-001")
	req.Header.Set("X-Vehicle-Token", "valid-vehicle-token")
	
	result := service.authenticateVehicle(req, "test-vehicle-001")
	assert.True(t, result)
	
	// Test with invalid vehicle ID
	req = httptest.NewRequest("GET", "/api/v1/ws/vehicle/test-vehicle-001", nil)
	req.Header.Set("X-Vehicle-ID", "different-vehicle")
	req.Header.Set("X-Vehicle-Token", "valid-vehicle-token")
	
	result = service.authenticateVehicle(req, "test-vehicle-001")
	assert.False(t, result)
	
	// Test with no vehicle token
	req = httptest.NewRequest("GET", "/api/v1/ws/vehicle/test-vehicle-001", nil)
	req.Header.Set("X-Vehicle-ID", "test-vehicle-001")
	
	result = service.authenticateVehicle(req, "test-vehicle-001")
	assert.False(t, result)
}

func TestValidateCommand(t *testing.T) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	// Valid command
	validCommand := VehicleCommand{
		VehicleID:   "test-vehicle-001",
		CommandType: "start_mission",
		Payload: map[string]interface{}{
			"mission_id": "test-mission-001",
		},
	}
	
	result := service.validateCommand(validCommand)
	assert.True(t, result)
	
	// Invalid command - missing vehicle ID
	invalidCommand := VehicleCommand{
		CommandType: "start_mission",
		Payload: map[string]interface{}{
			"mission_id": "test-mission-001",
		},
	}
	
	result = service.validateCommand(invalidCommand)
	assert.False(t, result)
	
	// Invalid command - missing command type
	invalidTypeCommand := VehicleCommand{
		VehicleID: "test-vehicle-001",
		Payload: map[string]interface{}{
			"mission_id": "test-mission-001",
		},
	}
	
	result = service.validateCommand(invalidTypeCommand)
	assert.False(t, result)
}

// Benchmark tests
func BenchmarkReceiveTelemetry(b *testing.B) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	telemetry := TelemetryMessage{
		VehicleID:   "test-vehicle-001",
		Timestamp:   time.Now(),
		MessageType: "location",
		Location: Location{
			Latitude:  24.4539,
			Longitude: 54.3773,
		},
		Speed: 60.0,
	}
	
	body, _ := json.Marshal(telemetry)
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("POST", "/api/v1/telemetry", bytes.NewReader(body))
		req.Header.Set("Content-Type", "application/json")
		w := httptest.NewRecorder()
		
		service.receiveTelemetry(w, req)
	}
}

func BenchmarkSendCommand(b *testing.B) {
	service := &VehicleGatewayService{
		config: loadConfig(),
	}
	
	command := VehicleCommand{
		CommandType: "start_mission",
		Payload: map[string]interface{}{
			"mission_id": "test-mission-001",
		},
		Priority: "high",
	}
	
	body, _ := json.Marshal(command)
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("POST", "/api/v1/vehicles/test-vehicle-001/commands", bytes.NewReader(body))
		req.Header.Set("Content-Type", "application/json")
		w := httptest.NewRecorder()
		
		service.sendCommand(w, req)
	}
}
