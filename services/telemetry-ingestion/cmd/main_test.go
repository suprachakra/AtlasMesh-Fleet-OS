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

func TestLoadConfig(t *testing.T) {
	config := loadConfig()
	
	assert.NotNil(t, config)
	assert.Equal(t, 8080, config.Port)
	assert.Equal(t, "info", config.LogLevel)
	assert.Equal(t, 1000, config.BufferSize)
}

func TestHealthCheck(t *testing.T) {
	// Create test service with mocks
	mockKafka := &MockKafkaWriter{}
	mockRedis := &MockRedisClient{}
	
	service := &TelemetryIngestionService{
		config:      loadConfig(),
		kafkaWriter: mockKafka,
		redis:       mockRedis,
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
	mockKafka := &MockKafkaWriter{}
	mockRedis := &MockRedisClient{}
	
	service := &TelemetryIngestionService{
		config:      loadConfig(),
		kafkaWriter: mockKafka,
		redis:       mockRedis,
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
	}
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(telemetry)
	req := httptest.NewRequest("POST", "/telemetry", bytes.NewReader(body))
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
	service := &TelemetryIngestionService{
		config: loadConfig(),
	}
	
	// Create test request with invalid JSON
	req := httptest.NewRequest("POST", "/telemetry", bytes.NewReader([]byte("invalid json")))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call receive telemetry
	service.receiveTelemetry(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestReceiveBatchTelemetry(t *testing.T) {
	// Create test service with mocks
	mockKafka := &MockKafkaWriter{}
	mockRedis := &MockRedisClient{}
	
	service := &TelemetryIngestionService{
		config:      loadConfig(),
		kafkaWriter: mockKafka,
		redis:       mockRedis,
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
	req := httptest.NewRequest("POST", "/telemetry/batch", bytes.NewReader(body))
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

func TestValidateTelemetryInput(t *testing.T) {
	service := &TelemetryIngestionService{
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
}

func TestProcessTelemetryMessage(t *testing.T) {
	// Create test service with mocks
	mockKafka := &MockKafkaWriter{}
	mockRedis := &MockRedisClient{}
	
	service := &TelemetryIngestionService{
		config:      loadConfig(),
		kafkaWriter: mockKafka,
		redis:       mockRedis,
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
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Process telemetry
	err := service.processTelemetryMessage(telemetry)
	
	// Assertions
	assert.NoError(t, err)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestProcessTelemetryMessageRedisError(t *testing.T) {
	// Create test service with mocks
	mockKafka := &MockKafkaWriter{}
	mockRedis := &MockRedisClient{}
	
	service := &TelemetryIngestionService{
		config:      loadConfig(),
		kafkaWriter: mockKafka,
		redis:       mockRedis,
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
	
	// Mock Redis set failure
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(assert.AnError)
	
	// Process telemetry
	err := service.processTelemetryMessage(telemetry)
	
	// Assertions
	assert.Error(t, err)
	mockRedis.AssertExpectations(t)
}

func TestProcessTelemetryMessageKafkaError(t *testing.T) {
	// Create test service with mocks
	mockKafka := &MockKafkaWriter{}
	mockRedis := &MockRedisClient{}
	
	service := &TelemetryIngestionService{
		config:      loadConfig(),
		kafkaWriter: mockKafka,
		redis:       mockRedis,
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
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write failure
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(assert.AnError)
	
	// Process telemetry
	err := service.processTelemetryMessage(telemetry)
	
	// Assertions
	assert.Error(t, err)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestStartTelemetryProcessor(t *testing.T) {
	// Create test service with mocks
	mockKafka := &MockKafkaWriter{}
	mockRedis := &MockRedisClient{}
	
	service := &TelemetryIngestionService{
		config:      loadConfig(),
		kafkaWriter: mockKafka,
		redis:       mockRedis,
	}
	
	// Create a small buffer for testing
	service.telemetryBuffer = make(chan TelemetryMessage, 1)
	
	// Start processor in background
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	
	go service.startTelemetryProcessor(ctx)
	
	// Send test message
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
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Send message to buffer
	select {
	case service.telemetryBuffer <- telemetry:
		// Message sent successfully
	case <-time.After(1 * time.Second):
		t.Fatal("Failed to send message to buffer")
	}
	
	// Wait a bit for processing
	time.Sleep(100 * time.Millisecond)
	
	// Cancel context to stop processor
	cancel()
	
	// Wait for processor to stop
	time.Sleep(100 * time.Millisecond)
	
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestMetricsCollection(t *testing.T) {
	service := &TelemetryIngestionService{
		config: loadConfig(),
	}
	
	// Test metrics collection
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	
	go service.startMetricsCollection(ctx)
	
	// Wait a bit for metrics collection
	time.Sleep(100 * time.Millisecond)
	
	// Cancel context to stop metrics collection
	cancel()
	
	// Wait for metrics collection to stop
	time.Sleep(100 * time.Millisecond)
}

func TestSetupRoutes(t *testing.T) {
	service := &TelemetryIngestionService{
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
	assert.Equal(t, 8080, config.Port)
	assert.Equal(t, "info", config.LogLevel)
	assert.Equal(t, 1000, config.BufferSize)
	assert.Equal(t, 5*time.Minute, config.RedisTTL)
}

func TestGenerateCorrelationID(t *testing.T) {
	service := &TelemetryIngestionService{
		config: loadConfig(),
	}
	
	// Test correlation ID generation
	corrID1 := service.generateCorrelationID()
	corrID2 := service.generateCorrelationID()
	
	assert.NotEmpty(t, corrID1)
	assert.NotEmpty(t, corrID2)
	assert.NotEqual(t, corrID1, corrID2)
}

func TestGenerateMessageID(t *testing.T) {
	service := &TelemetryIngestionService{
		config: loadConfig(),
	}
	
	// Test message ID generation
	msgID1 := service.generateMessageID()
	msgID2 := service.generateMessageID()
	
	assert.NotEmpty(t, msgID1)
	assert.NotEmpty(t, msgID2)
	assert.NotEqual(t, msgID1, msgID2)
}

// Benchmark tests
func BenchmarkReceiveTelemetry(b *testing.B) {
	service := &TelemetryIngestionService{
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
		req := httptest.NewRequest("POST", "/telemetry", bytes.NewReader(body))
		req.Header.Set("Content-Type", "application/json")
		w := httptest.NewRecorder()
		
		service.receiveTelemetry(w, req)
	}
}

func BenchmarkProcessTelemetryMessage(b *testing.B) {
	service := &TelemetryIngestionService{
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
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		service.processTelemetryMessage(telemetry)
	}
}
