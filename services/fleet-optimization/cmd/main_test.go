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
type MockDatabase struct {
	mock.Mock
}

func (m *MockDatabase) QueryContext(ctx context.Context, query string, args ...interface{}) (interface{}, error) {
	mockArgs := m.Called(ctx, query, args)
	return mockArgs.Get(0), mockArgs.Error(1)
}

func (m *MockDatabase) ExecContext(ctx context.Context, query string, args ...interface{}) (interface{}, error) {
	mockArgs := m.Called(ctx, query, args)
	return mockArgs.Get(0), mockArgs.Error(1)
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

func TestLoadConfig(t *testing.T) {
	config := loadConfig()
	
	assert.NotNil(t, config)
	assert.Equal(t, 8080, config.Port)
	assert.Equal(t, "info", config.LogLevel)
	assert.Equal(t, 1000, config.BufferSize)
}

func TestHealthCheck(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
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

func TestListOptimizationRuns(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/optimization-runs", nil)
	w := httptest.NewRecorder()
	
	// Call list optimization runs
	service.listOptimizationRuns(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["optimization_runs"])
	
	mockDB.AssertExpectations(t)
}

func TestCreateOptimizationRun(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test optimization run data
	optimizationRun := map[string]interface{}{
		"name":        "Test Optimization Run",
		"description": "Test optimization run for fleet efficiency",
		"type":        "route_optimization",
		"fleet_id":    "fleet-001",
		"goals": []map[string]interface{}{
			{
				"type":        "minimize_distance",
				"weight":      0.5,
				"target":      100.0,
			},
			{
				"type":        "minimize_time",
				"weight":      0.3,
				"target":      60.0,
			},
			{
				"type":        "minimize_cost",
				"weight":      0.2,
				"target":      50.0,
			},
		},
		"constraints": []map[string]interface{}{
			{
				"type":  "vehicle_capacity",
				"value": 1000.0,
			},
			{
				"type":  "time_window",
				"value": 480.0, // 8 hours in minutes
			},
		},
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(optimizationRun)
	req := httptest.NewRequest("POST", "/api/v1/optimization-runs", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create optimization run
	service.createOptimizationRun(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "optimization_run_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestCreateOptimizationRunInvalidJSON(t *testing.T) {
	service := &FleetOptimizationService{
		config: loadConfig(),
	}
	
	// Create test request with invalid JSON
	req := httptest.NewRequest("POST", "/api/v1/optimization-runs", bytes.NewReader([]byte("invalid json")))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create optimization run
	service.createOptimizationRun(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestGetOptimizationRun(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/optimization-runs/test-optimization-001", nil)
	w := httptest.NewRecorder()
	
	// Call get optimization run
	service.getOptimizationRun(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["optimization_run"])
	
	mockDB.AssertExpectations(t)
}

func TestCancelOptimizationRun(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	req := httptest.NewRequest("POST", "/api/v1/optimization-runs/test-optimization-001/cancel", nil)
	w := httptest.NewRecorder()
	
	// Call cancel optimization run
	service.cancelOptimizationRun(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "optimization_run_cancelled", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestGetOptimizationResults(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/optimization-runs/test-optimization-001/results", nil)
	w := httptest.NewRecorder()
	
	// Call get optimization results
	service.getOptimizationResults(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["results"])
	
	mockDB.AssertExpectations(t)
}

func TestListOptimizationGoals(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/optimization-goals", nil)
	w := httptest.NewRecorder()
	
	// Call list optimization goals
	service.listOptimizationGoals(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["goals"])
	
	mockDB.AssertExpectations(t)
}

func TestCreateOptimizationGoal(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test optimization goal data
	goal := map[string]interface{}{
		"name":        "Minimize Distance",
		"description": "Minimize total distance traveled",
		"type":        "minimize_distance",
		"weight":      0.5,
		"target":      100.0,
		"unit":        "kilometers",
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(goal)
	req := httptest.NewRequest("POST", "/api/v1/optimization-goals", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create optimization goal
	service.createOptimizationGoal(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "goal_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestGetOptimizationGoal(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/optimization-goals/test-goal-001", nil)
	w := httptest.NewRecorder()
	
	// Call get optimization goal
	service.getOptimizationGoal(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["goal"])
	
	mockDB.AssertExpectations(t)
}

func TestUpdateOptimizationGoal(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test optimization goal update data
	updateData := map[string]interface{}{
		"name":        "Updated Goal",
		"description": "Updated goal description",
		"weight":      0.7,
		"target":      80.0,
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(updateData)
	req := httptest.NewRequest("PUT", "/api/v1/optimization-goals/test-goal-001", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call update optimization goal
	service.updateOptimizationGoal(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "goal_updated", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestDeleteOptimizationGoal(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis delete success
	mockRedis.On("Del", mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	req := httptest.NewRequest("DELETE", "/api/v1/optimization-goals/test-goal-001", nil)
	w := httptest.NewRecorder()
	
	// Call delete optimization goal
	service.deleteOptimizationGoal(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "goal_deleted", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestGetOptimizationMetrics(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &FleetOptimizationService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/optimization/metrics", nil)
	w := httptest.NewRecorder()
	
	// Call get optimization metrics
	service.getOptimizationMetrics(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["metrics"])
	
	mockDB.AssertExpectations(t)
}

func TestValidateOptimizationRunInput(t *testing.T) {
	service := &FleetOptimizationService{
		config: loadConfig(),
	}
	
	// Valid optimization run
	validRun := map[string]interface{}{
		"name":        "Test Optimization Run",
		"description": "Test optimization run for fleet efficiency",
		"type":        "route_optimization",
		"fleet_id":    "fleet-001",
		"goals": []map[string]interface{}{
			{
				"type":   "minimize_distance",
				"weight": 0.5,
				"target": 100.0,
			},
		},
		"constraints": []map[string]interface{}{
			{
				"type":  "vehicle_capacity",
				"value": 1000.0,
			},
		},
	}
	
	err := service.validateOptimizationRunInput(validRun)
	assert.NoError(t, err)
	
	// Invalid optimization run - missing name
	invalidRun := map[string]interface{}{
		"description": "Test optimization run for fleet efficiency",
		"type":        "route_optimization",
		"fleet_id":    "fleet-001",
		"goals": []map[string]interface{}{
			{
				"type":   "minimize_distance",
				"weight": 0.5,
				"target": 100.0,
			},
		},
		"constraints": []map[string]interface{}{
			{
				"type":  "vehicle_capacity",
				"value": 1000.0,
			},
		},
	}
	
	err = service.validateOptimizationRunInput(invalidRun)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "name is required")
	
	// Invalid optimization run - empty goals
	invalidGoalsRun := map[string]interface{}{
		"name":        "Test Optimization Run",
		"description": "Test optimization run for fleet efficiency",
		"type":        "route_optimization",
		"fleet_id":    "fleet-001",
		"goals":       []map[string]interface{}{},
		"constraints": []map[string]interface{}{
			{
				"type":  "vehicle_capacity",
				"value": 1000.0,
			},
		},
	}
	
	err = service.validateOptimizationRunInput(invalidGoalsRun)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "at least one goal is required")
}

func TestValidateOptimizationGoalInput(t *testing.T) {
	service := &FleetOptimizationService{
		config: loadConfig(),
	}
	
	// Valid optimization goal
	validGoal := map[string]interface{}{
		"name":        "Minimize Distance",
		"description": "Minimize total distance traveled",
		"type":        "minimize_distance",
		"weight":      0.5,
		"target":      100.0,
		"unit":        "kilometers",
	}
	
	err := service.validateOptimizationGoalInput(validGoal)
	assert.NoError(t, err)
	
	// Invalid optimization goal - missing name
	invalidGoal := map[string]interface{}{
		"description": "Minimize total distance traveled",
		"type":        "minimize_distance",
		"weight":      0.5,
		"target":      100.0,
		"unit":        "kilometers",
	}
	
	err = service.validateOptimizationGoalInput(invalidGoal)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "name is required")
	
	// Invalid optimization goal - invalid weight
	invalidWeightGoal := map[string]interface{}{
		"name":        "Minimize Distance",
		"description": "Minimize total distance traveled",
		"type":        "minimize_distance",
		"weight":      1.5, // Invalid weight > 1.0
		"target":      100.0,
		"unit":        "kilometers",
	}
	
	err = service.validateOptimizationGoalInput(invalidWeightGoal)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "weight must be between 0 and 1")
}

func TestGenerateUUID(t *testing.T) {
	service := &FleetOptimizationService{
		config: loadConfig(),
	}
	
	// Test UUID generation
	uuid1 := service.generateUUID()
	uuid2 := service.generateUUID()
	
	assert.NotEmpty(t, uuid1)
	assert.NotEmpty(t, uuid2)
	assert.NotEqual(t, uuid1, uuid2)
	assert.Len(t, uuid1, 36) // UUID v4 format
	assert.Len(t, uuid2, 36) // UUID v4 format
}

func TestSetupRoutes(t *testing.T) {
	service := &FleetOptimizationService{
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
	service := &FleetOptimizationService{
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
	service := &FleetOptimizationService{
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
func BenchmarkCreateOptimizationRun(b *testing.B) {
	service := &FleetOptimizationService{
		config: loadConfig(),
	}
	
	optimizationRun := map[string]interface{}{
		"name":        "Test Optimization Run",
		"description": "Test optimization run for fleet efficiency",
		"type":        "route_optimization",
		"fleet_id":    "fleet-001",
		"goals": []map[string]interface{}{
			{
				"type":   "minimize_distance",
				"weight": 0.5,
				"target": 100.0,
			},
		},
		"constraints": []map[string]interface{}{
			{
				"type":  "vehicle_capacity",
				"value": 1000.0,
			},
		},
	}
	
	body, _ := json.Marshal(optimizationRun)
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("POST", "/api/v1/optimization-runs", bytes.NewReader(body))
		req.Header.Set("Content-Type", "application/json")
		w := httptest.NewRecorder()
		
		service.createOptimizationRun(w, req)
	}
}

func BenchmarkListOptimizationRuns(b *testing.B) {
	service := &FleetOptimizationService{
		config: loadConfig(),
	}
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("GET", "/api/v1/optimization-runs", nil)
		w := httptest.NewRecorder()
		
		service.listOptimizationRuns(w, req)
	}
}
