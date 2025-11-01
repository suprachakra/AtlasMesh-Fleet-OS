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
	
	service := &MissionManagementService{
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

func TestListMissions(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/missions", nil)
	w := httptest.NewRecorder()
	
	// Call list missions
	service.listMissions(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["missions"])
	
	mockDB.AssertExpectations(t)
}

func TestCreateMission(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test mission data
	mission := map[string]interface{}{
		"name":        "Test Mission",
		"description": "Test mission for delivery",
		"type":        "delivery",
		"priority":    "high",
		"fleet_id":    "fleet-001",
		"vehicle_ids": []string{"vehicle-001", "vehicle-002"},
		"start_time":  time.Now().Add(1 * time.Hour).Format(time.RFC3339),
		"end_time":    time.Now().Add(2 * time.Hour).Format(time.RFC3339),
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(mission)
	req := httptest.NewRequest("POST", "/api/v1/missions", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create mission
	service.createMission(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "mission_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestCreateMissionInvalidJSON(t *testing.T) {
	service := &MissionManagementService{
		config: loadConfig(),
	}
	
	// Create test request with invalid JSON
	req := httptest.NewRequest("POST", "/api/v1/missions", bytes.NewReader([]byte("invalid json")))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create mission
	service.createMission(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func TestGetMission(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/missions/test-mission-001", nil)
	w := httptest.NewRecorder()
	
	// Call get mission
	service.getMission(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["mission"])
	
	mockDB.AssertExpectations(t)
}

func TestUpdateMission(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test mission update data
	updateData := map[string]interface{}{
		"name":        "Updated Mission",
		"description": "Updated mission description",
		"priority":    "medium",
	}
	
	// Mock database exec success
	mockDB.On("ExecContext", mock.Anything, mock.Anything, mock.Anything).Return(nil, nil)
	
	// Mock Redis set success
	mockRedis.On("Set", mock.Anything, mock.Anything, mock.Anything, mock.Anything).Return(nil)
	
	// Mock Kafka write success
	mockKafka.On("WriteMessages", mock.Anything, mock.Anything).Return(nil)
	
	// Create test request
	body, _ := json.Marshal(updateData)
	req := httptest.NewRequest("PUT", "/api/v1/missions/test-mission-001", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call update mission
	service.updateMission(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "mission_updated", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestDeleteMission(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
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
	req := httptest.NewRequest("DELETE", "/api/v1/missions/test-mission-001", nil)
	w := httptest.NewRecorder()
	
	// Call delete mission
	service.deleteMission(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "mission_deleted", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestStartMission(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
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
	req := httptest.NewRequest("POST", "/api/v1/missions/test-mission-001/start", nil)
	w := httptest.NewRecorder()
	
	// Call start mission
	service.startMission(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "mission_started", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestPauseMission(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
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
	req := httptest.NewRequest("POST", "/api/v1/missions/test-mission-001/pause", nil)
	w := httptest.NewRecorder()
	
	// Call pause mission
	service.pauseMission(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "mission_paused", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestResumeMission(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
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
	req := httptest.NewRequest("POST", "/api/v1/missions/test-mission-001/resume", nil)
	w := httptest.NewRecorder()
	
	// Call resume mission
	service.resumeMission(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "mission_resumed", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestCompleteMission(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
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
	req := httptest.NewRequest("POST", "/api/v1/missions/test-mission-001/complete", nil)
	w := httptest.NewRecorder()
	
	// Call complete mission
	service.completeMission(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "mission_completed", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestCancelMission(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
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
	req := httptest.NewRequest("POST", "/api/v1/missions/test-mission-001/cancel", nil)
	w := httptest.NewRecorder()
	
	// Call cancel mission
	service.cancelMission(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "mission_cancelled", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestListMissionTemplates(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/mission-templates", nil)
	w := httptest.NewRecorder()
	
	// Call list mission templates
	service.listMissionTemplates(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["templates"])
	
	mockDB.AssertExpectations(t)
}

func TestCreateMissionTemplate(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Test mission template data
	template := map[string]interface{}{
		"name":        "Delivery Template",
		"description": "Template for delivery missions",
		"type":        "delivery",
		"priority":    "medium",
		"tasks": []map[string]interface{}{
			{
				"name":        "Pickup",
				"description": "Pick up package",
				"type":        "pickup",
				"order":       1,
			},
			{
				"name":        "Delivery",
				"description": "Deliver package",
				"type":        "delivery",
				"order":       2,
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
	body, _ := json.Marshal(template)
	req := httptest.NewRequest("POST", "/api/v1/mission-templates", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	
	// Call create mission template
	service.createMissionTemplate(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusCreated, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.Equal(t, "template_created", response["status"])
	
	mockDB.AssertExpectations(t)
	mockRedis.AssertExpectations(t)
	mockKafka.AssertExpectations(t)
}

func TestGetMissionStatus(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/missions/test-mission-001/status", nil)
	w := httptest.NewRecorder()
	
	// Call get mission status
	service.getMissionStatus(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["status"])
	
	mockDB.AssertExpectations(t)
}

func TestGetMissionProgress(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/missions/test-mission-001/progress", nil)
	w := httptest.NewRecorder()
	
	// Call get mission progress
	service.getMissionProgress(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["progress"])
	
	mockDB.AssertExpectations(t)
}

func TestGetMissionAuditLog(t *testing.T) {
	// Create test service with mocks
	mockDB := &MockDatabase{}
	mockRedis := &MockRedisClient{}
	mockKafka := &MockKafkaWriter{}
	
	service := &MissionManagementService{
		config:      loadConfig(),
		database:    mockDB,
		redis:       mockRedis,
		kafkaWriter: mockKafka,
	}
	
	// Mock database query success
	mockDB.On("QueryContext", mock.Anything, mock.Anything, mock.Anything).Return([]interface{}{}, nil)
	
	// Create test request
	req := httptest.NewRequest("GET", "/api/v1/missions/test-mission-001/audit-log", nil)
	w := httptest.NewRecorder()
	
	// Call get mission audit log
	service.getMissionAuditLog(w, req)
	
	// Assertions
	assert.Equal(t, http.StatusOK, w.Code)
	
	var response map[string]interface{}
	err := json.Unmarshal(w.Body.Bytes(), &response)
	assert.NoError(t, err)
	assert.NotNil(t, response["audit_log"])
	
	mockDB.AssertExpectations(t)
}

func TestValidateMissionInput(t *testing.T) {
	service := &MissionManagementService{
		config: loadConfig(),
	}
	
	// Valid mission
	validMission := map[string]interface{}{
		"name":        "Test Mission",
		"description": "Test mission for delivery",
		"type":        "delivery",
		"priority":    "high",
		"fleet_id":    "fleet-001",
		"vehicle_ids": []string{"vehicle-001", "vehicle-002"},
		"start_time":  time.Now().Add(1 * time.Hour).Format(time.RFC3339),
		"end_time":    time.Now().Add(2 * time.Hour).Format(time.RFC3339),
	}
	
	err := service.validateMissionInput(validMission)
	assert.NoError(t, err)
	
	// Invalid mission - missing name
	invalidMission := map[string]interface{}{
		"description": "Test mission for delivery",
		"type":        "delivery",
		"priority":    "high",
		"fleet_id":    "fleet-001",
		"vehicle_ids": []string{"vehicle-001", "vehicle-002"},
		"start_time":  time.Now().Add(1 * time.Hour).Format(time.RFC3339),
		"end_time":    time.Now().Add(2 * time.Hour).Format(time.RFC3339),
	}
	
	err = service.validateMissionInput(invalidMission)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "name is required")
	
	// Invalid mission - empty vehicle_ids
	invalidVehicleIdsMission := map[string]interface{}{
		"name":        "Test Mission",
		"description": "Test mission for delivery",
		"type":        "delivery",
		"priority":    "high",
		"fleet_id":    "fleet-001",
		"vehicle_ids": []string{},
		"start_time":  time.Now().Add(1 * time.Hour).Format(time.RFC3339),
		"end_time":    time.Now().Add(2 * time.Hour).Format(time.RFC3339),
	}
	
	err = service.validateMissionInput(invalidVehicleIdsMission)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "at least one vehicle is required")
}

func TestValidateMissionTemplateInput(t *testing.T) {
	service := &MissionManagementService{
		config: loadConfig(),
	}
	
	// Valid mission template
	validTemplate := map[string]interface{}{
		"name":        "Delivery Template",
		"description": "Template for delivery missions",
		"type":        "delivery",
		"priority":    "medium",
		"tasks": []map[string]interface{}{
			{
				"name":        "Pickup",
				"description": "Pick up package",
				"type":        "pickup",
				"order":       1,
			},
		},
	}
	
	err := service.validateMissionTemplateInput(validTemplate)
	assert.NoError(t, err)
	
	// Invalid template - missing name
	invalidTemplate := map[string]interface{}{
		"description": "Template for delivery missions",
		"type":        "delivery",
		"priority":    "medium",
		"tasks": []map[string]interface{}{
			{
				"name":        "Pickup",
				"description": "Pick up package",
				"type":        "pickup",
				"order":       1,
			},
		},
	}
	
	err = service.validateMissionTemplateInput(invalidTemplate)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "name is required")
	
	// Invalid template - empty tasks
	invalidTasksTemplate := map[string]interface{}{
		"name":        "Delivery Template",
		"description": "Template for delivery missions",
		"type":        "delivery",
		"priority":    "medium",
		"tasks":       []map[string]interface{}{},
	}
	
	err = service.validateMissionTemplateInput(invalidTasksTemplate)
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "at least one task is required")
}

func TestGenerateUUID(t *testing.T) {
	service := &MissionManagementService{
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
	service := &MissionManagementService{
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
	service := &MissionManagementService{
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
	service := &MissionManagementService{
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
func BenchmarkCreateMission(b *testing.B) {
	service := &MissionManagementService{
		config: loadConfig(),
	}
	
	mission := map[string]interface{}{
		"name":        "Test Mission",
		"description": "Test mission for delivery",
		"type":        "delivery",
		"priority":    "high",
		"fleet_id":    "fleet-001",
		"vehicle_ids": []string{"vehicle-001", "vehicle-002"},
		"start_time":  time.Now().Add(1 * time.Hour).Format(time.RFC3339),
		"end_time":    time.Now().Add(2 * time.Hour).Format(time.RFC3339),
	}
	
	body, _ := json.Marshal(mission)
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("POST", "/api/v1/missions", bytes.NewReader(body))
		req.Header.Set("Content-Type", "application/json")
		w := httptest.NewRecorder()
		
		service.createMission(w, req)
	}
}

func BenchmarkListMissions(b *testing.B) {
	service := &MissionManagementService{
		config: loadConfig(),
	}
	
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		req := httptest.NewRequest("GET", "/api/v1/missions", nil)
		w := httptest.NewRecorder()
		
		service.listMissions(w, req)
	}
}
